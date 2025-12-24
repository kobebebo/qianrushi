#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>

#include "appdef.h"
#include "museum_samples.h"

#define LOG_TAG "museum"
#define LOG_I(...) rt_kprintf("[museum] " __VA_ARGS__)
#define LOG_W(...) rt_kprintf("[museum][W] " __VA_ARGS__)
#define LOG_E(...) rt_kprintf("[museum][E] " __VA_ARGS__)

#define SOF0 0xAA
#define SOF1 0x55

#define FRAME_MAX 128
#define PAYLOAD_MAX 96

#define TYPE_PERIODIC 0x01
#define TYPE_ALARM    0x02
#define TYPE_CMD_REQ  0x10
#define TYPE_CMD_RESP 0x11

#define RAW_QUEUE_LEN 16
#define REPORT_QUEUE_LEN 16
#define CMD_QUEUE_LEN 4

#define AVG_WINDOW 5
#define CAP_SPIKE_THRESHOLD 180.0f
#define CAP_IIR_ALPHA 0.18f
#define CAP_BASELINE_LAMBDA 0.002f
#define ALARM_HOLD_TICKS 10
#define ALARM_CONSEC_REQ 3

#define DEFAULT_PERIOD_MS 500
#define DEFAULT_CAP_THRESHOLD 520.0f

static rt_device_t serial_dev = RT_NULL;
static struct rt_semaphore sample_sem;
static struct rt_semaphore rx_sem;
static rt_timer_t sample_timer = RT_NULL;
static rt_mq_t raw_mq = RT_NULL;
static rt_mq_t report_mq = RT_NULL;

static rt_thread_t sample_tid = RT_NULL;
static rt_thread_t process_tid = RT_NULL;
static rt_thread_t comm_tid = RT_NULL;
static rt_thread_t cmd_tid = RT_NULL;

static struct
{
    rt_uint32_t period_ms;
    float cap_threshold;
    rt_bool_t encrypt_enabled;
} g_config = {DEFAULT_PERIOD_MS, DEFAULT_CAP_THRESHOLD, RT_TRUE};

struct raw_msg
{
    rt_uint16_t index;
    museum_sample_t raw;
};

struct report_msg
{
    rt_uint16_t index;
    museum_sample_t raw;
    float temp_filt;
    float humi_filt;
    float light_filt;
    float cap_filt;
    rt_bool_t alarming;
    rt_uint8_t type;
};

struct cap_filter_state
{
    float filtered;
    float baseline;
    float last_raw;
};

struct avg_state
{
    float buf[AVG_WINDOW];
    int pos;
    int filled;
};

static rt_err_t uart_rx_cb(rt_device_t dev, rt_size_t size)
{
    /* ISR-friendly: only signal the worker */
    rt_sem_release(&rx_sem);
    return RT_EOK;
}

static void sample_timer_cb(void *param)
{
    rt_sem_release(&sample_sem);
}

static float update_avg(struct avg_state *st, float v)
{
    st->buf[st->pos] = v;
    st->pos = (st->pos + 1) % AVG_WINDOW;
    if (st->filled < AVG_WINDOW)
        st->filled++;
    float sum = 0.0f;
    for (int i = 0; i < st->filled; i++)
    {
        sum += st->buf[i];
    }
    return sum / st->filled;
}

static float update_cap_filter(struct cap_filter_state *st, float raw)
{
    float corr = raw;
    /* Step1: spike clamp for capacitive glitches */
    float delta = raw - st->last_raw;
    if (st->last_raw != 0 && rt_abs(delta) > CAP_SPIKE_THRESHOLD)
    {
        corr = st->last_raw + (delta > 0 ? CAP_SPIKE_THRESHOLD : -CAP_SPIKE_THRESHOLD);
    }
    /* Step2: slow baseline tracking to remove drift */
    st->baseline = st->baseline + CAP_BASELINE_LAMBDA * (corr - st->baseline);
    corr -= st->baseline * 0.05f; /* small baseline compensation */
    /* Step3: IIR smoothing */
    st->filtered = st->filtered * (1.0f - CAP_IIR_ALPHA) + corr * CAP_IIR_ALPHA;
    st->last_raw = corr;
    return st->filtered;
}

static rt_uint16_t crc16_ibm(const rt_uint8_t *data, rt_size_t len)
{
    rt_uint16_t crc = 0xFFFF;
    for (rt_size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* Tiny-TEA 64-bit block cipher */
static const rt_uint32_t tea_key[4] = {0x12345678, 0x9ABCDEF0, 0x0F1E2D3C, 0x44556677};
static void tea_encrypt_block(rt_uint32_t *v)
{
    rt_uint32_t v0 = v[0], v1 = v[1];
    rt_uint32_t sum = 0, delta = 0x9e3779b9;
    for (int i = 0; i < 32; i++)
    {
        sum += delta;
        v0 += ((v1 << 4) + tea_key[0]) ^ (v1 + sum) ^ ((v1 >> 5) + tea_key[1]);
        v1 += ((v0 << 4) + tea_key[2]) ^ (v0 + sum) ^ ((v0 >> 5) + tea_key[3]);
    }
    v[0] = v0; v[1] = v1;
}

static rt_size_t encrypt_payload(rt_uint8_t *payload, rt_size_t len)
{
    if (!g_config.encrypt_enabled) return len;
    rt_size_t padded = (len + 7) & ~((rt_size_t)7);
    for (rt_size_t i = len; i < padded; i++)
        payload[i] = 0; /* zero padding */
    for (rt_size_t i = 0; i < padded; i += 8)
    {
        tea_encrypt_block((rt_uint32_t *)(payload + i));
    }
    return padded;
}

static void send_frame(rt_uint8_t type, rt_uint8_t *payload, rt_size_t payload_len)
{
    rt_uint8_t frame[FRAME_MAX];
    rt_size_t len = payload_len;
    if (len > PAYLOAD_MAX) len = PAYLOAD_MAX;
    if (g_config.encrypt_enabled && (type == TYPE_PERIODIC || type == TYPE_ALARM))
    {
        len = encrypt_payload(payload, len);
    }
    frame[0] = SOF0;
    frame[1] = SOF1;
    frame[2] = (rt_uint8_t)len;
    frame[3] = type;
    rt_memcpy(&frame[4], payload, len);
    rt_uint16_t crc = crc16_ibm(&frame[2], 2 + len);
    frame[4 + len] = crc & 0xFF;
    frame[5 + len] = (crc >> 8) & 0xFF;
    rt_size_t total = 6 + len - 1; /* because len already counts payload */
    /* Correct total size */
    total = 4 + len + 2;
    rt_device_write(serial_dev, 0, frame, total);
}

static void build_and_send_report(const struct report_msg *msg)
{
    rt_uint8_t payload[PAYLOAD_MAX];
    /* Payload layout: idx(2) | raw cap(2) | filt cap(2) | temp0.1C | hum0.1 | light | alarm | enc */
    rt_uint16_t cap_raw = (rt_uint16_t)(msg->raw.cap_raw);
    rt_uint16_t cap_filt = (rt_uint16_t)(msg->cap_filt);
    rt_int16_t temp = (rt_int16_t)(msg->temp_filt * 10);
    rt_int16_t humi = (rt_int16_t)(msg->humi_filt * 10);
    rt_uint16_t light = (rt_uint16_t)(msg->light_filt);
    payload[0] = msg->index & 0xFF;
    payload[1] = (msg->index >> 8) & 0xFF;
    payload[2] = cap_raw & 0xFF;
    payload[3] = (cap_raw >> 8) & 0xFF;
    payload[4] = cap_filt & 0xFF;
    payload[5] = (cap_filt >> 8) & 0xFF;
    payload[6] = temp & 0xFF;
    payload[7] = (temp >> 8) & 0xFF;
    payload[8] = humi & 0xFF;
    payload[9] = (humi >> 8) & 0xFF;
    payload[10] = light & 0xFF;
    payload[11] = (light >> 8) & 0xFF;
    payload[12] = msg->alarming;
    payload[13] = g_config.encrypt_enabled;
    send_frame(msg->type, payload, 14);
}

static void send_cmd_response(const char *text)
{
    rt_size_t len = rt_strlen(text);
    if (len > PAYLOAD_MAX) len = PAYLOAD_MAX;
    rt_uint8_t payload[PAYLOAD_MAX];
    rt_memcpy(payload, text, len);
    send_frame(TYPE_CMD_RESP, payload, len);
}

static void sample_thread_entry(void *parameter)
{
    rt_uint16_t idx = 0;
    while (1)
    {
        rt_sem_take(&sample_sem, RT_WAITING_FOREVER);
        struct raw_msg msg = {0};
        const museum_sample_t *s = &museum_samples[idx];
        msg.index = idx;
        msg.raw = *s;
        if (rt_mq_send(raw_mq, &msg, sizeof(msg)) != RT_EOK)
        {
            LOG_W("raw mq full\n");
        }
        idx = (idx + 1) % museum_sample_count;
    }
}

static void process_thread_entry(void *parameter)
{
    struct avg_state temp_avg = {0}, humi_avg = {0}, light_avg = {0};
    struct cap_filter_state cap_state = {0};
    int alarm_consec = 0;
    int alarm_hold = 0;
    rt_bool_t alarming = RT_FALSE;
    struct raw_msg raw;
    while (1)
    {
        rt_memset(&raw, 0, sizeof(raw));
        rt_mq_recv(raw_mq, &raw, sizeof(raw), RT_WAITING_FOREVER);
        float temp_f = update_avg(&temp_avg, raw.raw.temp_c);
        float humi_f = update_avg(&humi_avg, raw.raw.humi_pct);
        float light_f = update_avg(&light_avg, raw.raw.light_lux);
        float cap_f = update_cap_filter(&cap_state, raw.raw.cap_raw);

        if (cap_f > g_config.cap_threshold)
        {
            alarm_consec++;
        }
        else
        {
            alarm_consec = 0;
        }
        if (!alarming && alarm_consec >= ALARM_CONSEC_REQ)
        {
            alarming = RT_TRUE;
            alarm_hold = ALARM_HOLD_TICKS;
            LOG_W("Touch alarm triggered at idx %d\n", raw.index);
        }
        if (alarming)
        {
            if (alarm_hold > 0)
                alarm_hold--;
            else if (cap_f < g_config.cap_threshold * 0.7f)
                alarming = RT_FALSE;
        }

        struct report_msg rep = {0};
        rep.index = raw.index;
        rep.raw = raw.raw;
        rep.temp_filt = temp_f;
        rep.humi_filt = humi_f;
        rep.light_filt = light_f;
        rep.cap_filt = cap_f;
        rep.alarming = alarming;
        rep.type = alarming ? TYPE_ALARM : TYPE_PERIODIC;

        if (rt_mq_send(report_mq, &rep, sizeof(rep)) != RT_EOK)
        {
            LOG_W("report mq full\n");
        }
    }
}

static void comm_thread_entry(void *parameter)
{
    struct report_msg rep;
    while (1)
    {
        rt_memset(&rep, 0, sizeof(rep));
        rt_mq_recv(report_mq, &rep, sizeof(rep), RT_WAITING_FOREVER);
        build_and_send_report(&rep);
        LOG_I("tx[%d] raw cap %.1f filt %.1f alarm %d enc %d\n", rep.index,
              rep.raw.cap_raw, rep.cap_filt, rep.alarming, g_config.encrypt_enabled);
    }
}

static void apply_config_change(void)
{
    if (sample_timer)
    {
        rt_timer_control(sample_timer, RT_TIMER_CTRL_SET_TIME, &g_config.period_ms);
        rt_timer_start(sample_timer);
    }
}

static void parse_command(const char *cmd)
{
    if (!cmd || cmd[0] == '\0') return;
    if (!rt_strncasecmp(cmd, "ENC ON", 6))
    {
        g_config.encrypt_enabled = RT_TRUE;
        send_cmd_response("ENC=ON\n");
    }
    else if (!rt_strncasecmp(cmd, "ENC OFF", 7))
    {
        g_config.encrypt_enabled = RT_FALSE;
        send_cmd_response("ENC=OFF\n");
    }
    else if (!rt_strncasecmp(cmd, "SET PERIOD", 10))
    {
        int val = atoi(cmd + 10);
        if (val >= 100 && val <= 5000)
        {
            g_config.period_ms = val;
            apply_config_change();
            send_cmd_response("PERIOD UPDATED\n");
        }
        else
        {
            send_cmd_response("PERIOD RANGE 100-5000\n");
        }
    }
    else if (!rt_strncasecmp(cmd, "SET CAP_TH", 11))
    {
        float th = atof(cmd + 11);
        if (th > 100 && th < 2000)
        {
            g_config.cap_threshold = th;
            send_cmd_response("CAP_TH UPDATED\n");
        }
        else
        {
            send_cmd_response("CAP_TH RANGE 100-2000\n");
        }
    }
    else if (!rt_strncasecmp(cmd, "GET STATUS", 10))
    {
        char resp[64];
        rt_snprintf(resp, sizeof(resp), "PER=%ums TH=%.1f ENC=%d\n", g_config.period_ms, g_config.cap_threshold, g_config.encrypt_enabled);
        send_cmd_response(resp);
    }
    else
    {
        send_cmd_response("UNKNOWN CMD\n");
    }
}

static void cmd_thread_entry(void *parameter)
{
    char line[64];
    int pos = 0;
    while (1)
    {
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        rt_uint8_t buf[32];
        rt_size_t n = rt_device_read(serial_dev, 0, buf, sizeof(buf));
        for (rt_size_t i = 0; i < n; i++)
        {
            char c = (char)buf[i];
            if (c == '\r' || c == '\n')
            {
                line[pos] = '\0';
                parse_command(line);
                pos = 0;
            }
            else if (pos < (int)sizeof(line) - 1)
            {
                line[pos++] = c;
            }
        }
    }
}

static int init_comm_device(void)
{
    serial_dev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
    if (!serial_dev)
    {
        LOG_E("uart %s not found\n", RT_CONSOLE_DEVICE_NAME);
        return -1;
    }
    rt_err_t err = rt_device_open(serial_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if (err != RT_EOK)
    {
        LOG_E("open uart err %d\n", err);
        return -1;
    }
    rt_device_set_rx_indicate(serial_dev, uart_rx_cb);
    return 0;
}

static void start_threads(void)
{
    raw_mq = rt_mq_create("rawq", sizeof(struct raw_msg), RAW_QUEUE_LEN, RT_IPC_FLAG_FIFO);
    report_mq = rt_mq_create("repq", sizeof(struct report_msg), REPORT_QUEUE_LEN, RT_IPC_FLAG_FIFO);
    rt_sem_init(&sample_sem, "s_smp", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&rx_sem, "s_rx", 0, RT_IPC_FLAG_PRIO);

    sample_tid = rt_thread_create("samp", sample_thread_entry, RT_NULL, 2048, 12, 10);
    process_tid = rt_thread_create("proc", process_thread_entry, RT_NULL, 4096, 13, 10);
    comm_tid = rt_thread_create("comm", comm_thread_entry, RT_NULL, 2048, 14, 10);
    cmd_tid = rt_thread_create("cmd", cmd_thread_entry, RT_NULL, 2048, 15, 10);

    if (sample_tid) rt_thread_startup(sample_tid);
    if (process_tid) rt_thread_startup(process_tid);
    if (comm_tid) rt_thread_startup(comm_tid);
    if (cmd_tid) rt_thread_startup(cmd_tid);

    sample_timer = rt_timer_create("smtmr", sample_timer_cb, RT_NULL, g_config.period_ms,
                                   RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (sample_timer) rt_timer_start(sample_timer);
}

static void print_banner(void)
{
    LOG_I("Museum showcase micro-environment monitor ready.\n");
    LOG_I("Commands: ENC ON/OFF, SET PERIOD <ms>, SET CAP_TH <v>, GET STATUS\n");
}

int rt_user_main(int argc, char **argv)
{
    print_banner();
    if (init_comm_device() != 0)
    {
        return -1;
    }
    start_threads();
    return 0;
}

MSH_CMD_EXPORT_ALIAS(rt_user_main, museum_demo, start museum monitor demo);

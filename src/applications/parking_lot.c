#include <rtthread.h>
#include "appdef.h"

#define MAX_LOT 16

static unsigned short lot_status = 0; // 低16位表示车位
static unsigned char exit_flag = 0;
static rt_thread_t tid_parking = RT_NULL;
static rt_thread_t tid_sim = RT_NULL;
static rt_thread_t tid_seg = RT_NULL;
#define DELAY 50
// 十进制转BCD（仅支持0~99）
static inline unsigned char dec_to_bcd(unsigned char dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}
// 线程1：将 lot_status 写入 LED
void lot_handler(void *parameter) {
    while (1) {
        // 只显示低16位
        SET_LED(lot_status & 0xFFFF);
        if (exit_flag) {
            rt_kprintf("lot_handler检测到退出条件，线程退出\n");
            break;
        }
        rt_thread_mdelay(DELAY);
    }
    tid_parking = RT_NULL;
}

// 线程2：检测拨码开关，模拟进库/出库，串口输出状态
void car_sim_thread(void *parameter) {
    unsigned short last_sw = 0;
	unsigned short last_lot = 0;
    while (1) {
        unsigned int sw = READ_SW();
        unsigned char in = sw & 0x1;         // SW0
        unsigned char out = (sw >> 1) & 0x1; // SW1
        exit_flag = (sw >> 15) & 0x1;

        rt_ubase_t level = rt_hw_interrupt_disable(); // 保护临界区

        // 进库
        if (in && !(last_sw & 0x1)) {
            for (int i = 0; i < MAX_LOT; i++) {
                if (((lot_status >> i) & 0x1) == 0) {
                    lot_status |= (1 << i);
                    break;
                }
            }
        }
        // 出库
        if (out && !((last_sw >> 1) & 0x1)) {
            for (int i = 0; i < MAX_LOT; i++) {
                if (((lot_status >> i) & 0x1) == 1) {
                    lot_status &= ~(1 << i);
                    break;
                }
            }
        }

        rt_hw_interrupt_enable(level); // 恢复中断

        last_sw = sw & 0x3; // 只记录SW0、SW1

        // 统计
        int used = 0;
        for (int i = 0; i < MAX_LOT; i++) {
            if ((lot_status >> i) & 0x1) used++;
        }
        int left = MAX_LOT - used;
		if (left != last_lot){
        	rt_kprintf("总车位:%d, 已占用:%d, 剩余:%d\n", MAX_LOT, used, left);
			last_lot = left;
		}

        if (exit_flag) {
            rt_kprintf("car_sim_thread检测到退出条件，线程退出\n");
            break;
        }
        rt_thread_mdelay(DELAY);
    }
    tid_sim = RT_NULL;
}

// 线程3：将剩余车位数写入七段管
void seg_display_thread(void *parameter) {
    while (1) {
        // 统计剩余车位
        int used = 0;
        for (int i = 0; i < MAX_LOT; i++) {
            if ((lot_status >> i) & 0x1) used++;
        }
        int left = MAX_LOT - used;
        unsigned char bcd = dec_to_bcd(left); // 转换为BCD码
        SET_SegEn(0x00);           // 使能所有数码管（根据硬件实际调整）
        SET_SegDig(bcd);           // 显示BCD码
        if (exit_flag) {
            rt_kprintf("seg_display_thread检测到退出条件，线程退出\n");
            break;
        }
        rt_thread_mdelay(DELAY);
    }
    tid_seg = RT_NULL;
}

int parking_lot(void) {
	exit_flag = 0;
    tid_parking = rt_thread_create("lot_handler", lot_handler, NULL,
                                   THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    tid_sim = rt_thread_create("car_sim", car_sim_thread, NULL,
                               THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    tid_seg = rt_thread_create("seg_disp", seg_display_thread, NULL,
                               THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);

    if (tid_parking != RT_NULL) rt_thread_startup(tid_parking);
    if (tid_sim != RT_NULL) rt_thread_startup(tid_sim);
    if (tid_seg != RT_NULL) rt_thread_startup(tid_seg);
	rt_kprintf("停车场管理系统初始化完成\n");
    return 0;
}

MSH_CMD_EXPORT(parking_lot, Parking lot LED status display);
#ifndef PR_H
#define PR_H

/* 来源：REG_PFC28379D.zip / REG_LED_BLINK/USER/PR.h。
 * 通用准 PR 的直接 I 型实现；本文件不依赖 TI/STM32 外设。
 * 输入 Vi 为电流误差(A)，输出 Vo 为电压校正量(V)。
 */
struct PR_PATH
{
    float A0, A1, A2;       /* 已归一化的分子系数。 */
    float B0, B1, B2;       /* B0 为归一化前分母；递推只用 B1/B2。 */
    float Vi, Vi1, Vi2;     /* e[k], e[k-1], e[k-2]。 */
    float Vo, Vo1, Vo2;     /* u[k], u[k-1], u[k-2]；均为限幅前值。 */
};

/* Ts 单位 s；Wc/W0 单位 rad/s。调用方先验证参数。
 * 初始化一次计算系数，PR_update 每个控制周期调用一次。
 */
void PR_init(struct PR_PATH *PR, float Ts, float Kp, float Kr,
             float Wc, float W0);
void PR_update(struct PR_PATH *PR, float Vi);

/* 移植增加的复位接口：只清历史，不改变系数。 */
void PR_reset(struct PR_PATH *PR);

#endif

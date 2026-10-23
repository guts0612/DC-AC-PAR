#include "PR.h"

void PR_reset(struct PR_PATH *PR)
{
    PR->Vi = 0.0f;
    PR->Vi1 = 0.0f;
    PR->Vi2 = 0.0f;
    PR->Vo = 0.0f;
    PR->Vo1 = 0.0f;
    PR->Vo2 = 0.0f;
}

void PR_init(struct PR_PATH *PR, float Ts, float Kp, float Kr,
             float Wc, float W0)
{
    PR_reset(PR);

    /* 保留压缩包 PR_init 的系数表达式及 float 存储方式。
     * G(s)=Kp+2*Kr*Wc*s/(s*s+2*Wc*s+W0*W0)，Tustin 离散。
     * double 显式转换等价于原式 4.0/8.0 造成的隐式提升，避免 ARMCC 警告。
     * 括号内原本的 float 乘加顺序保持不变；系数运算仅在初始化时发生。
     * B0 包含 4/Ts^2，不能直接把原 PFC 的 25 us 复制到本机。
     */
    PR->B0 = (float)((4.0 / (double)(Ts * Ts)) +
                    (4.0 * (double)Wc / (double)Ts) + (double)(W0 * W0));
    PR->A0 = (float)(((4.0 * (double)Kp / (double)(Ts * Ts)) +
                     (4.0 * (double)Wc * (double)(Kp + Kr) / (double)Ts) +
                     (double)(Kp * W0 * W0)) / (double)PR->B0);
    PR->A1 = (float)(((-8.0 * (double)Kp / (double)(Ts * Ts)) +
                     (2.0 * (double)Kp * (double)W0 * (double)W0)) / (double)PR->B0);
    PR->A2 = (float)(((4.0 * (double)Kp / (double)(Ts * Ts)) -
                     (4.0 * (double)Wc * (double)(Kp + Kr) / (double)Ts) +
                     (double)(Kp * W0 * W0)) / (double)PR->B0);
    PR->B1 = (float)(((-8.0 / (double)(Ts * Ts)) +
                     (2.0 * (double)W0 * (double)W0)) / (double)PR->B0);
    PR->B2 = (float)(((4.0 / (double)(Ts * Ts)) -
                     (4.0 * (double)Wc / (double)Ts) + (double)(W0 * W0)) / (double)PR->B0);
}

void PR_update(struct PR_PATH *PR, float Vi)
{
    PR->Vi = Vi;
    /* 与压缩包逐项同序：比例项已经包含在 A0/A1/A2，外部不再加 Kp*e。
     * 中断内仅使用 float 乘加和历史移位，无除法或系数重新计算。
     */
    PR->Vo = (-PR->B1 * PR->Vo1) - (PR->B2 * PR->Vo2) +
             (PR->A0 * PR->Vi) + (PR->A1 * PR->Vi1) + (PR->A2 * PR->Vi2);
    PR->Vo2 = PR->Vo1;
    PR->Vo1 = PR->Vo;
    PR->Vi2 = PR->Vi1;
    PR->Vi1 = PR->Vi;
}

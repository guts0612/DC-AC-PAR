#include "PR.h"

void PR_init(struct PR_PATH *PR,float Ts,float Kp,float Kr,float Wc,float W0)
{
    //清零结构体
    PR->Vi=0,PR->Vi1=0,PR->Vi2=0;
    PR->Vo=0,PR->Vo1=0,PR->Vo2=0;

    //常量计算
    PR->B0=(4.0/(Ts*Ts))+(4.0*Wc/Ts)+(W0*W0);
    PR->A0=((4.0*Kp/(Ts*Ts))+(4.0*Wc*(Kp+Kr)/Ts)+(Kp*W0*W0))/ PR->B0;
    PR->A1=((-8.0*Kp/(Ts*Ts))+(2.0*Kp*W0*W0))/ PR->B0;
    PR->A2=((4.0*Kp/(Ts*Ts))-(4.0*Wc*(Kp+Kr)/Ts)+(Kp*W0*W0))/ PR->B0;
    PR->B1=((-8.0/(Ts*Ts))+(2.0*W0*W0))/ PR->B0;
    PR->B2=((4.0/(Ts*Ts))-(4.0*Wc/Ts)+(W0*W0))/ PR->B0;

}

void PR_update(struct PR_PATH *PR,float Vi)
{
    //传递赋值
    PR->Vi=Vi;
    //递归计算
    PR->Vo=(-PR->B1*PR->Vo1)-(PR->B2*PR->Vo2)+(PR->A0*PR->Vi)+(PR->A1*PR->Vi1)+(PR->A2*PR->Vi2);
    //参数传递
    PR->Vo2=PR->Vo1;
    PR->Vo1=PR->Vo;
    PR->Vi2=PR->Vi1;
    PR->Vi1=PR->Vi;
}


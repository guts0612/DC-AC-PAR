#ifndef PR_H
#define PR_H




struct PR_PATH
{
    float A0,A1,A2;
    float B0,B1,B2;
    float Vi,Vi1,Vi2;//x[n],x[n-1],x[n-2]
    float Vo,Vo1,Vo2;//y[n],y[n-1],y[n-2]
};

void PR_init(struct PR_PATH *PR,float Ts,float Kp,float Kr,float Wc,float W0);

void PR_update(struct PR_PATH *PR,float Vi);


#endif

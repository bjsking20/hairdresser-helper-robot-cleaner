#ifndef __PID_H
#define __PID_H

typedef struct {
    float kp, ki, kd;
    float Ts;              // 0.01f (10ms)
    float i_term, prev_e;
    float out_min, out_max;
} PID_t;

static inline float clampf(float x,float a,float b){ return (x<a)?a:((x>b)?b:x); }

static inline void PID_Init(PID_t* p, float kp,float ki,float kd,float Ts,float omin,float omax){
    p->kp=kp; p->ki=ki; p->kd=kd; p->Ts=Ts;
    p->i_term=0.0f; p->prev_e=0.0f; p->out_min=omin; p->out_max=omax;
}

static inline float PID_Step(PID_t* p, float ref, float meas){
    float e = ref - meas;
    p->i_term += p->ki * e * p->Ts;
    float d = p->kd * (e - p->prev_e) / p->Ts;
    float u = p->kp*e + p->i_term + d;
    float u_sat = clampf(u, p->out_min, p->out_max);
    if (u != u_sat) p->i_term += 0.5f*(u_sat - u); // anti-windup(간단)
    p->prev_e = e;
    return u_sat;
}

// ★ PID 적분/상태 리셋 (정지/무장 해제 시 호출)
static inline void PID_Reset(PID_t* p){
    if (!p) return;
    p->i_term = 0.0f;
    p->prev_e = 0.0f;
}
#endif

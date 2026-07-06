// encoder_units.h 최종 수정본

#ifndef __ENCODER_UNITS_H
#define __ENCODER_UNITS_H

#include <math.h>

// === 모터/바퀴 사양 ===
#define CPR_MOTOR   64.0f
#define GEAR_RATIO  70.0f
#define QUAD        1.0f
// ⬇️ 올바른 값으로 수정: 4480.0f 이 아닌, 쿼드러처를 포함한 전체 계산식 사용
#define CPR_OUT     (CPR_MOTOR * GEAR_RATIO * QUAD)

#define ENC_LEFT_INVERT   1
#define ENC_RIGHT_INVERT 1

// 휠 지름/둘레 (단위: 미터)
#define WHEEL_D     0.115f
#define WHEEL_C     (M_PI * WHEEL_D)

// 1m 이동 시 필요한 엔코더 카운트 수
#define CNT_PER_M   (CPR_OUT / WHEEL_C)
#define WHEEL_BASE  0.45f

// --- 이하 함수들은 그대로 둡니다 ---
static inline float ticks10ms_from_mps(float v_mps){
    return v_mps * CNT_PER_M * 0.01f; // m/s → cnt/10ms
}
static inline float mps_from_ticks10ms(float t10){
    return (t10 / CNT_PER_M) / 0.01f; // cnt/10ms → m/s
}

#endif /* __ENCODER_UNITS_H */

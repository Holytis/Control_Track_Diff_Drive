#include <Arduino.h>
#include <ps5Controller.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"

// ==========================================
// 1. CONFIGURATION & PINS
// ==========================================
BTS7960_t motor_L = { 23, 22, 255 }; //
BTS7960_t motor_R = { 26, 25, 255 };
Encoder_t encLeft, encRight;

// ข้อมูลทางกายภาพ
const float WHEEL_PPR = 16.0 * 99.5 * 2; 
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH = 0.37;
const float MAX_SPEED_MS = 0.3; 
const float MAX_OMEGA = 1.0;

// การตั้งค่า S-Curve
unsigned long moveStartTime = 0;
float startWheelL = 0, startWheelR = 0;
float targetWheelL = 0, targetWheelR = 0;
const double RAMP_DURATION = 1000.0; // ระยะเวลาในการเร่ง (ms) ยิ่งมากยิ่งนุ่ม

// PID & Safety Limit
const int PWM_HARD_LIMIT = 128; // ลิมิต PWM ไว้ที่ 200 เพื่อความปลอดภัย
PID PIDMotorL(-PWM_HARD_LIMIT, PWM_HARD_LIMIT, 200.0, 0.0, 0.0); //
PID PIDMotorR(-PWM_HARD_LIMIT, PWM_HARD_LIMIT, 200.0, 0.0, 0.0);

unsigned long prevPidTime = 0;
const long pidInterval = 20;

// ==========================================
// 2. S-CURVE & KINEMATICS FUNCTIONS
// ==========================================

// ฟังก์ชันคำนวณความเร็วแบบ S-Curve
double SCurve(double t, double start, double delta_vel, double duration) {
    if (t >= duration) return start + delta_vel;
    t /= duration / 2.0;
    if (t < 1) return delta_vel / 2.0 * t * t * t + start;
    t -= 2.0;
    return delta_vel / 2.0 * (t * t * t + 2.0) + start;
}

void processJoyInput() {
    int joy_v = ps5.LStickY(); 
    int joy_w = ps5.RStickX();

    if (abs(joy_v) < 10) joy_v = 0;
    if (abs(joy_w) < 10) joy_w = 0;

    float v_req = ((float)joy_v / 128.0) * MAX_SPEED_MS;
    float w_req = -((float)joy_w / 128.0) * MAX_OMEGA;

    float newTargetL = v_req - (w_req * TRACK_WIDTH / 2.0);
    float newTargetR = v_req + (w_req * TRACK_WIDTH / 2.0);

    // ถ้ามีการเปลี่ยนเป้าหมายจากจอย ให้เริ่มนับเวลา S-Curve ใหม่
    if (newTargetL != targetWheelL || newTargetR != targetWheelR) {
        startWheelL = targetWheelL; 
        startWheelR = targetWheelR;
        targetWheelL = newTargetL;
        targetWheelR = newTargetR;
        moveStartTime = millis();
    }
}

// ==========================================
// 3. MAIN CONTROL
// ==========================================

void setup() {
    Serial.begin(115200);
    ps5.begin("10:18:49:ac:28:82");
    
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 19, 18);
    Encoder_Init(&encRight, 13, 14);
}

void loop() {
    if (!ps5.isConnected()) {
        BTS7960_Stop(&motor_L);
        BTS7960_Stop(&motor_R);
        return;
    }

    processJoyInput();

    unsigned long currentMillis = millis();
    if (currentMillis - prevPidTime >= pidInterval) {
        double dt = (currentMillis - prevPidTime) / 1000.0;
        prevPidTime = currentMillis;

        // 1. คำนวณความเร็วเป้าหมายผ่าน S-Curve
        unsigned long elapsed = currentMillis - moveStartTime;
        float currentProfiledL = SCurve((double)elapsed, startWheelL, targetWheelL - startWheelL, RAMP_DURATION);
        float currentProfiledR = SCurve((double)elapsed, startWheelR, targetWheelR - startWheelR, RAMP_DURATION);

        // 2. อ่านความเร็วจริงจาก Encoder
        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);
        float dist_per_tick = (2.0 * PI * WHEEL_RADIUS) / WHEEL_PPR;
        float meas_v_L = (delta_L * dist_per_tick) / dt;
        float meas_v_R = (delta_R * dist_per_tick) / dt;

        // 3. คำนวณ PID
        float output_L = PIDMotorL.compute(currentProfiledL, meas_v_L);
        float output_R = PIDMotorR.compute(currentProfiledR, meas_v_R);

        // 4. Safety Limit & Drive
        int final_L = constrain((int)output_L, -PWM_HARD_LIMIT, PWM_HARD_LIMIT);
        int final_R = constrain((int)output_R, -PWM_HARD_LIMIT, PWM_HARD_LIMIT);

        if (targetWheelL == 0 && targetWheelR == 0 && abs(meas_v_L) < 0.01) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
        } else {
            BTS7960_SetSpeed(&motor_L, final_L);
            BTS7960_SetSpeed(&motor_R, final_R);
        }

        // ==========================================
        // เพิ่มการโชว์ค่า (SERIAL MONITOR / PLOTTER)
        // ==========================================
        // รูปแบบ: ชื่อค่า:ค่า (เว้นวรรค) เพื่อให้ Serial Plotter แยกสีได้
        Serial.print("Tgt_L:"); Serial.print(currentProfiledL, 3);
        Serial.print(" Real_L:"); Serial.print(meas_v_L, 3);
        Serial.print(" PWM_L:"); Serial.print(final_L / 100.0); // หาร 100 เพื่อให้กราฟอยู่ในสเกลเดียวกับ m/s
        
        Serial.print(" | Tgt_R:"); Serial.print(currentProfiledR, 3);
        Serial.print(" Real_R:"); Serial.print(meas_v_R, 3);
        Serial.print(" PWM_R:"); Serial.println(final_R / 100.0);
    }
}
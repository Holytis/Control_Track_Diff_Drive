#include "bts7960.h"
#include "encoder.h"
// --- 1. ตั้งค่าพารามิเตอร์มอเตอร์และเกียร์ ---
const float MOTOR_PPR = 16.0;   //
const float GEAR_RATIO = 99.5;  //
const float WHEEL_PPR = MOTOR_PPR * GEAR_RATIO; // 1592.0

// --- 2. ประกาศตัวแปรสำหรับ BTS7960 และ Encoder ---
// โครงสร้าง BTS7960_t { RPWM, LPWM, MAX_PWM }
BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };

Encoder_t left_enc, right_enc; //

unsigned long test_timer = 0;

void setup() {
    Serial.begin(115200);

    // เริ่มต้นใช้งานมอเตอร์
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);

    // เริ่มต้นใช้งาน Encoder (A=18, B=19 และ A=13, B=14)
    Encoder_Init(&left_enc, 19, 18);
    Encoder_Init(&right_enc, 13, 14);

    Serial.println("--- BTS7960 Max RPM Test (Gear 99.5:1) ---");
    Serial.println("Starting in 3 seconds...");
    delay(3000);

    // สั่งรันมอเตอร์ที่ความเร็วสูงสุด (Max PWM)
    BTS7960_SetSpeed(&motor_L, 60);
    BTS7960_SetSpeed(&motor_R, 60);
}

void loop() {
    // คำนวณและแสดงผลทุก 1 วินาที
    if (millis() - test_timer >= 1000) {
        // อ่านค่า Delta (จำนวน Pulse ที่เปลี่ยนไปใน 1 วินาที)
        long p_L = Encoder_GetDelta(&left_enc);
        long p_R = Encoder_GetDelta(&right_enc);

        // คำนวณ RPM ของล้อ: (Pulses ต่อวินาที / Total PPR ล้อ) * 60 วินาที
        float wheel_rpm_L = (p_L / WHEEL_PPR) * 60.0;
        float wheel_rpm_R = (p_R / WHEEL_PPR) * 60.0;

        Serial.print("Wheel RPM L: "); Serial.print(wheel_rpm_L);
        Serial.print(" | Wheel RPM R: "); Serial.print(wheel_rpm_R);
        Serial.print(" | Raw Pulses/sec: "); Serial.println(p_L);

        test_timer = millis();
    }
}
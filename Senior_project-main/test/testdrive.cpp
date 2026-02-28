#include <Arduino.h>
#include "SmileEVO24.h"
extern "C" {
#include "encoder.h"
}

// ==========================================
// 1. CONFIGURATION
// ==========================================
const float TPR = 1592.0;         
const int TEST_PWM = 40;  // ความเร็วสูงสุด (255)
const long INTERVAL = 100; // อ่านค่าทุก 100ms

// ==========================================
// 2. PIN DEFINITIONS
// ==========================================
Encoder_t encLeft;
Encoder_t encRight;
const int encA[2] = {13, 18};
const int encB[2] = {14, 19};

SmileEVO24 Motor_L(27, 25, 26, -1);
SmileEVO24 Motor_R(23, 21, 22, -1);

unsigned long previousMillis = 0;

void setup() {
    Serial.begin(115200);
    
    // 1. เริ่มต้นระบบ
    Encoder_Init(&encLeft, encB[0], encA[0]);
    Encoder_Init(&encRight, encB[1], encA[1]);

    Motor_L.begin();
    Motor_R.begin();

    Serial.println("\n\n=== RPM TEST STARTED ===");
    Serial.println("Motors should spin NOW...");
    
    // 2. สั่งมอเตอร์ *ครั้งเดียว* ตรงนี้ (แก้ปัญหา PWM รวน)
    Motor_L.drive(TEST_PWM);
    Motor_R.drive(TEST_PWM);
    
    delay(1000); // รอให้รอบนิ่งสักนิด
}

void loop() {
    // ใน Loop ไม่ต้องสั่งมอเตอร์ซ้ำแล้ว ปล่อยให้มันหมุนไปเรื่อยๆ
    
    // จับเวลาเพื่อคำนวณ RPM
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= INTERVAL) {
        
        long dt = currentMillis - previousMillis;
        previousMillis = currentMillis;

        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);

        // ==========================================
        // [จุดกลับทิศทางตัวเลข]
        // ถ้าค่า RPM ติดลบ ให้เอา // ออก
        // ==========================================
        //delta_L = -delta_L; 
        delta_R = -delta_R; 

        // คำนวณ RPM
        float rpm_L = (delta_L * 60000.0) / (TPR * dt);
        float rpm_R = (delta_R * 60000.0) / (TPR * dt);

        Serial.printf("PWM: %d | L_RPM: %6.2f | R_RPM: %6.2f\n", 
                      TEST_PWM, rpm_L, rpm_R);
    }
}
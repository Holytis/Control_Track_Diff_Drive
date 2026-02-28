#include "SmileEVO24.h"
extern "C" {
  #include "encoder.h"
}

// --- การตั้งค่าพารามิเตอร์ ---
const float MOTOR_PPR = 16.0;   // Pulse ของมอเตอร์เดิม
const float GEAR_RATIO = 99.5;  // อัตราทดเกียร์
const float WHEEL_PPR = MOTOR_PPR * GEAR_RATIO; // Pulse ต่อรอบล้อ (1,592 PPR)

// การตั้งค่า Pin ตามที่ระบุ
SmileEVO24 Motor_L(23, 22, 21, -1); //
SmileEVO24 Motor_R(27, 26, 25, -1);
Encoder_t left_enc, right_enc;      //

unsigned long test_timer = 0;

void setup() {
  Serial.begin(115200);
  
  // เริ่มต้นการทำงานของ Driver และ Encoder
  Motor_L.begin(); //
  Motor_R.begin();
  Encoder_Init(&left_enc, 18, 19);  //
  Encoder_Init(&right_enc, 13, 14);

  Serial.println("--- Max RPM Test with Gearbox 99.5:1 ---");
  Serial.println("Motor will start in 3 seconds...");
  delay(3000);
  
  // สั่ง Max PWM (255) เพื่อดูความเร็วสูงสุดของล้อ
  Motor_L.drive(30); //
  Motor_R.drive(30);
}

void loop() {
  // คำนวณและแสดงผลทุก 1 วินาที
  if (millis() - test_timer >= 1000) { 
    // อ่านจำนวน Pulse ที่เกิดขึ้นจริงใน 1 วินาทีล่าสุด
    long p_L = -Encoder_GetDelta(&left_enc); //
    long p_R = Encoder_GetDelta(&right_enc);
    
    // คำนวณ RPM ของล้อ: (Pulses ต่อวินาที / Total PPR ของล้อ) * 60 วินาที
    float wheel_rpm_L = (p_L / WHEEL_PPR) * 60.0;
    float wheel_rpm_R = (p_R / WHEEL_PPR) * 60.0;

    Serial.print("Wheel RPM Left: "); Serial.print(wheel_rpm_L);
    Serial.print(" | Wheel RPM Right: "); Serial.println(wheel_rpm_R);
    
    // แสดงจำนวน Pulse รวมที่อ่านได้เผื่อตรวจสอบความแม่นยำ
    Serial.print("Pulses per sec (L): "); Serial.println(p_L);
    
    test_timer = millis();
  }
}
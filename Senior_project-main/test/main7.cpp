#include <Arduino.h>
#include <math.h>
#include "SmileEVO24.h"
#include "pid.h"
#include <ps5Controller.h>

extern "C" {
  #include "encoder.h"
}

// --- 1. การตั้งค่า Pin และ Encoder ---
Encoder_t encLeft;
Encoder_t encRight;
const int encA[2] = {13, 18};
const int encB[2] = {14, 19};

// --- 2. การตั้งค่า Motor ---
SmileEVO24 Motor_L(27, 25, 26, -1);
SmileEVO24 Motor_R(23, 21, 22, -1);

// --- 3. พารามิเตอร์หุ่นยนต์ (ต้องวัดจริง!) ---
float TPR = 600.0;          // Ticks Per Revolution (รวมเกียร์แล้ว)
float WHEEL_DIAMETER = 0.0325; // เส้นผ่านศูนย์กลางล้อ (เมตร) เช่น 10cm = 0.10
float TRACK_WIDTH = 0.37;    // ความกว้างฐานล้อ (เมตร)

// กำหนดความเร็วสูงสุด (m/s)
// ลองเริ่มที่ 0.5 - 1.0 m/s
const float MAX_SPEED_MS = 0.5; 

// --- 4. PID Parameters (จูนใหม่สำหรับ m/s) ---
// เนื่องจาก Error มีค่าน้อย (เช่น 0.1 m/s) Output ต้องเยอะ (PWM)
// Kp จึงต้องมีค่าสูงขึ้นกว่าตอนใช้ RPM มาก
double Kp1 = 100.0, Ki1 = 0.0, Kd1 = 0.0; 
double Kp2 = 100.0, Ki2 = 0.0, Kd2 = 0.0;

PID PIDMortorL(-255, 255, Kp1, Ki1, Kd1);
PID PIDMortorR(-255, 255, Kp2, Ki2, Kd2);

// ตัวแปรเก็บค่า (หน่วย m/s)
float Setpoint[2] = {0, 0}; 
float Speed_Real[2] = {0, 0};
float Output[2]   = {0, 0};

// --- 5. Timing ---
unsigned long prevPidTime = 0;
const long pidInterval = 10;    // 10ms (100Hz)
unsigned long prevSerialTime = 0;
const long serialInterval = 50; // 50ms

// ==========================================
// Function: แปลงจอย PS5 เป็น Target Speed (m/s)
// ==========================================
void Drive_equation(int joy_y, int joy_x, float &target_L, float &target_R) {
    if (abs(joy_y) < 10) joy_y = 0;
    if (abs(joy_x) < 10) joy_x = 0;

    // แปลงจอย (-128 ถึง 127) เป็นความเร็วเชิงเส้น (m/s)
    float v_req = ((float)joy_y / 128.0) * MAX_SPEED_MS;
    
    // แปลงจอยเป็นความเร็วเชิงมุม (rad/s) เพื่อการเลี้ยว
    // สมมติให้เลี้ยวสุด = หมุนตัวด้วยความเร็ว MAX_SPEED_MS
    float w_req = ((float)joy_x / 128.0) * (MAX_SPEED_MS * 2.0); 

    // Differential Drive Kinematics
    // v_L = v - (w * L / 2)
    // v_R = v + (w * L / 2)
    target_L = v_req - (w_req * TRACK_WIDTH / 2.0);
    target_R = v_req + (w_req * TRACK_WIDTH / 2.0);

    // Limit ไม่ให้เกิน Max Speed
    target_L = constrain(target_L, -MAX_SPEED_MS, MAX_SPEED_MS);
    target_R = constrain(target_R, -MAX_SPEED_MS, MAX_SPEED_MS);
}

void setup() {
    Serial.begin(115200);
    ps5.begin("10:18:49:ac:28:82"); 
    
    Encoder_Init(&encLeft, encA[0], encB[0]);
    Encoder_Init(&encRight, encA[1], encB[1]);
    
    Motor_L.begin();
    Motor_R.begin();

    Serial.println("System Ready (Mode: m/s)");
    while (!ps5.isConnected()) {
        delay(300);
    }
    Serial.println("PS5 Connected!");
}

void loop() {
    unsigned long currentMillis = millis();

    // --- Control Loop ---
    if (currentMillis - prevPidTime >= pidInterval) {
        float dt = (currentMillis - prevPidTime) / 1000.0; // dt เป็นวินาที
        prevPidTime = currentMillis;

        if (ps5.isConnected()) {
            Drive_equation(ps5.LStickY(), ps5.LStickX(), Setpoint[0], Setpoint[1]);
        } else {
            Setpoint[0] = 0; Setpoint[1] = 0;
        }

        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);
        delta_R = -delta_R; // กลับทิศถ้าจำเป็น

        // --- สูตรคำนวณ m/s ---
        // 1. หาเส้นรอบวงล้อ (เมตร) = PI * Diameter
        float circumference = M_PI * WHEEL_DIAMETER;
        
        // 2. ระยะทางต่อ 1 Tick (เมตร/tick)
        float dist_per_tick = circumference / TPR;

        // 3. ความเร็ว (m/s) = (delta_ticks * dist_per_tick) / dt
        Speed_Real[0] = (delta_L * dist_per_tick) / dt;
        Speed_Real[1] = (delta_R * dist_per_tick) / dt;

        // PID Calculation (Input & Setpoint are both in m/s)
        Output[0] = PIDMortorL.compute(Setpoint[0], Speed_Real[0]);
        Output[1] = PIDMortorR.compute(Setpoint[1], Speed_Real[1]);

        Motor_L.drive((int)Output[0]);
        Motor_R.drive((int)Output[1]);
    }

    // --- Serial Plotter Loop ---
    if (currentMillis - prevSerialTime >= serialInterval) {
        prevSerialTime = currentMillis;
        // ส่งค่าหน่วย m/s ไปพล็อตกราฟ
        Serial.printf("%.3f,%.3f,%.3f,%.3f\n", 
                      Setpoint[0], Speed_Real[0], 
                      Setpoint[1], Speed_Real[1]);
    }
}   
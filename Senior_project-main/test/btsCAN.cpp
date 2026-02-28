#include <Arduino.h>
#include <ps5Controller.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"
#include "driver/twai.h" // ไลบรารี CAN BUS สำหรับ ESP32

// ==========================================
// 1. CONFIGURATION & PINS
// ==========================================
BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };
Encoder_t encLeft, encRight;

// CAN BUS Pins (อ้างอิงจากตัวอย่างเดิม)
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define CAN_ID 0x100 

// พารามิเตอร์หุ่นยนต์
const float WHEEL_PPR = 16.0 * 99.5 / 3; 
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH = 0.37;
const float MAX_SPEED_MS = 0.3; 
const float MAX_OMEGA = 1.2;
float totalDistance = 0; // ระยะทางสะสมทั้งหมด (เมตร)

// การตั้งค่า S-Curve
unsigned long moveStartTime = 0;
float startWheelL = 0, startWheelR = 0;
float targetWheelL = 0, targetWheelR = 0;
const double RAMP_DURATION = 1000.0; 

// PID & Safety Limit
const int PWM_HARD_LIMIT = 255; 
PID PIDMotorL(-PWM_HARD_LIMIT, PWM_HARD_LIMIT, 200.0, 0.0, 0.0);
PID PIDMotorR(-PWM_HARD_LIMIT, PWM_HARD_LIMIT, 200.0, 0.0, 0.0);

// ตัวแปรควบคุมสถานะ
bool autoMode = false; // R2: สลับโหมด CAN (Auto) / Joy (Manual)
bool lastR2State = false;
float Can_Target_L = 0, Can_Target_R = 0; // เป้าหมายความเร็วจาก ROS 2

unsigned long prevPidTime = 0;                                                      
const long pidInterval = 20;

// ==========================================
// 2. CAN BUS FUNCTIONS (ROS 2 Interface)
// ==========================================

void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // ตั้งค่าความเร็ว 500k
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
    Serial.println("CAN BUS Started");
}

void readCAN() {
    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) {
        // ตรวจสอบ ID และความยาวข้อมูล (ID 0x100 รับค่า L, R อย่างละ 2 bytes)
        if (message.identifier == CAN_ID && message.data_length_code == 4) {
            int16_t raw_L = (message.data[1] << 8) | message.data[0];
            int16_t raw_R = (message.data[3] << 8) | message.data[2];
            // แปลงค่ากลับเป็นหน่วย m/s (หาร 100.0 ตามมาตรฐานโปรโตคอลของคุณ)
            Can_Target_L = (float)raw_L / 100.0f;
            Can_Target_R = (float)raw_R / 100.0f;
        }
    }
}

// ==========================================
// 3. KINEMATICS & S-CURVE
// ==========================================

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

    if (newTargetL != targetWheelL || newTargetR != targetWheelR) {
        startWheelL = targetWheelL; 
        startWheelR = targetWheelR;
        targetWheelL = newTargetL;
        targetWheelR = newTargetR;
        moveStartTime = millis();
    }
}

// ==========================================
// 4. MAIN LOOP
// ==========================================

void setup() {
    Serial.begin(115200);
    ps5.begin("10:18:49:ac:28:82");
    
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 18, 19);
    Encoder_Init(&encRight, 13, 14);
    
    setupCAN();
}

void loop() {
    readCAN(); // รับค่า Can_Target_L, Can_Target_R จาก ROS 2

    if (!ps5.isConnected()) {
        BTS7960_Stop(&motor_L);
        BTS7960_Stop(&motor_R);
        return;
    }

    // --- ระบบ Toggle Mode (R2) ---
    bool currentR2 = ps5.R2();
    if (currentR2 && !lastR2State) {
        autoMode = !autoMode;
        // เมื่อสลับโหมด ให้รีเซ็ตจุดเริ่มต้น S-Curve เพื่อป้องกันการกระชาก
        startWheelL = targetWheelL;
        startWheelR = targetWheelR;
        moveStartTime = millis();
        
        ps5.setLed(autoMode ? 0 : 0, autoMode ? 255 : 0, autoMode ? 0 : 255);
        ps5.sendToController();
    }
    lastR2State = currentR2;

    unsigned long currentMillis = millis();
    if (currentMillis - prevPidTime >= pidInterval) {
        double dt = (currentMillis - prevPidTime) / 1000.0;
        prevPidTime = currentMillis;

        // --- Logic การเลือกเป้าหมาย (Input Processing) ---
        if (autoMode) {
            // [AUTO] ตรวจสอบว่าค่าจาก CAN เปลี่ยนไปจาก Target เดิมหรือไม่
            if (Can_Target_L != targetWheelL || Can_Target_R != targetWheelR) {
                startWheelL = targetWheelL; // ใช้ Target ปัจจุบันเป็นจุดเริ่มใหม่
                startWheelR = targetWheelR;
                targetWheelL = Can_Target_L;
                targetWheelR = Can_Target_R;
                moveStartTime = currentMillis; // เริ่มนับเวลา S-Curve ใหม่
            }
        } 
        else {
            // [MANUAL] ประมวลผลจากจอย
            processJoyInput(); // ฟังก์ชันนี้จะคอยอัปเดต targetWheelL/R และ moveStartTime ให้เอง
        }

        // --- คำนวณความเร็วผ่าน S-Curve (ใช้ทั้ง 2 โหมด) ---
        unsigned long elapsed = currentMillis - moveStartTime;
        float setpoint_L = SCurve((double)elapsed, startWheelL, targetWheelL - startWheelL, RAMP_DURATION);
        float setpoint_R = SCurve((double)elapsed, startWheelR, targetWheelR - startWheelR, RAMP_DURATION);

        // --- อ่านค่าจริงจาก Encoder ---
        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);
        float dist_per_tick = (2.0 * PI * WHEEL_RADIUS) / WHEEL_PPR;
        // ระยะทางที่แต่ละล้อเคลื่อนที่ได้ในรอบนี้ (เมตร)
        float stepDistL = delta_L * dist_per_tick;
        float stepDistR = delta_R * dist_per_tick;
        float stepDistRobot = (stepDistL + stepDistR) / 2.0;
        // สะสมระยะทางรวม
        totalDistance += stepDistRobot;
        // --- คำนวณความเร็ว (Velocity) ---
        float meas_v_L = stepDistL / dt;
        float meas_v_R = stepDistR / dt;
        // --- คำนวณ PID และส่งคำสั่งมอเตอร์ ---
        float output_L = PIDMotorL.compute(setpoint_L, meas_v_L);
        float output_R = PIDMotorR.compute(setpoint_R, meas_v_R);

        int final_L = constrain((int)output_L, -PWM_HARD_LIMIT, PWM_HARD_LIMIT);
        int final_R = constrain((int)output_R, -PWM_HARD_LIMIT, PWM_HARD_LIMIT);

        if (abs(setpoint_L) < 0.001 && abs(setpoint_R) < 0.001 && abs(meas_v_L) < 0.01) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
        } else {
            BTS7960_SetSpeed(&motor_L, final_L);
            BTS7960_SetSpeed(&motor_R, final_R);
        }

        // แสดงผล
        Serial.printf("[%s] Tgt:%.2f,%.2f Real:%.2f,%.2f Dist:%.2f m\n", 
                      autoMode ? "AUTO" : "MAN", setpoint_L, setpoint_R, meas_v_L, meas_v_R, totalDistance);
    }
    }

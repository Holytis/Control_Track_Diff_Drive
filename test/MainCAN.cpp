#include <math.h>
#include "SmileEVO24.h"
#include "pid.h"
#include <Preferences.h>
#include "driver/twai.h" // ไลบรารี CAN Bus

extern "C"{
#include "encoder.h"
}

// ==========================================
// 1. CONFIGURATION (ตั้งค่าหุ่นยนต์)
// ==========================================
// *** ค่า TPR ที่ถูกต้อง (คำนวณจากเกียร์ 99.5) ***
const float TPR = 1592.0;           

// ขนาดยางและฐานล้อ (หน่วยเมตร)
const float WHEEL_RADIUS = 0.0325; 
const float TRACK_WIDTH = 0.36;    

// ลิมิตความเร็ว (กันมอเตอร์ไหม้)
const float MAX_RPM = 150.0;

// *** ตั้งค่าขา CAN (ต้องไม่ชนกับ Motor) ***
#define CAN_TX_PIN 5  // ต่อเข้าขา TX ของ TJA1051
#define CAN_RX_PIN 4  // ต่อเข้าขา RX ของ TJA1051
#define CAN_ID     0x100 // ID ประจำตัวหุ่นยนต์

// ==========================================
// 2. HARDWARE SETUP
// ==========================================
Encoder_t encLeft;
Encoder_t encRight;
const int encA[2] = {13, 18};
const int encB[2] = {14, 19};

// Motor Pins
SmileEVO24 Motor_L(27,25,26,-1);
SmileEVO24 Motor_R(23,21,22,-1);

// PID Parameters (ปรับจูนได้ที่นี่)
double Kp = 50.0, Ki = 10.0, Kd = 2.0; 

PID PIDMotorL(-255, 255, Kp, Ki, Kd);
PID PIDMotorR(-255, 255, Kp, Ki, Kd);

Preferences preferences;

// Variables
float Target_RPM_L = 0, Target_RPM_R = 0;
float Measured_RPM_L = 0, Measured_RPM_R = 0;
float Output_L = 0, Output_R = 0;

unsigned long previousMillis = 0;
const long interval = 20; // PID Loop 20ms
unsigned long lastCanRecvTime = 0; // จับเวลา Safety

// ==========================================
// 3. FUNCTIONS
// ==========================================

void setupCAN() {
    // Config CAN 500 kbps
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("CAN: Driver Installed");
    }
    if (twai_start() == ESP_OK) {
        Serial.println("CAN: Started");
    }
}

void readCAN() {
    twai_message_t message;
    
    // เช็คว่ามีข้อความเข้ามาหรือไม่ (Non-blocking)
    if (twai_receive(&message, 0) == ESP_OK) {
        
        // รับเฉพาะ ID 0x100 ที่มีข้อมูลครบ 4 Byte
        if (message.identifier == CAN_ID && message.data_length_code == 4) {
            
            // แกะข้อมูล (Byte 0-1 = Left, Byte 2-3 = Right)
            int16_t raw_L = (message.data[1] << 8) | message.data[0];
            int16_t raw_R = (message.data[3] << 8) | message.data[2];

            // แปลงกลับเป็น float (หาร 10)
            Target_RPM_L = (float)raw_L / 10.0f;
            Target_RPM_R = (float)raw_R / 10.0f;

            // Limit ค่า
            Target_RPM_L = constrain(Target_RPM_L, -MAX_RPM, MAX_RPM);
            Target_RPM_R = constrain(Target_RPM_R, -MAX_RPM, MAX_RPM);

            // อัปเดตเวลา Safety
            lastCanRecvTime = millis();
        }
    }
}

void loadPID() {
    preferences.begin("robot_pid", true);
    Kp = preferences.getFloat("kp", 50.0);
    Ki = preferences.getFloat("ki", 10.0);
    Kd = preferences.getFloat("kd", 2.0);
    preferences.end();
    PIDMotorL.updateConstants(Kp, Ki, Kd);
    PIDMotorR.updateConstants(Kp, Ki, Kd);
    Serial.printf("PID Loaded: %.1f, %.1f, %.1f\n", Kp, Ki, Kd);
}

// ==========================================
// 4. MAIN PROGRAM
// ==========================================

void setup() {
    Serial.begin(115200);
    
    Encoder_Init(&encLeft, encB[0], encA[0]);
    Encoder_Init(&encRight, encB[1], encA[1]);

    loadPID();   
    setupCAN();  
    
    Motor_L.begin();
    Motor_R.begin();
    
    Serial.println("ROBOT READY: Waiting for CAN ID 0x100...");
}

void loop() {
    // 1. อ่านคำสั่งจาก CAN (ทำตลอดเวลา)
    readCAN();

    // 2. Safety Watchdog: ถ้าเงียบเกิน 0.5 วินาที ให้จอด
    if (millis() - lastCanRecvTime > 500) {
        if (Target_RPM_L != 0 || Target_RPM_R != 0) {
            Serial.println("SAFETY: Stop (No Signal)");
        }
        Target_RPM_L = 0;
        Target_RPM_R = 0;
        Motor_L.brake();
        Motor_R.brake();
        
        // Reset PID เพื่อกันค่า I ค้าง
        PIDMotorL = PID(-255, 255, Kp, Ki, Kd);
        PIDMotorR = PID(-255, 255, Kp, Ki, Kd);
    }

    // 3. PID Loop (ทำงานทุก 20ms)
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        long dt = currentMillis - previousMillis;
        previousMillis = currentMillis;

        // ถ้า Safety ไม่ตัด ให้ขับมอเตอร์
        if (millis() - lastCanRecvTime <= 500) {
            
            // อ่าน Encoder
            long delta_L = Encoder_GetDelta(&encLeft);
            long delta_R = Encoder_GetDelta(&encRight);
            
            // ** ถ้าทิศทางกลับด้าน ให้แก้ตรงนี้ **
            // delta_R = -delta_R; 

            // คำนวณ RPM จริง (ใช้ TPR 1592)
            Measured_RPM_L = (delta_L * 60000.0) / (TPR * dt);
            Measured_RPM_R = (delta_R * 60000.0) / (TPR * dt);

            // คำนวณ PID Output (PWM)
            Output_L = PIDMotorL.compute(Target_RPM_L, Measured_RPM_L);
            Output_R = PIDMotorR.compute(Target_RPM_R, Measured_RPM_R);

            // สั่งมอเตอร์
            Motor_L.drive((int)Output_L);
            Motor_R.drive((int)Output_R);
        }
        
        // Debug แสดงผลทุก 0.2 วินาที
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 200) {
            lastPrint = millis();
            Serial.printf("T:%.1f/%.1f | M:%.1f/%.1f\n", 
                          Target_RPM_L, Target_RPM_R, Measured_RPM_L, Measured_RPM_R);
        }
    }
}
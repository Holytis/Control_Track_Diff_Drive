#include <Arduino.h>
#include <ps5Controller.h>
#include <math.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"
#include "driver/twai.h" 

// ==========================================
// 1. CONFIGURATION & PINS
// ==========================================
BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };
Encoder_t encLeft, encRight;

#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define CAN_ID 0x100 

// Robot Parameters
const float WHEEL_PPR = 16.0 * 99.5 ; 
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH = 0.37;
const float MAX_SPEED_MS = 0.3; 
const float MAX_OMEGA = 1.2;
float totalDistance = 0; 

// S-Curve Settings
unsigned long moveStartTime = 0;
float startWheelL = 0, startWheelR = 0;
float targetWheelL = 0, targetWheelR = 0;
const double RAMP_DURATION = 1000.0; 

// PID - ปลดปล่อยพลังงานเต็มที่ (Limit -255 ถึง 255)
PID PIDMotorL(-255, 255, 1000.0, 0.0, 0.0); 
PID PIDMotorR(-255, 255, 1000.0, 0.0, 0.0);

// Control States
bool autoMode = false; 
bool lastR2State = false;

float Can_Target_L = 0, Can_Target_R = 0;
unsigned long prevPidTime = 0;
const long pidInterval = 20;

bool isAutoDistance = false;
float distanceTarget = 0;
float startDistance = 0;

// ==========================================
// 2. HELPER FUNCTIONS
// ==========================================

void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
    Serial.println("CAN BUS Started");
}

void readCAN() {
    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) {
        if (message.identifier == CAN_ID && message.data_length_code == 4) {
            int16_t raw_L = (message.data[1] << 8) | message.data[0];
            int16_t raw_R = (message.data[3] << 8) | message.data[2];
            Can_Target_L = (float)raw_L / 100.0f;
            Can_Target_R = (float)raw_R / 100.0f;
        }
    }
}

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
// เพิ่มฟังก์ชันการส่งข้อมูลระยะทาง (Transmit CAN)
// ==========================================

void sendDistanceCAN() {
    twai_message_t message;
    message.identifier = 0x101; // ID สำหรับส่งข้อมูล Feedback (Distance)
    message.extd = 0;           // Standard ID
    message.data_length_code = 4; // ส่ง 4 bytes (สำหรับ int32_t)

    // แปลง float เป็น int32 (หน่วยเซนติเมตร) เพื่อส่งผ่าน CAN
    int32_t dist_cm = (int32_t)(totalDistance * 100.0f);

    message.data[0] = dist_cm & 0xFF;
    message.data[1] = (dist_cm >> 8) & 0xFF;
    message.data[2] = (dist_cm >> 16) & 0xFF;
    message.data[3] = (dist_cm >> 24) & 0xFF;

    // ส่งข้อความออกไป
    if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
        // Serial.println("Distance sent via CAN");
    }
}

// ==========================================
// 3. MAIN LOOP
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
    readCAN(); 

    if (!ps5.isConnected()) {
        BTS7960_Stop(&motor_L);
        BTS7960_Stop(&motor_R);
        return;
    }

    // --- BUTTON LOGIC ---
    
    // 1. ตรวจสอบปุ่ม D-Pad เพื่อตั้งระยะทางเป้าหมาย
    if (ps5.Up() || ps5.Right() || ps5.Down() || ps5.Left()) {
        autoMode = false;      // ปิดโหมด CAN
        isAutoDistance = true; // เปิดโหมดวิ่งตามระยะทาง
        startDistance = totalDistance; // บันทึกระยะทางเริ่มต้น ณ จุดที่กด
        
        if (ps5.Up())    distanceTarget = 5.0;
        if (ps5.Right()) distanceTarget = 10.0;
        if (ps5.Down())  distanceTarget = 15.0;
        if (ps5.Left())  distanceTarget = 20.0;

        // ตั้งเป้าหมายความเร็วสูงสุด และเริ่ม S-Curve
        startWheelL = targetWheelL; startWheelR = targetWheelR;
        targetWheelL = MAX_SPEED_MS; targetWheelR = MAX_SPEED_MS;
        moveStartTime = millis();
        
        Serial.printf(">>> Task Started: Move forward %.1f m <<<\n", distanceTarget);
    }

    // 2. ยกเลิก Task หากขยับ Analog หรือกด R2 สลับโหมด
    if (abs(ps5.LStickY()) > 20 || ps5.R2()) {
        if (isAutoDistance) {
            isAutoDistance = false;
            Serial.println(">>> Task Cancelled by User <<<");
        }
    }

    // 3. ระบบ Toggle Mode R2 (คงเดิม)
    bool currentR2 = ps5.R2();
    if (currentR2 && !lastR2State) {
        autoMode = !autoMode;
        isAutoDistance = false; 
        startWheelL = targetWheelL; startWheelR = targetWheelR;
        moveStartTime = millis();
        ps5.setLed(0, autoMode ? 255 : 0, autoMode ? 0 : 255); 
        ps5.sendToController();
    }
    lastR2State = currentR2;

    if (ps5.Options()) {
        totalDistance = 0;
        isAutoDistance = false;
        Serial.println("Distance Reset!");
    }

    // --- CONTROL TASK ---
    unsigned long currentMillis = millis();
    if (currentMillis - prevPidTime >= pidInterval) {
        double dt = (currentMillis - prevPidTime) / 1000.0;
        prevPidTime = currentMillis;

        // เช็คว่าวิ่งถึงระยะเป้าหมายหรือยัง
        if (isAutoDistance) {
            float traveled = totalDistance - startDistance;
            if (traveled >= distanceTarget) {
                isAutoDistance = false;
                targetWheelL = 0; targetWheelR = 0;
                startWheelL = MAX_SPEED_MS; startWheelR = MAX_SPEED_MS;
                moveStartTime = currentMillis;
                Serial.println(">>> Target Reached: Stopping <<<");
            }
        }

        if (autoMode) {
            if (Can_Target_L != targetWheelL || Can_Target_R != targetWheelR) {
                startWheelL = targetWheelL; startWheelR = targetWheelR;
                targetWheelL = Can_Target_L; targetWheelR = Can_Target_R;
                moveStartTime = currentMillis;
            }
        } else if (!isAutoDistance) {
            // โหมด Manual ปกติ (จะทำงานเมื่อไม่ได้อยู่ในโหมดวิ่งตามระยะทาง)
            processJoyInput(); 
        }

        unsigned long elapsed = currentMillis - moveStartTime;
        float setpoint_L = SCurve((double)elapsed, startWheelL, targetWheelL - startWheelL, RAMP_DURATION);
        float setpoint_R = SCurve((double)elapsed, startWheelR, targetWheelR - startWheelR, RAMP_DURATION);

        // คำนวณความเร็วและระยะทาง
        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);
        float dist_per_tick = (2.0 * PI * WHEEL_RADIUS) / WHEEL_PPR;
        float stepL = delta_L * dist_per_tick;
        float stepR = delta_R * dist_per_tick;
        totalDistance += (stepL + stepR) / 2.0;

        float meas_v_L = stepL / dt;
        float meas_v_R = stepR / dt;

        int final_L = (int)PIDMotorL.compute(setpoint_L, meas_v_L);
        int final_R = (int)PIDMotorR.compute(setpoint_R, meas_v_R);

        // --- MOTOR OUTPUT ---
        if (abs(setpoint_L) < 0.001 && abs(setpoint_R) < 0.001 && abs(meas_v_L) < 0.01) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
        } else {
            BTS7960_SetSpeed(&motor_L, final_L);
            BTS7960_SetSpeed(&motor_R, final_R);
        }

        sendDistanceCAN();
        
        // แสดงโหมดที่หน้าจอ Serial
        const char* modeStr = isAutoDistance ? "DIST" : (autoMode ? "AUTO" : "MAN");
        Serial.printf("[%s] TgtL:%.2f TgtR:%.2f RealL:%.2f RealR:%.2f Dist:%.2f/%s m\n", 
                      modeStr, setpoint_L,setpoint_R, meas_v_L, meas_v_R , totalDistance, 
                      isAutoDistance ? String(distanceTarget).c_str() : "--");
    }
}

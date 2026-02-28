#include <Arduino.h>
#include <math.h>
#include "SmileEVO24.h"
#include "pid.h"
#include <ps5Controller.h>
#include "driver/twai.h" 

extern "C" {
  #include "encoder.h"
}

// ==========================================
// 1. CONFIGURATION (ใช้ Pin เดิมของคุณ)
// ==========================================
Encoder_t encLeft;
Encoder_t encRight;
const int encA[2] = {18, 13};
const int encB[2] = {19, 14};

// Motor Pins (ตามโค้ดของคุณ)
SmileEVO24 Motor_L(23, 22, 21, -1);
SmileEVO24 Motor_R(27, 26, 25, -1);

// CAN BUS Pins
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define CAN_ID 0x100 

// Parameters (ตามโค้ดของคุณ)
const float TPR = 1592.0; 
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH = 0.37;
const float MAX_SPEED_MS = 0.2;
const float MAX_OMEGA = 1.8;      

// --- [Task 1] RAMP SETTINGS (แยกเร่ง/ผ่อน) ---
const float ACCEL_LIMIT = 0.5; // ตอนเร่ง: ไปไวหน่อย
const float DECEL_LIMIT = 0.5; // ตอนผ่อน: ไหลยาวๆ กันไฟกระชาก

// PID
double Kp = 120.0, Ki = 0.0, Kd = 0.0; 
PID PIDMortorL(-255, 255, Kp, Ki, Kd);
PID PIDMortorR(-255, 255, Kp, Ki, Kd);

// Variables
float Raw_Target_L = 0, Raw_Target_R = 0; // เป้าหมายดิบ (จากจอย)
float Setpoint_L = 0, Setpoint_R = 0;     // เป้าหมายจริง (ผ่าน Ramp แล้ว)
float Speed_Real_L = 0, Speed_Real_R = 0;
float Output_L = 0, Output_R = 0;

// CAN Variables
float Can_Target_L = 0, Can_Target_R = 0;

// Control States
bool autoMode = false;      // R2 Toggle: Auto(CAN) / Manual(Joy)
bool isSwingMode = false;   // L2 Toggle: Pivot / Swing Turn
bool lastR2State = false;
bool lastL2State = false;

// Timing
unsigned long prevPidTime = 0;
const long pidInterval = 20;    
unsigned long prevSerialTime = 0;
const long serialInterval = 50; 

// ==========================================
// 2. FUNCTIONS
// ==========================================

void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
    Serial.println("CAN Started");
}

void readCAN() {
    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) {
        if (message.identifier == CAN_ID && message.data_length_code == 4) {
            int16_t raw_L = (message.data[1] << 8) | message.data[0];
            int16_t raw_R = (message.data[3] << 8) | message.data[2];
            Can_Target_L = (float)raw_L / 10.0f;
            Can_Target_R = (float)raw_R / 10.0f;
        }
    }
}

// [Task 1] ฟังก์ชัน Ramp แบบ Asymmetric (เร่ง/ผ่อน ไม่เท่ากัน)
float rampVal(float current, float target, float dt) {
    // ถ้า target เยอะกว่า current (ในทิศทางเดียวกัน) แปลว่ากำลังเร่ง
    // ถ้า target น้อยกว่า (หรือกลับเข้าหา 0) แปลว่ากำลังผ่อน
    bool isAccelerating = abs(target) > abs(current);
    
    // เลือกค่า Limit ให้ถูกตัว
    float rate = isAccelerating ? ACCEL_LIMIT : DECEL_LIMIT;
    float max_change = rate * dt;

    float diff = target - current;
    if (abs(diff) <= max_change) return target;
    else return (diff > 0) ? current + max_change : current - max_change;
}

// [Task 2 & 3] คำนวณ Joy แบบแยกแกน + โหมด Swing
void Compute_Joy_Kinematics() {
    // 1. Split Controls: แยกจอยซ้าย/ขวา
    int joy_v = ps5.LStickY(); // เดินหน้า-ถอยหลัง
    int joy_w = ps5.RStickX(); // เลี้ยวซ้าย-ขวา

    // Deadzone
    if (abs(joy_v) < 10) joy_v = 0;
    if (abs(joy_w) < 10) joy_w = 0;

    float v_req = ((float)joy_v / 128.0) * MAX_SPEED_MS;
    float w_req = -((float)joy_w / 128.0) * MAX_OMEGA; 

    // Differential Drive Math
    float v_l = v_req - (w_req * TRACK_WIDTH / 2.0);
    float v_r = v_req + (w_req * TRACK_WIDTH / 2.0);

    // 2. [Task 3] Swing Turn Logic (ถ้าเปิดโหมด L2)
    if (isSwingMode) {
        // ห้ามล้อหมุนถอยหลังถ้ากำลังเดินหน้า (ทำให้ล้อในโค้งหยุดนิ่งแทนที่จะหมุนสวน)
        if (v_req >= 0) { // กรณีรถหยุดหรือเดินหน้า
            if (v_l < 0) v_l = 0;
            if (v_r < 0) v_r = 0;
        } else { // กรณีรถถอยหลัง
            if (v_l > 0) v_l = 0;
            if (v_r > 0) v_r = 0;
        }
    }

    Raw_Target_L = v_l;
    Raw_Target_R = v_r;
}

// ==========================================
// 3. MAIN SETUP & LOOP
// ==========================================

void setup() {
    Serial.begin(115200);
    ps5.begin("10:18:49:ac:28:82"); 
    
    Encoder_Init(&encLeft, encB[0], encA[0]);
    Encoder_Init(&encRight, encB[1], encA[1]);
    setupCAN();
    
    Motor_L.begin();
    Motor_R.begin();

    Serial.println("Ready. R2=Auto/Man, L2=Pivot/Swing");
}

void loop() {
    readCAN();

    if (!ps5.isConnected()) {
        Motor_L.brake(); Motor_R.brake();
        return;
    }

    // --- BUTTON LOGIC ---
    
    // 1. R2 Toggle (Auto/Manual)
    bool currentR2 = ps5.R2();
    if (currentR2 && !lastR2State) {
        autoMode = !autoMode;
        if (autoMode) {
            Serial.println("MODE: AUTO (Green)");
            ps5.setLed(0, 255, 0); // เขียว = Auto
        } else {
            Serial.println("MODE: MANUAL (Blue)");
            ps5.setLed(0, 0, 255); // น้ำเงิน = Manual
            Can_Target_L = 0; Can_Target_R = 0; // เคลียร์ค่าค้าง
        }
        ps5.sendToController();
    }
    lastR2State = currentR2;

    // 2. L2 Toggle (Swing/Pivot) - ทำงานเฉพาะ Manual
    bool currentL2 = ps5.L2();
    if (currentL2 && !lastL2State && !autoMode) {
        isSwingMode = !isSwingMode;
        if (isSwingMode) {
            Serial.println("TURN: SWING (Red)");
            ps5.setLed(255, 0, 0); // แดง = Swing Mode
        } else {
            Serial.println("TURN: PIVOT (Blue)");
            ps5.setLed(0, 0, 255); // น้ำเงิน = Normal
        }
        ps5.sendToController();
    }
    lastL2State = currentL2;

    // --- CONTROL LOOP ---
    unsigned long currentMillis = millis();
    if (currentMillis - prevPidTime >= pidInterval) {
        float dt = (currentMillis - prevPidTime) / 1000.0f; 
        prevPidTime = currentMillis;

        if (autoMode) {
            // [AUTO] ใช้ค่าจาก CAN ตรงๆ (Bypass Ramp เพื่อให้ ROS คุม)
            Setpoint_L = Can_Target_L;
            Setpoint_R = Can_Target_R;
        } 
        else {
            // [MANUAL] คำนวณจอย + Asymmetric Ramp
            Compute_Joy_Kinematics(); 
            Setpoint_L = rampVal(Setpoint_L, Raw_Target_L, dt);
            Setpoint_R = rampVal(Setpoint_R, Raw_Target_R, dt);
        }

        // Anti-Stuck (Zero Stop)
        if (abs(Setpoint_L) < 0.01 && abs(Setpoint_R) < 0.01 && 
            abs(Raw_Target_L) < 0.01 && abs(Raw_Target_R) < 0.01) {
            
            Motor_L.brake(); Motor_R.brake();
            Output_L = 0; Output_R = 0;
            PIDMortorL = PID(-255, 255, Kp, Ki, Kd);
            PIDMortorR = PID(-255, 255, Kp, Ki, Kd);
        } 
        else {
            long delta_L = Encoder_GetDelta(&encLeft);
            long delta_R = Encoder_GetDelta(&encRight);
            delta_R = -delta_R; 

            float dist_per_tick = (2.0 * M_PI * WHEEL_RADIUS) / TPR;
            Speed_Real_L = (delta_L * dist_per_tick) / dt;
            Speed_Real_R = (delta_R * dist_per_tick) / dt;

            Output_L = PIDMortorL.compute(Setpoint_L, Speed_Real_L);
            Output_R = PIDMortorR.compute(Setpoint_R, Speed_Real_R);

            Motor_L.drive((int)Output_L);
            Motor_R.drive((int)Output_R);
        }
    }

    if (currentMillis - prevSerialTime >= serialInterval) {
        prevSerialTime = currentMillis;
        Serial.printf("[%s|%s] Tgt:%.2f,%.2f Real:%.2f,%.2f\n", 
                      autoMode ? "AUTO" : "MAN", 
                      isSwingMode ? "SWING" : "PIVOT",
                      Setpoint_L, Setpoint_R, Speed_Real_L, Speed_Real_R);
    }
}
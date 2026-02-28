#include <Arduino.h>
#include <ps5Controller.h>
#include <math.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"
#include "driver/twai.h"
#include "web_server.h"

// ==========================================
// 1. CONFIGURATION
// ==========================================
BTS7960_t motor_L = { 23, 22, 255 };
BTS7960_t motor_R = { 26, 25, 255 };
Encoder_t encLeft, encRight;

#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define CAN_ID 0x100 

const float WHEEL_PPR = 16.0 * 99.5;
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH = 0.37;
const float MAX_SPEED_MS = 0.3; 
const float MAX_OMEGA = 1.0;

unsigned long moveStartTime = 0;
float startWheelL = 0, startWheelR = 0;
float targetWheelL = 0, targetWheelR = 0;
const double RAMP_DURATION = 1000.0; 

const int PWM_HARD_LIMIT = 128; 
PID PIDMotorL(-PWM_HARD_LIMIT, PWM_HARD_LIMIT, 200.0, 50.0, 0.0);
PID PIDMotorR(-PWM_HARD_LIMIT, PWM_HARD_LIMIT, 200.0, 50.0, 0.0);

bool autoMode = false; 
bool lastR2State = false;
float Can_Target_L = 0, Can_Target_R = 0;
unsigned long prevPidTime = 0;
const long pidInterval = 20;

// ==========================================
// 2. HELPER FUNCTIONS
// ==========================================

void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
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
// 3. MAIN
// ==========================================

void setup() {
    Serial.begin(115200);
    ps5.begin("10:18:49:ac:28:82");
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 18, 19);
    Encoder_Init(&encRight, 13, 14);
    setupCAN();
    setupWeb();
}

void loop() {
    readCAN();
    if (!ps5.isConnected()) {
        BTS7960_Stop(&motor_L);
        BTS7960_Stop(&motor_R);
        return;
    }

    bool currentR2 = ps5.R2();
    if (currentR2 && !lastR2State) {
        autoMode = !autoMode;
        ps5.setLed(autoMode ? 0 : 0, autoMode ? 255 : 0, autoMode ? 0 : 255);
        ps5.sendToController();
    }
    lastR2State = currentR2;

    unsigned long currentMillis = millis();
    if (currentMillis - prevPidTime >= pidInterval) {
        double dt = (currentMillis - prevPidTime) / 1000.0;
        prevPidTime = currentMillis;

        float setL, setR;
        if (autoMode) {
            setL = Can_Target_L; setR = Can_Target_R;
        } else {
            processJoyInput();
            unsigned long elapsed = currentMillis - moveStartTime;
            setL = SCurve((double)elapsed, startWheelL, targetWheelL - startWheelL, RAMP_DURATION);
            setR = SCurve((double)elapsed, startWheelR, targetWheelR - startWheelR, RAMP_DURATION);
        }

        long dL = Encoder_GetDelta(&encLeft);
        long dR = Encoder_GetDelta(&encRight);
        float res = (2.0 * PI * WHEEL_RADIUS) / WHEEL_PPR;
        float realL = (dL * res) / dt;
        float realR = (dR * res) / dt;

        int outL = (int)PIDMotorL.compute(setL, realL);
        int outR = (int)PIDMotorR.compute(setR, realR);

        if (abs(setL) < 0.001 && abs(setR) < 0.001 && abs(realL) < 0.01) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
        } else {
            BTS7960_SetSpeed(&motor_L, outL);
            BTS7960_SetSpeed(&motor_R, outR);
        }

        updateWebData(setL, realL, setR, realR);
        Serial.printf("Tgt_L:%.2f Real_L:%.2f PWM:%d\n", setL, realL, outL);
    }
}
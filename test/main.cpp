#include <math.h>
// #include "bts7960.h"
#include "SmileEVO24.h"
#include "pid.h"
extern "C"{
#include "encoder.h"
}
#include <ps5Controller.h>

Encoder_t encLeft;
Encoder_t encRight;

const int N_WHEEL = 2;
const int encA[N_WHEEL] = {13, 18};
const int encB[N_WHEEL] = {14, 19};

// SmileEVO24 motor(PWM_PIN, INA_PIN, INB_PIN, CS_PIN);
SmileEVO24 Motor_L(27,25,26,-1);
SmileEVO24 Motor_R(23,21,22,-1);
// SmileEVO24 Motor_R = {
//     .RPWM_Pin = 22,  // ขา D5 (PWM)
//     .LPWM_Pin = 23,  // ขา D6 (PWM)
//     .max_pwm = 255  // สำหรับ Arduino 8-bit PWM
// };

// สร้าง Object Array สำหรับ Encoder


float TPR = 600;
double wheelRPM[2] = {0,0};


// timer 
unsigned long previousMillis = 0;
const long interval = 5;

//Pid parameter ssss
float Setpoint[3]={0,0};
float Input[3]={0,0};
float Output[3]={0,0};

double   Kp1=100, Ki1=20.00, Kd1=10.00;
double   Kp2=35, Ki2=1.00, Kd2=50.00;

PID PIDMortorL(-255, 255, Kp1, Ki1, Kd1);
PID PIDMortorR(-255, 255, Kp2, Ki2, Kd2);

float TRACK_WIDTH = 0.36;
void Drive_equation(int target_v ,int target_w,float &out1, float &out2){
    if (abs(target_v) < 10) target_v = 0;
    if (abs(target_w) < 10) target_w = 0;
    // if (abs(W) < 10) W = 0;
    // if (abs(x) < 100) x = x/2;
    // if (abs(y) < 100) y = y/2;
    // if (abs(W) < 100) W = W/2;
    target_v = map(target_v,-127,127,-2,2);
    target_w = map(target_w,-127,127,-2,2);
    out1 = target_v - (target_w * TRACK_WIDTH / 2.0f);
    out2 = target_v + (target_w * TRACK_WIDTH / 2.0f);
}

// float target_v = 0, target_w = 0; // ตัวแปรรับค่าจาก Serial
unsigned long lastSerialTime = 0;

void setup() {
    Serial.begin(115200);
    // ps5.begin("10:18:49:ac:28:82");
    ps5.begin("10:18:49:ac:28:82");
    
    Encoder_Init(&encLeft, encB[0], encA[0]);  // ขา A=13, B=14
    Encoder_Init(&encRight,  encB[1], encA[1]); // ขา A=18, B=19
    Serial.println("Encoders Initialized");

    while (ps5.isConnected() == false)  
    { 
    Serial.println("PS5 controller not found");
    delay(300);
    }
    Motor_L.begin();
    Motor_R.begin();
    Serial.println("Ready");
}

void loop(){

    Drive_equation(ps5.LStickY(),ps5.LStickX(),Setpoint[0],Setpoint[1]);
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
    // save the current time as the last time the action happened
    previousMillis = currentMillis;
    // --- 1. การอ่าน Serial Command ---
    // if (Serial.available() > 0) {
    //     // รับรูปแบบ: "v,w\n" เช่น "0.5,0.2\n"
    //     target_v = Serial.parseFloat(); 
    //     if (Serial.read() == ',') {
    //         target_w = Serial.parseFloat();
    //     }
    //     Serial.printf("target_v: %ld, target_w: %ld\n", target_v, target_w);
    //     while (Serial.available()) Serial.read(); // Clear buffer
    //     lastSerialTime = currentMillis;
    // }
    // Safety: ถ้าไม่มีคำสั่งส่งมาเกิน 1 วินาที ให้หยุดหุ่นยนต์
    

    long delta_countL = Encoder_GetDelta(&encLeft);
    long delta_countR = Encoder_GetDelta(&encRight);
    delta_countR = - delta_countR;
    float rpm_L = (delta_countL * (1000.0 / interval) * 60.0) / (TPR * 100);
    float rpm_R = (delta_countR * (1000.0 / interval) * 60.0) / (TPR*100);

    
    Output[0] = PIDMortorL.compute(Setpoint[0],float(rpm_L));
    Output[1] = PIDMortorR.compute(Setpoint[1],float(rpm_R));
    
    //  (target_v, target_w, Setpoint[0], Setpoint[1]);
    // BTS7960_SetSpeed(&Motor_L,int(Setpoint[0]));
    // BTS7960_SetSpeed(&Motor_R,int(Setpoint[1]));
    Motor_L.drive(int(Output[0]));
    Motor_R.drive(int(Output[1]));
    // computeWheelKinematics(interval);
    if (currentMillis - lastSerialTime > 1000) {
        lastSerialTime = currentMillis;
        // target_v = 0; target_w = 0;
        if (Serial.availableForWrite() > 20){
        Serial.print("rX: "); Serial.print(int(ps5.LStickX())); Serial.print("rY: ");Serial.println(int(ps5.LStickY()));
        Serial.print("Setpoint[0]: "); Serial.print(Setpoint[0],2);Serial.print("Setpoint[1]: "); Serial.println(Setpoint[1],2);
        Serial.print("Output[0]: "); Serial.print(Output[0],2);Serial.print("Output[1]: "); Serial.println(Output[1],2);
        Serial.printf("encoder L: %0.2f, R: %0.2f\n", rpm_L, rpm_R);
        }
    }
    // Motor_L.drive(int(Setpoint[0])*100);
    // Motor_R.drive(int(Setpoint[1])*100);

  }
  
//   Serial.printf("RPM1: %.2f , RPM2: %d \n", wheelRPM[0], wheelRPM[1]);
}
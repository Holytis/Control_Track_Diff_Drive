#include "Arduino.h"
#include "driver/twai.h"

// กำหนดขาให้ตรงกับที่ต่อ SN65HVD230 หรือ TJA1051
#define RX_PIN 4
#define TX_PIN 5

void setup() {
    Serial.begin(115200);

    // 1. ตั้งค่าพื้นฐาน (ขา TX, ขา RX, โหมดทำ7งาน)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
    
    // 2. ตั้งค่า Bitrate (ต้องตรงกับฝั่งส่ง)
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
    
    // 3. ตั้งค่า Filter (รับทุก ID ที่ส่งมา)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // ติดตั้งและเริ่มใช้งาน Driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("Driver installed");
    }
    if (twai_start() == ESP_OK) {
        Serial.println("Driver started");
    }
}

void loop() {
    twai_message_t message;

    // 4. รอรับข้อมูล (Timeout 1 วินาที)
    if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        
        // แสดงผล ID ของข้อความ (Hex)
        Serial.print("New packet received! ID: 0x");
        Serial.print(message.identifier, HEX);

        // ตรวจสอบว่าเป็นเฟรมข้อมูล (Data Frame) หรือไม่
        if (!(message.flags & TWAI_MSG_FLAG_RTR)) {
            Serial.print(" | Data: ");
            for (int i = 0; i < message.data_length_code; i++) {
                Serial.printf("%02X ", message.data[i]);
            }
        }
        Serial.println();
    } else {
        // หากไม่มีข้อมูลเข้ามาในช่วงเวลาที่กำหนด
        Serial.println("Waiting for data...");
    }
}
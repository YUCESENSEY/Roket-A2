////////////////////////////////////
// ÜLGEN UKB - ORTAK KONFİGÜRASYON (REFERANS DOSYASI)
// Arduino IDE klasör dışındaki dosyaları kolay bulamadığı için
// test kodlarının her birinde bu değerler kendi içinde de tanımlıdır.
// Pin veya eşik değiştirirsek önce BURAYI güncelleyip sonra
// test kodlarına kopyalarız. Böylece tek bir doğru kaynak olur.
////////////////////////////////////
#pragma once

////////////////////////////////////
// Pinler (Arduino Nano ESP32)
#define buzzer       D2

#define ApogeeMosfet A0
#define MainMosfet   A1

#define LORA_RX      D4
#define LORA_TX      D5
#define LORA_M0      A7
#define LORA_M1      A6
#define LORA_AUX     D3

#define LED_GUC      A3
#define LED_MOD      A2
#define LED_UCUS     D10

// UART atamaları
// GPS      : UART0  -> RX = D1, TX = D0, 9600 baud
// RS232    : UART1  -> RX = D8, TX = D7, 115200 baud
// LoRa     : UART2  -> RX = D4, TX = D5, 115200 baud (konfig modunda 9600)
// I2C      : BMP390 (0x77 veya 0x76), BNO055 (0x28)

////////////////////////////////////
// Uçuş algoritması eşikleri (UKB-Kod.ino ile aynı olmalı)
#define KALKIS_ESIGI_DEGER   5      // m/s²
#define YANMA_ESIGI_DEGER    15     // m/s²
#define GOVDE_ACISI_DEGER    60     // derece
#define MIN_IRTIFA_DEGER     2400   // metre
#define APOGEE_MARGIN_DEGER  2.5    // metre
#define MAIN_IRTIFA_DEGER    500    // metre

////////////////////////////////////
// Piro çıkış süreleri (ms)
#define APOGEE_MOSFET_SURE   4000
#define MAIN_MOSFET_SURE     4000
#define BUZZER_SURE          1000

////////////////////////////////////
// Filtre sabitleri
#define KALMAN_Q_POZISYON    0.001f
#define KALMAN_Q_HIZ         0.003f
#define KALMAN_R_BARO        2.0f
#define TAMAMLAYICI_ALPHA    0.98f

////////////////////////////////////
// Paket başlıkları
#define LORA_PAKET_BASLIK    0xBC   // 35 byte telemetri paketi
#define RS232_SIT_BASLIK     0xAB   // 36 byte sensör paketi
#define RS232_KOMUT_BASLIK   0xAA   // komut / durum paketi

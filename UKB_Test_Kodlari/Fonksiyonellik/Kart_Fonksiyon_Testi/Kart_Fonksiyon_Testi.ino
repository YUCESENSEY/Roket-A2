#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPS++.h>

////////////////////////////////////
// ÜLGEN UKB - KART FONKSİYON TESTİ
// Karttaki her parçayı tek tek (veya hepsini birden) test eder.
// Serial Monitor: 115200 baud, "Yeni satır" (Newline) seçili olsun.
// Sonuçlar Serial'e yazılır; buzzer ve LED'ler gözle de doğrulanır.
////////////////////////////////////

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(55, 0x28, &Wire);
TinyGPSPlus gps;
HardwareSerial GPS(0);   // UART0
HardwareSerial RS232(1); // UART1
HardwareSerial LoRaSerial(2);

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

////////////////////////////////////
bool bmpHazir = false;
bool bnoHazir = false;

// Test numaraları ve sonuçları
// -1 = yapılmadı, 0 = BAŞARISIZ, 1 = BAŞARILI, 2 = gözle kontrol edilmeli
enum { T_I2C, T_BMP, T_BNO, T_GPS, T_BUZZER, T_LED, T_LORA, T_RS232, T_SAYI };
int8_t sonuc[T_SAYI] = {-1, -1, -1, -1, -1, -1, -1, -1};
const char* testAdi[T_SAYI] = {
  "I2C tarama", "BMP390", "BNO055", "GPS",
  "Buzzer", "LED'ler", "LoRa modülü", "RS232 loopback"
};

////////////////////////////////////
float basincIrtifa(float hPa) {
  return 44330.0f * (1.0f - powf(hPa / 1013.25f, 0.19029f));
}

////////////////////////////////////
// Serial'den bir satır bekler (zaman aşımı ms). Büyük harfe çevirip döner.
String satirBekle(unsigned long sure) {
  String s = "";
  unsigned long bas = millis();
  while (millis() - bas < sure) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (s.length() > 0) { s.trim(); s.toUpperCase(); return s; }
      } else {
        s += c;
      }
    }
  }
  s.trim();
  s.toUpperCase();
  return s;
}

////////////////////////////////////
// 1) I2C TARAMA
////////////////////////////////////
bool i2cTara() {
  Serial.println(F("\n--- I2C TARAMA ---"));
  int bulunan = 0;
  bool bmpVar = false, bnoVar = false;

  for (uint8_t adres = 1; adres < 127; adres++) {
    Wire.beginTransmission(adres);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  Cihaz bulundu: 0x"));
      if (adres < 0x10) Serial.print("0");
      Serial.print(adres, HEX);
      if (adres == 0x77 || adres == 0x76) { Serial.print(F("  -> BMP390"));  bmpVar = true; }
      if (adres == 0x28 || adres == 0x29) { Serial.print(F("  -> BNO055"));  bnoVar = true; }
      Serial.println();
      bulunan++;
    }
  }

  Serial.print(F("Toplam cihaz: ")); Serial.println(bulunan);
  if (!bmpVar) Serial.println(F("UYARI: BMP390 bulunamadı (0x77 / 0x76)"));
  if (!bnoVar) Serial.println(F("UYARI: BNO055 bulunamadı (0x28 / 0x29)"));
  return bmpVar && bnoVar;
}

////////////////////////////////////
// 2) BMP390 TESTİ
////////////////////////////////////
bool bmpBaslat() {
  if (bmpHazir) return true;
  bmpHazir = bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76);
  if (bmpHazir) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
  return bmpHazir;
}

bool bmpTest() {
  Serial.println(F("\n--- BMP390 TESTİ ---"));
  if (!bmpBaslat()) {
    Serial.println(F("HATA: BMP390 başlatılamadı. Kablo / lehim / adres kontrol et."));
    return false;
  }

  int basarili = 0;
  float minIrt = 99999.0f, maxIrt = -99999.0f;
  float sonBasinc = 0, sonSicaklik = 0;

  for (int i = 0; i < 20; i++) {
    if (bmp.performReading()) {
      float p   = bmp.pressure / 100.0f;
      float irt = basincIrtifa(p);
      sonBasinc   = p;
      sonSicaklik = bmp.temperature;
      basarili++;
      if (i >= 2) {                       // ilk 2 okumayı atla
        if (irt < minIrt) minIrt = irt;
        if (irt > maxIrt) maxIrt = irt;
      }
      if (i % 5 == 0) {
        Serial.printf("  Basınç: %.2f hPa | Sıcaklık: %.2f C | İrtifa: %.2f m\n", p, bmp.temperature, irt);
      }
    }
    delay(50);
  }

  Serial.printf("Başarılı okuma: %d / 20\n", basarili);
  Serial.printf("İrtifa gürültüsü (min-max): %.2f m\n", maxIrt - minIrt);

  bool ok = basarili >= 15 && sonBasinc > 300 && sonBasinc < 1100 &&
            sonSicaklik > -40 && sonSicaklik < 85;
  if (!ok) Serial.println(F("HATA: Değerler mantıksız veya okuma eksik!"));
  return ok;
}

////////////////////////////////////
// 3) BNO055 TESTİ
////////////////////////////////////
bool bnoTest() {
  Serial.println(F("\n--- BNO055 TESTİ (kartı sabit tut) ---"));
  if (!bnoHazir) {
    bnoHazir = bno.begin();
    if (bnoHazir) bno.setExtCrystalUse(true);
  }
  if (!bnoHazir) {
    Serial.println(F("HATA: BNO055 başlatılamadı. Kablo / adres (0x28) kontrol et."));
    return false;
  }
  delay(100);

  float sonMag = 0;
  for (int i = 0; i < 5; i++) {
    imu::Vector<3> acc  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> eul  = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    sonMag = sqrtf(acc.x()*acc.x() + acc.y()*acc.y() + acc.z()*acc.z());

    Serial.printf("  İvme: %.2f %.2f %.2f (|a|=%.2f) | Euler: %.1f %.1f %.1f | Jiro: %.2f %.2f %.2f\n",
                  acc.x(), acc.y(), acc.z(), sonMag,
                  eul.x(), eul.y(), eul.z(),
                  gyro.x(), gyro.y(), gyro.z());
    delay(200);
  }

  uint8_t kSis, kJiro, kIvme, kMag;
  bno.getCalibration(&kSis, &kJiro, &kIvme, &kMag);
  Serial.printf("Kalibrasyon (0-3): Sistem=%d Jiro=%d İvme=%d Mag=%d\n", kSis, kJiro, kIvme, kMag);
  Serial.printf("BNO sıcaklık: %d C\n", bno.getTemp());

  bool ok = sonMag > 8.0f && sonMag < 11.5f;   // durağanken yerçekimi ~9.81
  if (!ok) Serial.println(F("HATA: İvme büyüklüğü ~9.81 olmalı (kart sabit mi?)"));
  return ok;
}

////////////////////////////////////
// 4) GPS TESTİ (5 saniye dinler)
////////////////////////////////////
bool gpsTest() {
  Serial.println(F("\n--- GPS TESTİ (5 sn) ---"));
  uint32_t baslangicKarakter = gps.charsProcessed();
  unsigned long bas = millis();

  while (millis() - bas < 5000) {
    while (GPS.available()) gps.encode(GPS.read());
  }

  uint32_t gelen = gps.charsProcessed() - baslangicKarakter;
  Serial.printf("Gelen karakter: %lu | Geçerli cümle: %lu | Hatalı checksum: %lu\n",
                (unsigned long)gelen,
                (unsigned long)gps.sentencesWithFix(),
                (unsigned long)gps.failedChecksum());
  Serial.printf("Uydu sayısı: %lu\n", (unsigned long)gps.satellites.value());

  if (gps.location.isValid()) {
    Serial.printf("FIX VAR -> Enlem: %.6f  Boylam: %.6f  İrtifa: %.1f m\n",
                  gps.location.lat(), gps.location.lng(), gps.altitude.meters());
  } else {
    Serial.println(F("Henüz FIX yok (açık havada bekle). Bu, modülün bozuk olduğu anlamına gelmez."));
  }

  if (gelen == 0) {
    Serial.println(F("HATA: GPS'ten hiç veri gelmiyor! RX/TX ve besleme kontrol et."));
    return false;
  }
  return true;
}

////////////////////////////////////
// 5) BUZZER TESTİ
////////////////////////////////////
void buzzerTest() {
  Serial.println(F("\n--- BUZZER TESTİ: 3 kısa bip duymalısın ---"));
  for (int i = 0; i < 3; i++) {
    digitalWrite(buzzer, HIGH); delay(200);
    digitalWrite(buzzer, LOW);  delay(200);
  }
}

////////////////////////////////////
// 6) LED TESTİ
////////////////////////////////////
void ledTest() {
  Serial.println(F("\n--- LED TESTİ: GÜÇ -> MOD -> UÇUŞ sırayla yanıp sönmeli ---"));
  const uint8_t ledler[3] = {LED_GUC, LED_MOD, LED_UCUS};
  const char*   adlar[3]  = {"LED_GUC", "LED_MOD", "LED_UCUS"};
  for (int i = 0; i < 3; i++) {
    Serial.print("  "); Serial.println(adlar[i]);
    for (int k = 0; k < 3; k++) {
      digitalWrite(ledler[i], HIGH); delay(150);
      digitalWrite(ledler[i], LOW);  delay(150);
    }
  }
  digitalWrite(LED_GUC, HIGH);   // güç LED'i normalde hep yanık
}

////////////////////////////////////
// 7) LORA MODÜL TESTİ (AUX + konfigürasyon okuma)
////////////////////////////////////
void auxBekle(unsigned long sure) {
  unsigned long bas = millis();
  while (digitalRead(LORA_AUX) == LOW && millis() - bas < sure);
  delay(5);
}

bool loraTest() {
  Serial.println(F("\n--- LoRa MODÜL TESTİ ---"));
  bool auxYuksek = (digitalRead(LORA_AUX) == HIGH);
  Serial.print(F("AUX pini: ")); Serial.println(auxYuksek ? F("HIGH (modül hazır)") : F("LOW (modül meşgul/bağlı değil)"));

  // Konfigürasyon moduna gir
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, HIGH);
  delay(50);
  auxBekle(1000);

  LoRaSerial.end();
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(50);
  while (LoRaSerial.available()) LoRaSerial.read();

  // Konfigürasyonu oku: C1 00 09
  uint8_t komut[3] = {0xC1, 0x00, 0x09};
  LoRaSerial.write(komut, 3);

  uint8_t yanit[12];
  int n = 0;
  unsigned long bas = millis();
  while (millis() - bas < 1000 && n < 12) {
    if (LoRaSerial.available()) yanit[n++] = LoRaSerial.read();
  }

  // Normal moda dön
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(50);
  auxBekle(1000);
  LoRaSerial.end();
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);

  Serial.print(F("Modül yanıtı: "));
  for (int i = 0; i < n; i++) {
    if (yanit[i] < 0x10) Serial.print("0");
    Serial.print(yanit[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  bool ok = (n == 12 && yanit[0] == 0xC1);
  if (ok) {
    Serial.printf("Adres: %02X%02X | NETID: %02X | REG0: %02X | REG1: %02X | Kanal: %d | REG3: %02X\n",
                  yanit[3], yanit[4], yanit[5], yanit[6], yanit[7], yanit[8], yanit[9]);
  } else {
    Serial.println(F("HATA: Modül konfigürasyon komutuna yanıt vermedi! M0/M1/RX/TX/besleme kontrol et."));
  }
  return ok;
}

////////////////////////////////////
// 8) RS232 LOOPBACK TESTİ
// RS232 TX (D7) ile RX (D8) pinlerini kısa devre yap (veya MAX3232 çıkışında TX-RX bağla)
////////////////////////////////////
bool rs232Test() {
  Serial.println(F("\n--- RS232 LOOPBACK TESTİ (TX-RX birbirine bağlı olmalı) ---"));
  while (RS232.available()) RS232.read();

  const uint8_t gonder[6] = {0xAA, 0x55, 0x12, 0x34, 0x0D, 0x0A};
  RS232.write(gonder, 6);

  uint8_t alinan[6];
  int n = 0;
  unsigned long bas = millis();
  while (millis() - bas < 300 && n < 6) {
    if (RS232.available()) alinan[n++] = RS232.read();
  }

  bool ok = (n == 6 && memcmp(gonder, alinan, 6) == 0);
  Serial.printf("Gönderilen 6 bayt, alınan %d bayt -> %s\n", n, ok ? "EŞLEŞTİ" : "EŞLEŞMEDİ");
  if (!ok) Serial.println(F("Loopback bağlı değilse bu hata normaldir."));
  return ok;
}

////////////////////////////////////
// 9) PİRO MOSFET TESTİ (TEHLİKELİ - onay ister)
////////////////////////////////////
void piroTest() {
  Serial.println(F("\n!!! DİKKAT: PİRO MOSFET TESTİ !!!"));
  Serial.println(F("Fünye / e-match / ejeksiyon yükü karta TAKILI OLMAMALI!"));
  Serial.println(F("Bunun yerine bir LED+direnç veya multimetre kullan."));
  Serial.println(F("Kanal seç: A = Apogee, M = Main, başka bir şey = iptal (15 sn)"));

  String kanal = satirBekle(15000);
  uint8_t pin;
  if (kanal == "A")      pin = ApogeeMosfet;
  else if (kanal == "M") pin = MainMosfet;
  else { Serial.println(F("İptal edildi.")); return; }

  Serial.println(F("Devam etmek için ONAY yaz (15 sn):"));
  String onay = satirBekle(15000);
  if (onay != "ONAY") { Serial.println(F("İptal edildi.")); return; }

  Serial.println(F("3... 2... 1..."));
  delay(3000);
  digitalWrite(pin, HIGH);
  Serial.println(F("MOSFET AÇIK (500 ms)"));
  delay(500);
  digitalWrite(pin, LOW);
  Serial.println(F("MOSFET KAPALI. Test bitti."));
}

////////////////////////////////////
void ozetYazdir() {
  Serial.println(F("\n========== TEST ÖZETİ =========="));
  for (int i = 0; i < T_SAYI; i++) {
    Serial.printf("%-16s : ", testAdi[i]);
    if (sonuc[i] == -1)      Serial.println(F("yapılmadı"));
    else if (sonuc[i] == 0)  Serial.println(F("BAŞARISIZ  [X]"));
    else if (sonuc[i] == 1)  Serial.println(F("BAŞARILI   [OK]"));
    else                     Serial.println(F("gözle kontrol et"));
  }
  Serial.println(F("================================"));
}

void tumunuTestEt() {
  sonuc[T_I2C]    = i2cTara()  ? 1 : 0;
  sonuc[T_BMP]    = bmpTest()  ? 1 : 0;
  sonuc[T_BNO]    = bnoTest()  ? 1 : 0;
  sonuc[T_GPS]    = gpsTest()  ? 1 : 0;
  buzzerTest();  sonuc[T_BUZZER] = 2;
  ledTest();     sonuc[T_LED]    = 2;
  sonuc[T_LORA]   = loraTest() ? 1 : 0;
  sonuc[T_RS232]  = rs232Test() ? 1 : 0;
  ozetYazdir();
}

////////////////////////////////////
void menuYazdir() {
  Serial.println(F("\n========== ÜLGEN UKB KART FONKSİYON TESTİ =========="));
  Serial.println(F(" 1 - I2C tarama"));
  Serial.println(F(" 2 - BMP390 testi"));
  Serial.println(F(" 3 - BNO055 testi"));
  Serial.println(F(" 4 - GPS testi"));
  Serial.println(F(" 5 - Buzzer testi"));
  Serial.println(F(" 6 - LED testi"));
  Serial.println(F(" 7 - LoRa modül testi"));
  Serial.println(F(" 8 - RS232 loopback testi"));
  Serial.println(F(" 9 - PİRO MOSFET testi (tehlikeli, onay ister)"));
  Serial.println(F(" 0 - Hepsini test et (piro hariç)"));
  Serial.println(F(" O - Özet göster    M - Menüyü tekrar yaz"));
  Serial.println(F("====================================================="));
}

////////////////////////////////////
void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  GPS.begin(9600, SERIAL_8N1, D1, D0);
  RS232.begin(115200, SERIAL_8N1, D8, D7);
  Wire.begin();

  pinMode(MainMosfet,   OUTPUT); digitalWrite(MainMosfet,   LOW);
  pinMode(ApogeeMosfet, OUTPUT); digitalWrite(ApogeeMosfet, LOW);
  pinMode(buzzer,       OUTPUT); digitalWrite(buzzer,       LOW);
  pinMode(LORA_M0, OUTPUT); digitalWrite(LORA_M0, LOW);
  pinMode(LORA_M1, OUTPUT); digitalWrite(LORA_M1, LOW);
  pinMode(LORA_AUX, INPUT);

  pinMode(LED_GUC,  OUTPUT); digitalWrite(LED_GUC,  HIGH);
  pinMode(LED_MOD,  OUTPUT); digitalWrite(LED_MOD,  LOW);
  pinMode(LED_UCUS, OUTPUT); digitalWrite(LED_UCUS, LOW);

  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);

  menuYazdir();
}

////////////////////////////////////
void loop() {
  while (GPS.available()) gps.encode(GPS.read());   // GPS verisi birikmesin

  if (Serial.available()) {
    char k = toupper(Serial.read());
    switch (k) {
      case '1': sonuc[T_I2C]   = i2cTara()   ? 1 : 0; break;
      case '2': sonuc[T_BMP]   = bmpTest()   ? 1 : 0; break;
      case '3': sonuc[T_BNO]   = bnoTest()   ? 1 : 0; break;
      case '4': sonuc[T_GPS]   = gpsTest()   ? 1 : 0; break;
      case '5': buzzerTest(); sonuc[T_BUZZER] = 2;    break;
      case '6': ledTest();    sonuc[T_LED]    = 2;    break;
      case '7': sonuc[T_LORA]  = loraTest()  ? 1 : 0; break;
      case '8': sonuc[T_RS232] = rs232Test() ? 1 : 0; break;
      case '9': piroTest();                           break;
      case '0': tumunuTestEt();                       break;
      case 'O': ozetYazdir();                         break;
      case 'M': menuYazdir();                         break;
      default: break;                                 // \n \r vb. yok say
    }
  }
}

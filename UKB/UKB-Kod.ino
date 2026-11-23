#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPS++.h>
#include "esp_system.h"

////////////////////////////////////

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(55, 0x28, &Wire);
bool bnoHazir = false;
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
// Eşikler
float KALKIS_ESIGI  = 5;    // m/s²
float YANMA_ESIGI   = 15;    // m/s²
float GOVDE_ACISI   = 60;    // derece
float MIN_IRTIFA    = 2400;  // metre (TEKNOFEST A2: 8000 ft ~= 2438m)
float APOGEE_MARGIN = 2.5;   // metre
float MAIN_IRTIFA   = 500;   // metre

////////////////////////////////////
// Uçuş durum bayrakları
bool kalkti       = false;
bool yandi        = false;
bool govdeAcisi   = false;
bool irtifaOk     = false;
bool apogee       = false;
bool ApogeePyroOk = false;
bool Main         = false;
bool MainPyroOk   = false;
float zirveIrtifa = 0;
float yerIrtifa   = 0;

////////////////////////////////////
struct SensorVerisi {
  float basinc;
  float irtifa;
  float sicaklik;
  float aciX;
  float aciY;
  float aciZ;
  float ivmeX;
  float ivmeY;
  float ivmeZ;
  float enlem;
  float boylam;
  float gpsIrtifa;
  int   uyduSayisi;
  int   saat;
  int   dakika;
  int   saniye;
};

////////////////////////////////////
// Yer irtifası biriktirme (SUT modu için)
bool  yerIrtifaHazir  = false;
int   yerIrtifaSayac  = 0;
float yerIrtifaToplam = 0.0f;
#define YER_IRTIFA_ORNEK 3

////////////////////////////////////
// Zamanlayıcılar
unsigned long simdi           = 0;
bool          buzzerA         = false;
unsigned long sonbuzzer       = 0;
unsigned long sonZaman        = 0;
unsigned long sonRS232Zamani  = 0;
unsigned long sonDurumZamani  = 0;
unsigned long sensorzamani    = 0;
unsigned long sonApogeeMosfet = 0;
unsigned long sonMainMosfet   = 0;
unsigned long sonZamanFiltre  = 0;
unsigned long sonLoraSensorZamani = 0;

float filtrePitch = 0.0f;
float filtreRoll  = 0.0f;

////////////////////////////////////
// LED DURUM DEĞİŞKENLERİ (Senaryo A)
////////////////////////////////////
bool          ledModDurum   = false;
bool          ledUcusDurum  = false;
unsigned long sonLedMod     = 0;
unsigned long sonLedUcus    = 0;

////////////////////////////////////
// LORA KONFİGÜRASYON DEĞİŞKENLERİ
////////////////////////////////////
bool    loraKonfigAktif = false;
#define LORA_HEX_MAKS 64
uint8_t loraHexBuffer[LORA_HEX_MAKS];
uint8_t loraHexSayac    = 0;
char    loraSatir[200];
uint8_t loraSatirIndex  = 0;

////////////////////////////////////

float aciyiSinirla(float aci) {
  while (aci >  180.0f) aci -= 360.0f;
  while (aci < -180.0f) aci += 360.0f;
  return aci;
}

////////////////////////////////////
// KALMAN FİLTRESİ
////////////////////////////////////
float kalmanIrtifa = 0.0f;
float kalmanHiz    = 0.0f;
float P_00 = 1.0f, P_01 = 0.0f;
float P_10 = 0.0f, P_11 = 1.0f;
unsigned long sonKalmanZamani = 0;

const float Q_pozisyon = 0.001f;
const float Q_hiz      = 0.003f;
const float R_baro     = 2.0f;

void kalmanFiltresiGuncelle(float baroIrtifa, float dikeyIvme) {
  unsigned long simdiK = millis();
  float dt = (simdiK - sonKalmanZamani) / 1000.0f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.025f;
  sonKalmanZamani = simdiK;

  float tahminIrtifa = kalmanIrtifa + (kalmanHiz * dt) + (0.5f * dikeyIvme * dt * dt);
  float tahminHiz    = kalmanHiz + (dikeyIvme * dt);

  P_00 += dt * (P_10 + P_01) + dt * dt * P_11 + Q_pozisyon;
  P_01 += dt * P_11;
  P_10 += dt * P_11;
  P_11 += Q_hiz;

  float S   = P_00 + R_baro;
  float K_0 = P_00 / S;
  float K_1 = P_10 / S;
  float hata = baroIrtifa - tahminIrtifa;

  kalmanIrtifa = tahminIrtifa + (K_0 * hata);
  kalmanHiz    = tahminHiz    + (K_1 * hata);

  float p00_t = P_00, p01_t = P_01;
  P_00 -= K_0 * p00_t;
  P_01 -= K_0 * p01_t;
  P_10 -= K_1 * p00_t;
  P_11 -= K_1 * p01_t;
}

////////////////////////////////////
// TAMAMLAYICI FİLTRE
////////////////////////////////////
const float ALPHA = 0.98f;

void tamamlayiciFiltreUygula(float ax, float ay, float az,
                              float gx, float gy, float gz) {
  unsigned long simdiC = millis();
  float dt = (simdiC - sonZamanFiltre) / 1000.0f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.025f;
  sonZamanFiltre = simdiC;

  float rollAcc  = atan2f(ay, az)  * 180.0f / M_PI;
  float pitchAcc = atan2f(-ax, az) * 180.0f / M_PI;

  float rollGyro  = filtreRoll  + (gx * dt);
  float pitchGyro = filtrePitch + (gy * dt);

  filtreRoll  = ALPHA * rollGyro  + (1.0f - ALPHA) * rollAcc;
  filtrePitch = ALPHA * pitchGyro + (1.0f - ALPHA) * pitchAcc;

  filtreRoll  = aciyiSinirla(filtreRoll);
  filtrePitch = aciyiSinirla(filtrePitch);
}

////////////////////////////////////
// BYTE <-> FLOAT DÖNÜŞÜM
////////////////////////////////////
float bytesToFloat(const uint8_t *kaynak) {
  uint8_t hedef[4];
  hedef[0] = kaynak[3];
  hedef[1] = kaynak[2];
  hedef[2] = kaynak[1];
  hedef[3] = kaynak[0];
  float d;
  memcpy(&d, hedef, 4);
  return d;
}

void floatToBytes(float deger, uint8_t *hedef) {
  uint8_t kaynak[4];
  memcpy(kaynak, &deger, 4);
  hedef[0] = kaynak[3];
  hedef[1] = kaynak[2];
  hedef[2] = kaynak[1];
  hedef[3] = kaynak[0];
}

////////////////////////////////////
// SAPMA HESABI
////////////////////////////////////
float sapmaHesapla(float aciX, float aciY) {
  float r = aciX * M_PI / 180.0f;
  float p = aciY * M_PI / 180.0f;
  float c = cosf(r) * cosf(p);
  c = constrain(c, -1.0f, 1.0f);
  return acosf(c) * 180.0f / M_PI;
}

////////////////////////////////////
// UÇUŞ ALGORİTMASI
////////////////////////////////////
void algoritmaCalistir(const SensorVerisi &v) {
  if (v.irtifa > zirveIrtifa) zirveIrtifa = v.irtifa;

  float toplamIvme = sqrtf(v.ivmeX*v.ivmeX + v.ivmeY*v.ivmeY + v.ivmeZ*v.ivmeZ);
  float sapma      = sapmaHesapla(v.aciX, v.aciY);

  if (!kalkti && toplamIvme > KALKIS_ESIGI) kalkti = true;
  if (kalkti  && !yandi      && toplamIvme < YANMA_ESIGI)                    yandi     = true;
  if (yandi   && !irtifaOk   && (v.irtifa - yerIrtifa) > MIN_IRTIFA)         irtifaOk  = true;
  if (kalkti  && !govdeAcisi && irtifaOk && sapma > GOVDE_ACISI)             govdeAcisi= true;
  if (irtifaOk && !apogee && govdeAcisi && (zirveIrtifa - v.irtifa) > APOGEE_MARGIN) apogee = true;

  if (kalkti && yandi && govdeAcisi && irtifaOk && apogee && !ApogeePyroOk) {
    ApogeePyroOk = true;
    digitalWrite(ApogeeMosfet, HIGH);
    sonApogeeMosfet = simdi;
    digitalWrite(buzzer, HIGH);
    sonbuzzer = simdi;
    buzzerA   = true;
  }

  if (kalkti && yandi && govdeAcisi && irtifaOk && apogee && ApogeePyroOk
      && !Main && (v.irtifa - yerIrtifa) <= MAIN_IRTIFA) {
    Main = true;
  }

  if (kalkti && yandi && govdeAcisi && irtifaOk && apogee && ApogeePyroOk
      && Main && !MainPyroOk) {
    MainPyroOk = true;
    digitalWrite(MainMosfet, HIGH);
    sonMainMosfet = simdi;
    digitalWrite(buzzer, HIGH);
    sonbuzzer = simdi;
    buzzerA   = true;
  }
}

////////////////////////////////////
// LED DURUM GÖSTERGESİ (Senaryo A)
////////////////////////////////////
void ledModGuncelle() {
  if (mevcutMod == MOD_NORMAL) {
    digitalWrite(LED_MOD, LOW);
    ledModDurum = false;
    return;
  }

  unsigned long aralik;
  if (mevcutMod == MOD_SIT) aralik = 400;
  else                      aralik = 120;

  if (simdi - sonLedMod >= aralik) {
    sonLedMod   = simdi;
    ledModDurum = !ledModDurum;
    digitalWrite(LED_MOD, ledModDurum ? HIGH : LOW);
  }
}

void ledUcusGuncelle() {
  if (MainPyroOk) {
    digitalWrite(LED_UCUS, HIGH);
    return;
  }

  if (!kalkti) {
    digitalWrite(LED_UCUS, LOW);
    ledUcusDurum = false;
    return;
  }

  unsigned long onSure  = 150;
  unsigned long offSure = ApogeePyroOk ? 300 : 700;
  unsigned long aralik  = ledUcusDurum ? onSure : offSure;

  if (simdi - sonLedUcus >= aralik) {
    sonLedUcus   = simdi;
    ledUcusDurum = !ledUcusDurum;
    digitalWrite(LED_UCUS, ledUcusDurum ? HIGH : LOW);
  }
}

////////////////////////////////////
float ikiBasamak(float v) {
  return roundf(v * 100.0f) / 100.0f;
}

////////////////////////////////////
// RS232 VERİ PAKETİ GÖNDER (MOD_SIT)
////////////////////////////////////
void rs232Gonder(const SensorVerisi &v) {
  uint8_t paket[36];
  paket[0] = 0xAB;

  floatToBytes(ikiBasamak(v.irtifa), &paket[1]);
  floatToBytes(ikiBasamak(v.basinc), &paket[5]);
  floatToBytes(ikiBasamak(v.ivmeX),  &paket[9]);
  floatToBytes(ikiBasamak(v.ivmeY),  &paket[13]);
  floatToBytes(ikiBasamak(v.ivmeZ),  &paket[17]);
  floatToBytes(ikiBasamak(v.aciX),   &paket[21]);
  floatToBytes(ikiBasamak(v.aciY),   &paket[25]);
  floatToBytes(ikiBasamak(v.aciZ),   &paket[29]);

  uint8_t toplam = 0;
  for (int i = 0; i < 33; i++) toplam += paket[i];
  paket[33] = toplam;
  paket[34] = 0x0D;
  paket[35] = 0x0A;

  RS232.write(paket, 36);
}

////////////////////////////////////
// DURUM PAKETİ GÖNDER (MOD_SUT)
////////////////////////////////////
void durumPaketiGonder() {
  uint8_t data1 = 0;
  if (kalkti)       data1 |= (1 << 0);
  if (yandi)        data1 |= (1 << 1);
  if (irtifaOk)     data1 |= (1 << 2);
  if (govdeAcisi)   data1 |= (1 << 3);
  if (apogee)       data1 |= (1 << 4);
  if (ApogeePyroOk) data1 |= (1 << 5);
  if (Main)         data1 |= (1 << 6);
  if (MainPyroOk)   data1 |= (1 << 7);

  uint8_t paket[6];
  paket[0] = 0xAA;
  paket[1] = data1;
  paket[2] = 0x00;

  uint8_t toplam = 0;
  for (int i = 0; i < 3; i++) toplam += paket[i];
  paket[3] = toplam;
  paket[4] = 0x0D;
  paket[5] = 0x0A;

  RS232.write(paket, 6);
}

////////////////////////////////////
// MOD TANIMI
////////////////////////////////////
enum Mod { MOD_NORMAL, MOD_SIT, MOD_SUT };
Mod mevcutMod = MOD_NORMAL;

////////////////////////////////////
// SERİAL ALICI STATE MACHINE
////////////////////////////////////
enum AliciDurum { BEKLE_HEADER, KOMUT_OKU, VERI_OKU };
AliciDurum    aliciDurum  = BEKLE_HEADER;
uint8_t       rxBuffer[40];
uint8_t       rxIndex     = 0;
unsigned long rxBaslangic = 0;

void serialVeriIsle() {
  while (RS232.available()) {
    uint8_t b         = RS232.read();
    unsigned long simdiS = millis();

    if (aliciDurum == BEKLE_HEADER) {
      if (b == 0xAA) {
        rxBuffer[0] = b;
        rxIndex      = 1;
        aliciDurum   = KOMUT_OKU;
        rxBaslangic  = simdiS;
      } else if (b == 0xAB && mevcutMod == MOD_SUT) {
        rxBuffer[0] = b;
        rxIndex      = 1;
        aliciDurum   = VERI_OKU;
        rxBaslangic  = simdiS;
      }
    }
    else if (aliciDurum == KOMUT_OKU) {
      rxBuffer[rxIndex++] = b;
      if (rxIndex >= 5) {
        if (rxBuffer[1] == 0x20) {
          mevcutMod = MOD_SIT;
        } else if (rxBuffer[1] == 0x22) {
          mevcutMod = MOD_SUT;
          // Kalman sıfırla
          kalmanIrtifa    = 0.0f;
          kalmanHiz       = 0.0f;
          P_00 = 1.0f; P_01 = 0.0f;
          P_10 = 0.0f; P_11 = 1.0f;
          sonKalmanZamani = 0;
          // Yer irtifası sıfırla
          yerIrtifaHazir  = false;
          yerIrtifaSayac  = 0;
          yerIrtifaToplam = 0.0f;
          // Uçuş sıfırla
          zirveIrtifa = 0;
          kalkti = yandi = irtifaOk = apogee = govdeAcisi
                 = ApogeePyroOk = Main = MainPyroOk = false;
        } else if (rxBuffer[1] == 0x24) {
          mevcutMod = MOD_NORMAL;
          kalkti = yandi = irtifaOk = apogee = govdeAcisi
                 = ApogeePyroOk = Main = MainPyroOk = false;
          zirveIrtifa = 0;
          buzzerA     = false;
          digitalWrite(ApogeeMosfet, LOW);
          digitalWrite(MainMosfet,   LOW);
          digitalWrite(buzzer,       LOW);
        }

        aliciDurum = BEKLE_HEADER;
        rxIndex    = 0;
      }
    }
    else if (aliciDurum == VERI_OKU) {
      rxBuffer[rxIndex++] = b;
      if (rxIndex >= 36) {
        uint8_t toplam = 0;
        for (int i = 0; i < 33; i++) toplam += rxBuffer[i];

        if (toplam == rxBuffer[33]) {
          SensorVerisi sutVeri = {0};
          float hamIrtifa  = bytesToFloat(&rxBuffer[1]);
          sutVeri.basinc   = bytesToFloat(&rxBuffer[5]);
          sutVeri.ivmeX    = bytesToFloat(&rxBuffer[9]);
          sutVeri.ivmeY    = bytesToFloat(&rxBuffer[13]);
          sutVeri.ivmeZ    = bytesToFloat(&rxBuffer[17]);
          sutVeri.aciX     = bytesToFloat(&rxBuffer[21]);
          sutVeri.aciY     = bytesToFloat(&rxBuffer[25]);
          sutVeri.aciZ     = bytesToFloat(&rxBuffer[29]);

          // Kalman: sadece SUT verisiyle güncelle
          kalmanFiltresiGuncelle(hamIrtifa, sutVeri.ivmeZ);
          sutVeri.irtifa = kalmanIrtifa;

          tamamlayiciFiltreUygula(sutVeri.ivmeX, sutVeri.ivmeY, sutVeri.ivmeZ,
                                  sutVeri.aciX,  sutVeri.aciY,  sutVeri.aciZ);

          // Önce yer irtifasını biriktir, hazır olunca algoritmayı çalıştır
          if (!yerIrtifaHazir) {
            yerIrtifaToplam += hamIrtifa;
            yerIrtifaSayac++;
            if (yerIrtifaSayac >= YER_IRTIFA_ORNEK) {
              yerIrtifa      = yerIrtifaToplam / (float)YER_IRTIFA_ORNEK;
              yerIrtifaHazir = true;
            }
          } else {
            algoritmaCalistir(sutVeri);
          }
        }

        aliciDurum = BEKLE_HEADER;
        rxIndex    = 0;
      }
    }

    // Timeout
    if (aliciDurum != BEKLE_HEADER && simdiS - rxBaslangic > 100) {
      aliciDurum = BEKLE_HEADER;
      rxIndex    = 0;
    }
  }
}
////////////////////////////////////
void loraSensorGonder(const SensorVerisi &v) {
  uint8_t paket[35];
  int idx = 0;

  auto writeUint16 = [&](uint16_t val) {
    paket[idx++] = (val >> 8) & 0xFF;
    paket[idx++] = val & 0xFF;
  };

  auto writeInt16 = [&](int16_t val) {
    paket[idx++] = (val >> 8) & 0xFF;
    paket[idx++] = val & 0xFF;
  };

  // Header
  paket[idx++] = 0xBC;

  // İrtifa: uint16, 0.1m hassasiyet, 0-6553.5m
  writeUint16((uint16_t)(ikiBasamak(v.irtifa) * 10));
  // Kalman hız: int16, 0.01m/s hassasiyet
  writeInt16((int16_t)(ikiBasamak(kalmanHiz) * 100));
  // Basınç: uint16, 0.1hPa hassasiyet, 0-6553.5hPa (negatif olamayacağı için uint)
  writeUint16((uint16_t)(ikiBasamak(v.basinc) * 10));
  // İvmeler: int16, 0.01m/s² hassasiyet
  writeInt16((int16_t)(ikiBasamak(v.ivmeX) * 100));
  writeInt16((int16_t)(ikiBasamak(v.ivmeY) * 100));
  writeInt16((int16_t)(ikiBasamak(v.ivmeZ) * 100));
  // Açılar: int16, 0.1° hassasiyet
  writeInt16((int16_t)(ikiBasamak(v.aciX) * 10));
  writeInt16((int16_t)(ikiBasamak(v.aciY) * 10));
  writeInt16((int16_t)(ikiBasamak(v.aciZ) * 10));

  // GPS koordinatları float kalıyor (hassasiyet kritik)
  floatToBytes(v.enlem,  &paket[idx]); idx += 4;
  floatToBytes(v.boylam, &paket[idx]); idx += 4;

  // GPS irtifa: uint16, 1m hassasiyet, 0-65535m
  writeUint16((uint16_t)(v.gpsIrtifa));

  // Tekli byte'lar
  paket[idx++] = (uint8_t)v.uyduSayisi;
  paket[idx++] = (uint8_t)v.saat;
  paket[idx++] = (uint8_t)v.dakika;
  paket[idx++] = (uint8_t)v.saniye;

  // Durum bayrakları
  uint8_t data1 = 0;
  if (kalkti)       data1 |= (1 << 0);
  if (yandi)        data1 |= (1 << 1);
  if (irtifaOk)     data1 |= (1 << 2);
  if (govdeAcisi)   data1 |= (1 << 3);
  if (apogee)       data1 |= (1 << 4);
  if (ApogeePyroOk) data1 |= (1 << 5);
  if (Main)         data1 |= (1 << 6);
  if (MainPyroOk)   data1 |= (1 << 7);
  paket[idx++] = data1;

  // Checksum (idx burada 34, yani sadece gerçekten yazılmış byte'lar toplanıyor)
  uint8_t toplam = 0;
  for (int i = 0; i < idx; i++) toplam += paket[i];
  paket[idx++] = toplam;  // idx = 35

  LoRaSerial.write(paket, 35);
}

////////////////////////////////////
// LORA KONFİGÜRASYON (USB Serial üzerinden hex komut)
////////////////////////////////////
// Kullanım: Serial Monitor'e boşlukla ayrılmış hex byte'lar yazıp
// Enter'a basmak yeterli, örn: C0 00 09 00 00 1A 17 00 00 62 00 00
void loraKonfigurasyonUygula() {
  loraKonfigAktif = true;

  // Konfigürasyon modu (Mod 2): M0 = LOW, M1 = HIGH
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, HIGH);
  delay(20);

  // E22 serisi konfigürasyon modunda sabit 9600 baud kullanır
  LoRaSerial.end();
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  delay(20);

  LoRaSerial.write(loraHexBuffer, loraHexSayac);

  Serial.print("LORA_YANIT: ");
  unsigned long yanitT = millis();
  while (millis() - yanitT < 500) {
    while (LoRaSerial.available()) {
      uint8_t b = LoRaSerial.read();
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
      yanitT = millis();
    }
  }
  Serial.println();

  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(20);

  LoRaSerial.end();
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);

  loraKonfigAktif = false;
}

void serialUsbIsle() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (loraSatirIndex > 0) {
        loraSatir[loraSatirIndex] = '\0';
        loraHexSayac = 0;

        char *tok = strtok(loraSatir, " ");
        while (tok != NULL && loraHexSayac < LORA_HEX_MAKS) {
          loraHexBuffer[loraHexSayac++] = (uint8_t)strtol(tok, NULL, 16);
          tok = strtok(NULL, " ");
        }

        if (loraHexSayac > 0) loraKonfigurasyonUygula();
        loraSatirIndex = 0;
      }
    } else if (loraSatirIndex < sizeof(loraSatir) - 1) {
      loraSatir[loraSatirIndex++] = c;
    }
  }
}

////////////////////////////////////
// SENSÖR OKUMA
////////////////////////////////////
SensorVerisi sensorOku() {
  SensorVerisi v = {0};

  if (bnoHazir) {
    imu::Vector<3> acc     = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro    = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> linIvme = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    v.ivmeX = linIvme.x();
    v.ivmeY = linIvme.y();
    v.ivmeZ = linIvme.z();

    tamamlayiciFiltreUygula(acc.x(), acc.y(), acc.z(),
                             gyro.x(), gyro.y(), gyro.z());
    v.aciX = filtreRoll;
    v.aciY = filtrePitch;

    imu::Quaternion q = bno.getQuat();
    float siny = 2.0f * (q.w() * q.z() + q.x() * q.y());
    float cosy = 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z());
    v.aciZ = atan2f(siny, cosy) * 180.0f / M_PI;
  }

  if (bmp.performReading()) {
    v.basinc   = bmp.pressure / 100.0f;
    v.sicaklik = bmp.temperature;
  }

  float mutlakBaroIrtifa = 44330.0f * (1.0f - pow((v.basinc / 1013.25f), 0.19029f));
  kalmanFiltresiGuncelle(mutlakBaroIrtifa, v.ivmeZ);
  v.irtifa = kalmanIrtifa;

  while (GPS.available()) gps.encode(GPS.read());
  if (gps.location.isValid()) {
    v.enlem      = gps.location.lat();
    v.boylam     = gps.location.lng();
    v.gpsIrtifa  = gps.altitude.meters();
    v.uyduSayisi = gps.satellites.value();
    v.saat       = gps.time.hour() + 3;
    if (v.saat >= 24) v.saat -= 24;
    v.dakika = gps.time.minute();
    v.saniye = gps.time.second();
  }

  return v;
}

////////////////////////////////////
SensorVerisi veri = {0};

////////////////////////////////////
void setup() {

  Serial.begin(115200);
  GPS.begin(9600,SERIAL_8N1, D1, D0);
  RS232.begin(115200, SERIAL_8N1, D8, D7);
  Wire.begin();
  bmp.begin_I2C();

  pinMode(MainMosfet,   OUTPUT); digitalWrite(MainMosfet,   LOW);
  pinMode(ApogeeMosfet, OUTPUT); digitalWrite(ApogeeMosfet, LOW);
  pinMode(buzzer,       OUTPUT); digitalWrite(buzzer,       LOW);
  pinMode(LORA_M0, OUTPUT); digitalWrite(LORA_M0, LOW);
  pinMode(LORA_M1, OUTPUT); digitalWrite(LORA_M1, LOW);
  pinMode(LORA_AUX, INPUT);

  pinMode(LED_GUC,  OUTPUT); digitalWrite(LED_GUC,  HIGH);
  pinMode(LED_MOD,  OUTPUT); digitalWrite(LED_MOD,  LOW);
  pinMode(LED_UCUS, OUTPUT); digitalWrite(LED_UCUS, LOW);

  if (bno.begin()) {
    bnoHazir = true;
    bno.setExtCrystalUse(true);
  }

  // Modül hazır olana kadar bekle (maks 3 sn)
  unsigned long loraT = millis();
  while (digitalRead(LORA_AUX) == LOW && millis() - loraT < 30000);
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);


  // MOD_NORMAL için yer irtifasını setup'ta hesapla
  float toplamYer = 0;
  for (int i = 0; i < 50; i++) {
    bmp.performReading();
    toplamYer += 44330.0f * (1.0f - pow((bmp.pressure / 100.0f / 1013.25f), 0.19029f));
    delay(20);
  }
  yerIrtifa      = toplamYer / 50.0f;
  yerIrtifaHazir = true;

}

////////////////////////////////////
void loop() {
  simdi = millis();
  serialVeriIsle();
  serialUsbIsle();

  if (mevcutMod != MOD_SUT) {
    if (simdi - sensorzamani >= 25) {
      sensorzamani = simdi;
      veri = sensorOku();
    }
  }

  if (mevcutMod == MOD_NORMAL) {
    algoritmaCalistir(veri);
  }

  ledModGuncelle();
  ledUcusGuncelle();

  if (!loraKonfigAktif && simdi - sonLoraSensorZamani >= 100) {
    sonLoraSensorZamani = simdi;
    loraSensorGonder(veri);
  }

  // Buzzer kapat (1 saniye sonra)
  if (buzzerA && simdi - sonbuzzer >= 1000) {
    buzzerA = false;
    digitalWrite(buzzer, LOW);
    sonbuzzer = 0;
  }

  // Apogee MOSFET kapat (4 saniye sonra)
  if (sonApogeeMosfet && simdi - sonApogeeMosfet >= 4000) {
    digitalWrite(ApogeeMosfet, LOW);
    sonApogeeMosfet = 0;
  }

  // Main MOSFET kapat (4 saniye sonra)
  if (sonMainMosfet && simdi - sonMainMosfet >= 4000) {
    digitalWrite(MainMosfet, LOW);
    sonMainMosfet = 0;
  }

  // MOD_SUT: durum paketi gönder (100ms)
  if (mevcutMod == MOD_SUT && simdi - sonDurumZamani >= 100) {
    sonDurumZamani = simdi;
    durumPaketiGonder();
  }

  // MOD_SIT: sensör verisi gönder (100ms)
  if (mevcutMod == MOD_SIT && simdi - sonRS232Zamani >= 100) {
    sonRS232Zamani = simdi;
    rs232Gonder(veri);
  }
}
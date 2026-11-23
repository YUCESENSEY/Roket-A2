// ============================================================
// ÖZGÜN UÇUŞ KONTROL BİLGİSAYARI
// UKB FIRMWARE
//
// RS232:
//   Mevcut 36-byte FLOAT protokolü korunmuştur.
//
// LoRa:
//   Ayrı 36-byte sıkıştırılmış binary telemetri protokolü.
//
// LoRa paket formatı:
//   1  byte   başlangıç
//   2  byte   paket_no
//   4  byte   zaman
//   4  byte   irtifa       (0.01 m)
//   2  byte   basınç       (0.1 hPa)
//   6  byte   ivme        (0.01 m/s² x3)
//   6  byte   açı         (0.01 derece x3)
//   8  byte   GPS         (1e-6 derece x2)
//   1  byte   checksum
//   1  byte   footer
//   1  byte   footer
//   ----------------------
//   36 byte
// ============================================================


// ============================================================
// KÜTÜPHANELER
// ============================================================

#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <HardwareSerial.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <TinyGPS++.h>


// ============================================================
// PIN TANIMLARI
// ============================================================

#define GPS_RX_PIN        1
#define GPS_TX_PIN        0
#define GPS_BAUD          9600

#define RS232_RX_PIN      7
#define RS232_TX_PIN      8
#define RS232_BAUD        115200

#define LORA_RX_PIN       5
#define LORA_TX_PIN       4
#define LORA_AUX_PIN      3
#define LORA_BAUD         115200

#define AUX_MIRROR_PIN    A7

#define PYRO1_PIN         A0
#define BUZZER_PIN        2

#define BMP390_I2C_ADDR   0x77
#define BNO055_I2C_ADDR   0x28


// ============================================================
// UART NESNELERİ
// ============================================================

HardwareSerial GPS(0);
HardwareSerial RS232(1);
HardwareSerial LoRaSerial(2);


// ============================================================
// HABERLEŞME PARAMETRELERİ
// ============================================================

// LoRa 11 Hz'de çalışır (değiştirilmedi)
#define LORA_TELEMETRY_HZ          11
#define LORA_TELEMETRY_PERIOD_MS   (1000 / LORA_TELEMETRY_HZ)

// LoRa başlangıç gecikmesi (1 dakika)
//
// NOT: millis() + offset KULLANILMAZ.
// Unsigned long taşması yanlış sonuç verir:
//   Örnek: millis()=2000, offset=60000
//   sonLoRa = 62000
//   loop'ta simdi=3000
//   3000 - 62000 → unsigned taşır → devasa sayı
//   devasa >= 90 → TRUE → hemen gönderir!
//
// Doğru yöntem: ayrı bayrak + başlangıç zamanı.
#define LORA_BASLANGIC_GECIKMESI_MS  60000UL

// SIT/SUT RS232 telemetrisi dokümana göre 10 Hz
#define RS232_TELEMETRY_HZ         10
#define RS232_TELEMETRY_PERIOD_MS  (1000 / RS232_TELEMETRY_HZ)

#define MODE_ACTIVATE_DELAY_MS     1000
#define SUT_TIMEOUT_MS             1000   // Dokümanda 1000 ms olarak belirtilmiş
#define FRAME_TIMEOUT_MS           100

#define STATE_SAVE_PERIOD_MS       500
#define I2C_TIMEOUT_MS             10
#define NVS_SAVE_PERIOD_MS         5000


// ============================================================
// BUZZER
// ============================================================

#define STARTUP_BEEP_COUNT       5
#define STARTUP_BEEP_ON_MS       300
#define STARTUP_BEEP_OFF_MS      200

#define HEARTBEAT_INTERVAL_MS    10000
#define HEARTBEAT_DURATION_MS    1000


// ============================================================
// UÇUŞ ALGORİTMASI
// ============================================================

#define DENIZ_SEVIYESI_BASINC_HPA   1013.25f

#define KALKIS_IVME_ESIGI_MS2       20.0f

#define YANMA_MIN_SURE_MS           500UL
#define YANMA_IVME_ESIGI_MS2        15.0f
#define YANMA_BITIS_ONAY_ORNEK      3

#define MIN_IRTIFA_ESIGI_M          1300.0f

#define GOVDE_ACISI_ESIGI_DERECE    60.0f
#define ACI_ONAY_ARDISIK_ORNEK      2

#define APOGEE_DUSUS_MARGIN_M       0.1f
#define APOGEE_DUSUS_HIZ_ESIGI_MS   0.0f
#define APOGEE_ONAY_ORNEK_SAYISI    2

#define SURUKLENME_GECIKME_MS       50UL
#define SURUKLENME_HIZ_ESIGI_MS     -0.5f
#define SURUKLENME_ONAY_ORNEK_SAYISI 2

#define BASLANGIC_KALIBRASYON_ORNEK 20


// ============================================================
// DURUM BİTLERİ
// ============================================================

#define BIT_KTE  (1 << 0)
#define BIT_YSD  (1 << 1)
#define BIT_IEA  (1 << 2)
#define BIT_GAA  (1 << 3)
#define BIT_ATE  (1 << 4)
#define BIT_SPE  (1 << 5)


// ============================================================
// RTC
// ============================================================

#define RTC_MAGIC 0x524F4B45UL

RTC_NOINIT_ATTR uint32_t rtc_magic;
RTC_NOINIT_ATTR uint16_t rtc_durumBitleri;
RTC_NOINIT_ATTR float rtc_zirveIrtifa;
RTC_NOINIT_ATTR uint8_t rtc_pyroAteslendi;
RTC_NOINIT_ATTR uint8_t rtc_modu;
RTC_NOINIT_ATTR uint32_t rtc_kalkisZamani;
RTC_NOINIT_ATTR uint32_t rtc_apogeeTespitMs;

Preferences nvPrefs;


// ============================================================
// TEST MODLARI
// ============================================================

enum TestModu {
  MOD_NORMAL,
  MOD_SIT_BEKLEME,
  MOD_SIT_AKTIF,
  MOD_SUT_BEKLEME,
  MOD_SUT_AKTIF
};


// ============================================================
// RS232 ALICI DURUMLARI
// ============================================================

enum AliciDurum {
  BEKLE_HEADER,
  KOMUT_OKU,
  VERI_OKU
};


// ============================================================
// UÇUŞ VERİSİ
// ============================================================

struct UcusVerisi {
  float irtifa;
  float basinc;

  float ivmeX;
  float ivmeY;
  float ivmeZ;

  float aciX;
  float aciY;
  float aciZ;
};


// ============================================================
// QUATERNION
// ============================================================

struct QuaternionSimple {
  float w;
  float x;
  float y;
  float z;
};


// ============================================================
// LORA PAKETİ
//
// TOPLAM = 36 BYTE
//
// 0       uint8_t   baslangic
// 1-2     uint16_t  paket_no
// 3-6     uint32_t  zaman
// 7-10    uint32_t  irtifa_x100
// 11-12   uint16_t  basinc_x10
// 13-14   int16_t   ivme_x_x100
// 15-16   int16_t   ivme_y_x100
// 17-18   int16_t   ivme_z_x100
// 19-20   int16_t   aci_x_x100
// 21-22   int16_t   aci_y_x100
// 23-24   int16_t   aci_z_x100
// 25-28   int32_t   gps_enlem_x1e6
// 29-32   int32_t   gps_boylam_x1e6
// 33      uint8_t   checksum
// 34      uint8_t   footer1
// 35      uint8_t   footer2
// ============================================================

struct __attribute__((packed)) LoRaPaketi {

  uint8_t baslangic;

  uint16_t paket_no;

  uint32_t zaman;

  uint32_t irtifa_x100;

  uint16_t basinc_x10;

  int16_t ivme_x_x100;
  int16_t ivme_y_x100;
  int16_t ivme_z_x100;

  int16_t aci_x_x100;
  int16_t aci_y_x100;
  int16_t aci_z_x100;

  int32_t gps_enlem_x1e6;
  int32_t gps_boylam_x1e6;

  uint8_t checksum;

  uint8_t footer1;
  uint8_t footer2;
};


// Derleme sırasında boyutun gerçekten 36 olduğundan emin ol.
static_assert(sizeof(LoRaPaketi) == 36, "LoRaPaketi 36 byte olmali");


// ============================================================
// SENSOR NESNELERİ
// ============================================================

Adafruit_BMP3XX bmp;
bool bmpHazir = false;
uint8_t bmpAktifAdres = 0;

Adafruit_BNO055 bno(55, BNO055_I2C_ADDR, &Wire);
bool bnoHazir = false;


// ============================================================
// AÇI REFERANSI
// ============================================================

float aciReferansX = 0.0f;
float aciReferansY = 0.0f;
float aciReferansZ = 0.0f;

bool aciReferansiHazir = false;


// ============================================================
// MOD VE RS232 DURUMU
// ============================================================

TestModu modu = MOD_NORMAL;

unsigned long modBeklemeBaslangic = 0;

AliciDurum aliciDurumu = BEKLE_HEADER;

uint8_t rxBuffer[40];
uint8_t rxIndex = 0;

unsigned long rxBaslangic = 0;


// ============================================================
// SENTETİK VERİ
// ============================================================

UcusVerisi sonSentetikVeri = {0};

unsigned long sonSentetikVeriZamani = 0;


// ============================================================
// ZAMANLAMA
// ============================================================

unsigned long sonLoRaTelemetriZamani  = 0;
unsigned long sonRS232TelemetriZamani = 0;
unsigned long sonDurumPaketiZamani    = 0;
unsigned long sonDurumKayitZamani     = 0;


// ============================================================
// LORA GECİKME DEĞİŞKENLERİ
// ============================================================
//
// millis() + offset yerine ayrı bayrak kullanılır.
// Bunun nedeni: unsigned long taşması.
//
// Örnek hata senaryosu:
//   millis() = 2000
//   sonLoRaTelemetriZamani = 2000 + 60000 = 62000
//   loop'ta simdi = 3000
//   3000 - 62000 → unsigned taşır → devasa sayı
//   devasa sayı >= 90 → TRUE → hemen gönderir!
//
// Doğru yöntem: ayrı bayrak ve başlangıç zamanı.
// ============================================================

bool loraBasladi = false;
unsigned long loraBaslangicZamani = 0;


// ============================================================
// UÇUŞ DURUMU
// ============================================================

uint16_t durumBitleri = 0;

float zirveIrtifa = -1000.0f;

uint16_t ardisikAcisayaci = 0;

uint8_t dususOnaySayaci = 0;
uint8_t suruklenmeOnaySayaci = 0;
uint8_t yanmaBitisOnaySayaci = 0;

unsigned long kalkisZamani = 0;
unsigned long apogeeTespitMs = 0;

unsigned long pyroBaslangicZamani = 0;

bool pyroAktif = false;
bool pyroAteslendi = false;


// ============================================================
// RAMPA REFERANSI
// ============================================================

float rampaIrtifasi = 0.0f;
bool rampaReferansiHazir = false;

float rampaBasincHpa = 0.0f;


// ============================================================
// GYRO
// ============================================================

float gyroRateX = 0.0f;
float gyroRateY = 0.0f;
float gyroRateZ = 0.0f;


// ============================================================
// KALMAN
// ============================================================

struct Kalman1D {

  float x[2];

  float P[2][2];
  float Q[2][2];

  float R;

  bool initialized;


  Kalman1D()
    : R(25.0f),
      initialized(false) {}


  void init(float altitude) {

    x[0] = altitude;
    x[1] = 0.0f;

    P[0][0] = 1.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 1.0f;

    Q[0][0] = 0.002f;
    Q[0][1] = 0.0f;

    Q[1][0] = 0.0f;
    Q[1][1] = 0.30f;

    R = 6.0f;

    initialized = true;
  }


  void predict(float dt) {

    x[0] += x[1] * dt;

    float p00 = P[0][0];
    float p01 = P[0][1];
    float p10 = P[1][0];
    float p11 = P[1][1];

    P[0][0] =
      p00 +
      dt * p10 +
      dt * p01 +
      dt * dt * p11 +
      Q[0][0];

    P[0][1] =
      p01 +
      dt * p11 +
      Q[0][1];

    P[1][0] =
      p10 +
      dt * p11 +
      Q[1][0];

    P[1][1] =
      p11 +
      Q[1][1];
  }


  void update(float z) {

    float y = z - x[0];

    float S = P[0][0] + R;

    if (S <= 0.0f)
      return;

    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    float p00 = P[0][0];
    float p01 = P[0][1];
    float p10 = P[1][0];
    float p11 = P[1][1];

    x[0] += K0 * y;
    x[1] += K1 * y;

    P[0][0] = p00 - K0 * p00;
    P[0][1] = p01 - K0 * p01;
    P[1][0] = p10 - K1 * p00;
    P[1][1] = p11 - K1 * p01;
  }


  float altitude() const {
    return x[0];
  }


  float velocity() const {
    return x[1];
  }


  void reset() {

    initialized = false;

    x[0] = 0.0f;
    x[1] = 0.0f;
  }
};


Kalman1D kalman;

unsigned long kalmanLastMs = 0;

float filtreliHiz = 0.0f;


// ============================================================
// GPS
// ============================================================

TinyGPSPlus gps;

#define GPS_FIX_TIMEOUT_MS 2000UL

bool gpsSonKonumGecerli = false;

float gpsSonEnlem = 0.0f;
float gpsSonBoylam = 0.0f;

uint8_t gpsSonUydu = 0;


// ============================================================
// LORA SAYACI
// ============================================================

uint16_t loraPaketSayaci = 0;


// ============================================================
// NVS / HEARTBEAT
// ============================================================

bool nvsReady = false;

unsigned long sonNvsKayitZamani = 0;

unsigned long heartbeatLastMs = 0;

bool heartbeatActive = false;


// ============================================================
// AÇI NORMALİZASYONU
// ============================================================

float normalizeAngle180(float angle) {

  while (angle > 180.0f)
    angle -= 360.0f;

  while (angle < -180.0f)
    angle += 360.0f;

  return angle;
}


// ============================================================
// RESET FONKSİYONLARI
// ============================================================

void resetAciKalman() {

  // BNO055 kendi sensör füzyonunu kullandığı
  // için harici açı filtresi yok.
}


void resetIrtifaKalman() {

  kalman.reset();

  kalmanLastMs = 0;

  filtreliHiz = 0.0f;

  rampaReferansiHazir = false;

  rampaIrtifasi = 0.0f;
}


float filtreliHizUret(float yeniHiz) {

  // Kalman zaten irtifa + hız durumunu filtreliyor.
  // Apogee tespitinde ek low-pass kullanmak gecikme oluşturuyordu.
  // Bu nedenle Kalman hızını doğrudan kullanıyoruz.
  filtreliHiz = yeniHiz;

  return filtreliHiz;
}


void resetUcusTespitleri() {

  durumBitleri = 0;

  zirveIrtifa = -1000.0f;

  ardisikAcisayaci = 0;

  dususOnaySayaci = 0;

  suruklenmeOnaySayaci = 0;

  yanmaBitisOnaySayaci = 0;

  kalkisZamani = 0;

  apogeeTespitMs = 0;

  pyroBaslangicZamani = 0;

  pyroAktif = false;

  pyroAteslendi = false;
}


// ============================================================
// CHECKSUM
// ============================================================

uint8_t hesaplaChecksum(
  const uint8_t *veri,
  size_t uzunluk
) {

  uint32_t toplam = 0;

  for (size_t i = 0; i < uzunluk; i++) {

    toplam += veri[i];
  }

  return (uint8_t)(toplam % 256);
}


// ============================================================
// FLOAT <-> BYTE DÖNÜŞÜMÜ
//
// RS232 için çalışan eski sürümdeki dönüşüm aynen korunmuştur.
// ============================================================

void floatToBytes(
  float deger,
  uint8_t *hedef
) {

  uint8_t kaynak[4];

  memcpy(kaynak, &deger, 4);

  hedef[0] = kaynak[3];
  hedef[1] = kaynak[2];
  hedef[2] = kaynak[1];
  hedef[3] = kaynak[0];
}


float bytesToFloat(
  const uint8_t *kaynak
) {

  uint8_t hedef[4];

  hedef[0] = kaynak[3];
  hedef[1] = kaynak[2];
  hedef[2] = kaynak[1];
  hedef[3] = kaynak[0];

  float d;

  memcpy(&d, hedef, 4);

  return d;
}


float ikiOndalikYuvarla(float deger) {

  return roundf(deger * 100.0f) / 100.0f;
}


// ============================================================
// QUATERNION
// ============================================================

QuaternionSimple quatConjugate(
  const QuaternionSimple &q
) {

  return {
    q.w,
    -q.x,
    -q.y,
    -q.z
  };
}


QuaternionSimple quatMultiply(
  const QuaternionSimple &a,
  const QuaternionSimple &b
) {

  QuaternionSimple r;

  r.w =
    a.w * b.w -
    a.x * b.x -
    a.y * b.y -
    a.z * b.z;

  r.x =
    a.w * b.x +
    a.x * b.w +
    a.y * b.z -
    a.z * b.y;

  r.y =
    a.w * b.y -
    a.x * b.z +
    a.y * b.w +
    a.z * b.x;

  r.z =
    a.w * b.z +
    a.x * b.y -
    a.y * b.x +
    a.z * b.w;

  return r;
}


void quatNormalize(
  QuaternionSimple &q
) {

  float n =
    sqrtf(
      q.w * q.w +
      q.x * q.x +
      q.y * q.y +
      q.z * q.z
    );

  if (n <= 0.0f) {

    q.w = 1.0f;

    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;

    return;
  }

  q.w /= n;
  q.x /= n;
  q.y /= n;
  q.z /= n;
}


// ============================================================
// AÇI KALMAN
// ============================================================

void updateAciKalman(
  const UcusVerisi &v
) {

  (void)v;
}


// ============================================================
// BAŞLANGIÇ AÇI REFERANSI
// ============================================================

void baslangicAciReferansiAl() {

  if (!bnoHazir) {

    aciReferansiHazir = false;

    return;
  }


  const int ORNEK_SAYISI = 20;

  float rollToplam = 0.0f;
  float pitchToplam = 0.0f;

  float yawSinToplam = 0.0f;
  float yawCosToplam = 0.0f;

  int gecerli = 0;


  for (int i = 0; i < ORNEK_SAYISI; i++) {

    imu::Vector<3> euler =
      bno.getVector(
        Adafruit_BNO055::VECTOR_EULER
      );

    float yaw = euler.x();
    float roll = euler.y();
    float pitch = euler.z();


    rollToplam += roll;
    pitchToplam += pitch;


    float yawRad =
      yaw * DEG_TO_RAD;

    yawSinToplam += sinf(yawRad);
    yawCosToplam += cosf(yawRad);


    gecerli++;

    delay(20);

    esp_task_wdt_reset();
  }


  if (gecerli > 0) {

    aciReferansX =
      rollToplam /
      (float)gecerli;

    aciReferansY =
      pitchToplam /
      (float)gecerli;


    float yawOrtalama =
      atan2f(
        yawSinToplam,
        yawCosToplam
      ) * RAD_TO_DEG;


    aciReferansZ =
      normalizeAngle180(
        yawOrtalama
      );

    aciReferansiHazir = true;
  }
}


// ============================================================
// SENSOR OKUMA
// ============================================================

UcusVerisi sensorleriOku() {

  UcusVerisi v = {0};


  // ----------------------------------------------------------
  // BMP390
  // ----------------------------------------------------------

  if (bmpHazir &&
      bmp.performReading()) {

    v.basinc =
      bmp.pressure / 100.0f;


    if (v.basinc > 0.0f) {

      float oran =
        v.basinc /
        DENIZ_SEVIYESI_BASINC_HPA;


      v.irtifa =
        44330.0f *
        (
          1.0f -
          powf(
            oran,
            0.19029495f
          )
        );

    } else {

      v.irtifa = 0.0f;
    }
  }


  // ----------------------------------------------------------
  // BNO055
  // ----------------------------------------------------------

  if (!bnoHazir) {

    gyroRateX = 0.0f;
    gyroRateY = 0.0f;
    gyroRateZ = 0.0f;

    return v;
  }


  imu::Vector<3> ivme =
    bno.getVector(
      Adafruit_BNO055::VECTOR_LINEARACCEL
    );


  imu::Vector<3> euler =
    bno.getVector(
      Adafruit_BNO055::VECTOR_EULER
    );


  v.ivmeX = ivme.x();
  v.ivmeY = ivme.y();
  v.ivmeZ = ivme.z();


  float hamRoll = euler.y();
  float hamPitch = euler.z();
  float hamYaw = euler.x();


  if (!aciReferansiHazir) {

    aciReferansX = hamRoll;
    aciReferansY = hamPitch;
    aciReferansZ = hamYaw;

    aciReferansiHazir = true;


    v.aciX = 0.0f;
    v.aciY = 0.0f;
    v.aciZ = 0.0f;

  } else {

    v.aciX =
      normalizeAngle180(
        hamRoll -
        aciReferansX
      );

    v.aciY =
      normalizeAngle180(
        hamPitch -
        aciReferansY
      );

    v.aciZ =
      normalizeAngle180(
        hamYaw -
        aciReferansZ
      );
  }


  gyroRateX = 0.0f;
  gyroRateY = 0.0f;
  gyroRateZ = 0.0f;


  return v;
}


// ============================================================
// RS232 SİT PAKETİ
// ============================================================

void gonderSitPaketi(
  const UcusVerisi &v
) {

  uint8_t paket[36];


  paket[0] = 0xAB;


  floatToBytes(
    ikiOndalikYuvarla(v.irtifa),
    &paket[1]
  );


  floatToBytes(
    ikiOndalikYuvarla(v.basinc),
    &paket[5]
  );


  floatToBytes(
    ikiOndalikYuvarla(v.ivmeX),
    &paket[9]
  );


  floatToBytes(
    ikiOndalikYuvarla(v.ivmeY),
    &paket[13]
  );


  floatToBytes(
    ikiOndalikYuvarla(v.ivmeZ),
    &paket[17]
  );


  floatToBytes(
    ikiOndalikYuvarla(
      normalizeAngle180(v.aciX)
    ),
    &paket[21]
  );


  floatToBytes(
    ikiOndalikYuvarla(
      normalizeAngle180(v.aciY)
    ),
    &paket[25]
  );


  floatToBytes(
    ikiOndalikYuvarla(
      normalizeAngle180(v.aciZ)
    ),
    &paket[29]
  );


  paket[33] =
    hesaplaChecksum(
      paket,
      33
    );


  paket[34] = 0x0D;
  paket[35] = 0x0A;


  RS232.write(
    paket,
    sizeof(paket)
  );
}


// ============================================================
// RS232 DURUM PAKETİ
// ============================================================

void gonderDurumPaketi(
  uint16_t bitler
) {

  uint8_t paket[6];


  paket[0] = 0xAA;

  paket[1] =
    (uint8_t)(
      bitler & 0xFF
    );

  paket[2] =
    (uint8_t)(
      (bitler >> 8) & 0xFF
    );


  paket[3] =
    hesaplaChecksum(
      paket,
      3
    );


  paket[4] = 0x0D;
  paket[5] = 0x0A;


  RS232.write(
    paket,
    sizeof(paket)
  );
}


// ============================================================
// LORA DEĞERİ SINIRLAMA
// ============================================================

int16_t floatToInt16Scaled(
  float value,
  float scale
) {

  float scaled =
    value * scale;


  if (scaled > 32767.0f)
    return 32767;


  if (scaled < -32768.0f)
    return -32768;


  return (int16_t)lroundf(
    scaled
  );
}


uint16_t floatToUint16Scaled(
  float value,
  float scale
) {

  float scaled =
    value * scale;


  if (scaled < 0.0f)
    return 0;


  if (scaled > 65535.0f)
    return 65535;


  return (uint16_t)lroundf(
    scaled
  );
}


uint32_t floatToUint32Scaled(
  float value,
  float scale
) {

  float scaled =
    value * scale;


  if (scaled < 0.0f)
    return 0;


  if (scaled > 4294967295.0f)
    return 4294967295UL;


  return (uint32_t)llroundf(
    scaled
  );
}


int32_t floatToInt32Scaled(
  float value,
  float scale
) {

  double scaled =
    (double)value *
    (double)scale;


  if (scaled > 2147483647.0)
    return 2147483647L;


  if (scaled < -2147483648.0)
    return (-2147483647L - 1L);


  return (int32_t)llround(
    scaled
  );
}


// ============================================================
// LORA PAKETİ HAZIRLAMA
// ============================================================

void hazirlaLoRaPaketi(
  LoRaPaketi &p,
  const UcusVerisi &v,
  float kalmanAlt,
  float kalmanVel
) {

  p.baslangic = 0xAB;

  p.paket_no =
    loraPaketSayaci++;

  p.zaman =
    millis();

  p.irtifa_x100 =
    floatToUint32Scaled(
      v.irtifa,
      100.0f
    );

  p.basinc_x10 =
    floatToUint16Scaled(
      v.basinc,
      10.0f
    );

  p.ivme_x_x100 =
    floatToInt16Scaled(
      v.ivmeX,
      100.0f
    );

  p.ivme_y_x100 =
    floatToInt16Scaled(
      v.ivmeY,
      100.0f
    );

  p.ivme_z_x100 =
    floatToInt16Scaled(
      v.ivmeZ,
      100.0f
    );

  p.aci_x_x100 =
    floatToInt16Scaled(
      normalizeAngle180(v.aciX),
      100.0f
    );

  p.aci_y_x100 =
    floatToInt16Scaled(
      normalizeAngle180(v.aciY),
      100.0f
    );

  p.aci_z_x100 =
    floatToInt16Scaled(
      normalizeAngle180(v.aciZ),
      100.0f
    );

  bool gpsGuncel =
    gps.location.isValid() &&
    gps.location.age() <= GPS_FIX_TIMEOUT_MS;


  if (gpsGuncel) {

    gpsSonEnlem =
      (float)gps.location.lat();

    gpsSonBoylam =
      (float)gps.location.lng();

    gpsSonUydu =
      (uint8_t)gps.satellites.value();

    gpsSonKonumGecerli = true;
  }


  if (gpsSonKonumGecerli) {

    p.gps_enlem_x1e6 =
      floatToInt32Scaled(
        gpsSonEnlem,
        1000000.0f
      );

    p.gps_boylam_x1e6 =
      floatToInt32Scaled(
        gpsSonBoylam,
        1000000.0f
      );

  } else {

    p.gps_enlem_x1e6 = 0;
    p.gps_boylam_x1e6 = 0;
  }


  p.checksum = 0;

  p.footer1 = 0x0D;
  p.footer2 = 0x0A;


  uint8_t *ptr =
    (uint8_t *)&p;


  p.checksum =
    hesaplaChecksum(
      ptr,
      33
    );


  (void)kalmanAlt;
  (void)kalmanVel;
}


// ============================================================
// LORA PAKETİNİ DİREKT GÖNDER
//
// AUX LOW  -> modül meşgul, paketi at
// AUX HIGH -> gönder
// ============================================================

void gonderLoRaPaketi(
  const UcusVerisi &v,
  float kalmanAlt,
  float kalmanVel
) {

  // AUX LOW ise modül meşgul, bu paketi at
  if (
    digitalRead(LORA_AUX_PIN)
    != HIGH
  ) {
    return;
  }


  LoRaPaketi p;

  hazirlaLoRaPaketi(
    p,
    v,
    kalmanAlt,
    kalmanVel
  );


  uint8_t *ptr = (uint8_t *)&p;


  LoRaSerial.write(
    ptr,
    sizeof(LoRaPaketi)
  );


  // UART tamponunun tamamen boşalmasını bekle
  LoRaSerial.flush();
}


// ============================================================
// GPS UART
// ============================================================

void gpsUARTUpdate() {

  while (GPS.available()) {

    gps.encode(
      GPS.read()
    );
  }
}


// ============================================================
// UÇUŞ ALGORİTMASI
// ============================================================


void ucusAlgoritmasiCalistir(
  const UcusVerisi &v
) {
 
  updateAciKalman(v);


  unsigned long now =
    millis();


  // ----------------------------------------------------------
  // KALMAN
  // ----------------------------------------------------------

  if (!kalman.initialized) {

    kalman.init(
      v.irtifa
    );

    kalmanLastMs = now;

  } else {

    float dt =
      (float)(
        now - kalmanLastMs
      ) / 1000.0f;


    kalmanLastMs = now;


    if (
      dt > 0.0f &&
      dt < 1.0f
    ) {

      kalman.predict(dt);
    }


    kalman.update(
      v.irtifa
    );
  }


  float filtAlt =
    kalman.altitude();


  // ----------------------------------------------------------
  // RAMPA REFERANSI
  // ----------------------------------------------------------

  if (!rampaReferansiHazir) {

    rampaIrtifasi =
      filtAlt;

    rampaReferansiHazir =
      true;
  }


  float ucusIrtifasi =
    filtAlt -
    rampaIrtifasi;


  // ----------------------------------------------------------
  // GÖVDE AÇISI
  // ----------------------------------------------------------

  float govdeAcisiX =
    normalizeAngle180(
      v.aciX
    );

  float govdeAcisiY =
    normalizeAngle180(
      v.aciY
    );

  bool govdeAcisiAsildi =
    fabsf(govdeAcisiX) > GOVDE_ACISI_ESIGI_DERECE ||
    fabsf(govdeAcisiY) > GOVDE_ACISI_ESIGI_DERECE;


  float irtifaHizi =
    filtreliHizUret(
      kalman.velocity()
    );


  bool irtifaHiziGecerli =
    kalman.initialized;


  // ----------------------------------------------------------
  // TOPLAM İVME
  // ----------------------------------------------------------

  float toplamIvme =
    sqrtf(
      v.ivmeX * v.ivmeX +
      v.ivmeY * v.ivmeY +
      v.ivmeZ * v.ivmeZ
    );


  // ----------------------------------------------------------
  // KALKIŞ
  // ----------------------------------------------------------

  if (
    !(durumBitleri & BIT_KTE) &&
    toplamIvme >
      KALKIS_IVME_ESIGI_MS2
  ) {

    durumBitleri |= BIT_KTE;

    kalkisZamani = now;
  }


  // ----------------------------------------------------------
  // YANMA SONU
  // ----------------------------------------------------------

  if (
    (durumBitleri & BIT_KTE) &&
    !(durumBitleri & BIT_YSD)
  ) {

    if (
      now - kalkisZamani >=
      YANMA_MIN_SURE_MS
    ) {

      if (
        toplamIvme <
        YANMA_IVME_ESIGI_MS2
      ) {

        if (
          yanmaBitisOnaySayaci <
          255
        ) {

          yanmaBitisOnaySayaci++;
        }

      } else {

        yanmaBitisOnaySayaci = 0;
      }


      if (
        yanmaBitisOnaySayaci >=
        YANMA_BITIS_ONAY_ORNEK
      ) {

        durumBitleri |= BIT_YSD;
      }
    }
  }


  // ----------------------------------------------------------
  // MİNİMUM İRTİFA
  // ----------------------------------------------------------

  if (
    !(durumBitleri & BIT_IEA) &&
    ucusIrtifasi >=
      MIN_IRTIFA_ESIGI_M
  ) {

    durumBitleri |= BIT_IEA;
  }


  // ----------------------------------------------------------
  // GÖVDE AÇISI
  // ----------------------------------------------------------

  if (
    !(durumBitleri & BIT_GAA)
  ) {

    if (
      govdeAcisiAsildi
    ) {

      ardisikAcisayaci++;

    } else {

      ardisikAcisayaci = 0;
    }


    if (
      ardisikAcisayaci >=
      ACI_ONAY_ARDISIK_ORNEK
    ) {

      durumBitleri |= BIT_GAA;
    }
  }


  // ----------------------------------------------------------
  // ZİRVE
  // ----------------------------------------------------------

  if (
    ucusIrtifasi >
    zirveIrtifa
  ) {

    zirveIrtifa =
      ucusIrtifasi;
  }


  // ----------------------------------------------------------
  // APOGEE
  // ----------------------------------------------------------

  if (
    !(durumBitleri & BIT_ATE) &&
    (durumBitleri & BIT_KTE) &&
    (durumBitleri & BIT_YSD) &&
    (durumBitleri & BIT_IEA)
  ) {

    bool apogeeIrtifaKosulu = false;
    bool apogeeHizKosulu = false;
    bool apogeeKosulu = false;

  apogeeIrtifaKosulu =
    (
      zirveIrtifa -
      ucusIrtifasi
    ) >
      APOGEE_DUSUS_MARGIN_M;

  apogeeHizKosulu =
    irtifaHiziGecerli &&
    irtifaHizi <
      APOGEE_DUSUS_HIZ_ESIGI_MS;

  apogeeKosulu =
    apogeeIrtifaKosulu &&
    apogeeHizKosulu;


    if (apogeeKosulu) {

      if (
        dususOnaySayaci <
        255
      ) {

        dususOnaySayaci++;
      }

    } else {

      dususOnaySayaci = 0;
    }


    if (
      dususOnaySayaci >=
      APOGEE_ONAY_ORNEK_SAYISI
    ) {

      durumBitleri |= BIT_ATE;

      apogeeTespitMs = now;

      suruklenmeOnaySayaci = 0;
    }

  } else if (
    !(durumBitleri & BIT_ATE)
  ) {

    dususOnaySayaci = 0;
  }


  // ----------------------------------------------------------
  // SÜRÜKLENME / PYRO
  //
  // DÜZELTME: Pyro ve buzzer yalnızca MOD_NORMAL'de fiziksel
  // olarak tetiklenir. MOD_SIT_AKTIF ve MOD_SUT_AKTIF modlarında
  // sadece durum biti set edilir, pine kesinlikle dokunulmaz.
  // Bu sayede SİT sırasında UKB sarsılsa bile pyro ateşlenmez.
  // ----------------------------------------------------------

  if (
    (durumBitleri & BIT_KTE) &&
    (durumBitleri & BIT_YSD) &&
    (durumBitleri & BIT_IEA) &&
    (durumBitleri & BIT_ATE) &&
    (durumBitleri & BIT_GAA) &&
    !(durumBitleri & BIT_SPE) &&
    !pyroAteslendi
  ) {

    durumBitleri |= BIT_SPE;

    pyroAteslendi = true;

    // Pyro ve buzzer yalnızca gerçek uçuş modunda (MOD_NORMAL)
    // fiziksel olarak aktive edilir.
    // SİT veya SUT modunda pin'e kesinlikle dokunulmaz.
    if (modu == MOD_NORMAL) {

      digitalWrite(
        BUZZER_PIN,
        LOW
      );

      digitalWrite(
        PYRO1_PIN,
        LOW
      );

      pyroBaslangicZamani =
        millis();

      pyroAktif = false;
    }
  }
}


// ============================================================
// RS232 KOMUT FRAME
//
// DÜZELTME: Tablo 1'deki checksum değerleri doğrulanıyor.
// SİT: 0xAA+0x20 = 0x8C
// SUT: 0xAA+0x22 = 0x8E
// Durdur: 0xAA+0x24 = 0x90
// Yanlış checksum gelen komutlar işleme alınmıyor.
// ============================================================


void islemKomutFrame(
  uint8_t *f
) {

  // ----------------------------------------------------------
  // SIT
  // ----------------------------------------------------------

  if (
    f[0] == 0xAA &&
    f[1] == 0x20
  ) {

    if (
      modu == MOD_NORMAL
    ) {

      modu =
        MOD_SIT_BEKLEME;

      modBeklemeBaslangic =
        millis();
    }
  }


  // ----------------------------------------------------------
  // SUT
  // ----------------------------------------------------------

  else if (
    f[0] == 0xAA &&
    f[1] == 0x22
  ) {

    if (
      modu == MOD_NORMAL
    ) {

      modu =
        MOD_SUT_BEKLEME;

      modBeklemeBaslangic =
        millis();


      resetUcusTespitleri();
      resetIrtifaKalman();
      resetAciKalman();


      digitalWrite(
        PYRO1_PIN,
        LOW
      );

      digitalWrite(
        BUZZER_PIN,
        LOW
      );
    }
  }


  // ----------------------------------------------------------
  // RESET
  // ----------------------------------------------------------

  else if (
    f[0] == 0xAA &&
    f[1] == 0x24
  ) {

    modu =
      MOD_NORMAL;

    aliciDurumu =
      BEKLE_HEADER;

    resetUcusTespitleri();
    resetIrtifaKalman();
    resetAciKalman();


    digitalWrite(
      PYRO1_PIN,
      LOW
    );

    digitalWrite(
      BUZZER_PIN,
      LOW
    );
  }
}


// ============================================================
// SUT VERİ FRAME
// ============================================================

void islemVeriFrame(
  uint8_t *f
) {

  if (
    f[34] != 0x0D ||
    f[35] != 0x0A
  )
    return;


  if (
    hesaplaChecksum(
      f,
      33
    ) != f[33]
  )
    return;


  sonSentetikVeri.irtifa =
    bytesToFloat(
      &f[1]
    );


  sonSentetikVeri.basinc =
    bytesToFloat(
      &f[5]
    );


  sonSentetikVeri.ivmeX =
    bytesToFloat(
      &f[9]
    );


  sonSentetikVeri.ivmeY =
    bytesToFloat(
      &f[13]
    );


  sonSentetikVeri.ivmeZ =
    bytesToFloat(
      &f[17]
    );


  sonSentetikVeri.aciX =
    normalizeAngle180(
      bytesToFloat(
        &f[21]
      )
    );


  sonSentetikVeri.aciY =
    normalizeAngle180(
      bytesToFloat(
        &f[25]
      )
    );


  sonSentetikVeri.aciZ =
    normalizeAngle180(
      bytesToFloat(
        &f[29]
      )
    );


  sonSentetikVeriZamani =
    millis();


  if (
    modu ==
    MOD_SUT_AKTIF
  ) {

    gyroRateX = 0.0f;
    gyroRateY = 0.0f;
    gyroRateZ = 0.0f;


    ucusAlgoritmasiCalistir(
      sonSentetikVeri
    );


    gonderDurumPaketi(
      durumBitleri
    );
  }
}


// ============================================================
// RS232 SERIAL PARSER
// ============================================================

void serialVeriIsle() {

  while (
    RS232.available()
  ) {

    uint8_t b =
      RS232.read();


    unsigned long simdi =
      millis();


    if (
      aliciDurumu ==
      BEKLE_HEADER
    ) {

      if (
        b == 0xAA
      ) {

        rxBuffer[0] = b;

        rxIndex = 1;

        aliciDurumu =
          KOMUT_OKU;

        rxBaslangic =
          simdi;

      } else if (
        b == 0xAB &&
        modu == MOD_SUT_AKTIF
      ) {

        rxBuffer[0] = b;

        rxIndex = 1;

        aliciDurumu =
          VERI_OKU;

        rxBaslangic =
          simdi;
      }
    }


    else if (
      aliciDurumu ==
      KOMUT_OKU
    ) {

      rxBuffer[rxIndex++] =
        b;


      if (
        rxIndex >= 5
      ) {

        islemKomutFrame(
          rxBuffer
        );


        aliciDurumu =
          BEKLE_HEADER;

        rxIndex = 0;
      }
    }


    else if (
      aliciDurumu ==
      VERI_OKU
    ) {

      rxBuffer[rxIndex++] =
        b;


      if (
        rxIndex >= 36
      ) {

        islemVeriFrame(
          rxBuffer
        );


        aliciDurumu =
          BEKLE_HEADER;

        rxIndex = 0;
      }
    }


    if (
      aliciDurumu !=
      BEKLE_HEADER &&
      simdi - rxBaslangic >
        FRAME_TIMEOUT_MS
    ) {

      aliciDurumu =
        BEKLE_HEADER;

      rxIndex = 0;
    }
  }
}


// ============================================================
// UÇUŞ DURUMUNU KAYDET
// ============================================================

void kaydetUcusDurumu(
  bool zorla = false
) {

  rtc_magic =
    RTC_MAGIC;

  rtc_durumBitleri =
    durumBitleri;

  rtc_zirveIrtifa =
    zirveIrtifa;

  rtc_pyroAteslendi =
    pyroAteslendi ? 1 : 0;

  rtc_modu =
    (uint8_t)modu;

  rtc_kalkisZamani =
    kalkisZamani;

  rtc_apogeeTespitMs =
    apogeeTespitMs;


  if (!nvsReady)
    return;


  static uint16_t lastDurum =
    0xFFFF;

  static uint8_t lastPyro =
    0xFF;

  static uint8_t lastMod =
    0xFF;

  static uint32_t lastKalkis =
    0xFFFFFFFF;

  static uint32_t lastApogee =
    0xFFFFFFFF;


  bool degisti =
    (durumBitleri != lastDurum) ||
    (
      (pyroAteslendi ? 1 : 0)
      != lastPyro
    ) ||
    (
      (uint8_t)modu
      != lastMod
    ) ||
    (
      kalkisZamani
      != lastKalkis
    ) ||
    (
      apogeeTespitMs
      != lastApogee
    );


  unsigned long now =
    millis();


  if (
    !zorla &&
    !degisti &&
    (
      now -
      sonNvsKayitZamani <
      NVS_SAVE_PERIOD_MS
    )
  )
    return;


  nvPrefs.putUInt(
    "magic",
    RTC_MAGIC
  );


  nvPrefs.putUShort(
    "durum",
    durumBitleri
  );


  nvPrefs.putFloat(
    "zirve",
    zirveIrtifa
  );


  nvPrefs.putUChar(
    "pyro",
    pyroAteslendi ? 1 : 0
  );


  nvPrefs.putUChar(
    "mod",
    (uint8_t)modu
  );


  nvPrefs.putULong(
    "kalkis",
    kalkisZamani
  );


  nvPrefs.putULong(
    "apogee",
    apogeeTespitMs
  );


  lastDurum =
    durumBitleri;

  lastPyro =
    pyroAteslendi ? 1 : 0;

  lastMod =
    (uint8_t)modu;

  lastKalkis =
    kalkisZamani;

  lastApogee =
    apogeeTespitMs;

  sonNvsKayitZamani =
    now;
}


// ============================================================
// UÇUŞ DURUMUNU YÜKLE
// ============================================================

void yukleUcusDurumu() {

  bool rtcGecerli =
    (
      rtc_magic ==
      RTC_MAGIC
    );


  if (rtcGecerli) {

    durumBitleri =
      rtc_durumBitleri;

    zirveIrtifa =
      rtc_zirveIrtifa;

    pyroAteslendi =
      (
        rtc_pyroAteslendi
        != 0
      );

    modu =
      (TestModu)rtc_modu;

    kalkisZamani =
      rtc_kalkisZamani;

    apogeeTespitMs =
      rtc_apogeeTespitMs;

  } else if (
    nvsReady
  ) {

    uint32_t magic =
      nvPrefs.getUInt(
        "magic",
        0
      );


    if (
      magic ==
      RTC_MAGIC
    ) {

      durumBitleri =
        nvPrefs.getUShort(
          "durum",
          0
        );


      zirveIrtifa =
        nvPrefs.getFloat(
          "zirve",
          -1000.0f
        );


      pyroAteslendi =
        (
          nvPrefs.getUChar(
            "pyro",
            0
          ) != 0
        );


      modu =
        (TestModu)
        nvPrefs.getUChar(
          "mod",
          0
        );


      kalkisZamani =
        nvPrefs.getULong(
          "kalkis",
          0
        );


      apogeeTespitMs =
        nvPrefs.getULong(
          "apogee",
          0
        );
    }
  }


  if (
    pyroAteslendi ||
    (
      durumBitleri &
      BIT_SPE
    )
  ) {

    digitalWrite(
      PYRO1_PIN,
      HIGH
    );

    digitalWrite(
      BUZZER_PIN,
      HIGH
    );


    pyroAktif = false;

    pyroAteslendi = true;

    durumBitleri |=
      BIT_SPE;
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  // ----------------------------------------------------------
  // WATCHDOG
  // ----------------------------------------------------------

#if defined(ESP_IDF_VERSION_MAJOR) && \
    (ESP_IDF_VERSION_MAJOR >= 5)

  esp_task_wdt_config_t twdt_config = {

    .timeout_ms = 10000,

    .idle_core_mask =
      (1 << portNUM_PROCESSORS) - 1,

    .trigger_panic = true
  };


  esp_task_wdt_init(
    &twdt_config
  );

#else

  esp_task_wdt_init(
    10,
    true
  );

#endif


  esp_task_wdt_add(
    NULL
  );


  // ----------------------------------------------------------
  // NVS
  // ----------------------------------------------------------

  nvsReady =
    nvPrefs.begin(
      "ukb",
      false
    );


  // ----------------------------------------------------------
  // ÇIKIŞ PINLERİ
  // ----------------------------------------------------------

  pinMode(
    PYRO1_PIN,
    OUTPUT
  );

  digitalWrite(
    PYRO1_PIN,
    LOW
  );


  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  pinMode(
    LORA_AUX_PIN,
    INPUT_PULLUP
  );


  pinMode(
    AUX_MIRROR_PIN,
    OUTPUT
  );

  digitalWrite(
    AUX_MIRROR_PIN,
    LOW
  );


  // ----------------------------------------------------------
  // AÇILIŞ BİPİ
  // ----------------------------------------------------------

  for (
    int i = 0;
    i < STARTUP_BEEP_COUNT;
    i++
  ) {

    digitalWrite(
      BUZZER_PIN,
      HIGH
    );


    delay(
      STARTUP_BEEP_ON_MS
    );


    digitalWrite(
      BUZZER_PIN,
      LOW
    );


    delay(
      STARTUP_BEEP_OFF_MS
    );


    esp_task_wdt_reset();
  }


  heartbeatLastMs =
    millis();


  // ----------------------------------------------------------
  // SERİ PORTLAR
  // ----------------------------------------------------------

  GPS.begin(
    GPS_BAUD,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );


  RS232.begin(
    RS232_BAUD,
    SERIAL_8N1,
    RS232_RX_PIN,
    RS232_TX_PIN
  );


  LoRaSerial.begin(
    LORA_BAUD,
    SERIAL_8N1,
    LORA_RX_PIN,
    LORA_TX_PIN
  );


  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin();

  Wire.setClock(
    400000
  );


#if defined(WIRE_HAS_TIMEOUT)

  Wire.setTimeOut(
    I2C_TIMEOUT_MS
  );

#endif


  // ----------------------------------------------------------
  // BMP390
  // ----------------------------------------------------------

  bmpHazir =
    bmp.begin_I2C(
      BMP390_I2C_ADDR,
      &Wire
    );


  if (bmpHazir) {

    bmpAktifAdres =
      BMP390_I2C_ADDR;

  } else {

    uint8_t alternatifAdres =
      (
        BMP390_I2C_ADDR ==
        0x77
      )
      ? 0x76
      : 0x77;


    bmpHazir =
      bmp.begin_I2C(
        alternatifAdres,
        &Wire
      );


    if (bmpHazir)
      bmpAktifAdres =
        alternatifAdres;
  }


  if (bmpHazir) {

    bmp.setTemperatureOversampling(
      BMP3_OVERSAMPLING_2X
    );


    bmp.setPressureOversampling(
      BMP3_OVERSAMPLING_8X
    );


    bmp.setIIRFilterCoeff(
      BMP3_IIR_FILTER_COEFF_1
    );


    bmp.setOutputDataRate(
      BMP3_ODR_50_HZ
    );
  }


  // ----------------------------------------------------------
  // BNO055
  // ----------------------------------------------------------

  bnoHazir =
    bno.begin();


  if (bnoHazir) {

    delay(700);

    bno.setExtCrystalUse(
      true
    );

    delay(100);

    aciReferansiHazir =
      false;
  }


  delay(500);


  // ----------------------------------------------------------
  // BASINÇ REFERANS KALİBRASYONU
  // ----------------------------------------------------------

  float basincToplam =
    0.0f;

  int basincGecerliOrnek =
    0;


  if (bmpHazir) {

    int isinmaOrnegi =
      0;


    while (
      isinmaOrnegi < 20
    ) {

      if (
        bmp.performReading()
      ) {

        isinmaOrnegi++;
      }


      delay(25);

      esp_task_wdt_reset();
    }


    while (
      basincGecerliOrnek < 40
    ) {

      if (
        bmp.performReading()
      ) {

        float basincHpa =
          bmp.pressure /
          100.0f;


        if (
          basincHpa > 100.0f &&
          basincHpa < 1200.0f
        ) {

          basincToplam +=
            basincHpa;

          basincGecerliOrnek++;
        }
      }


      delay(25);

      esp_task_wdt_reset();
    }
  }


  if (
    basincGecerliOrnek > 0
  ) {

    rampaBasincHpa =
      basincToplam /
      (float)basincGecerliOrnek;

  } else {

    rampaBasincHpa =
      0.0f;
  }


  rampaIrtifasi = 0.0f;

  rampaReferansiHazir =
    false;


  // ----------------------------------------------------------
  // UÇUŞ DURUMLARI
  // ----------------------------------------------------------

  durumBitleri = 0;

  zirveIrtifa =
    -1000.0f;

  pyroAteslendi =
    false;

  pyroAktif =
    false;

  kalkisZamani =
    0;

  apogeeTespitMs =
    0;


  resetUcusTespitleri();
  resetIrtifaKalman();
  resetAciKalman();


  kaydetUcusDurumu(
    true
  );


  // ----------------------------------------------------------
  // ZAMANLAYICI SIFIRLAMA
  // ----------------------------------------------------------

  unsigned long simdi = millis();

  sonRS232TelemetriZamani = simdi;
  sonDurumKayitZamani     = simdi;
  sonLoRaTelemetriZamani  = simdi;


  // ----------------------------------------------------------
  // LORA GECİKME BAŞLANGICI
  // ----------------------------------------------------------
  //
  // loraBasladi = false olduğu sürece loop()
  // gonderLoRaPaketi() çağırmaz.
  //
  // loraBaslangicZamani'nden itibaren
  // LORA_BASLANGIC_GECIKMESI_MS geçince
  // loraBasladi = true yapılır ve gönderim başlar.
  //
  // NOT: millis() + offset KULLANILMAZ çünkü
  // unsigned long taşması yanlış sonuç verir.
  // ----------------------------------------------------------

  loraBasladi = false;
  loraBaslangicZamani = simdi;
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // WATCHDOG
  // ----------------------------------------------------------

  esp_task_wdt_reset();


  // ----------------------------------------------------------
  // RS232 GELEN VERİ
  // ----------------------------------------------------------

  serialVeriIsle();


  // ----------------------------------------------------------
  // LORA AUX LED
  // ----------------------------------------------------------

  bool auxReady =
    (
      digitalRead(
        LORA_AUX_PIN
      ) == HIGH
    );


  digitalWrite(
    AUX_MIRROR_PIN,
    auxReady ? LOW : HIGH
  );


  // ----------------------------------------------------------
  // ZAMAN
  // ----------------------------------------------------------

  unsigned long simdi =
    millis();


  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  gpsUARTUpdate();


  // ----------------------------------------------------------
  // PYRO + HEARTBEAT
  // ----------------------------------------------------------

  if (pyroAktif) {

    if (
      simdi -
      pyroBaslangicZamani >=
      4000
    ) {

      digitalWrite(
        PYRO1_PIN,
        HIGH
      );

      pyroAktif =
        false;

    } else if (
      simdi -
      pyroBaslangicZamani >=
      3000
    ) { 

      digitalWrite(
        BUZZER_PIN,
        LOW
      );

      heartbeatActive =
        false;
    }

  } else {

    if (
      !heartbeatActive &&
      (
        simdi -
        heartbeatLastMs >=
        HEARTBEAT_INTERVAL_MS
      )
    ) {

      digitalWrite(
        BUZZER_PIN,
        HIGH
      );

      heartbeatActive =
        true;

      heartbeatLastMs =
        simdi;

    } else if (
      heartbeatActive &&
      (
        simdi -
        heartbeatLastMs >=
        HEARTBEAT_DURATION_MS
      )
    ) {

      digitalWrite(
        BUZZER_PIN,
        LOW
      );

      heartbeatActive =
        false;

      heartbeatLastMs =
        simdi;
    }
  }


  // ----------------------------------------------------------
  // MOD GEÇİŞİ - SIT
  // ----------------------------------------------------------

  if (
    modu ==
    MOD_SIT_BEKLEME &&
    simdi -
    modBeklemeBaslangic >=
    MODE_ACTIVATE_DELAY_MS
  ) {

    modu =
      MOD_SIT_AKTIF;

    resetAciKalman();
  }


  // ----------------------------------------------------------
  // MOD GEÇİŞİ - SUT
  // ----------------------------------------------------------

  if (
    modu ==
    MOD_SUT_BEKLEME &&
    simdi -
    modBeklemeBaslangic >=
    MODE_ACTIVATE_DELAY_MS
  ) {

    modu =
      MOD_SUT_AKTIF;


    sonSentetikVeriZamani =
      simdi;



    resetUcusTespitleri();
    resetIrtifaKalman();
    resetAciKalman();


    digitalWrite(
      PYRO1_PIN,
      LOW
    );

    digitalWrite(
      BUZZER_PIN,
      LOW
    );
  }


  // ----------------------------------------------------------
  // SUT TIMEOUT
  // ----------------------------------------------------------

  if (
    modu ==
    MOD_SUT_AKTIF &&
    simdi -
    sonSentetikVeriZamani >
    SUT_TIMEOUT_MS
  ) {

    modu =
      MOD_NORMAL;


    resetUcusTespitleri();
    resetIrtifaKalman();
    resetAciKalman();


    digitalWrite(
      PYRO1_PIN,
      LOW
    );

    digitalWrite(
      BUZZER_PIN,
      LOW
    );
  }


  // ----------------------------------------------------------
  // RS232 SIT TELEMETRİSİ — 10 Hz
  // ----------------------------------------------------------

  if (
    simdi -
    sonRS232TelemetriZamani >=
    RS232_TELEMETRY_PERIOD_MS
  ) {

    sonRS232TelemetriZamani =
      simdi;


    UcusVerisi raw =
      sensorleriOku();


    if (isnan(raw.basinc))
      raw.basinc = 0.0f;

    if (isnan(raw.irtifa))
      raw.irtifa = 0.0f;


    UcusVerisi v = raw;


    // Gerçek uçuş algoritması sadece normal modda
    // bu zamanlayıcıdan çalışır. SUT'ta algoritma
    // islemVeriFrame() içinden sentetik veriyle çalışır.
    if (modu != MOD_SUT_AKTIF) {

      ucusAlgoritmasiCalistir(v);
    }


    // SIT modunda sensör verisini gönder
    if (modu == MOD_SIT_AKTIF) {

      gonderSitPaketi(v);
    }
  }


  // ----------------------------------------------------------
  // LORA TELEMETRİSİ — 11 Hz
  //
  // loraBasladi false iken sadece gecikme sayılır,
  // gönderim yapılmaz.
  //
  // loraBasladi true olduktan sonra normal
  // periyodik gönderim çalışır.
  // ----------------------------------------------------------

  if (!loraBasladi) {

    if (
      simdi - loraBaslangicZamani >=
      LORA_BASLANGIC_GECIKMESI_MS
    ) {

      loraBasladi = true;

      // İlk gönderimi hemen tetikle
      sonLoRaTelemetriZamani = simdi;
    }

  } else if (
    simdi -
    sonLoRaTelemetriZamani >=
    LORA_TELEMETRY_PERIOD_MS
  ) {

    sonLoRaTelemetriZamani =
      simdi;


    // LoRa her zaman gerçek sensör verisini gönderir.
    // SUT sırasında zemin istasyonu gerçek sensör verisini görür.
    UcusVerisi loraVeri =
      sensorleriOku();

    if (isnan(loraVeri.basinc))
      loraVeri.basinc = 0.0f;

    if (isnan(loraVeri.irtifa))
      loraVeri.irtifa = 0.0f;


    gonderLoRaPaketi(
      loraVeri,
      kalman.altitude(),
      filtreliHiz
    );
  }


  // ----------------------------------------------------------
  // DURUM KAYDI
  // ----------------------------------------------------------

  if (
    simdi -
    sonDurumKayitZamani >=
    STATE_SAVE_PERIOD_MS
  ) {

    sonDurumKayitZamani =
      simdi;


    kaydetUcusDurumu();
  }
}

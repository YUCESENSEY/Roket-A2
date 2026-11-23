#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

////////////////////////////////////
// ÜLGEN UKB - SENSÖR TESTİ (BMP390 + BNO055)
// Hem HAM hem FİLTRELİ (UKB-Kod.ino'daki filtrelerle aynı) değerleri yazar.
// Serial Monitor: 115200 baud, "Yeni satır" seçili.
// Komutlar:
//   M -> Okunabilir mod / Serial Plotter modu arasında geçiş
//   Z -> Yer irtifasını yeniden ölç (kartı sabit tut)
////////////////////////////////////

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(55, 0x28, &Wire);
bool bmpHazir = false;
bool bnoHazir = false;

////////////////////////////////////
// Sensör ham verileri
float basinc = 0, sicaklik = 0, hamIrtifa = 0;
float yerIrtifa = 0;
float accX = 0, accY = 0, accZ = 0;       // yerçekimli ivme
float linX = 0, linY = 0, linZ = 0;       // lineer ivme (yerçekimsiz)
float gyroX = 0, gyroY = 0, gyroZ = 0;    // jiroskop (derece/sn kabul edilir)
float eulerH = 0, eulerR = 0, eulerP = 0; // BNO'nun kendi füzyonu (referans)
uint8_t kalSis = 0, kalJiro = 0, kalIvme = 0, kalMag = 0;

////////////////////////////////////
// Zamanlayıcılar
unsigned long simdi          = 0;
unsigned long sensorzamani   = 0;
unsigned long sonYazdirma    = 0;
unsigned long sonZamanFiltre = 0;
bool plotterMod = false;

////////////////////////////////////
float filtrePitch = 0.0f;
float filtreRoll  = 0.0f;

float aciyiSinirla(float aci) {
  while (aci >  180.0f) aci -= 360.0f;
  while (aci < -180.0f) aci += 360.0f;
  return aci;
}

float basincIrtifa(float hPa) {
  return 44330.0f * (1.0f - pow((hPa / 1013.25f), 0.19029f));
}

////////////////////////////////////
// KALMAN FİLTRESİ (UKB-Kod.ino ile aynı)
////////////////////////////////////
float kalmanIrtifa = 0.0f;
float kalmanHiz    = 0.0f;
float P_00 = 1.0f, P_01 = 0.0f;
float P_10 = 0.0f, P_11 = 1.0f;
unsigned long sonKalmanZamani = 0;
bool kalmanBasladi = false;

const float Q_pozisyon = 0.001f;
const float Q_hiz      = 0.003f;
const float R_baro     = 2.0f;

void kalmanFiltresiGuncelle(float baroIrtifa, float dikeyIvme) {
  // Test için ilk ölçümle başlat (UKB'de 0'dan başlıyor, bkz. açıklama)
  if (!kalmanBasladi) {
    kalmanIrtifa = baroIrtifa;
    kalmanHiz    = 0.0f;
    kalmanBasladi = true;
  }

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
// TAMAMLAYICI FİLTRE (UKB-Kod.ino ile aynı)
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
// YER İRTİFASI ÖLÇ (50 örnek ortalaması)
////////////////////////////////////
void yerIrtifaOlc() {
  Serial.println(F("Yer irtifası ölçülüyor (kartı sabit tut)..."));
  float toplam = 0;
  int adet = 0;
  for (int i = 0; i < 50; i++) {
    if (bmp.performReading()) {
      toplam += basincIrtifa(bmp.pressure / 100.0f);
      adet++;
    }
    delay(20);
  }
  if (adet > 0) {
    yerIrtifa = toplam / adet;
    Serial.printf("Yer irtifası: %.2f m (mutlak)\n", yerIrtifa);
  } else {
    Serial.println(F("HATA: BMP390 okunamadı!"));
  }
}

////////////////////////////////////
// SENSÖR OKU
////////////////////////////////////
void sensorOku() {
  if (bnoHazir) {
    imu::Vector<3> acc   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> lin   = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    accX = acc.x();  accY = acc.y();  accZ = acc.z();
    linX = lin.x();  linY = lin.y();  linZ = lin.z();
    gyroX = gyro.x(); gyroY = gyro.y(); gyroZ = gyro.z();
    eulerH = euler.x(); eulerR = euler.y(); eulerP = euler.z();

    tamamlayiciFiltreUygula(accX, accY, accZ, gyroX, gyroY, gyroZ);
    bno.getCalibration(&kalSis, &kalJiro, &kalIvme, &kalMag);
  }

  if (bmpHazir && bmp.performReading()) {
    basinc    = bmp.pressure / 100.0f;
    sicaklik  = bmp.temperature;
    hamIrtifa = basincIrtifa(basinc);
    kalmanFiltresiGuncelle(hamIrtifa, linZ);   // UKB gibi lineer Z ivmesi kullanılıyor
  }
}

////////////////////////////////////
// YAZDIRMA
////////////////////////////////////
void okunabilirYazdir() {
  Serial.printf("\n=========== t = %.2f sn ===========\n", simdi / 1000.0f);

  Serial.println(F("--- BMP390 HAM ---"));
  Serial.printf("  Basınç: %.2f hPa | Sıcaklık: %.2f C\n", basinc, sicaklik);
  Serial.printf("  İrtifa (mutlak): %.2f m | İrtifa (yere göre): %.2f m\n", hamIrtifa, hamIrtifa - yerIrtifa);

  Serial.println(F("--- BMP390 FİLTRELİ (Kalman) ---"));
  Serial.printf("  İrtifa (yere göre): %.2f m | Hız: %.3f m/s\n", kalmanIrtifa - yerIrtifa, kalmanHiz);

  Serial.println(F("--- BNO055 HAM ---"));
  Serial.printf("  İvme (yerçekimli): %.2f %.2f %.2f m/s²\n", accX, accY, accZ);
  Serial.printf("  Lineer ivme      : %.2f %.2f %.2f m/s²\n", linX, linY, linZ);
  Serial.printf("  Jiroskop         : %.2f %.2f %.2f\n", gyroX, gyroY, gyroZ);
  Serial.printf("  Euler (BNO füzyon): Heading %.1f | Roll %.1f | Pitch %.1f\n", eulerH, eulerR, eulerP);

  Serial.println(F("--- BNO055 FİLTRELİ (Tamamlayıcı) ---"));
  Serial.printf("  Roll: %.2f | Pitch: %.2f derece\n", filtreRoll, filtrePitch);

  Serial.printf("Kalibrasyon (0-3): Sistem=%d Jiro=%d İvme=%d Mag=%d\n", kalSis, kalJiro, kalIvme, kalMag);
}

void plotterYazdir() {
  // Arduino IDE -> Araçlar -> Serial Plotter ile çizdirilebilir
  Serial.printf("HamIrtifa:%.2f,KalmanIrtifa:%.2f,KalmanHiz:%.3f,LinZ:%.2f,Roll:%.2f,Pitch:%.2f,BnoRoll:%.2f,BnoPitch:%.2f\n",
                hamIrtifa - yerIrtifa, kalmanIrtifa - yerIrtifa, kalmanHiz, linZ,
                filtreRoll, filtrePitch, eulerR, eulerP);
}

////////////////////////////////////
void komutIsle() {
  while (Serial.available()) {
    char k = toupper(Serial.read());
    if (k == 'M') {
      plotterMod = !plotterMod;
      Serial.println(plotterMod ? F("[Serial Plotter modu]") : F("[Okunabilir mod]"));
    } else if (k == 'Z') {
      if (bmpHazir) yerIrtifaOlc();
    }
  }
}

////////////////////////////////////
void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  Wire.begin();

  bmpHazir = bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76);
  if (bmpHazir) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println(F("BMP390: OK"));
  } else {
    Serial.println(F("BMP390: HATA (bulunamadı)"));
  }

  if (bno.begin()) {
    bnoHazir = true;
    bno.setExtCrystalUse(true);
    Serial.println(F("BNO055: OK"));
  } else {
    Serial.println(F("BNO055: HATA (bulunamadı)"));
  }

  if (bmpHazir) yerIrtifaOlc();
  Serial.println(F("Komutlar: M = plotter/okunabilir, Z = yer irtifasını yeniden ölç"));
}

////////////////////////////////////
void loop() {
  simdi = millis();
  komutIsle();

  if (simdi - sensorzamani >= 25) {
    sensorzamani = simdi;
    sensorOku();

    if (plotterMod) {
      if (simdi - sonYazdirma >= 50) { sonYazdirma = simdi; plotterYazdir(); }
    } else {
      if (simdi - sonYazdirma >= 500) { sonYazdirma = simdi; okunabilirYazdir(); }
    }
  }
}

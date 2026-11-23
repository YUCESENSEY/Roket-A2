#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

////////////////////////////////////
// ÜLGEN UKB - İRTİFA FİLTRESİ KARŞILAŞTIRMA
// BMP390'dan gelen ham irtifayı 4 farklı filtreden geçirip yan yana çizer.
//   1) Hareketli ortalama   (basit, gecikmeli)
//   2) Medyan filtresi      (ani sıçramaları / spike'ları temizler)
//   3) Üssel ortalama (EMA) (çok hafif, tek parametre)
//   4) Kalman               (şu an UKB'de kullanılan; baro + ivme birleştirir)
// Çıktı: Arduino IDE -> Araçlar -> Serial Plotter (115200 baud)
// Komutlar:
//   T -> Gürültü (standart sapma) tablosunu yazdır
//   S -> İstatistikleri sıfırla (kartı sabit tutup tekrar ölç)
//   Z -> Yer irtifasını yeniden ölç
////////////////////////////////////

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno(55, 0x28, &Wire);
bool bmpHazir = false;
bool bnoHazir = false;

////////////////////////////////////
float yerIrtifa = 0;
float linZ      = 0;      // BNO lineer ivme Z (Kalman için; BNO yoksa 0)

unsigned long simdi        = 0;
unsigned long sensorzamani = 0;
unsigned long sonYazdirma  = 0;

float basincIrtifa(float hPa) {
  return 44330.0f * (1.0f - pow((hPa / 1013.25f), 0.19029f));
}

////////////////////////////////////
// 1) HAREKETLİ ORTALAMA
// N büyüdükçe daha pürüzsüz ama daha gecikmeli olur.
////////////////////////////////////
#define HO_N 10
struct HareketliOrtalama {
  float tampon[HO_N];
  int   idx   = 0;
  int   adet  = 0;
  float toplam = 0;

  float guncelle(float x) {
    if (adet < HO_N) adet++;
    else toplam -= tampon[idx];
    tampon[idx] = x;
    toplam += x;
    idx = (idx + 1) % HO_N;
    return toplam / adet;
  }
};

////////////////////////////////////
// 2) MEDYAN FİLTRESİ
// Tek seferlik hatalı ölçümleri (spike) tamamen yok eder.
////////////////////////////////////
#define MED_N 5
struct MedyanFiltre {
  float tampon[MED_N];
  int   idx  = 0;
  int   adet = 0;

  float guncelle(float x) {
    tampon[idx] = x;
    idx = (idx + 1) % MED_N;
    if (adet < MED_N) adet++;

    float s[MED_N];
    for (int i = 0; i < adet; i++) s[i] = tampon[i];
    for (int i = 1; i < adet; i++) {           // basit sıralama
      float anahtar = s[i];
      int j = i - 1;
      while (j >= 0 && s[j] > anahtar) { s[j + 1] = s[j]; j--; }
      s[j + 1] = anahtar;
    }
    return s[adet / 2];
  }
};

////////////////////////////////////
// 3) ÜSSEL ORTALAMA (EMA)
// ALFA küçüldükçe daha yumuşak ama daha gecikmeli. (0.0 - 1.0)
////////////////////////////////////
const float USSEL_ALFA = 0.2f;
struct UsselFiltre {
  float deger = 0;
  bool  hazir = false;

  float guncelle(float x) {
    if (!hazir) { deger = x; hazir = true; }
    else deger = USSEL_ALFA * x + (1.0f - USSEL_ALFA) * deger;
    return deger;
  }
};

////////////////////////////////////
// 4) KALMAN (UKB-Kod.ino ile aynı)
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
  if (!kalmanBasladi) { kalmanIrtifa = baroIrtifa; kalmanHiz = 0; kalmanBasladi = true; }

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
// İSTATİSTİK (gürültü ölçümü - Welford yöntemi)
////////////////////////////////////
struct Istatistik {
  uint32_t n = 0;
  double ort = 0, m2 = 0;
  void ekle(float x) { n++; double d = x - ort; ort += d / n; m2 += d * (x - ort); }
  float std() { return n > 1 ? (float)sqrt(m2 / (n - 1)) : 0.0f; }
  void sifirla() { n = 0; ort = 0; m2 = 0; }
};

HareketliOrtalama ho;
MedyanFiltre      med;
UsselFiltre       ussel;
Istatistik        istHam, istHo, istMed, istUssel, istKalman;

////////////////////////////////////
void yerIrtifaOlc() {
  Serial.println(F("Yer irtifası ölçülüyor (kartı sabit tut)..."));
  float toplam = 0;
  int adet = 0;
  for (int i = 0; i < 50; i++) {
    if (bmp.performReading()) { toplam += basincIrtifa(bmp.pressure / 100.0f); adet++; }
    delay(20);
  }
  if (adet > 0) {
    yerIrtifa = toplam / adet;
    Serial.printf("Yer irtifası: %.2f m\n", yerIrtifa);
  }
}

void istatistikYazdir() {
  Serial.println(F("\n===== GÜRÜLTÜ (standart sapma, m) - kart SABİTKEN anlamlı ====="));
  Serial.printf("  Ham            : %.3f\n", istHam.std());
  Serial.printf("  Hareketli ort. : %.3f\n", istHo.std());
  Serial.printf("  Medyan         : %.3f\n", istMed.std());
  Serial.printf("  Üssel (EMA)    : %.3f\n", istUssel.std());
  Serial.printf("  Kalman         : %.3f\n", istKalman.std());
  Serial.printf("  Örnek sayısı   : %lu\n", (unsigned long)istHam.n);
  Serial.println(F("================================================================\n"));
}

void komutIsle() {
  while (Serial.available()) {
    char k = toupper(Serial.read());
    if (k == 'T') istatistikYazdir();
    else if (k == 'S') {
      istHam.sifirla(); istHo.sifirla(); istMed.sifirla(); istUssel.sifirla(); istKalman.sifirla();
      Serial.println(F("[İstatistikler sıfırlandı]"));
    } else if (k == 'Z') yerIrtifaOlc();
  }
}

////////////////////////////////////
void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  Wire.begin();
  bmpHazir = bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76);
  if (!bmpHazir) { Serial.println(F("HATA: BMP390 bulunamadı!")); while (1) delay(1000); }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  if (bno.begin()) { bnoHazir = true; bno.setExtCrystalUse(true); }
  else Serial.println(F("UYARI: BNO055 yok, Kalman ivme girişi 0 kullanılacak."));

  yerIrtifaOlc();
  Serial.println(F("Komutlar: T = gürültü tablosu, S = istatistik sıfırla, Z = yer irtifası"));
}

////////////////////////////////////
void loop() {
  simdi = millis();
  komutIsle();

  if (simdi - sensorzamani >= 25) {
    sensorzamani = simdi;

    if (bnoHazir) {
      imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      linZ = lin.z();
    }

    if (bmp.performReading()) {
      float ham = basincIrtifa(bmp.pressure / 100.0f) - yerIrtifa;

      float fHo     = ho.guncelle(ham);
      float fMed    = med.guncelle(ham);
      float fUssel  = ussel.guncelle(ham);
      kalmanFiltresiGuncelle(ham + yerIrtifa, linZ);
      float fKalman = kalmanIrtifa - yerIrtifa;

      istHam.ekle(ham); istHo.ekle(fHo); istMed.ekle(fMed);
      istUssel.ekle(fUssel); istKalman.ekle(fKalman);

      if (simdi - sonYazdirma >= 50) {
        sonYazdirma = simdi;
        Serial.printf("Ham:%.2f,HareketliOrt:%.2f,Medyan:%.2f,Ussel:%.2f,Kalman:%.2f\n",
                      ham, fHo, fMed, fUssel, fKalman);
      }
    }
  }
}

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

////////////////////////////////////
// ÜLGEN UKB - AÇI (YÖNELİM) FİLTRESİ KARŞILAŞTIRMA
// BNO055'in ham ivme + jiroskop verisinden roll/pitch hesaplayan filtreleri
// yan yana çizer:
//   BNO      -> BNO055'in kendi iç füzyonu (REFERANS; işaret farkı olabilir)
//   Ivme     -> sadece ivmeölçer (gürültülü, ivmelenirken yanlış)
//   Jiro     -> sadece jiroskop integrali (zamanla kayar / drift)
//   Tamamlayici -> şu an UKB'de kullanılan filtre (alpha = 0.98)
//   Madgwick -> kullanmadığımız alternatif (6 eksen, kuvaterniyon tabanlı)
// Çıktı: Serial Plotter (115200 baud)
// Komutlar:
//   E -> Çizilecek ekseni değiştir (Roll / Pitch)
//   S -> Filtreleri sıfırla
// NOT: Jiroskop değerlerinin derece/sn olduğu varsayılır. Kartı döndürürken
//      Jiro çizgisi gerçek açıdan çok farklı büyüyorsa birim (rad/sn) sorunu vardır.
////////////////////////////////////

Adafruit_BNO055 bno(55, 0x28, &Wire);

unsigned long simdi         = 0;
unsigned long sensorzamani  = 0;
unsigned long sonYazdirma   = 0;
unsigned long sonZamanFiltre = 0;
bool pitchCiz = false;

float aciyiSinirla(float aci) {
  while (aci >  180.0f) aci -= 360.0f;
  while (aci < -180.0f) aci += 360.0f;
  return aci;
}

////////////////////////////////////
// TAMAMLAYICI FİLTRE (UKB-Kod.ino ile aynı)
////////////////////////////////////
const float ALPHA = 0.98f;
float tamRoll = 0, tamPitch = 0;

void tamamlayiciGuncelle(float ax, float ay, float az, float gx, float gy, float dt) {
  float rollAcc  = atan2f(ay, az)  * 180.0f / M_PI;
  float pitchAcc = atan2f(-ax, az) * 180.0f / M_PI;

  float rollGyro  = tamRoll  + (gx * dt);
  float pitchGyro = tamPitch + (gy * dt);

  tamRoll  = aciyiSinirla(ALPHA * rollGyro  + (1.0f - ALPHA) * rollAcc);
  tamPitch = aciyiSinirla(ALPHA * pitchGyro + (1.0f - ALPHA) * pitchAcc);
}

////////////////////////////////////
// MADGWICK FİLTRESİ (6 eksen: jiro + ivme)
// BETA büyürse ivmeölçere daha çok güvenir (hızlı toparlar, gürültülü).
// BETA küçülürse jiroya daha çok güvenir (pürüzsüz, yavaş toparlar).
////////////////////////////////////
const float MADGWICK_BETA = 0.1f;
struct Madgwick {
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  float roll = 0, pitch = 0;

  void sifirla() { q0 = 1; q1 = q2 = q3 = 0; roll = pitch = 0; }

  // gx, gy, gz: derece/sn   ax, ay, az: m/s² (birim önemli değil, normalize edilir)
  void guncelle(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    gx *= M_PI / 180.0f; gy *= M_PI / 180.0f; gz *= M_PI / 180.0f;

    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    float norm = ax * ax + ay * ay + az * az;
    if (norm > 0.0f) {
      norm = 1.0f / sqrtf(norm);
      ax *= norm; ay *= norm; az *= norm;

      float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
      float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
      float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
      float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

      float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

      norm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;

      qDot1 -= MADGWICK_BETA * s0;
      qDot2 -= MADGWICK_BETA * s1;
      qDot3 -= MADGWICK_BETA * s2;
      qDot4 -= MADGWICK_BETA * s3;
    }

    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
    norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;

    roll  = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 180.0f / M_PI;
    pitch = asinf(constrain(-2.0f * (q1 * q3 - q0 * q2), -1.0f, 1.0f)) * 180.0f / M_PI;
  }
};

Madgwick madgwick;

////////////////////////////////////
float jiroRoll = 0, jiroPitch = 0;

void filtreleriSifirla() {
  tamRoll = tamPitch = 0;
  jiroRoll = jiroPitch = 0;
  madgwick.sifirla();
  Serial.println(F("[Filtreler sıfırlandı]"));
}

void komutIsle() {
  while (Serial.available()) {
    char k = toupper(Serial.read());
    if (k == 'E') {
      pitchCiz = !pitchCiz;
      Serial.println(pitchCiz ? F("[PITCH çiziliyor]") : F("[ROLL çiziliyor]"));
    } else if (k == 'S') filtreleriSifirla();
  }
}

////////////////////////////////////
void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  Wire.begin();
  if (!bno.begin()) {
    Serial.println(F("HATA: BNO055 bulunamadı!"));
    while (1) delay(1000);
  }
  bno.setExtCrystalUse(true);

  // Başlangıç açısını ivmeölçerden al ki filtreler doğru yerden başlasın
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  tamRoll  = jiroRoll  = atan2f(acc.y(), acc.z())  * 180.0f / M_PI;
  tamPitch = jiroPitch = atan2f(-acc.x(), acc.z()) * 180.0f / M_PI;

  sonZamanFiltre = micros();
  Serial.println(F("Komutlar: E = roll/pitch değiştir, S = sıfırla"));
}

////////////////////////////////////
void loop() {
  simdi = millis();
  komutIsle();

  if (simdi - sensorzamani >= 10) {   // 100 Hz
    sensorzamani = simdi;

    unsigned long simdiMikro = micros();
    float dt = (simdiMikro - sonZamanFiltre) / 1000000.0f;
    sonZamanFiltre = simdiMikro;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

    imu::Vector<3> acc   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    float ax = acc.x(),  ay = acc.y(),  az = acc.z();
    float gx = gyro.x(), gy = gyro.y(), gz = gyro.z();

    float ivmeRoll  = atan2f(ay, az)  * 180.0f / M_PI;
    float ivmePitch = atan2f(-ax, az) * 180.0f / M_PI;

    jiroRoll  = aciyiSinirla(jiroRoll  + gx * dt);
    jiroPitch = aciyiSinirla(jiroPitch + gy * dt);

    tamamlayiciGuncelle(ax, ay, az, gx, gy, dt);
    madgwick.guncelle(gx, gy, gz, ax, ay, az, dt);

    if (simdi - sonYazdirma >= 30) {
      sonYazdirma = simdi;
      if (!pitchCiz) {
        Serial.printf("BNO:%.2f,Ivme:%.2f,Jiro:%.2f,Tamamlayici:%.2f,Madgwick:%.2f\n",
                      euler.y(), ivmeRoll, jiroRoll, tamRoll, madgwick.roll);
      } else {
        Serial.printf("BNO:%.2f,Ivme:%.2f,Jiro:%.2f,Tamamlayici:%.2f,Madgwick:%.2f\n",
                      euler.z(), ivmePitch, jiroPitch, tamPitch, madgwick.pitch);
      }
    }
  }
}

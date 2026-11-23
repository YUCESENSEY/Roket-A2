////////////////////////////////////
// ÜLGEN UKB - UÇUŞ ALGORİTMASI MASA SİMÜLASYONU
// Sensör, RS232 veya ikinci kart GEREKMEZ. Sanal bir roket uçuşu üretir,
// UKB-Kod.ino'daki algoritmayı (Kalman dahil, SUT modundaki gibi) üzerinde çalıştırır
// ve hangi olay hangi irtifada tetiklendi Serial'e yazar.
// Piro MOSFET'leri çalıştırılmaz; sadece buzzer öter ve Serial'e yazılır.
//
// Serial Monitor: 115200 baud, "Yeni satır" seçili.
// Komutlar:
//   B -> Simülasyonu başlat
//   D -> Simülasyonu durdur
//   H -> Hız: 1x / 10x (10x ile uçuş çok daha çabuk biter)
//   G -> İvme türü: lineer (yerçekimsiz) / ivmeölçer (yerçekimli)
//   K -> Kalman başlangıcı: UKB gibi 0'dan / ilk ölçümden
////////////////////////////////////

#define buzzer D2

////////////////////////////////////
// Eşikler (UKB-Kod.ino ile aynı tutun)
float KALKIS_ESIGI  = 5;     // m/s²
float YANMA_ESIGI   = 15;    // m/s²
float GOVDE_ACISI   = 60;    // derece
float MIN_IRTIFA    = 2400;  // metre
float APOGEE_MARGIN = 2.5;   // metre
float MAIN_IRTIFA   = 500;   // metre

////////////////////////////////////
// Simülasyon ayarları (kendi roketinize göre değiştirin)
const float G                = 9.81f;
const float YER_RAKIM        = 850.0f;   // m, deniz seviyesinden
const float RAMPA_SURESI     = 3.0f;     // s, rampada bekleme
const float YANMA_SURESI     = 3.5f;     // s, motor yanma süresi
const float MOTOR_IVME       = 85.0f;    // m/s², motorun verdiği ivme
const float SURTUNME_KAT     = 0.0002f;  // havanın sürtünme katsayısı
const float DROGUE_INIS_HIZI = 35.0f;    // m/s
const float MAIN_INIS_HIZI   = 8.0f;     // m/s
const float PARASUT_ZAMAN_SABITI = 1.0f; // s, paraşüt açılma yumuşaklığı
const float DT               = 0.025f;   // s, UKB sensör periyodu (25 ms)

bool GRAVITELI_IVME     = false;  // false: UKB NORMAL moddaki gibi lineer ivme
bool KALMAN_ILK_OLCUMLE = false;  // false: UKB gibi Kalman 0'dan başlar

////////////////////////////////////
// Uçuş durum bayrakları (UKB-Kod.ino ile aynı)
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

bool  yerIrtifaHazir  = false;
int   yerIrtifaSayac  = 0;
float yerIrtifaToplam = 0.0f;
#define YER_IRTIFA_ORNEK 3

struct SensorVerisi {
  float irtifa;
  float ivmeX, ivmeY, ivmeZ;
  float aciX, aciY, aciZ;
};

////////////////////////////////////
unsigned long simdi     = 0;
unsigned long sonbuzzer = 0;
unsigned long sonAdim   = 0;
bool buzzerA = false;

////////////////////////////////////
// KALMAN (UKB-Kod.ino ile aynı, sadece dt dışarıdan verilir)
float kalmanIrtifa = 0.0f;
float kalmanHiz    = 0.0f;
float P_00 = 1.0f, P_01 = 0.0f;
float P_10 = 0.0f, P_11 = 1.0f;
const float Q_pozisyon = 0.001f;
const float Q_hiz      = 0.003f;
const float R_baro     = 2.0f;

void kalmanFiltresiGuncelle(float baroIrtifa, float dikeyIvme, float dt) {
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

float sapmaHesapla(float aciX, float aciY) {
  float r = aciX * M_PI / 180.0f;
  float p = aciY * M_PI / 180.0f;
  float c = cosf(r) * cosf(p);
  c = constrain(c, -1.0f, 1.0f);
  return acosf(c) * 180.0f / M_PI;
}

////////////////////////////////////
// Piro tetiklemesi simülasyonu (gerçek MOSFET yok)
void piroTetikle(const char* ad) {
  Serial.printf(">>> %s PİRO TETİKLENDİ (simülasyon)\n", ad);
  digitalWrite(buzzer, HIGH);
  sonbuzzer = simdi;
  buzzerA   = true;
}

////////////////////////////////////
// UÇUŞ ALGORİTMASI (UKB-Kod.ino ile aynı mantık)
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
    piroTetikle("APOGEE");
  }

  if (kalkti && yandi && govdeAcisi && irtifaOk && apogee && ApogeePyroOk
      && !Main && (v.irtifa - yerIrtifa) <= MAIN_IRTIFA) {
    Main = true;
  }

  if (kalkti && yandi && govdeAcisi && irtifaOk && apogee && ApogeePyroOk
      && Main && !MainPyroOk) {
    MainPyroOk = true;
    piroTetikle("MAIN");
  }
}

////////////////////////////////////
// SANAL ROKET (fizik modeli)
////////////////////////////////////
bool  simAktif = false;
int   faz = 0;                 // 0 rampa, 1 yanma, 2 süzülme/iniş, 5 bitti
int   hizCarpani = 1;
float simT = 0, simH = 0, simV = 0, simA = 0;
float yanmaBas = 0, yanmaBitis = 0;
float zirveGercek = 0, zirveGercekZamani = 0;
float sonDurumSaniye = -1;

const char* olayAdi[8] = {"KALKTI", "YANDI (motor bitti)", "İRTİFA OK", "GÖVDE AÇISI",
                          "APOGEE", "APOGEE PİRO", "MAIN", "MAIN PİRO"};
uint8_t sonBayrak = 0;
float olayZaman[8], olayIrtifa[8], olayKalmanIrtifa[8];
bool  olayVar[8];

float gurultu(float genlik) {
  return (random(-1000, 1001) / 1000.0f) * genlik;
}

uint8_t bayrakByte() {
  uint8_t d = 0;
  if (kalkti)       d |= (1 << 0);
  if (yandi)        d |= (1 << 1);
  if (irtifaOk)     d |= (1 << 2);
  if (govdeAcisi)   d |= (1 << 3);
  if (apogee)       d |= (1 << 4);
  if (ApogeePyroOk) d |= (1 << 5);
  if (Main)         d |= (1 << 6);
  if (MainPyroOk)   d |= (1 << 7);
  return d;
}

void simBaslat() {
  kalkti = yandi = irtifaOk = apogee = govdeAcisi = ApogeePyroOk = Main = MainPyroOk = false;
  zirveIrtifa = 0;
  yerIrtifaHazir = false; yerIrtifaSayac = 0; yerIrtifaToplam = 0;
  kalmanIrtifa = 0; kalmanHiz = 0;
  P_00 = 1; P_01 = 0; P_10 = 0; P_11 = 1;

  faz = 0; simT = 0; simH = 0; simV = 0; simA = 0;
  zirveGercek = 0; zirveGercekZamani = 0; sonDurumSaniye = -1;
  sonBayrak = 0;
  for (int i = 0; i < 8; i++) olayVar[i] = false;
  simAktif = true;

  Serial.println(F("\n===== SİMÜLASYON BAŞLADI ====="));
  Serial.printf("Yer rakımı: %.0f m | İvme türü: %s | Kalman başlangıcı: %s | Hız: %dx\n",
                YER_RAKIM, GRAVITELI_IVME ? "ivmeölçer (yerçekimli)" : "lineer (yerçekimsiz)",
                KALMAN_ILK_OLCUMLE ? "ilk ölçüm" : "0 (UKB gibi)", hizCarpani);
}

void simFizikAdimi() {
  simT += DT;

  if (faz == 0 && simT >= RAMPA_SURESI) { faz = 1; yanmaBas = simT; }
  if (faz == 1 && simT - yanmaBas >= YANMA_SURESI) { faz = 2; yanmaBitis = simT; }

  if (faz == 0) {
    simA = 0; simV = 0; simH = 0;
    return;
  }

  float aMotor = (faz == 1) ? MOTOR_IVME : 0.0f;

  if (MainPyroOk)        simA = (-MAIN_INIS_HIZI   - simV) / PARASUT_ZAMAN_SABITI;
  else if (ApogeePyroOk) simA = (-DROGUE_INIS_HIZI - simV) / PARASUT_ZAMAN_SABITI;
  else                   simA = aMotor - G - SURTUNME_KAT * simV * fabsf(simV);

  simV += simA * DT;
  simH += simV * DT;

  if (simH > zirveGercek) { zirveGercek = simH; zirveGercekZamani = simT; }

  if (faz >= 2 && simV < 0 && simH <= 0) faz = 5;   // yere indi
}

void olaylariKontrolEt(float kalmanAGL) {
  uint8_t simdikiBayrak = bayrakByte();
  uint8_t degisen = simdikiBayrak ^ sonBayrak;
  for (int i = 0; i < 8; i++) {
    if (degisen & (1 << i)) {
      olayVar[i] = true;
      olayZaman[i] = simT;
      olayIrtifa[i] = simH;
      olayKalmanIrtifa[i] = kalmanAGL;
      Serial.printf("[t=%6.2f s] %-20s | gerçek irtifa: %7.1f m | UKB irtifası: %7.1f m | hız: %6.1f m/s\n",
                    simT, olayAdi[i], simH, kalmanAGL, simV);
    }
  }
  sonBayrak = simdikiBayrak;
}

void ozetYazdir() {
  Serial.println(F("\n================ SİMÜLASYON ÖZETİ ================"));
  Serial.printf("Gerçek zirve       : %.1f m (t = %.2f s)\n", zirveGercek, zirveGercekZamani);
  Serial.printf("UKB'nin gördüğü zirve: %.1f m\n", zirveIrtifa - yerIrtifa);

  if (olayVar[5]) {
    Serial.printf("Apogee piro        : t = %.2f s, gerçek irtifa %.1f m (zirveden %.1f m aşağıda, %.2f s sonra)\n",
                  olayZaman[5], olayIrtifa[5], zirveGercek - olayIrtifa[5], olayZaman[5] - zirveGercekZamani);
  } else {
    Serial.println(F("Apogee piro        : TETİKLENMEDİ  [X]"));
  }

  if (olayVar[7]) {
    Serial.printf("Main piro          : gerçek irtifa %.1f m (hedef %.0f m)\n", olayIrtifa[7], MAIN_IRTIFA);
  } else {
    Serial.println(F("Main piro          : TETİKLENMEDİ  [X]"));
  }

  bool ok = olayVar[5] && olayVar[7] &&
            fabsf(zirveGercek - olayIrtifa[5]) < 60.0f &&
            fabsf(olayIrtifa[7] - MAIN_IRTIFA) < 60.0f;
  Serial.println(ok ? F("SONUÇ: Algoritma beklenen şekilde çalıştı [OK]")
                    : F("SONUÇ: Beklenmeyen davranış var, yukarıdaki olay zamanlarını incele [X]"));
  Serial.println(F("==================================================\n"));
}

void simAdimi() {
  simFizikAdimi();

  float hamIrtifa = YER_RAKIM + simH + gurultu(0.6f);

  SensorVerisi v;
  v.ivmeZ = simA + (GRAVITELI_IVME ? G : 0.0f) + gurultu(0.15f);
  v.ivmeX = gurultu(0.1f);
  v.ivmeY = gurultu(0.1f);

  // Açılar: rampada ve yanmada neredeyse dik, yanma bitince yavaşça yatar
  if (faz == 0)       { v.aciX = gurultu(0.3f);  v.aciY = gurultu(0.3f); }
  else if (faz == 1)  { v.aciX = 2.0f * sinf(simT * 3.0f); v.aciY = 2.0f * sinf(simT * 2.0f); }
  else                { v.aciX = 2.0f * sinf(simT * 3.0f); v.aciY = min(110.0f, 4.0f * (simT - yanmaBitis)); }
  v.aciZ = 0;

  if (KALMAN_ILK_OLCUMLE && yerIrtifaSayac == 0 && !yerIrtifaHazir) kalmanIrtifa = hamIrtifa;
  kalmanFiltresiGuncelle(hamIrtifa, v.ivmeZ, DT);
  v.irtifa = kalmanIrtifa;

  // UKB'deki gibi: önce yer irtifasını biriktir, sonra algoritmayı çalıştır
  if (!yerIrtifaHazir) {
    yerIrtifaToplam += hamIrtifa;
    yerIrtifaSayac++;
    if (yerIrtifaSayac >= YER_IRTIFA_ORNEK) {
      yerIrtifa = yerIrtifaToplam / (float)YER_IRTIFA_ORNEK;
      yerIrtifaHazir = true;
    }
  } else {
    algoritmaCalistir(v);
  }

  float kalmanAGL = kalmanIrtifa - yerIrtifa;
  olaylariKontrolEt(kalmanAGL);

  // Her simülasyon saniyesinde bir durum satırı
  if ((int)simT != (int)sonDurumSaniye) {
    sonDurumSaniye = simT;
    float sapma = sapmaHesapla(v.aciX, v.aciY);
    Serial.printf("t=%3d s | gerçek: %7.1f m %7.1f m/s | UKB: %7.1f m | |a|: %6.1f | sapma: %5.1f derece\n",
                  (int)simT, simH, simV, kalmanAGL,
                  sqrtf(v.ivmeX*v.ivmeX + v.ivmeY*v.ivmeY + v.ivmeZ*v.ivmeZ), sapma);
  }

  if (faz == 5) {
    simAktif = false;
    Serial.println(F("--- Roket yere indi ---"));
    ozetYazdir();
  }
}

////////////////////////////////////
void komutIsle() {
  while (Serial.available()) {
    char k = toupper(Serial.read());
    if (k == 'B') simBaslat();
    else if (k == 'D') { simAktif = false; Serial.println(F("[Simülasyon durduruldu]")); }
    else if (k == 'H') {
      hizCarpani = (hizCarpani == 1) ? 10 : 1;
      Serial.printf("[Hız: %dx]\n", hizCarpani);
    } else if (k == 'G') {
      GRAVITELI_IVME = !GRAVITELI_IVME;
      Serial.println(GRAVITELI_IVME ? F("[İvme türü: ivmeölçer (yerçekimli)]") : F("[İvme türü: lineer (yerçekimsiz)]"));
    } else if (k == 'K') {
      KALMAN_ILK_OLCUMLE = !KALMAN_ILK_OLCUMLE;
      Serial.println(KALMAN_ILK_OLCUMLE ? F("[Kalman ilk ölçümle başlayacak]") : F("[Kalman 0'dan başlayacak (UKB gibi)]"));
    }
  }
}

////////////////////////////////////
void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  randomSeed(micros());

  Serial.println(F("ÜLGEN UKB - ALGORİTMA MASA SİMÜLASYONU"));
  Serial.println(F("Komutlar: B = başlat, D = durdur, H = hız 1x/10x, G = ivme türü, K = Kalman başlangıcı"));
}

////////////////////////////////////
void loop() {
  simdi = millis();
  komutIsle();

  if (buzzerA && simdi - sonbuzzer >= 300) {
    buzzerA = false;
    digitalWrite(buzzer, LOW);
  }

  if (simAktif && simdi - sonAdim >= (unsigned long)(DT * 1000)) {
    sonAdim = simdi;
    for (int i = 0; i < hizCarpani && simAktif; i++) simAdimi();
  }
}

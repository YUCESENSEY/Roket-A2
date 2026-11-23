////////////////////////////////////
// ÜLGEN UKB - SİT / SUT YER SİMÜLATÖRÜ
// Bu kod İKİNCİ bir Arduino Nano ESP32 (test kartı) üzerinde çalışır ve
// yarışmadaki SİT/SUT bilgisayarı gibi davranır. UKB kartına RS232 hattından bağlanır.
//
// BAĞLANTI (UKB kartı <-> bu kart), TX/RX ÇAPRAZ, GND ORTAK:
//   Bu kartın D7 (TX)  ->  UKB RS232 RX (D8)
//   Bu kartın D8 (RX)  <-  UKB RS232 TX (D7)
//   GND - GND
//   (UKB tarafında MAX3232 kullanılıyorsa onun TTL tarafına bağlayın, RS232 tarafına değil.)
//
// Serial Monitor: 115200 baud, "Yeni satır" seçili. UKB'de UKB-Kod.ino yüklü olmalı.
// Komutlar:
//   I -> SİT başlat: UKB sensör verisini gönderir, burada ekrana yazılır
//   U -> SUT başlat: Bu kart sanal bir uçuş verisi gönderir, UKB'nin durum bayrakları izlenir
//   N -> Normal moda dön / durdur
//   D -> Durum bilgisi
//   G -> İvme türü: lineer / ivmeölçer (UKB algoritması hangisine göre ayarlıysa)
////////////////////////////////////

HardwareSerial RS232(1);
#define RS232_RX D8
#define RS232_TX D7

////////////////////////////////////
// Simülasyon ayarları
const float G                = 9.81f;
const float YER_RAKIM        = 850.0f;
const float RAMPA_SURESI     = 3.0f;
const float YANMA_SURESI     = 3.5f;
const float MOTOR_IVME       = 85.0f;
const float SURTUNME_KAT     = 0.0002f;
const float DROGUE_INIS_HIZI = 35.0f;
const float MAIN_INIS_HIZI   = 8.0f;
const float PARASUT_ZAMAN_SABITI = 1.0f;
const unsigned long PAKET_ARALIGI = 50;     // ms (20 Hz)
const float DT = 0.05f;
float MAIN_IRTIFA_HEDEF = 500;               // UKB'deki MAIN_IRTIFA ile aynı olmalı
bool GRAVITELI_IVME = false;

////////////////////////////////////
char mod = 'N';           // N = normal, I = SİT, U = SUT
unsigned long simdi = 0;
unsigned long sonPaketZamani = 0;
unsigned long komutZamani = 0;
unsigned long sonUkbPaketi = 0;
unsigned long sonUyari = 0;

// UKB durum bayrakları (UKB'den okunan)
uint8_t ukbBayrak = 0;
bool ukbApogeePiro = false;
bool ukbMainPiro   = false;

// Sanal roket
int   faz = 0;
float simT = 0, simH = 0, simV = 0, simA = 0;
float yanmaBitis = 0, yanmaBas = 0;
float zirveGercek = 0, zirveGercekZamani = 0;
bool  ucusBitti = false;
const char* olayAdi[8] = {"KALKTI", "YANDI", "İRTİFA OK", "GÖVDE AÇISI",
                          "APOGEE", "APOGEE PİRO", "MAIN", "MAIN PİRO"};
bool  olayVar[8];
float olayZaman[8], olayIrtifa[8];

// Alıcı
uint8_t rxTampon[40];
uint8_t rxSayac = 0, rxBeklenen = 0, rxDurum = 0;
unsigned long rxBaslangic = 0;
uint32_t sitPaketSayisi = 0, durumPaketSayisi = 0, hataliPaket = 0;
unsigned long sonSitYazdirma = 0;

////////////////////////////////////
float bytesToFloat(const uint8_t *kaynak) {
  uint8_t hedef[4];
  hedef[0] = kaynak[3]; hedef[1] = kaynak[2]; hedef[2] = kaynak[1]; hedef[3] = kaynak[0];
  float d;
  memcpy(&d, hedef, 4);
  return d;
}

void floatToBytes(float deger, uint8_t *hedef) {
  uint8_t kaynak[4];
  memcpy(kaynak, &deger, 4);
  hedef[0] = kaynak[3]; hedef[1] = kaynak[2]; hedef[2] = kaynak[1]; hedef[3] = kaynak[0];
}

float gurultu(float genlik) {
  return (random(-1000, 1001) / 1000.0f) * genlik;
}

////////////////////////////////////
// UKB'ye komut gönder: AA KOMUT CHECKSUM 0D 0A
void komutGonder(uint8_t komut) {
  uint8_t p[5] = {0xAA, komut, (uint8_t)(0xAA + komut), 0x0D, 0x0A};
  RS232.write(p, 5);
  komutZamani = millis();
  sonUkbPaketi = millis();
}

// SUT sensör paketi gönder (36 bayt)
void sutPaketiGonder(float irtifa, float basinc, float ax, float ay, float az,
                     float rx, float ry, float rz) {
  uint8_t paket[36];
  paket[0] = 0xAB;
  floatToBytes(irtifa, &paket[1]);
  floatToBytes(basinc, &paket[5]);
  floatToBytes(ax, &paket[9]);
  floatToBytes(ay, &paket[13]);
  floatToBytes(az, &paket[17]);
  floatToBytes(rx, &paket[21]);
  floatToBytes(ry, &paket[25]);
  floatToBytes(rz, &paket[29]);

  uint8_t toplam = 0;
  for (int i = 0; i < 33; i++) toplam += paket[i];
  paket[33] = toplam;
  paket[34] = 0x0D;
  paket[35] = 0x0A;
  RS232.write(paket, 36);
}

////////////////////////////////////
// SANAL ROKET
void simSifirla() {
  faz = 0; simT = 0; simH = 0; simV = 0; simA = 0;
  zirveGercek = 0; zirveGercekZamani = 0;
  ucusBitti = false;
  ukbBayrak = 0; ukbApogeePiro = false; ukbMainPiro = false;
  for (int i = 0; i < 8; i++) olayVar[i] = false;
}

void simFizikAdimi() {
  simT += DT;

  if (faz == 0 && simT >= RAMPA_SURESI) { faz = 1; yanmaBas = simT; }
  if (faz == 1 && simT - yanmaBas >= YANMA_SURESI) { faz = 2; yanmaBitis = simT; }

  if (faz == 0) { simA = 0; simV = 0; simH = 0; return; }

  float aMotor = (faz == 1) ? MOTOR_IVME : 0.0f;

  // Paraşütler UKB'nin gönderdiği durum bayraklarına göre açılır (kapalı çevrim test)
  if (ukbMainPiro)        simA = (-MAIN_INIS_HIZI   - simV) / PARASUT_ZAMAN_SABITI;
  else if (ukbApogeePiro) simA = (-DROGUE_INIS_HIZI - simV) / PARASUT_ZAMAN_SABITI;
  else                    simA = aMotor - G - SURTUNME_KAT * simV * fabsf(simV);

  simV += simA * DT;
  simH += simV * DT;

  if (simH > zirveGercek) { zirveGercek = simH; zirveGercekZamani = simT; }
  if (faz >= 2 && simV < 0 && simH <= 0) ucusBitti = true;
}

void sutAdimi() {
  simFizikAdimi();

  float hamIrtifa = YER_RAKIM + simH + gurultu(0.6f);
  float basinc    = 1013.25f * powf(1.0f - hamIrtifa / 44330.0f, 5.2553f);
  float ivmeZ     = simA + (GRAVITELI_IVME ? G : 0.0f) + gurultu(0.15f);
  float ivmeX     = gurultu(0.1f);
  float ivmeY     = gurultu(0.1f);

  float aciX, aciY;
  if (faz == 0)      { aciX = gurultu(0.3f);  aciY = gurultu(0.3f); }
  else if (faz == 1) { aciX = 2.0f * sinf(simT * 3.0f); aciY = 2.0f * sinf(simT * 2.0f); }
  else               { aciX = 2.0f * sinf(simT * 3.0f); aciY = min(110.0f, 4.0f * (simT - yanmaBitis)); }

  sutPaketiGonder(hamIrtifa, basinc, ivmeX, ivmeY, ivmeZ, aciX, aciY, 0.0f);
}

void ozetYazdir() {
  Serial.println(F("\n================ SUT ÖZETİ ================"));
  Serial.printf("Gerçek zirve: %.1f m (t = %.2f s)\n", zirveGercek, zirveGercekZamani);
  for (int i = 0; i < 8; i++) {
    if (olayVar[i])
      Serial.printf("  %-14s : t = %6.2f s | gerçek irtifa %7.1f m\n", olayAdi[i], olayZaman[i], olayIrtifa[i]);
    else
      Serial.printf("  %-14s : GELMEDİ\n", olayAdi[i]);
  }
  bool ok = olayVar[5] && olayVar[7] &&
            fabsf(zirveGercek - olayIrtifa[5]) < 60.0f &&
            fabsf(olayIrtifa[7] - MAIN_IRTIFA_HEDEF) < 60.0f;
  Serial.println(ok ? F("SONUÇ: UKB SUT'u doğru tamamladı [OK]")
                    : F("SONUÇ: Beklenmeyen davranış var [X]"));
  Serial.println(F("===========================================\n"));
}

////////////////////////////////////
// UKB'den gelen paketleri işle
void paketiIsle() {
  if (rxTampon[0] == 0xAA) {
    // Durum paketi: AA data1 00 checksum 0D 0A
    uint8_t toplam = (uint8_t)(rxTampon[0] + rxTampon[1] + rxTampon[2]);
    if (toplam != rxTampon[3] || rxTampon[4] != 0x0D || rxTampon[5] != 0x0A) { hataliPaket++; return; }

    durumPaketSayisi++;
    sonUkbPaketi = millis();
    uint8_t yeni = rxTampon[1];
    uint8_t degisen = yeni ^ ukbBayrak;

    for (int i = 0; i < 8; i++) {
      if ((degisen & (1 << i)) && (yeni & (1 << i))) {
        olayVar[i] = true;
        olayZaman[i] = simT;
        olayIrtifa[i] = simH;
        Serial.printf("[t=%6.2f s] UKB: %-12s | gerçek irtifa: %7.1f m | hız: %6.1f m/s\n",
                      simT, olayAdi[i], simH, simV);
      }
    }
    ukbBayrak     = yeni;
    ukbApogeePiro = yeni & (1 << 5);
    ukbMainPiro   = yeni & (1 << 7);

  } else if (rxTampon[0] == 0xAB) {
    // SİT paketi: AB + 8 float + checksum + 0D 0A
    uint8_t toplam = 0;
    for (int i = 0; i < 33; i++) toplam += rxTampon[i];
    if (toplam != rxTampon[33] || rxTampon[34] != 0x0D || rxTampon[35] != 0x0A) { hataliPaket++; return; }

    sitPaketSayisi++;
    sonUkbPaketi = millis();

    if (millis() - sonSitYazdirma >= 500) {
      sonSitYazdirma = millis();
      Serial.printf("SİT | İrtifa: %.2f m | Basınç: %.2f hPa | İvme: %.2f %.2f %.2f | Açı: %.2f %.2f %.2f | (paket: %lu, hatalı: %lu)\n",
                    bytesToFloat(&rxTampon[1]), bytesToFloat(&rxTampon[5]),
                    bytesToFloat(&rxTampon[9]), bytesToFloat(&rxTampon[13]), bytesToFloat(&rxTampon[17]),
                    bytesToFloat(&rxTampon[21]), bytesToFloat(&rxTampon[25]), bytesToFloat(&rxTampon[29]),
                    (unsigned long)sitPaketSayisi, (unsigned long)hataliPaket);
    }
  }
}

void ukbYanitiniOku() {
  while (RS232.available()) {
    uint8_t b = RS232.read();
    unsigned long t = millis();

    if (rxDurum == 0) {
      if (b == 0xAA)      { rxTampon[0] = b; rxSayac = 1; rxBeklenen = 6;  rxDurum = 1; rxBaslangic = t; }
      else if (b == 0xAB) { rxTampon[0] = b; rxSayac = 1; rxBeklenen = 36; rxDurum = 1; rxBaslangic = t; }
    } else {
      rxTampon[rxSayac++] = b;
      if (rxSayac >= rxBeklenen) {
        paketiIsle();
        rxDurum = 0;
        rxSayac = 0;
      }
    }

    if (rxDurum != 0 && t - rxBaslangic > 100) { rxDurum = 0; rxSayac = 0; }
  }
}

////////////////////////////////////
void komutIsle() {
  while (Serial.available()) {
    char k = toupper(Serial.read());
    if (k == 'I') {
      mod = 'I';
      sitPaketSayisi = 0; hataliPaket = 0;
      komutGonder(0x20);
      Serial.println(F("[SİT komutu gönderildi, UKB'den sensör verisi bekleniyor]"));
    } else if (k == 'U') {
      mod = 'U';
      simSifirla();
      durumPaketSayisi = 0; hataliPaket = 0;
      komutGonder(0x22);
      delay(100);
      Serial.println(F("[SUT komutu gönderildi, sanal uçuş başlıyor (3 sn rampada bekler)]"));
    } else if (k == 'N') {
      mod = 'N';
      komutGonder(0x24);
      Serial.println(F("[Normal mod komutu gönderildi]"));
    } else if (k == 'D') {
      Serial.printf("Mod: %c | SİT paketi: %lu | Durum paketi: %lu | Hatalı: %lu | Son UKB paketi: %lu ms önce\n",
                    mod, (unsigned long)sitPaketSayisi, (unsigned long)durumPaketSayisi,
                    (unsigned long)hataliPaket, millis() - sonUkbPaketi);
    } else if (k == 'G') {
      GRAVITELI_IVME = !GRAVITELI_IVME;
      Serial.println(GRAVITELI_IVME ? F("[İvme türü: ivmeölçer (yerçekimli)]") : F("[İvme türü: lineer (yerçekimsiz)]"));
    }
  }
}

////////////////////////////////////
void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  RS232.begin(115200, SERIAL_8N1, RS232_RX, RS232_TX);
  randomSeed(micros());

  Serial.println(F("ÜLGEN UKB - SİT/SUT YER SİMÜLATÖRÜ"));
  Serial.println(F("Komutlar: I = SİT, U = SUT, N = normal, D = durum, G = ivme türü"));
}

////////////////////////////////////
void loop() {
  simdi = millis();
  komutIsle();
  ukbYanitiniOku();

  // SUT: sanal uçuş verisini gönder
  if (mod == 'U' && !ucusBitti && simdi - sonPaketZamani >= PAKET_ARALIGI) {
    sonPaketZamani = simdi;
    sutAdimi();
    if (ucusBitti) {
      Serial.println(F("--- Sanal roket yere indi ---"));
      ozetYazdir();
    }
  }

  // UKB cevap vermiyorsa uyar
  if ((mod == 'I' || mod == 'U') && !ucusBitti && simdi - sonUkbPaketi > 2000 && simdi - sonUyari > 3000) {
    sonUyari = simdi;
    Serial.println(F("UYARI: UKB'den 2 sn'dir paket gelmiyor! TX/RX çaprazlaması, GND ve baud (115200) kontrol et."));
  }
}

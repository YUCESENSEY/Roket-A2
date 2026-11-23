////////////////////////////////////
// ÜLGEN UKB - YER İSTASYONU ALICI
// Yer tarafındaki ESP32 + LoRa modülünde çalışır.
// Roketten (UKB veya Roket_Verici_Test) gelen 35 baytlık telemetri paketini
// yakalar, checksum doğrular, çözüp Serial'e yazar ve paket istatistiklerini tutar.
//
// Paket formatı için bkz. Dosyalar/Paket_Formatlari.md
// LoRa modülleri aynı kanal / hava hızı / adres ayarlarında olmalıdır.
// Serial Monitor: 115200 baud, "Yeni satır" seçili.
// Komutlar:
//   R -> İstatistikleri sıfırla
//   S -> Her paketi yaz / sadece saniyede 2 kez yaz
////////////////////////////////////

HardwareSerial LoRaSerial(2);

#define LORA_RX      D4
#define LORA_TX      D5
#define LORA_M0      A7
#define LORA_M1      A6
#define LORA_AUX     D3

#define LED_UCUS     D10

#define PAKET_BASLIK 0xBC
#define PAKET_BOYUT  35

////////////////////////////////////
uint8_t rxTampon[PAKET_BOYUT];
uint8_t rxSayac = 0;
unsigned long rxSon = 0;

unsigned long simdi          = 0;
unsigned long sonYazdirma    = 0;
unsigned long sonIstatistik  = 0;
unsigned long sonPaketZamani = 0;
unsigned long istatistikBas  = 0;
uint32_t dogruPaket = 0;
uint32_t hataliPaket = 0;
bool hepsiniYaz = false;

const char* bayrakAdi[8] = {"KALKTI", "YANDI", "İRTİFA OK", "GÖVDE AÇISI",
                            "APOGEE", "APOGEE PİRO", "MAIN", "MAIN PİRO"};

////////////////////////////////////
float bytesToFloat(const uint8_t *kaynak) {
  uint8_t hedef[4];
  hedef[0] = kaynak[3]; hedef[1] = kaynak[2]; hedef[2] = kaynak[1]; hedef[3] = kaynak[0];
  float d;
  memcpy(&d, hedef, 4);
  return d;
}

uint16_t oku16u(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
int16_t  oku16s(const uint8_t *p) { return (int16_t)oku16u(p); }

////////////////////////////////////
// PAKETİ ÇÖZ VE YAZDIR
////////////////////////////////////
void paketiCozVeYazdir(const uint8_t *p) {
  float irtifa    = oku16u(&p[1])  / 10.0f;
  float hiz       = oku16s(&p[3])  / 100.0f;
  float basinc    = oku16u(&p[5])  / 10.0f;
  float ivmeX     = oku16s(&p[7])  / 100.0f;
  float ivmeY     = oku16s(&p[9])  / 100.0f;
  float ivmeZ     = oku16s(&p[11]) / 100.0f;
  float aciX      = oku16s(&p[13]) / 10.0f;
  float aciY      = oku16s(&p[15]) / 10.0f;
  float aciZ      = oku16s(&p[17]) / 10.0f;
  float enlem     = bytesToFloat(&p[19]);
  float boylam    = bytesToFloat(&p[23]);
  uint16_t gpsIrt = oku16u(&p[27]);
  uint8_t uydu    = p[29];
  uint8_t saat    = p[30], dakika = p[31], saniye = p[32];
  uint8_t bayrak  = p[33];

  Serial.println(F("\n------------- PAKET -------------"));
  Serial.printf("İrtifa: %.1f m | Hız: %.2f m/s | Basınç: %.1f hPa\n", irtifa, hiz, basinc);
  Serial.printf("İvme: X %.2f | Y %.2f | Z %.2f m/s²\n", ivmeX, ivmeY, ivmeZ);
  Serial.printf("Açı : X %.1f | Y %.1f | Z %.1f derece\n", aciX, aciY, aciZ);
  Serial.printf("GPS : %.6f, %.6f | İrtifa %u m | Uydu %u\n", enlem, boylam, gpsIrt, uydu);
  Serial.printf("Saat: %02u:%02u:%02u\n", saat, dakika, saniye);

  Serial.print(F("Durum: "));
  bool hicbiri = true;
  for (int i = 0; i < 8; i++) {
    if (bayrak & (1 << i)) { Serial.print(bayrakAdi[i]); Serial.print(" | "); hicbiri = false; }
  }
  if (hicbiri) Serial.print(F("(rampada, hiçbir bayrak yok)"));
  Serial.println();
}

////////////////////////////////////
// LORA'DAN GELEN BAYTLARI PAKETE DÖNÜŞTÜR (hatalı pakette yeniden senkronize olur)
////////////////////////////////////
void loraVeriIsle() {
  while (LoRaSerial.available()) {
    uint8_t b = LoRaSerial.read();
    if (rxSayac == 0 && b != PAKET_BASLIK) continue;   // başlığa kadar at

    rxTampon[rxSayac++] = b;
    rxSon = millis();

    if (rxSayac >= PAKET_BOYUT) {
      uint8_t toplam = 0;
      for (int i = 0; i < PAKET_BOYUT - 1; i++) toplam += rxTampon[i];

      if (toplam == rxTampon[PAKET_BOYUT - 1]) {
        dogruPaket++;
        sonPaketZamani = millis();
        digitalWrite(LED_UCUS, !digitalRead(LED_UCUS));

        if (hepsiniYaz || millis() - sonYazdirma >= 500) {
          sonYazdirma = millis();
          paketiCozVeYazdir(rxTampon);
        }
        rxSayac = 0;
      } else {
        hataliPaket++;
        // Tamponda bir sonraki başlığı ara
        int k = 1;
        while (k < PAKET_BOYUT && rxTampon[k] != PAKET_BASLIK) k++;
        if (k < PAKET_BOYUT) {
          memmove(rxTampon, &rxTampon[k], PAKET_BOYUT - k);
          rxSayac = PAKET_BOYUT - k;
        } else {
          rxSayac = 0;
        }
      }
    }
  }

  // Yarım kalan paket 300 ms içinde tamamlanmadıysa at
  if (rxSayac > 0 && millis() - rxSon > 300) rxSayac = 0;
}

////////////////////////////////////
void istatistikYazdir() {
  float sure = (millis() - istatistikBas) / 1000.0f;
  float hz   = sure > 0 ? dogruPaket / sure : 0;
  uint32_t toplam = dogruPaket + hataliPaket;
  float hataYuzde = toplam > 0 ? 100.0f * hataliPaket / toplam : 0;
  // Roket 10 Hz gönderiyor: beklenen paket = süre * 10
  float kayipYuzde = sure > 5 ? 100.0f * (1.0f - dogruPaket / (sure * 10.0f)) : 0;
  if (kayipYuzde < 0) kayipYuzde = 0;

  Serial.printf("\n[İSTATİSTİK] Doğru: %lu | Hatalı: %lu (%%%.1f) | Hız: %.1f paket/sn | Tahmini kayıp: %%%.1f | Son paket: %.1f sn önce\n",
                (unsigned long)dogruPaket, (unsigned long)hataliPaket, hataYuzde, hz, kayipYuzde,
                sonPaketZamani ? (millis() - sonPaketZamani) / 1000.0f : -1.0f);
}

void komutIsle() {
  while (Serial.available()) {
    char k = toupper(Serial.read());
    if (k == 'R') {
      dogruPaket = hataliPaket = 0;
      istatistikBas = millis();
      Serial.println(F("[İstatistikler sıfırlandı]"));
    } else if (k == 'S') {
      hepsiniYaz = !hepsiniYaz;
      Serial.println(hepsiniYaz ? F("[Her paket yazılacak]") : F("[Saniyede 2 paket yazılacak]"));
    }
  }
}

////////////////////////////////////
void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  pinMode(LORA_M0, OUTPUT); digitalWrite(LORA_M0, LOW);   // normal mod
  pinMode(LORA_M1, OUTPUT); digitalWrite(LORA_M1, LOW);
  pinMode(LORA_AUX, INPUT);
  pinMode(LED_UCUS, OUTPUT); digitalWrite(LED_UCUS, LOW);

  unsigned long loraT = millis();
  while (digitalRead(LORA_AUX) == LOW && millis() - loraT < 3000);
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);

  istatistikBas = millis();
  Serial.println(F("ÜLGEN UKB - YER İSTASYONU ALICI başladı"));
  Serial.println(F("Komutlar: R = istatistik sıfırla, S = her paketi yaz / yavaşlat"));
}

////////////////////////////////////
void loop() {
  simdi = millis();
  komutIsle();
  loraVeriIsle();

  if (simdi - sonIstatistik >= 5000) {
    sonIstatistik = simdi;
    istatistikYazdir();
    if (dogruPaket == 0 && simdi - istatistikBas > 5000) {
      Serial.println(F("UYARI: Hiç paket gelmiyor! Kanal, hava hızı, adres, UART baud ve anten bağlantısını kontrol et."));
    }
  }
}

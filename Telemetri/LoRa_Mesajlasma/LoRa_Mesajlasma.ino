////////////////////////////////////
// ÜLGEN UKB - LoRa MESAJLAŞMA / PING TESTİ
// AYNI kod iki tarafa da (roket ve yer istasyonu) yüklenir.
// Serial Monitor'e yazdığın metin LoRa ile karşıya gider, karşıdan gelen metin ekrana yazılır.
// Menzil ve gecikme testi için ping komutu vardır.
//
// Serial Monitor: 115200 baud, "Yeni satır" seçili.
// Komutlar (başına / koy):
//   /ping    -> Karşıya PING gönderir, cevap gelince gidiş-dönüş süresini yazar
//   /otomatik-> Her 2 sn'de bir otomatik TEST mesajı gönderir (aç/kapat)
//   /yardim  -> Komutları yazar
// Başka her yazı normal mesaj olarak gönderilir.
////////////////////////////////////

// Bu kartın adı: yer istasyonuna yüklerken "YER", roket tarafına yüklerken "ROKET" yap
#define ISTASYON_ADI "YER"

HardwareSerial LoRaSerial(2);

#define LORA_RX      D4
#define LORA_TX      D5
#define LORA_M0      A7
#define LORA_M1      A6
#define LORA_AUX     D3

////////////////////////////////////
#define SATIR_MAKS 200
char usbSatir[SATIR_MAKS];
uint8_t usbIndex = 0;
char loraSatir[SATIR_MAKS];
uint8_t loraIndex = 0;

unsigned long simdi        = 0;
unsigned long sonOtomatik  = 0;
unsigned long pingZamani   = 0;
bool otomatikAktif = false;
uint32_t pingNo = 0, testNo = 0;
uint32_t gonderilen = 0, alinan = 0;

////////////////////////////////////
void loraGonder(const String &metin) {
  LoRaSerial.print(metin);
  LoRaSerial.print('\n');
  gonderilen++;
}

void yardimYazdir() {
  Serial.println(F("Komutlar: /ping  /otomatik  /yardim   (diğer yazılar mesaj olarak gönderilir)"));
}

////////////////////////////////////
// Karşıdan gelen satırı işle
void gelenSatirIsle(const char *satir) {
  alinan++;
  String s = String(satir);

  if (s.startsWith("PING,")) {
    // Hemen PONG ile cevap ver
    loraGonder("PONG," + s.substring(5));
    Serial.printf("[%lu ms] PING alındı, PONG gönderildi\n", millis());
  } else if (s.startsWith("PONG,")) {
    uint32_t no = s.substring(5).toInt();
    if (no == pingNo && pingZamani != 0) {
      Serial.printf("[PONG] Gidiş-dönüş süresi: %lu ms\n", millis() - pingZamani);
      pingZamani = 0;
    }
  } else {
    Serial.printf("[%lu ms] KARŞIDAN: %s\n", millis(), satir);
  }
}

////////////////////////////////////
void loraOku() {
  while (LoRaSerial.available()) {
    char c = LoRaSerial.read();
    if (c == '\n' || c == '\r') {
      if (loraIndex > 0) {
        loraSatir[loraIndex] = '\0';
        gelenSatirIsle(loraSatir);
        loraIndex = 0;
      }
    } else if (loraIndex < SATIR_MAKS - 1) {
      loraSatir[loraIndex++] = c;
    }
  }
}

void usbOku() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbIndex > 0) {
        usbSatir[usbIndex] = '\0';
        String s = String(usbSatir);
        s.trim();
        usbIndex = 0;

        if (s == "/ping") {
          pingNo++;
          pingZamani = millis();
          loraGonder("PING," + String(pingNo));
          Serial.printf("[PING #%lu gönderildi, cevap bekleniyor...]\n", (unsigned long)pingNo);
        } else if (s == "/otomatik") {
          otomatikAktif = !otomatikAktif;
          Serial.println(otomatikAktif ? F("[Otomatik gönderim AÇIK]") : F("[Otomatik gönderim KAPALI]"));
        } else if (s == "/yardim") {
          yardimYazdir();
        } else if (s.length() > 0) {
          loraGonder(String(ISTASYON_ADI) + ": " + s);
          Serial.printf("[GÖNDERİLDİ] %s\n", s.c_str());
        }
      }
    } else if (usbIndex < SATIR_MAKS - 1) {
      usbSatir[usbIndex++] = c;
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

  unsigned long loraT = millis();
  while (digitalRead(LORA_AUX) == LOW && millis() - loraT < 3000);
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);

  Serial.printf("ÜLGEN UKB - LoRa MESAJLAŞMA [%s] başladı\n", ISTASYON_ADI);
  yardimYazdir();
}

////////////////////////////////////
void loop() {
  simdi = millis();
  usbOku();
  loraOku();

  if (otomatikAktif && simdi - sonOtomatik >= 2000) {
    sonOtomatik = simdi;
    testNo++;
    loraGonder(String(ISTASYON_ADI) + ": TEST " + String(testNo));
    Serial.printf("[OTOMATİK] TEST %lu gönderildi (toplam gönderilen: %lu, alınan: %lu)\n",
                  (unsigned long)testNo, (unsigned long)gonderilen, (unsigned long)alinan);
  }

  // Ping cevabı 3 sn içinde gelmediyse bildir
  if (pingZamani != 0 && simdi - pingZamani > 3000) {
    Serial.println(F("[PING] Cevap gelmedi (zaman aşımı). Menzil, anten ve ayarları kontrol et."));
    pingZamani = 0;
  }
}

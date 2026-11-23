////////////////////////////////////
// ÜLGEN UKB - E22 LoRa KONFİGÜRASYON ARACI
// Hex yazmak yerine okunabilir komutlarla modül ayarlarını okur ve değiştirir.
// Her değiştirme komutu önce mevcut ayarı okur, sadece istenen alanı değiştirir
// ve kalıcı olarak (flash'a) yazar.
//
// Serial Monitor: 115200 baud, "Yeni satır" seçili.
// Komutlar:
//   OKU                 -> Mevcut ayarları oku ve çöz
//   KANAL 23            -> Kanal (0-80)
//   HAVA 2400           -> Hava hızı: 2400, 4800, 9600, 19200, 38400, 62500
//   BAUD 115200         -> UART hızı: 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
//   GUC 0               -> Çıkış gücü: 0 = en yüksek, 1, 2, 3 = en düşük
//   ADRES 0 0           -> Adres (ADDH ADDL)
//   NETID 0             -> Ağ kimliği (0-255)
//   HAM C1 00 09        -> Ham hex komut gönder (ileri seviye)
//
// UYARI: Roket ve yer istasyonu modüllerinde ADRES, NETID, KANAL ve HAVA hızı AYNI olmalı.
// BAUD'u değiştirirsen UKB-Kod.ino'daki LoRaSerial.begin() hızını da değiştirmelisin.
////////////////////////////////////

HardwareSerial LoRaSerial(2);

#define LORA_RX      D4
#define LORA_TX      D5
#define LORA_M0      A7
#define LORA_M1      A6
#define LORA_AUX     D3

////////////////////////////////////
// cfg[]: [0]=komut [1]=0x00 [2]=0x09 [3]=ADDH [4]=ADDL [5]=NETID
//        [6]=REG0 [7]=REG1 [8]=REG2(kanal) [9]=REG3 [10]=CRYPT_H [11]=CRYPT_L
uint8_t cfg[12];
uint32_t normalBaud = 115200;

const uint32_t BAUD_TABLOSU[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
const char* HAVA_TABLOSU[8] = {"2.4k", "2.4k", "2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"};
const char* GUC_TABLOSU[4]  = {"En yüksek (30 dBm, E22-xxxT30D)", "-3 dB (27 dBm)", "-6 dB (24 dBm)", "-9 dB (21 dBm)"};

////////////////////////////////////
void auxBekle(unsigned long sure) {
  unsigned long bas = millis();
  while (digitalRead(LORA_AUX) == LOW && millis() - bas < sure);
  delay(5);
}

void konfigModunaGir() {
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, HIGH);
  delay(50);
  auxBekle(1000);
  LoRaSerial.end();
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);   // konfigürasyon modunda sabit 9600
  delay(50);
  while (LoRaSerial.available()) LoRaSerial.read();
}

void normalModaDon() {
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(50);
  auxBekle(1000);
  LoRaSerial.end();
  LoRaSerial.begin(normalBaud, SERIAL_8N1, LORA_RX, LORA_TX);
}

// n bayt gelene kadar (zaman aşımı ms) oku, kaç bayt okunduğunu döndür
int yanitOku(uint8_t *hedef, int n, unsigned long sure) {
  int say = 0;
  unsigned long bas = millis();
  while (millis() - bas < sure && say < n) {
    if (LoRaSerial.available()) hedef[say++] = LoRaSerial.read();
  }
  return say;
}

////////////////////////////////////
bool konfigOku() {
  konfigModunaGir();
  uint8_t komut[3] = {0xC1, 0x00, 0x09};
  LoRaSerial.write(komut, 3);
  int n = yanitOku(cfg, 12, 1000);
  bool ok = (n == 12 && cfg[0] == 0xC1);
  if (ok) normalBaud = BAUD_TABLOSU[(cfg[6] >> 5) & 0x07];
  normalModaDon();
  return ok;
}

bool konfigYaz() {
  uint8_t gonder[12];
  memcpy(gonder, cfg, 12);
  gonder[0] = 0xC0;                       // 0xC0 = kalıcı yaz
  gonder[1] = 0x00;
  gonder[2] = 0x09;

  konfigModunaGir();
  LoRaSerial.write(gonder, 12);
  uint8_t yanit[12];
  int n = yanitOku(yanit, 12, 1000);
  bool ok = (n >= 3 && yanit[0] == 0xC1);
  if (ok) normalBaud = BAUD_TABLOSU[(gonder[6] >> 5) & 0x07];
  normalModaDon();
  return ok;
}

////////////////////////////////////
void konfigYazdir() {
  Serial.println(F("\n---------- E22 AYARLARI ----------"));
  Serial.print(F("Ham: "));
  for (int i = 0; i < 12; i++) { if (cfg[i] < 0x10) Serial.print("0"); Serial.print(cfg[i], HEX); Serial.print(" "); }
  Serial.println();

  Serial.printf("Adres        : %u (ADDH=%u, ADDL=%u)\n", (cfg[3] << 8) | cfg[4], cfg[3], cfg[4]);
  Serial.printf("NETID        : %u\n", cfg[5]);
  Serial.printf("UART baud    : %lu\n", (unsigned long)BAUD_TABLOSU[(cfg[6] >> 5) & 0x07]);
  Serial.printf("Hava hızı    : %s bps\n", HAVA_TABLOSU[cfg[6] & 0x07]);
  Serial.printf("Çıkış gücü   : %s\n", GUC_TABLOSU[cfg[7] & 0x03]);
  Serial.printf("Kanal        : %u  (E22-400: 410.125 + kanal MHz, E22-900: 850.125 + kanal MHz)\n", cfg[8]);
  Serial.printf("RSSI baytı   : %s\n", (cfg[9] & 0x80) ? "AÇIK (pakete 1 bayt eklenir!)" : "kapalı");
  Serial.printf("İletim modu  : %s\n", (cfg[9] & 0x40) ? "Sabit noktalı" : "Şeffaf");
  Serial.println(F("----------------------------------"));
}

////////////////////////////////////
void komutIsle(String s) {
  s.trim();
  s.toUpperCase();
  if (s.length() == 0) return;

  String komut = s;
  String arg = "";
  int bosluk = s.indexOf(' ');
  if (bosluk > 0) { komut = s.substring(0, bosluk); arg = s.substring(bosluk + 1); arg.trim(); }

  if (komut == "OKU") {
    if (konfigOku()) konfigYazdir();
    else Serial.println(F("HATA: Modül yanıt vermedi! M0/M1/RX/TX/besleme kontrol et."));
    return;
  }

  if (komut == "HAM") {
    // Örnek: HAM C1 00 09
    uint8_t buf[40];
    int say = 0;
    int bas = 0;
    while (bas < (int)arg.length() && say < 40) {
      int son = arg.indexOf(' ', bas);
      if (son < 0) son = arg.length();
      buf[say++] = (uint8_t)strtol(arg.substring(bas, son).c_str(), NULL, 16);
      bas = son + 1;
    }
    konfigModunaGir();
    LoRaSerial.write(buf, say);
    uint8_t yanit[40];
    int n = yanitOku(yanit, 40, 800);
    normalModaDon();
    Serial.print(F("Yanıt: "));
    for (int i = 0; i < n; i++) { if (yanit[i] < 0x10) Serial.print("0"); Serial.print(yanit[i], HEX); Serial.print(" "); }
    Serial.println();
    return;
  }

  // Aşağıdakiler: önce oku, alanı değiştir, yaz, tekrar oku ve doğrula
  if (!konfigOku()) { Serial.println(F("HATA: Mevcut ayar okunamadı, değişiklik yapılmadı.")); return; }

  if (komut == "KANAL") {
    int k = arg.toInt();
    if (k < 0 || k > 80) { Serial.println(F("Kanal 0-80 arasında olmalı.")); return; }
    cfg[8] = k;
  } else if (komut == "HAVA") {
    int h = arg.toInt();
    uint8_t kod;
    if      (h == 2400)  kod = 2;
    else if (h == 4800)  kod = 3;
    else if (h == 9600)  kod = 4;
    else if (h == 19200) kod = 5;
    else if (h == 38400) kod = 6;
    else if (h == 62500) kod = 7;
    else { Serial.println(F("Geçersiz hava hızı. 2400/4800/9600/19200/38400/62500")); return; }
    cfg[6] = (cfg[6] & 0xF8) | kod;
  } else if (komut == "BAUD") {
    uint32_t b = arg.toInt();
    int kod = -1;
    for (int i = 0; i < 8; i++) if (BAUD_TABLOSU[i] == b) kod = i;
    if (kod < 0) { Serial.println(F("Geçersiz baud.")); return; }
    cfg[6] = (cfg[6] & 0x1F) | (kod << 5);
  } else if (komut == "GUC") {
    int g = arg.toInt();
    if (g < 0 || g > 3) { Serial.println(F("Güç 0-3 arasında olmalı.")); return; }
    cfg[7] = (cfg[7] & 0xFC) | g;
  } else if (komut == "ADRES") {
    int bosluk2 = arg.indexOf(' ');
    if (bosluk2 < 0) { Serial.println(F("Kullanım: ADRES <ADDH> <ADDL>")); return; }
    cfg[3] = arg.substring(0, bosluk2).toInt();
    cfg[4] = arg.substring(bosluk2 + 1).toInt();
  } else if (komut == "NETID") {
    cfg[5] = arg.toInt();
  } else {
    Serial.println(F("Bilinmeyen komut. OKU / KANAL / HAVA / BAUD / GUC / ADRES / NETID / HAM"));
    return;
  }

  if (!konfigYaz()) { Serial.println(F("HATA: Yazma başarısız!")); return; }
  Serial.println(F("Yazıldı. Doğrulamak için tekrar okunuyor..."));
  delay(100);
  if (konfigOku()) konfigYazdir();
  else Serial.println(F("UYARI: Doğrulama okunamadı."));
}

////////////////////////////////////
String usbSatir = "";

void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  pinMode(LORA_M0, OUTPUT); digitalWrite(LORA_M0, LOW);
  pinMode(LORA_M1, OUTPUT); digitalWrite(LORA_M1, LOW);
  pinMode(LORA_AUX, INPUT);

  auxBekle(3000);
  LoRaSerial.begin(normalBaud, SERIAL_8N1, LORA_RX, LORA_TX);

  Serial.println(F("ÜLGEN UKB - E22 LoRa KONFİGÜRASYON ARACI"));
  Serial.println(F("Komutlar: OKU, KANAL n, HAVA n, BAUD n, GUC n, ADRES h l, NETID n, HAM <hex...>"));
  if (konfigOku()) konfigYazdir();
  else Serial.println(F("UYARI: Modül yanıt vermedi."));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbSatir.length() > 0) { komutIsle(usbSatir); usbSatir = ""; }
    } else {
      usbSatir += c;
    }
  }
}

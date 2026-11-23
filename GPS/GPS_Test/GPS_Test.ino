#include <TinyGPS++.h>

////////////////////////////////////
// ÜLGEN UKB - GPS TESTİ
// GPS modülünden gelen TÜM verileri Serial'e yazar.
// Serial Monitor: 115200 baud, "Yeni satır" seçili.
// Komutlar:
//   H -> Ham NMEA verisini aç/kapat (GPS'in ne gönderdiğini aynen görmek için)
//   S -> Ayrıntılı yazdırmayı aç/kapat
//   R -> İstatistikleri ve ilk fix süresini sıfırla
////////////////////////////////////

TinyGPSPlus gps;
HardwareSerial GPS(0);   // UART0 (RX = D1, TX = D0)

////////////////////////////////////
unsigned long simdi          = 0;
unsigned long sonYazdirma    = 0;
unsigned long sonVeriZamani  = 0;
unsigned long baslangicZamani = 0;
unsigned long ilkFixSuresi   = 0;
unsigned long sonUyariZamani = 0;
bool fixAlindi    = false;
bool hamNmea      = false;
bool ayrintiliMod = true;

////////////////////////////////////
void tarihSaatYazdir() {
  if (gps.date.isValid()) {
    Serial.printf("  Tarih (UTC)     : %02d.%02d.%04d\n", gps.date.day(), gps.date.month(), gps.date.year());
  } else {
    Serial.println(F("  Tarih (UTC)     : geçersiz"));
  }

  if (gps.time.isValid()) {
    int tsiSaat = gps.time.hour() + 3;      // Türkiye saati (UTC+3)
    if (tsiSaat >= 24) tsiSaat -= 24;
    Serial.printf("  Saat (UTC)      : %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.printf("  Saat (TSİ)      : %02d:%02d:%02d\n", tsiSaat, gps.time.minute(), gps.time.second());
  } else {
    Serial.println(F("  Saat            : geçersiz"));
  }
}

void gpsDurumYazdir() {
  Serial.println(F("\n---------- GPS DURUMU ----------"));
  Serial.printf("  Çalışma süresi  : %lu sn\n", (millis() - baslangicZamani) / 1000);

  if (gps.location.isValid()) {
    Serial.println(F("  FIX             : VAR"));
    Serial.printf("  Enlem           : %.6f\n", gps.location.lat());
    Serial.printf("  Boylam          : %.6f\n", gps.location.lng());
    Serial.printf("  Konum yaşı      : %lu ms\n", gps.location.age());
  } else {
    Serial.println(F("  FIX             : YOK (açık havada bekle)"));
  }

  if (gps.altitude.isValid())
    Serial.printf("  İrtifa          : %.1f m\n", gps.altitude.meters());
  else
    Serial.println(F("  İrtifa          : geçersiz"));

  if (gps.speed.isValid())
    Serial.printf("  Hız             : %.2f km/h  (%.2f m/s)\n", gps.speed.kmph(), gps.speed.mps());
  else
    Serial.println(F("  Hız             : geçersiz"));

  if (gps.course.isValid())
    Serial.printf("  Yön             : %.1f derece\n", gps.course.deg());
  else
    Serial.println(F("  Yön             : geçersiz"));

  Serial.printf("  Uydu sayısı     : %lu\n", (unsigned long)gps.satellites.value());

  if (gps.hdop.isValid())
    Serial.printf("  HDOP            : %.2f  (düşük = iyi, <2 çok iyi)\n", gps.hdop.hdop());
  else
    Serial.println(F("  HDOP            : geçersiz"));

  tarihSaatYazdir();

  if (fixAlindi) Serial.printf("  İlk fix süresi  : %.1f sn\n", ilkFixSuresi / 1000.0f);

  Serial.printf("  Karakter        : %lu | Fixli cümle: %lu | Doğru checksum: %lu | Hatalı checksum: %lu\n",
                (unsigned long)gps.charsProcessed(),
                (unsigned long)gps.sentencesWithFix(),
                (unsigned long)gps.passedChecksum(),
                (unsigned long)gps.failedChecksum());
  Serial.println(F("--------------------------------"));
}

////////////////////////////////////
void komutIsle() {
  while (Serial.available()) {
    char k = toupper(Serial.read());
    if (k == 'H') {
      hamNmea = !hamNmea;
      Serial.println(hamNmea ? F("\n[Ham NMEA AÇIK]") : F("\n[Ham NMEA KAPALI]"));
    } else if (k == 'S') {
      ayrintiliMod = !ayrintiliMod;
      Serial.println(ayrintiliMod ? F("\n[Ayrıntılı yazdırma AÇIK]") : F("\n[Ayrıntılı yazdırma KAPALI]"));
    } else if (k == 'R') {
      baslangicZamani = millis();
      fixAlindi = false;
      ilkFixSuresi = 0;
      Serial.println(F("\n[Sayaç sıfırlandı]"));
    }
  }
}

////////////////////////////////////
void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  GPS.begin(9600, SERIAL_8N1, D1, D0);
  baslangicZamani = millis();

  Serial.println(F("ÜLGEN UKB - GPS TESTİ başladı"));
  Serial.println(F("Komutlar: H = ham NMEA, S = ayrıntılı yazdırma, R = sıfırla"));
}

////////////////////////////////////
void loop() {
  simdi = millis();

  while (GPS.available()) {
    char c = GPS.read();
    sonVeriZamani = simdi;
    if (hamNmea) Serial.write(c);
    gps.encode(c);
  }

  komutIsle();

  if (!fixAlindi && gps.location.isValid()) {
    fixAlindi = true;
    ilkFixSuresi = simdi - baslangicZamani;
    Serial.printf("\n*** İLK FIX ALINDI! Süre: %.1f sn ***\n", ilkFixSuresi / 1000.0f);
  }

  // GPS'ten hiç veri gelmiyorsa uyar
  if (simdi - sonVeriZamani > 3000 && simdi - sonUyariZamani > 3000) {
    sonUyariZamani = simdi;
    Serial.println(F("UYARI: GPS'ten veri gelmiyor! RX/TX kablosu, besleme ve baud (9600) kontrol et."));
  }

  if (ayrintiliMod && !hamNmea && simdi - sonYazdirma >= 1000) {
    sonYazdirma = simdi;
    gpsDurumYazdir();
  }
}

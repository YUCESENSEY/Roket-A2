////////////////////////////////////
// ÜLGEN UKB - ROKET TELEMETRİ VERİCİ TESTİ
// Roket tarafı: UKB'deki loraSensorGonder() ile BİREBİR aynı 35 baytlık paketi,
// sensör olmadan SANAL (üretilmiş) verilerle 10 Hz gönderir.
// Böylece yer istasyonu kodu, menzil ve paket kaybı sensörsüz test edilir.
// UKB kartının kendisinde veya aynı LoRa bağlantısı olan herhangi bir ESP32'de çalışır.
// Yer tarafında Yer_Istasyonu_Alici.ino çalıştırılır.
//
// Sanal uçuş 60 sn'lik döngüdür: yükselir -> zirve -> iner -> tekrar başlar.
// Serial Monitor: 115200 baud, "Yeni satır" seçili.
// Komutlar:
//   G -> Gönderimi durdur / devam ettir
//   R -> Sanal uçuşu başa al
////////////////////////////////////

HardwareSerial LoRaSerial(2);

#define LORA_RX      D4
#define LORA_TX      D5
#define LORA_M0      A7
#define LORA_M1      A6
#define LORA_AUX     D3

#define LED_UCUS     D10

////////////////////////////////////
struct SensorVerisi {
  float basinc;
  float irtifa;
  float hiz;
  float aciX, aciY, aciZ;
  float ivmeX, ivmeY, ivmeZ;
  float enlem, boylam, gpsIrtifa;
  int   uyduSayisi;
  int   saat, dakika, saniye;
};

unsigned long simdi              = 0;
unsigned long sonLoraSensorZamani = 0;
unsigned long sonYazdirma        = 0;
unsigned long ucusBaslangic      = 0;
uint32_t gonderilenPaket = 0;
bool gonderimAktif = true;

////////////////////////////////////
float ikiBasamak(float v) {
  return roundf(v * 100.0f) / 100.0f;
}

void floatToBytes(float deger, uint8_t *hedef) {
  uint8_t kaynak[4];
  memcpy(kaynak, &deger, 4);
  hedef[0] = kaynak[3]; hedef[1] = kaynak[2]; hedef[2] = kaynak[1]; hedef[3] = kaynak[0];
}

// Taşmayı engelleyen dönüşümler
uint16_t u16(float deger) { return (uint16_t)constrain(deger, 0.0f, 65535.0f); }
int16_t  i16(float deger) { return (int16_t)constrain(deger, -32768.0f, 32767.0f); }

////////////////////////////////////
// SANAL VERİ ÜRET (t: saniye, 60 sn'lik döngü)
////////////////////////////////////
uint8_t simVeriUret(float t, SensorVerisi &v) {
  float d = fmodf(t, 60.0f);

  v.irtifa = 3000.0f * (1.0f - fabsf(d - 30.0f) / 30.0f);     // 0 -> 3000 -> 0
  v.hiz    = (d < 30.0f) ? 100.0f : -100.0f;
  v.basinc = 1013.25f * powf(1.0f - (850.0f + v.irtifa) / 44330.0f, 5.2553f);

  v.ivmeX = 0.5f * sinf(t * 2.0f);
  v.ivmeY = 0.5f * cosf(t * 2.0f);
  v.ivmeZ = 20.0f * sinf(t);

  v.aciX = 30.0f * sinf(t / 3.0f);
  v.aciY = 45.0f * sinf(t / 5.0f);
  v.aciZ = fmodf(t * 6.0f, 360.0f) - 180.0f;

  v.enlem  = 39.9334f + 0.0001f * d;     // sabit test koordinatı + küçük kayma
  v.boylam = 32.8597f + 0.0001f * d;
  v.gpsIrtifa = 850.0f + v.irtifa;
  v.uyduSayisi = 9;

  unsigned long sn = (unsigned long)t;
  v.saat   = 12 + (sn / 3600) % 12;
  v.dakika = (sn / 60) % 60;
  v.saniye = sn % 60;

  // Durum bayrakları uçuş evrelerine göre
  uint8_t data1 = 0;
  if (d > 2)    data1 |= (1 << 0);   // kalktı
  if (d > 5)    data1 |= (1 << 1);   // yandı
  if (d > 10)   data1 |= (1 << 2);   // irtifa OK
  if (d > 25)   data1 |= (1 << 3);   // gövde açısı
  if (d > 30)   data1 |= (1 << 4);   // apogee
  if (d > 30.5) data1 |= (1 << 5);   // apogee piro
  if (d > 45)   data1 |= (1 << 6);   // main
  if (d > 45.5) data1 |= (1 << 7);   // main piro
  return data1;
}

////////////////////////////////////
// LORA PAKETİ (UKB-Kod.ino'daki loraSensorGonder ile aynı format)
////////////////////////////////////
void loraSensorGonder(const SensorVerisi &v, uint8_t data1) {
  uint8_t paket[35];
  int idx = 0;

  auto writeUint16 = [&](uint16_t val) {
    paket[idx++] = (val >> 8) & 0xFF;
    paket[idx++] = val & 0xFF;
  };
  auto writeInt16 = [&](int16_t val) {
    paket[idx++] = (val >> 8) & 0xFF;
    paket[idx++] = val & 0xFF;
  };

  paket[idx++] = 0xBC;

  writeUint16(u16(ikiBasamak(v.irtifa) * 10));
  writeInt16 (i16(ikiBasamak(v.hiz)    * 100));
  writeUint16(u16(ikiBasamak(v.basinc) * 10));
  writeInt16 (i16(ikiBasamak(v.ivmeX)  * 100));
  writeInt16 (i16(ikiBasamak(v.ivmeY)  * 100));
  writeInt16 (i16(ikiBasamak(v.ivmeZ)  * 100));
  writeInt16 (i16(ikiBasamak(v.aciX)   * 10));
  writeInt16 (i16(ikiBasamak(v.aciY)   * 10));
  writeInt16 (i16(ikiBasamak(v.aciZ)   * 10));

  floatToBytes(v.enlem,  &paket[idx]); idx += 4;
  floatToBytes(v.boylam, &paket[idx]); idx += 4;

  writeUint16(u16(v.gpsIrtifa));

  paket[idx++] = (uint8_t)v.uyduSayisi;
  paket[idx++] = (uint8_t)v.saat;
  paket[idx++] = (uint8_t)v.dakika;
  paket[idx++] = (uint8_t)v.saniye;
  paket[idx++] = data1;

  uint8_t toplam = 0;
  for (int i = 0; i < idx; i++) toplam += paket[i];
  paket[idx++] = toplam;

  LoRaSerial.write(paket, 35);
}

////////////////////////////////////
void komutIsle() {
  while (Serial.available()) {
    char k = toupper(Serial.read());
    if (k == 'G') {
      gonderimAktif = !gonderimAktif;
      Serial.println(gonderimAktif ? F("[Gönderim devam ediyor]") : F("[Gönderim durduruldu]"));
    } else if (k == 'R') {
      ucusBaslangic = millis();
      Serial.println(F("[Sanal uçuş başa alındı]"));
    }
  }
}

////////////////////////////////////
void setup() {
  Serial.begin(115200);
  unsigned long bekle = millis();
  while (!Serial && millis() - bekle < 3000);

  pinMode(LORA_M0, OUTPUT); digitalWrite(LORA_M0, LOW);   // M0 = M1 = LOW -> normal mod
  pinMode(LORA_M1, OUTPUT); digitalWrite(LORA_M1, LOW);
  pinMode(LORA_AUX, INPUT);
  pinMode(LED_UCUS, OUTPUT); digitalWrite(LED_UCUS, LOW);

  // Modül hazır olana kadar bekle (maks 3 sn)
  unsigned long loraT = millis();
  while (digitalRead(LORA_AUX) == LOW && millis() - loraT < 3000);
  LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX, LORA_TX);

  ucusBaslangic = millis();
  Serial.println(F("ÜLGEN UKB - ROKET TELEMETRİ VERİCİ TESTİ başladı (10 Hz)"));
  Serial.println(F("Komutlar: G = durdur/devam, R = sanal uçuşu başa al"));
}

////////////////////////////////////
void loop() {
  simdi = millis();
  komutIsle();

  if (gonderimAktif && simdi - sonLoraSensorZamani >= 100) {
    sonLoraSensorZamani = simdi;

    SensorVerisi v;
    uint8_t bayrak = simVeriUret((simdi - ucusBaslangic) / 1000.0f, v);
    loraSensorGonder(v, bayrak);
    gonderilenPaket++;

    digitalWrite(LED_UCUS, (gonderilenPaket % 10) < 5);   // gönderirken yanıp söner
  }

  if (simdi - sonYazdirma >= 1000) {
    sonYazdirma = simdi;
    Serial.printf("Gönderilen paket: %lu | AUX: %s\n", (unsigned long)gonderilenPaket,
                  digitalRead(LORA_AUX) ? "HIGH" : "LOW");
  }

  // Yer istasyonundan gelen bir şey olursa ekrana yaz
  while (LoRaSerial.available()) {
    uint8_t b = LoRaSerial.read();
    Serial.printf("[Yerden gelen bayt: 0x%02X]\n", b);
  }
}

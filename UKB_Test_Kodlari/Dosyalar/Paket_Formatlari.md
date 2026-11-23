# ÜLGEN UKB - Paket Formatları

Tüm çok baytlı sayılar **big-endian** (yüksek bayt önce) gönderilir.
`float` değerler `floatToBytes()` ile 4 bayta ters çevrilerek yazılır (big-endian).

## 1) RS232 Komut Paketi (Bilgisayar -> UKB), 5 bayt
| Bayt | Anlamı |
|------|--------|
| 0 | 0xAA |
| 1 | Komut: 0x20 = SİT başlat, 0x22 = SUT başlat, 0x24 = Normal moda dön / durdur |
| 2 | Checksum = (bayt0 + bayt1) & 0xFF |
| 3 | 0x0D |
| 4 | 0x0A |

## 2) RS232 Sensör Paketi (SİT: UKB -> Bilgisayar, SUT: Bilgisayar -> UKB), 36 bayt
| Bayt | Anlamı |
|------|--------|
| 0 | 0xAB |
| 1-4 | İrtifa (float, m) |
| 5-8 | Basınç (float, hPa) |
| 9-12 | İvme X (float, m/s²) |
| 13-16 | İvme Y |
| 17-20 | İvme Z |
| 21-24 | Açı X / Roll (float, derece) |
| 25-28 | Açı Y / Pitch |
| 29-32 | Açı Z / Yaw |
| 33 | Checksum = bayt 0..32 toplamı & 0xFF |
| 34 | 0x0D |
| 35 | 0x0A |

## 3) RS232 Durum Paketi (SUT sırasında UKB -> Bilgisayar), 6 bayt
| Bayt | Anlamı |
|------|--------|
| 0 | 0xAA |
| 1 | Durum bayrakları (aşağıda) |
| 2 | 0x00 |
| 3 | Checksum = bayt 0..2 toplamı & 0xFF |
| 4 | 0x0D |
| 5 | 0x0A |

Durum bayrakları: bit0 kalktı, bit1 yandı, bit2 irtifa OK, bit3 gövde açısı,
bit4 apogee, bit5 apogee piro, bit6 main, bit7 main piro.

## 4) LoRa Telemetri Paketi (UKB -> Yer İstasyonu), 35 bayt
| Bayt | Tür | Anlamı | Ölçek |
|------|-----|--------|-------|
| 0 | u8 | Başlık 0xBC | |
| 1-2 | u16 | İrtifa | /10 (m) |
| 3-4 | i16 | Kalman hız | /100 (m/s) |
| 5-6 | u16 | Basınç | /10 (hPa) |
| 7-8 | i16 | İvme X | /100 |
| 9-10 | i16 | İvme Y | /100 |
| 11-12 | i16 | İvme Z | /100 |
| 13-14 | i16 | Açı X | /10 (derece) |
| 15-16 | i16 | Açı Y | /10 |
| 17-18 | i16 | Açı Z | /10 |
| 19-22 | float | Enlem | |
| 23-26 | float | Boylam | |
| 27-28 | u16 | GPS irtifa | 1 m |
| 29 | u8 | Uydu sayısı | |
| 30 | u8 | Saat (TSİ) | |
| 31 | u8 | Dakika | |
| 32 | u8 | Saniye | |
| 33 | u8 | Durum bayrakları (2. paragraftaki bit sırası) | |
| 34 | u8 | Checksum = bayt 0..33 toplamı & 0xFF | |

## 5) E22 LoRa Konfigürasyon Komutları (M0 = LOW, M1 = HIGH, 9600 baud)
- Oku   : `C1 00 09`
- Yaz   : `C0 00 09 ADDH ADDL NETID REG0 REG1 REG2 REG3 CRYPT_H CRYPT_L`
- REG0  : bit7-5 UART baud, bit4-3 parite, bit2-0 hava hızı
- REG1  : bit7-6 paket boyutu, bit5 RSSI ortam gürültüsü, bit1-0 çıkış gücü
- REG2  : kanal
- REG3  : bit7 RSSI byte, bit6 sabit/şeffaf mod, bit5 röle, bit4 LBT, bit3 WOR, bit2-0 WOR çevrimi

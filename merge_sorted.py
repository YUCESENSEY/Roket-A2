# -*- coding: utf-8 -*-
"""
Merge Sorted Arrays (Sıralı Dizileri Birleştirme)
==================================================
İki önceden sıralanmış listeyi tek bir sıralı listede birleştiren fonksiyon.

Karmaşıklık
-----------
    - Zaman : O(n + m)  -> her eleman en fazla bir kez karşılaştırılır / yazılır.
    - Alan  : O(1)      -> ek (auxiliary) alan yalnızca birkaç işaretli değişken.
                         Birleştirme, daha büyük girdi listesinin üzerine
                         "yerinde" (in-place) yapılır; yeni O(n+m) liste yok.

Nasıl çalışır? (LeetCode 88 / "extra space" tekniği)
-----------------------------------------------------
İki sıralı bloğu doğrudan aynı dizi içinde sondan birleştirmek, ikinci blok
gerçek veri içerdiğinde okunmamış elemanları ezer. Bunu önlemek için:
    1) Daha büyük listeye (a) m adet "dolgu" (0) ekleriz -> a boyutu n+m olur.
    2) a[0..n-1] = orijinal a (sıralı), a[n..n+m-1] = dolgu.
    3) b (ayrı liste) ve a[0..n-1] 'den okuyup, a'nın sonundan (dolgu bölgesi)
       başlayarak en büyük elemanı sona yerleştiririz.
    4) Dolgu bölgesi dizinin sonunda olduğundan, hiçbir okunmamış eleman
       ezilmez. Kalan b elemanları (varsa) başa kopyalanır.
"""


def merge_sorted_inplace(a, b):
    """
    İki sıralı listeyi tek bir sıralı listede birleştirir (in-place).

    Parametreler
    ------------
    a : list  -> İlk sıralı liste (artan sırada).
    b : list  -> İkinci sıralı liste (artan sırada).

    Dönüş
    -----
    list : Birleştirilmiş, artan sırada tek liste.
           (Daha büyük olan girdi listesi yerinde güncellenir ve döndürülür.)

    Not: Girdi listelerinden biri (daha büyüğü) yerinde değiştirilir.
         Değer döndürmek istemeyen çağrıcı `merge_sorted_inplace(a, b)`
         ifadesinin sonucunu kullanabilir.
    """

    # --- Adım 0: a'yı her zaman "daha büyük" liste yap -----------------------
    # Daha büyük listeye birleştirme yaparsak taşınan eleman sayısı azalır
    # ve döndürülen liste de daha büyük olanıdır.
    if len(a) < len(b):
        a, b = b, a

    n = len(a)   # a'nın (birleştirme öncesi) uzunluğu
    m = len(b)   # b'nin uzunluğu

    # --- Adım 1: a'ya m adet dolgu (0) ekle (in-place genişletme) ------------
    # Artık a boyutu n+m:  a[0..n-1] = orijinal a (sıralı), a[n..n+m-1] = 0.
    # Bu dolgu bölgesi, birleştirme sırasında yazma alanı olarak kullanılır.
    # NOT: Döngüyle tek tek ekliyoruz; böylece geçici bir [0]*m listesi
    #      oluşturmayız ve O(1) ek (auxiliary) alan garantisi tam korunur.
    for _ in range(m):
        a.append(0)

    # --- Adım 2: işaretli başlangıç değerleri --------------------------------
    i = n - 1        # a'nın (orijinal) son elemanı
    j = m - 1        # b'nin son elemanı
    k = n + m - 1    # yazma konumu (dizinin en sonu)

    # --- Adım 3: sondan iki işaretli birleştirme -----------------------------
    # Her adımda a[i] ile b[j] 'yi karşılaştırıp BÜYÜK olanı a[k] konumuna
    # yerleştiririz. k her zaman dolgu bölgesinde (veya onun solunda) ilerler;
    # b ayrı bir liste olduğundan okunmamış b elemanı asla ezilmez.
    while i >= 0 and j >= 0:
        if a[i] >= b[j]:
            a[k] = a[i]   # a'nın en büyük kalan elemanı sona gider
            i -= 1
        else:
            a[k] = b[j]   # b'nin en büyük kalan elemanı sona gider
            j -= 1
        k -= 1

    # --- Adım 4: b'nin kalan elemanlarını (varsa) başa kopyala ---------------
    # a bittiyse (i < 0) ama b hâlâ eleman kaldıysa, o elemanlar zaten
    # en küçük olanlardır ve dizinin başına yerleştirilmelidir.
    while j >= 0:
        a[k] = b[j]
        j -= 1
        k -= 1

    # (a'nın kalan elemanları zaten sıralı ve dizinin başında olduğundan
    #  ek kopyalama gerekmez -> O(1) ek alan korunur.)

    return a


# ===========================================================================
# TEST SENARYOLARI
# ===========================================================================
def _check(ad, a, b, beklenen):
    """Yardımcı: sonucu beklenenle karşılaştırıp okunabilir çıktı üretir."""
    sonuc = merge_sorted_inplace(list(a), list(b))
    durum = "GEÇTİ" if sonuc == beklenen else "BAŞARISIZ"
    print(f"[{ad}]  a={a}   b={b}")
    print(f"        beklenen : {beklenen}")
    print(f"        bulunan  : {sonuc}")
    print(f"        durum    : {durum}\n")
    return sonuc == beklenen


def run_tests():
    print("=" * 62)
    print("TEST SENARYOLARI")
    print("=" * 62)

    # Senaryo 1: Farklı uzunluklar, iç içe geçen (interleaved) değerler
    ok1 = _check(
        "Senaryo 1",
        a=[1, 3, 5, 7],
        b=[2, 4, 6, 8, 10],
        beklenen=[1, 2, 3, 4, 5, 6, 7, 8, 10],
    )

    # Senaryo 2: Aralıkların örtüşmediği (ayık) iki liste
    ok2 = _check(
        "Senaryo 2",
        a=[1, 2, 3],
        b=[10, 20, 30, 40],
        beklenen=[1, 2, 3, 10, 20, 30, 40],
    )

    print("=" * 62)
    print("SONUÇ:", "TÜM TESTLER GEÇTİ" if (ok1 and ok2) else "BAŞARISIZ TEST VAR")
    print("=" * 62)


if __name__ == "__main__":
    run_tests()

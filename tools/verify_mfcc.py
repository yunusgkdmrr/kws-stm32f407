"""
Kartın hesapladığı MFCC ile PC'nin hesapladığı MFCC'yi karşılaştırır.

Kullanım:
    python3 mfcc_dogrula.py <port> <mfcc_consts.h yolu>
Örnek:
    python3 mfcc_dogrula.py /dev/tty.usbserial-1230 ~/yunus/stmproject/mic_test/Core/Inc/mfcc_consts.h

Karttaki mavi düğmeye her bastığında:
  kart 1 sn kaydeder -> kendi MFCC'sini hesaplar -> sesi ve MFCC'yi gönderir
  bu betik aynı sesin MFCC'sini PC'de hesaplar ve iki sonucu karşılaştırır.
PC tarafındaki hesap, Colab'da TensorFlow ile 0,000003 fark verdiği doğrulanan
mfcc_ref() ile aynıdır ve AYNI sabitleri (mfcc_consts.h) kullanır.
"""
import pathlib
import re
import sys
import wave

import numpy as np
import serial

PORT = sys.argv[1]
CONSTS = pathlib.Path(sys.argv[2]).expanduser()


# ---- mfcc_consts.h içindeki dizileri oku --------------------------------------
def load_consts(path):
    txt = path.read_text()
    arrs = {}
    for name, body in re.findall(r"static const float (\w+)\[\d+\] = \{(.*?)\};", txt, re.S):
        arrs[name] = np.array(re.findall(r"(-?\d\.\d+e[+-]\d+)f", body), dtype=np.float32)
    return arrs


C = load_consts(CONSTS)
WIN = C["MFCC_WINDOW"]
MELW = C["MFCC_MELW"].reshape(257, 40)
DCT = C["MFCC_DCT"].reshape(40, 10)


def mfcc_ref(x):
    out = np.zeros((49, 10), np.float32)
    for f in range(49):
        seg = x[f * 320: f * 320 + 640] * WIN
        spec = np.abs(np.fft.rfft(seg, n=1024))[:257]
        out[f] = np.log(spec @ MELW + 1e-6) @ DCT
    return out


# ---- Karttan veri al ve karşılaştır -------------------------------------------
ser = serial.Serial(PORT, 115200, timeout=10)
print("Hazir. Karttaki mavi dugmeye bas ve bir kelime soyle.\n")
n = 0
while True:
    line = ser.readline().decode(errors="replace").strip()
    if line.startswith(("TAHMIN", "EMIN", "MODEL", "DINLIYOR", "dinliyor", "DEBUG", "taban", "[mic_test")):
        print("kart>", line)                            # kartın tahminini göster
        continue
    if not line.startswith("BEGIN"):
        continue

    count = int(line.split()[1])
    audio = ser.read(count * 2)
    line = ser.readline().decode(errors="replace").strip()
    while not line.startswith("MFCC"):                 # aradaki boş satırları atla
        line = ser.readline().decode(errors="replace").strip()
    nm = int(line.split()[1])
    board = np.frombuffer(ser.read(nm * 4), dtype="<f4").reshape(49, 10)
    line = ser.readline().decode(errors="replace").strip()
    while not line.startswith("END"):
        line = ser.readline().decode(errors="replace").strip()
    sure = line.split("=")[-1]

    n += 1
    x = np.frombuffer(audio, dtype="<i2").astype(np.float32) / 32768.0
    pc = mfcc_ref(x)
    diff = np.abs(board - pc)

    with wave.open(f"dogrula_{n:02d}.wav", "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000); w.writeframes(audio)
    np.save(f"dogrula_{n:02d}_kart_mfcc.npy", board)

    print(f"[{n}] kartta 49 pencere MFCC suresi: {sure} us")
    print(f"    MFCC araligi (PC): {pc.min():.2f} .. {pc.max():.2f}")
    print(f"    en buyuk fark: {diff.max():.6f}   ortalama fark: {diff.mean():.6f}")
    print("    katsayi basina en buyuk fark:", " ".join(f"{v:.4f}" for v in diff.max(axis=0)))
    print("    SONUC:", "AYNI (basarili)" if diff.max() < 0.01 else "FARK VAR, incelemek gerek")
    print()

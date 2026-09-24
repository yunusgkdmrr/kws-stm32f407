import array, pathlib, sys, wave
import serial
PORT, WORD = sys.argv[1], sys.argv[2]
out = pathlib.Path(WORD); out.mkdir(exist_ok=True)
n = max([int(p.stem.split("_")[-1]) for p in out.glob(f"{WORD}_*.wav")], default=0)
ser = serial.Serial(PORT, 115200, timeout=1)
print(f"Otomatik kayit -> {out.resolve()}. Serbestce konus (KOMUT KELIMELERINI SOYLEME). Cikis: Ctrl + C\n")
while True:
    line = ser.readline().decode(errors="replace").strip()
    if not line.startswith("BEGIN"):
        continue
    count = int(line.split()[1]); ser.timeout = 10
    data = ser.read(count * 2); ser.timeout = 1
    if len(data) != count * 2:
        continue
    peak = max(abs(s) for s in array.array("h", data))
    if peak < 3000:
        continue
    n += 1
    with wave.open(str(out / f"{WORD}_{n:03d}.wav"), "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000); w.writeframes(data)
    print(f"[{n:3d}] kaydedildi (tepe = {peak})")

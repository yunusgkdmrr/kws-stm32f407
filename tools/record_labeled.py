import array, pathlib, subprocess, sys, wave
import serial

PORT, WORD = sys.argv[1], sys.argv[2]
out_dir = pathlib.Path(WORD)
out_dir.mkdir(exist_ok=True)
nums = [int(p.stem.split("_")[-1]) for p in out_dir.glob(f"{WORD}_*.wav")]
n = max(nums, default=0)
ser = serial.Serial(PORT, 115200, timeout=1)
print(f"'{WORD}' kaydediliyor -> {out_dir.resolve()}  (su an {len(nums)} kayit var)")
print("Karttaki MAVI dugmeye bas, TURUNCU LED yaninca kelimeyi soyle. Cikis: Ctrl + C\n")

while True:
    line = ser.readline().decode(errors="replace").strip()
    if line.startswith("ZAMAN ASIMI"):
        print("    konusma algilanmadi, dugmeye tekrar bas\n")
        continue
    if not line.startswith("BEGIN"):
        continue
    count = int(line.split()[1])
    ser.timeout = 10
    data = ser.read(count * 2)
    ser.timeout = 1
    if len(data) != count * 2:
        print("    eksik veri, tekrar dene\n")
        continue
    peak = max(abs(s) for s in array.array("h", data))
    tmp = pathlib.Path("/tmp/kws_onizleme.wav")
    with wave.open(str(tmp), "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000); w.writeframes(data)
    subprocess.run(["afplay", str(tmp)])
    cevap = input(f"[{n + 1:2d}] tepe = {peak}  -> Enter = kaydet, s = sil: ")
    ser.reset_input_buffer()
    if cevap.strip().lower() == "s":
        print("    silindi\n")
        continue
    n += 1
    fn = out_dir / f"{WORD}_{n:02d}.wav"
    tmp.replace(fn)
    print(f"    kaydedildi: {fn.name}\n")

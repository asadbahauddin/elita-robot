# ELITA - Robot Pemotong Rumput Otomatis

## Komponen

- Arduino Uno
- ESP32 DevKit v1
- Driver motor L298N
- Servo MG996R
- Relay cutter

## Isi Folder

```
python/     - program dashboard laptop (main.py) + requirements.txt
firmware/
  Arduino/  - firmware Arduino Uno (Arduino.ino)
  Esp/      - firmware ESP32 (Esp.ino)
docs/       - skematik rangkaian
CARA_INSTALL.txt - panduan install & menjalankan program Python
```

## Upload Firmware

1. Install [Arduino IDE](https://www.arduino.cc/en/software).
2. **Arduino Uno**: buka `firmware/Arduino/Arduino.ino`, pilih board **"Arduino Uno"**,
   pilih port, lalu Upload.
3. **ESP32**:
   - Pasang library **"WebSockets" by Markus Sattler** lewat
     *Sketch > Include Library > Manage Libraries*.
   - Buka `firmware/Esp/Esp.ino`, pilih board **"ESP32 Dev Module"**,
     pilih port, lalu Upload.

## Menjalankan Program Python

Lihat [CARA_INSTALL.txt](CARA_INSTALL.txt).

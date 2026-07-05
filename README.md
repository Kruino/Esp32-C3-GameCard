# ESP32-C3 Steam E-Paper Display

Use an **ESP32-C3** with the **WeAct Studio 2.9" Black & White E-Paper Module** to display Steam game artwork. The display receives images over USB Serial, remembers the associated Steam App ID, and can automatically request that the PC launches the game when the device is plugged in.

> **Panel:** DEPG0290BS (SSD1680 controller, 128 × 296)  
> **Library:** GxEPD2 (`GxEPD2_290_BS`)

---

## Features

- 🖼️ Receives images over USB Serial and displays them instantly.
- 💾 Stores the associated Steam App ID in flash (NVS).
- ⚡ Requests the PC to launch the stored Steam game after the board is plugged in.
- 🔋 E-paper image survives resets and power loss (the display is not cleared during boot).
- 📡 Simple line-based serial protocol.
- 🔌 Works over a single USB connection.

---

## Hardware

### Supported Display

- WeAct Studio 2.9" Black & White E-Paper Module
- Panel: **DEPG0290BS**
- Controller: **SSD1680**
- Resolution: **128 × 296**

Module reference:

https://github.com/WeActStudio/WeActStudio.EpaperModule

---

## Wiring

Connect the WeAct 2.9" B/W module to the ESP32-C3.

> **⚠️ Use 3.3 V only. Never connect the display to 5 V.**

| Module Pin | ESP32-C3 |
|------------|-----------|
| BUSY | GPIO3 |
| RST (RES) | GPIO2 |
| DC | GPIO1 |
| CS | GPIO7 |
| SCK (SCL/CLK) | GPIO4 |
| MOSI (SDA/DIN) | GPIO6 |
| GND | GND |
| VCC | 3V3 |

![ESP32-C3 Wiring Guide](docs/wiring.png)

> The GPIO assignments are defined near the top of the firmware. If you wire the display differently, update the pin definitions and re-flash the firmware.

---

## How It Works

When the PC wants to display a Steam game:

1. Convert the artwork into a 1-bit image.
2. Send it over USB Serial.
3. The ESP32 immediately draws the image.
4. The supplied Steam App ID is saved in flash memory.
5. The next time the board is plugged in, the PC asks for the stored ID.
6. The firmware replies with a launch request once, allowing the PC application to open:

```
steam://launch/<appid>
```

The displayed artwork remains visible even if the ESP32 resets, because e-paper retains its image without power.

---

## Serial Protocol

**Baud Rate:** `115200`

### Host → Device

#### Display an Image

```
IMG:<width>,<height>,<base64>\n
```

The image format is:

- 296 × 128 (landscape) or 128 × 296 (portrait)
- Row-major
- Top-left origin
- `ceil(width / 8)` bytes per row
- MSB = left-most pixel
- `1 = Black`
- `0 = White`

---

#### Store a Steam App ID

```
SETID:<appid>\n
```

An empty value clears the stored ID.

Example:

```
SETID:730
```

---

#### Retrieve the Stored ID

```
GETID
```

---

## Device → Host

### Device Ready

```
READY
```

Sent once after boot.

---

### Launch Request

```
LAUNCH:<appid>
```

Returned only the **first** time `GETID` is called after the board is plugged in (provided an App ID is stored).

The PC should launch:

```
steam://launch/<appid>
```

---

### Current Stored ID

```
CURID:<appid>
```

Returned on subsequent `GETID` requests.

If no ID is stored:

```
CURID:
```

---

### Success Responses

```
OK
```

Image drawn successfully.

```
IDOK
```

Steam App ID stored successfully.

---

### Error

```
ERR <reason>
```

Returned if an operation fails.

---

## Why Launch Requests Use `GETID`

The firmware does **not** send an unsolicited launch request immediately after boot.

When the ESP32 is plugged in, the USB CDC serial port is not yet open while the device is enumerating. Any boot-time serial output would likely be lost.

Instead, the host application opens the serial port and sends:

```
GETID
```

If this is the first request after power-up and an App ID is stored, the firmware responds with:

```
LAUNCH:<appid>
```

This guarantees the host never misses the launch request.

---

## Libraries

- Arduino ESP32
- GxEPD2
- Preferences (ESP32 NVS)

---

## License

See the repository license for details.

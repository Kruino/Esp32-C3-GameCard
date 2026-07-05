/*
 * ESP32-C3 + WeAct Studio 2.9" B/W e-paper module
 * Panel: DEPG0290BS (SSD1680 controller, 128 x 296)
 * Display library: GxEPD2 (class GxEPD2_290_BS)
 *
 * Module ref: https://github.com/WeActStudio/WeActStudio.EpaperModule
 *
 * Behaviour:
 *   - Receives an image over the USB serial line and draws it immediately.
 *   - The image bytes are not kept, but the associated Steam app id IS stored
 *     in flash (NVS) so the board can ask the PC to launch the game on plug-in.
 *   - On boot the panel is left as-is (e-paper retains the last image), so a
 *     reset / site refresh does not wipe the shown card.
 *
 * Serial protocol (115200 baud, line based):
 *   Host -> device:
 *     "IMG:" <W> "," <H> "," <base64> "\n"   draw a 1-bpp frame (see packing below)
 *     "IMG4:" <W> "," <H> "," <base64> "\n"  draw a 4-level grayscale frame (see below)
 *     "SETID:" <appid> "\n"                  remember the Steam app id (empty clears)
 *     "GETID" "\n"                           ask for the stored app id
 *     IMG packing: 296x128 landscape / 128x296 portrait, row-major, top-left origin,
 *       ceil(W/8) bytes per row, MSB = leftmost pixel, bit 1 = BLACK, 0 = WHITE.
 *     IMG4 packing: 128x296 portrait only, row-major, top-left origin, 2 bits per
 *       pixel, 4 pixels per byte, MSB-first (leftmost pixel = bits 7..6). Level
 *       codes: 3=white, 2=light gray, 1=dark gray, 0=black. ceil(W/4) bytes/row.
 *       Rendered via the SSD1680 4-gray waveform (custom LUT), bypassing GxEPD2.
 *   Device -> host:
 *     "READY\n"          once after boot
 *     "LAUNCH:" <id>     reply to GETID the FIRST time after a plug-in, if an app id
 *                        is stored -> host opens steam://launch/<id>. Fires once per boot.
 *     "CURID:" <id>      reply to GETID otherwise (id may be empty; no launch)
 *     "OK\n"             frame drawn
 *     "IDOK\n"           app id stored
 *     "ERR <reason>\n"   on failure
 *
 * The launch is delivered via GETID (a pull) rather than an unsolicited boot
 * print, because on plug-in the host cannot open the USB CDC port until after
 * the device enumerates -- a boot-time print would be missed.
 *
 * ---- Wiring (module pin -> ESP32-C3 GPIO) ----
 *   BUSY -> GPIO3
 *   RST  -> GPIO2   (module label: RES)
 *   DC   -> GPIO1
 *   CS   -> GPIO7
 *   SCK  -> GPIO4   (module label: SCL / CLK)
 *   MOSI -> GPIO6   (module label: SDA / DIN)
 *   GND  -> GND
 *   VCC  -> 3V3
 */

#include <Arduino.h>
#include <Preferences.h>
#include <mbedtls/base64.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>

// ---- Pin assignments ----
#define EPD_CS   7
#define EPD_DC   1
#define EPD_RST  2
#define EPD_BUSY 3
#define EPD_SCK  4
#define EPD_MOSI 6

// ---- Panel geometry (landscape) ----
static const int    PANEL_W   = 296;
static const int    PANEL_H   = 128;
static const int    ROW_BYTES = (PANEL_W + 7) / 8;             // 37
static const size_t IMG_BYTES = (size_t)ROW_BYTES * PANEL_H;   // 4736

// ---- 4-gray geometry (portrait, native panel) ----
static const int    GRAY_W       = 128;
static const int    GRAY_H       = 296;
static const int    GRAY_ROW     = (GRAY_W + 3) / 4;                 // 32 bytes/row (4 px/byte)
static const size_t GRAY_BYTES   = (size_t)GRAY_ROW * GRAY_H;        // 9472 (2 bpp)
static const size_t GRAY_PLANE   = (size_t)(GRAY_W / 8) * GRAY_H;    // 4736 bytes per bit-plane

// The decode buffer must hold the largest payload (the 2-bpp gray frame).
static const size_t DECODE_MAX = GRAY_BYTES; // 9472 >= IMG_BYTES

// base64 of DECODE_MAX, plus the command prefix and a little slack.
static const size_t LINE_BUF_MAX = ((DECODE_MAX + 2) / 3) * 4 + 32; // ~12664

// WeAct 2.9" B/W panel: DEPG0290BS -> GxEPD2_290_BS (128 x 296)
GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(
    GxEPD2_290_BS(/*CS=*/ EPD_CS, /*DC=*/ EPD_DC, /*RST=*/ EPD_RST, /*BUSY=*/ EPD_BUSY));

// Single reusable frame buffer. Received image data is decoded in, drawn, and
// then left to be overwritten by the next frame -- never saved anywhere.
// Sized for the larger 2-bpp grayscale frame; the 1-bpp path uses a prefix.
static uint8_t frameBuf[DECODE_MAX];

// SSD1680 4-level grayscale waveform LUT (159 bytes).
// Source: Waveshare e-Paper 2.9" V2 (SSD1680, 128x296) reference driver, Gray4[].
// https://github.com/waveshareteam/e-Paper (Arduino/epd2in9_V2). The DEPG0290BS
// shares the SSD1680 controller and panel size; verify tone/ghosting on hardware.
static const uint8_t GRAY4_LUT[159] = {
  0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L0
  0x20, 0x60, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L1
  0x28, 0x60, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L2
  0x2A, 0x60, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L3
  0x00, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VS L4
  0x00, 0x02, 0x00, 0x05, 0x14, 0x00, 0x00,                               // Group0
  0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x01,                               // Group1
  0x00, 0x02, 0x00, 0x05, 0x14, 0x00, 0x00,                               // Group2
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // Group3
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // Group4
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // Group5
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // Group6
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // Group7
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // Group8
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // Group9
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // Group10
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                               // Group11
  0x24, 0x22, 0x22, 0x22, 0x23, 0x32, 0x00, 0x00, 0x00,                   // FR, XON
  0x22, 0x17, 0x41, 0xAE, 0x32, 0x28,                                     // EOPT VGH VSH1 VSH2 VSL VCOM
};

// Serial line accumulator.
static char   lineBuf[LINE_BUF_MAX];
static size_t lineLen = 0;
static bool   lineOverflow = false;

// Persistent settings (Steam app id + a flag that something has been drawn).
static Preferences prefs;
static String      storedAppId;
static bool        hasShown = false;

// True from boot until the first GETID consumes it: "this is a fresh plug-in,
// launch the stored game once".
static bool        launchPending = false;

// True once a raw 4-gray frame has taken over the panel from GxEPD2 (see below).
static bool        grayActive = false;


// Drawing helpers

// Draw a text message (used for the boot / status screen).
static void showMessage(const char *line1, const char *line2)
{
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);

  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(8, 40);
    display.print(line1);
    if (line2)
    {
      display.setCursor(8, 70);
      display.print(line2);
    }
  } while (display.nextPage());
}

// Push the packed 1-bpp buffer in frameBuf straight to the panel, using the
// orientation implied by the requested W x H.
static void drawFrameBuffer(int w, int h)
{
  const int rowBytes = (w + 7) / 8;
  // If the raw grayscale path last drove the panel (and left it in deep sleep),
  // re-init GxEPD2 so it hardware-resets and reloads its own B/W waveform.
  if (grayActive)
  {
    display.init(115200);
    grayActive = false;
  }
  display.setRotation(w > h ? 1 : 0); // landscape vs portrait
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    for (int y = 0; y < h; y++)
    {
      const uint8_t *row = &frameBuf[(size_t)y * rowBytes];
      for (int x = 0; x < w; x++)
      {
        // bit 1 = black, MSB is the leftmost pixel
        if (row[x >> 3] & (0x80 >> (x & 7)))
          display.drawPixel(x, y, GxEPD_BLACK);
      }
    }
  } while (display.nextPage());
}

// 4-level grayscale (raw SSD1680)
//
// GxEPD2 only drives this panel in 1-bpp black/white, so the grayscale path
// talks to the SSD1680 directly over the same SPI bus and control pins, using a
// custom waveform LUT and the controller's two-bit-plane update. After a gray
// frame the panel is left in deep sleep; the next 1-bpp frame re-inits GxEPD2
// (see drawFrameBuffer) to recover it.

static const uint32_t EPD_SPI_HZ = 4000000;

static inline void epdCmd(uint8_t c)
{
  digitalWrite(EPD_DC, LOW);
  digitalWrite(EPD_CS, LOW);
  SPI.transfer(c);
  digitalWrite(EPD_CS, HIGH);
}

static inline void epdData(uint8_t d)
{
  digitalWrite(EPD_DC, HIGH);
  digitalWrite(EPD_CS, LOW);
  SPI.transfer(d);
  digitalWrite(EPD_CS, HIGH);
}

static void epdWaitIdle()
{
  // SSD1680 BUSY is HIGH while the controller is busy.
  while (digitalRead(EPD_BUSY) == HIGH)
    delay(1);
}

static void epdHardReset()
{
  digitalWrite(EPD_RST, HIGH);
  delay(20);
  digitalWrite(EPD_RST, LOW);
  delay(5);
  digitalWrite(EPD_RST, HIGH);
  delay(20);
}

// Load the 4-gray waveform LUT and its voltage/VCOM tail (bytes 153..158).
static void epdLoadGrayLut()
{
  epdCmd(0x32);
  for (int i = 0; i < 153; i++)
    epdData(GRAY4_LUT[i]);
  epdWaitIdle();

  epdCmd(0x3F); epdData(GRAY4_LUT[153]);
  epdCmd(0x03); epdData(GRAY4_LUT[154]);              // gate voltage
  epdCmd(0x04); epdData(GRAY4_LUT[155]);              // VSH1
                epdData(GRAY4_LUT[156]);              // VSH2
                epdData(GRAY4_LUT[157]);              // VSL
  epdCmd(0x2C); epdData(GRAY4_LUT[158]);              // VCOM
}

// Given a 2-bit level code (3=white .. 0=black) return the bit contributed to
// each of the two SSD1680 planes, matching Waveshare's Display4Gray mapping:
//   3(white): p1=0 p2=0   0(black): p1=1 p2=1
//   2(lgray): p1=1 p2=0   1(dgray): p1=0 p2=1
static inline uint8_t grayPlane1Bit(uint8_t code) { return (code == 0 || code == 2) ? 1 : 0; }
static inline uint8_t grayPlane2Bit(uint8_t code) { return (code == 0 || code == 1) ? 1 : 0; }

// Send one bit-plane (0x24 or 0x26) built from the 2-bpp frameBuf.
static void epdSendGrayPlane(uint8_t cmd, bool plane2)
{
  epdCmd(cmd);
  // 4736 output bytes: each packs 8 pixels taken from 2 input bytes (8 * 2bpp).
  for (size_t i = 0; i < GRAY_PLANE; i++)
  {
    uint8_t out = 0;
    for (int j = 0; j < 2; j++)
    {
      uint8_t in = frameBuf[i * 2 + j];
      for (int k = 0; k < 4; k++)
      {
        uint8_t code = (in >> 6) & 0x03;             // top pixel first
        uint8_t bit  = plane2 ? grayPlane2Bit(code) : grayPlane1Bit(code);
        out = (out << 1) | bit;
        in <<= 2;
      }
    }
    epdData(out);
  }
}

// Render the 2-bpp frame currently in frameBuf as a 4-gray image.
static void drawGrayFrame()
{
  SPI.beginTransaction(SPISettings(EPD_SPI_HZ, MSBFIRST, SPI_MODE0));

  epdHardReset();
  epdWaitIdle();
  epdCmd(0x12);                                       // SWRESET
  epdWaitIdle();

  epdCmd(0x01); epdData(0x27); epdData(0x01); epdData(0x00); // driver output: 296 gates
  epdCmd(0x11); epdData(0x03);                        // data entry: X+ Y+

  // RAM window: full panel, x bytes 1..16 (128 px), y 0..295. The SSD1680 maps
  // this panel's visible columns to RAM byte 1 onward (byte 0 is off-screen), so
  // the window and pointer start at byte 1 -- matching Waveshare's driver. Using
  // byte 0 shifts the image left and leaves the right column unwritten (black).
  epdCmd(0x44); epdData(0x01); epdData(0x10);
  epdCmd(0x45); epdData(0x00); epdData(0x00); epdData(0x27); epdData(0x01);

  epdCmd(0x3C); epdData(0x04);                        // border waveform

  epdCmd(0x4E); epdData(0x01);                        // RAM x pointer
  epdCmd(0x4F); epdData(0x00); epdData(0x00);         // RAM y pointer
  epdWaitIdle();

  epdLoadGrayLut();

  epdSendGrayPlane(0x24, false);                      // plane 1
  epdSendGrayPlane(0x26, true);                       // plane 2

  epdCmd(0x22); epdData(0xC7);                        // display update (with custom LUT)
  epdCmd(0x20);
  epdWaitIdle();

  epdCmd(0x10); epdData(0x01);                        // deep sleep mode 1
  delay(20);

  SPI.endTransaction();
  grayActive = true;
}

// Serial protocol

// Handle one complete "IMG:<W>,<H>,<base64>" payload (after the "IMG:").
static void handleImageLine(const char *payload)
{
  // Parse "W,H," prefix.
  char *c1 = strchr((char *)payload, ',');
  if (!c1)
  {
    Serial.println("ERR bad header");
    return;
  }
  char *c2 = strchr(c1 + 1, ',');
  if (!c2)
  {
    Serial.println("ERR bad header");
    return;
  }

  int w = atoi(payload);
  int h = atoi(c1 + 1);
  const char *b64 = c2 + 1;
  size_t b64Len = strlen(b64);

  if (w <= 0 || h <= 0)
  {
    Serial.println("ERR bad dimensions");
    return;
  }
  const size_t expected = (size_t)((w + 7) / 8) * h;
  if (expected == 0 || expected > IMG_BYTES)
  {
    Serial.println("ERR unsupported size");
    return;
  }

  size_t decoded = 0;
  int rc = mbedtls_base64_decode(
      frameBuf, IMG_BYTES, &decoded,
      reinterpret_cast<const unsigned char *>(b64), b64Len);

  if (rc != 0)
  {
    Serial.println("ERR invalid base64");
    return;
  }
  if (decoded != expected)
  {
    Serial.print("ERR bad size: got ");
    Serial.print(decoded);
    Serial.print(" expected ");
    Serial.println(expected);
    return;
  }

  drawFrameBuffer(w, h);
  display.hibernate();

  if (!hasShown)
  {
    hasShown = true;
    prefs.putBool("shown", true); 
  }
  Serial.println("OK");
}

// Handle one complete "IMG4:<W>,<H>,<base64>" payload (after the "IMG4:").
// The base64 carries a 2-bpp, 4-px-per-byte, 128x296 portrait frame.
static void handleImage4Line(const char *payload)
{
  char *c1 = strchr((char *)payload, ',');
  if (!c1) { Serial.println("ERR bad header"); return; }
  char *c2 = strchr(c1 + 1, ',');
  if (!c2) { Serial.println("ERR bad header"); return; }

  int w = atoi(payload);
  int h = atoi(c1 + 1);
  const char *b64 = c2 + 1;

  if (w != GRAY_W || h != GRAY_H)
  {
    Serial.println("ERR gray needs 128x296");
    return;
  }

  size_t decoded = 0;
  int rc = mbedtls_base64_decode(
      frameBuf, sizeof(frameBuf), &decoded,
      reinterpret_cast<const unsigned char *>(b64), strlen(b64));

  if (rc != 0) { Serial.println("ERR invalid base64"); return; }
  if (decoded != GRAY_BYTES)
  {
    Serial.print("ERR bad size: got ");
    Serial.print(decoded);
    Serial.print(" expected ");
    Serial.println(GRAY_BYTES);
    return;
  }

  drawGrayFrame();

  if (!hasShown)
  {
    hasShown = true;
    prefs.putBool("shown", true);
  }
  Serial.println("OK");
}

// Store (or clear) the Steam app id in flash.
static void handleSetId(const char *id)
{
  storedAppId = id;
  storedAppId.trim();
  prefs.putString("appid", storedAppId);
  Serial.println("IDOK");
}

// Dispatch a full received line (newline stripped).
static void processLine()
{
  if (lineOverflow)
  {
    Serial.println("ERR line too long");
    return;
  }
  lineBuf[lineLen] = '\0';

  if (strncmp(lineBuf, "IMG4:", 5) == 0)
    handleImage4Line(lineBuf + 5);
  else if (strncmp(lineBuf, "IMG:", 4) == 0)
    handleImageLine(lineBuf + 4);
  else if (strncmp(lineBuf, "SETID:", 6) == 0)
    handleSetId(lineBuf + 6);
  else if (strcmp(lineBuf, "GETID") == 0)
  {
    // First poll after a plug-in requests a launch; later polls just report.
    if (launchPending && storedAppId.length())
    {
      launchPending = false;
      Serial.println("LAUNCH:" + storedAppId);
    }
    else
    {
      Serial.println("CURID:" + storedAppId);
    }
  }
  else if (lineLen > 0)
    Serial.println("ERR unknown command");
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  // Route hardware SPI to the wired pins (ESP32-C3 supports remapping).
  SPI.begin(EPD_SCK, /*MISO=*/ -1, EPD_MOSI, EPD_CS);
  display.init(115200);

  // Load persisted state.
  prefs.begin("epaper", false);
  storedAppId = prefs.getString("appid", "");
  hasShown    = prefs.getBool("shown", false);

  // Only draw the ready screen on a truly fresh device; otherwise the e-paper
  // already shows the last card and we must not wipe it on reset / replug.
  if (!hasShown)
    showMessage("Ready.", "Send an image over serial.");

  // Arm a one-shot launch for this boot; the host claims it via GETID.
  launchPending = storedAppId.length() > 0;

  Serial.println("READY");
}

void loop()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\r')
      continue;
    if (c == '\n')
    {
      processLine();
      lineLen = 0;
      lineOverflow = false;
      continue;
    }
    if (lineLen < LINE_BUF_MAX - 1)
      lineBuf[lineLen++] = c;
    else
      lineOverflow = true;
  }
}

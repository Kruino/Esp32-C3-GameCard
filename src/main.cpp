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
 *     "IMG:" <W> "," <H> "," <base64> "\n"   draw a frame (see packing below)
 *     "SETID:" <appid> "\n"                  remember the Steam app id (empty clears)
 *     "GETID" "\n"                           ask for the stored app id
 *     Packing: 296x128 landscape / 128x296 portrait, row-major, top-left origin,
 *       ceil(W/8) bytes per row, MSB = leftmost pixel, bit 1 = BLACK, 0 = WHITE.
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

// base64 of IMG_BYTES, plus the "IMG:" prefix and a little slack.
static const size_t LINE_BUF_MAX = ((IMG_BYTES + 2) / 3) * 4 + 16; // ~6332

// WeAct 2.9" B/W panel: DEPG0290BS -> GxEPD2_290_BS (128 x 296)
GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(
    GxEPD2_290_BS(/*CS=*/ EPD_CS, /*DC=*/ EPD_DC, /*RST=*/ EPD_RST, /*BUSY=*/ EPD_BUSY));

// Single reusable frame buffer. Received image data is decoded in, drawn, and
// then left to be overwritten by the next frame -- never saved anywhere.
static uint8_t frameBuf[IMG_BYTES];

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

  if (strncmp(lineBuf, "IMG:", 4) == 0)
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

#include <Adafruit_SSD1306.h>
#include <RF24.h>

#include "../lib/commands.hpp"
#include "../lib/io.hpp"
#include "../lib/panic.hpp"

using namespace panic;

#define PIN_PANIC 4

#define PIN_NRF_CE 9
#define PIN_NRF_SC 10
// #define PIN_NRF_.. 11, 12, 13

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define FONT_WIDTH 6
#define FONT_HEIGHT 8
#define PAD_X 3

#define STATUS_EXPIRE_MS 2000

RF24 nrf24(PIN_NRF_CE, PIN_NRF_SC);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

static char PANIC_BUFFER[16];
static millis_t panic_time = 0;
static void panic_snprintf(const char* kind, uint8_t code) {
  snprintf(PANIC_BUFFER, sizeof(PANIC_BUFFER), "%s: 0x%x", kind, code);
  panic_time = millis();
}

static void panic_draw() {
  display.setCursor(PAD_X, SCREEN_HEIGHT - 1 * FONT_HEIGHT);
  display.println(PANIC_BUFFER);
}

void setup() {
  analogReference(INTERNAL);

  Serial.begin(19200);
  panic_init(PIN_PANIC);
  panic::init_printer([](const char* kind, uint8_t code) {
    panic_snprintf(kind, code);
    Serial.println(PANIC_BUFFER);
  });

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    return halt(Error::DISPLAY_INIT);
  }
  display.clearDisplay();
  panic::init_printer([](const char* kind, uint8_t code) {
    panic_snprintf(kind, code);
    Serial.println(PANIC_BUFFER);
    display.setTextSize(1);
    panic_draw();
    display.display();
  });
  display.dim(true);

  for (auto r = io::init(nrf24); r != Error::OK;) {
    return halt(r);
  }
  nrf24.enableAckPayload();
  nrf24.openWritingPipe(PIPE_ADDRESS);
  nrf24.stopListening();

  report(Error::OK);
}

commands::reply_status status;
millis_t status_time = -STATUS_EXPIRE_MS;
uint16_t transmitterCentVolts;

io::message_t message;

static void send_pwm(millis_t ms, millis_t period) {
  static millis_t lastMs = 0;

  auto dt = ms - lastMs;
  if (dt < period) {
    return;
  }
  lastMs = ms;

  // Serial.print(millis()); Serial.print(" ");
  // Serial.println(panic::counter);
  auto a0 = analogRead(A0);
  auto a1 = analogRead(A1);
  auto pos =
      constrain(map(a0, 0, a1 - 10, INT8_MIN, INT8_MAX), INT8_MIN, INT8_MAX);
  transmitterCentVolts = map(a1, 445, 635, 419, 446);

  commands::set_pwm cmd;
  cmd.value = pos;
  // Serial.println(pos);
  auto len = io::prepare_send(message, cmd.begin(), sizeof(cmd));
  if (!nrf24.writeBlocking(message, len, 50)) {
    report(Error::SEND);
  }
}

static void handle_command(const commands::_base* cmd, millis_t ms) {
  switch (cmd->id) {
    case commands::reply_status::ID: {
      status = *static_cast<const commands::reply_status*>(cmd);
      status_time = ms;
    } break;
    default:
      panic::report(Error::UNKNOWN_CMD);
      return;
  }
}

void drawBattery(int16_t x, int16_t y, uint8_t percent, uint8_t max = 100) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  char buff[5];
  snprintf(buff, sizeof(buff), "%d%%", percent);
  auto len = strlen(buff);
  display.setCursor(x + 4 + FONT_WIDTH * (4 - len), y + 4);
  display.print(buff);
  display.drawFastVLine(x, y + 4, 7, SSD1306_WHITE);
  const auto W = 5 + FONT_WIDTH * 4;
  display.drawRect(x + 1, y, W, 15, SSD1306_WHITE);
  auto w = map(constrain(percent, 0, max), 0, max, 0, W - 2);
  display.fillRect(x + W - w, y + 1, w, 13, SSD1306_INVERSE);
}

void loop() {
  auto ms = millis();

  send_pwm(ms, 10);

  uint8_t len = sizeof(message);
  auto rr = io::recv(nrf24, message, &len);
  switch (rr) {
    case Error::OK:
      if (len) {
        auto cmd = reinterpret_cast<const commands::_base*>(message);
        handle_command(cmd, ms);
      }
      break;
    default:
      report(rr);
      break;
  }

  bool statusExpired = (ms - status_time) >= STATUS_EXPIRE_MS;
  char buff[16];

  display.clearDisplay();

  drawBattery(PAD_X, 0, map(transmitterCentVolts, 330, 420, 0, 100));
  if (!statusExpired) {
    display.setCursor(SCREEN_WIDTH / 2 - 2 * FONT_WIDTH, 2);
    display.print(itoa(status.tempFET, buff, 10));
    display.print(" C");
    display.drawCircle(SCREEN_WIDTH / 2 + FONT_WIDTH / 2 - 1, 2 + 1, 1,
                       SSD1306_WHITE);

    drawBattery(SCREEN_WIDTH - 30 - PAD_X, 0,
                map(constrain(status.voltInput, 198, 252), 198, 252, 0, 100));
  }

  const auto YR = 2 * FONT_HEIGHT + 4;
  if (!statusExpired) {
    display.setCursor(PAD_X, YR + 0 * FONT_HEIGHT);
    display.print(F("Ma: "));
    display.println(dtostrf((double)status.ampsMotor / 10.0, 4, 1, buff));

    display.setCursor(PAD_X, YR + 1 * FONT_HEIGHT);
    display.print(F("Ba: "));
    display.println(dtostrf((double)status.ampsInput / 10.0, 4, 1, buff));

    display.setCursor(PAD_X, YR + 2 * FONT_HEIGHT);
    display.print(F("Bv: "));
    display.println(dtostrf((double)status.voltInput / 10.0, 4, 1, buff));
  }

  display.setCursor(PAD_X, YR + 3 * FONT_HEIGHT);
  display.print(F("Tv: "));
  display.println(dtostrf((double)transmitterCentVolts / 100.0, 4, 1, buff));

  if ((ms - panic_time) < STATUS_EXPIRE_MS) {
    panic_draw();
  }

  if (!statusExpired) {
    display.setTextSize(3);
    auto ptr = dtostrf((double)status.speed / 10.0, 0, 1, buff);
    len = strlen(ptr);
    display.setCursor(SCREEN_WIDTH - len * 3 * FONT_WIDTH, YR);
    display.print(ptr);

    display.setTextSize(1);
    display.setCursor(SCREEN_WIDTH - 5 * FONT_WIDTH, YR + 3 * FONT_HEIGHT);
    display.print(F("km/h"));
  }

  if (ms & 0x10000) {  // ~1 minute
    // screensaver
    display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_INVERSE);
  }
  display.display();
}

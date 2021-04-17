#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiredDevice.h>
#include <RegisterBasedWiredDevice.h>
#include <Accelerometer.h>
#include <AccelerometerMMA8451.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>

#define BUZZER_PIN D8
#define DISPLAY_CS_PIN D3
#define DISPLAY_DC_PIN D0
#define DISPLAY_RS_PIN D4
#endif

#ifdef __AVR__
#include "LowPower.h"

#define BUZZER_PIN 6
#define DISPLAY_CS_PIN 10
#define DISPLAY_DC_PIN 9
#define DISPLAY_RS_PIN 8
#endif

#define BALLSIZE 3
#define DELAY 150
#define MAX_TIMER 30*1000/DELAY
#define MAX_LENGTH 100
#define LEN_PER_LEVEL 3

/**
 * Depending on how the sensor is oriented in relation to the display, we need to adjust sensor readings.
 * Uncomment only one of ORN_X_ and ORN_Y_ so that the X and Y are read from correct sensor direction:
 */
//#define ORN_X_FROM_X
#define ORN_X_FROM_Y
//#define ORN_X_FROM_Z
#define ORN_Y_FROM_X
//#define ORN_Y_FROM_Y
//#define ORN_Y_FROM_Z
/**
 * Depending on how the sensor is oriented, the reading needs to be inverted or not. Uncomment if any of
 * X or Y reading needs to be inverted:
 */
//#define ORN_X_INV
//#define ORN_Y_INV

#define DEBUG true

U8G2_PCD8544_84X48_F_4W_HW_SPI u8g2(U8G2_R0, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RS_PIN);
AccelerometerMMA8451 acc(0);

typedef struct {
  uint8_t x;
  uint8_t y;
} upoint_t;

uint8_t displayWidth, displayHeight, max_x, max_y, level, timer = MAX_TIMER;
upoint_t snake[MAX_LENGTH];
uint8_t headIndex = 0;
uint8_t snakeLen = 1;
uint8_t dir = 0;
upoint_t flag;
uint16_t tonesFlag[][2] = {{698, 1}, {880, 1}, {1047, 1}, {0, 0}};
uint16_t tonesLevel[][2] = {{1047, 1}, {988, 1}, {1047, 1}, {988, 1}, {1047, 1}, {0, 0}};
uint16_t tonesSad[][2] = {{262, 1}, {247, 1}, {233, 1}, {220, 3}, {0, 0}};

uint8_t melodyIndex;
uint16_t (*currentMelody)[2];

void startMotionDetection() {
  acc.standby();
  acc.setMotionDetection(false, true, 0x03);
  acc.setMotionDetectionThreshold(false, 0x1a);
  acc.setMotionDetectionCount(0x10);

  // Register 0x2D, Control Register 4 configures all embedded features for interrupt detection.
  // To set this device up to run an interrupt service routine:
  // Program the Pulse Detection bit in Control Register 4.
  // Set bit 3 to enable the pulse detection "INT_PULSE".
  acc.enableInterrupt(AccelerometerMMA8451::INT_FF_MT);

  // Register 0x2E is Control Register 5 which gives the option of routing the interrupt to either INT1 or INT2
  acc.routeInterruptToInt1(AccelerometerMMA8451::INT_FF_MT);

  // Put the device in Active Mode
  acc.activate();
}

void startTransientDetection() {  
  // Put the part into Standby Mode
  acc.standby();

  // This will enable the transient detection.
  acc.setTransientDetection(true, 0x03, 0x00);

  // Set the transient threshold.
  acc.setTransientThreshold(true, 0x60);

  // Set the debounce counter
  acc.setTransientCount(0x02);

//  // Configure the INT pins for Open Drain
//  acc.setPushPullOpenDrain(AccelerometerMMA8451::PUSH_PULL);
//
//  // Configure the INT pins for Active Low
//  acc.setInterruptPolarity(AccelerometerMMA8451::ACTIVE_LOW);

  // Register 0x2D, Control Register 4 configures all embedded features for interrupt detection.
  // To set this device up to run an interrupt service routine: 
  // Program the Transient Detection bit in Control Register 4. 
  // Set bit 5 to enable the transient detection "INT_TRANS".
  acc.enableInterrupt(AccelerometerMMA8451::INT_TRANS);

  // Register 0x2E is Control Register 5 which gives the option of routing the interrupt to either INT1 or INT2
  acc.routeInterruptToInt1(AccelerometerMMA8451::INT_TRANS);

  // Put the device in Active Mode
  acc.activate();
}

void setupMMA()
{
  // Put the part into Standby Mode
  acc.standby();
  acc.disableInterrupt(AccelerometerMMA8451::INT_ALL);
  acc.setDynamicRange(AccelerometerMMA8451::DR_2G);
  // Set the data rate to 50 Hz (for example, but can choose any sample rate).
  acc.setOutputDataRate(AccelerometerMMA8451::ODR_50HZ_20_MS);
  // Put the device in Active Mode
  acc.activate();
}

void drawBoard(void) {
  static char buf[12];
  u8g2.clearBuffer();
  // draw snake, head to tail
  uint8_t currentIndex = headIndex;
  for (int i = 0; i < snakeLen; i++) {
    u8g2.drawBox(snake[currentIndex].x * BALLSIZE, snake[currentIndex].y * BALLSIZE, BALLSIZE, BALLSIZE);
    if (currentIndex == 0) currentIndex = MAX_LENGTH;
    currentIndex--;
  }
  // draw fruit
  u8g2.drawCircle(flag.x * BALLSIZE + BALLSIZE / 2, flag.y * BALLSIZE + BALLSIZE / 2, BALLSIZE/2);
  
  // write points and time
  itoa(level, buf, 10);
  u8g2.drawStr(0, 5, buf);
  itoa(timer/10, buf, 10);
  uint8_t width = u8g2.getStrWidth(buf);
  u8g2.drawStr(displayWidth - width, 5, buf);
  u8g2.sendBuffer();
}

void showPopup(char *line_1, char *line_2) {
  u8g2.clearBuffer();
  u8g2.drawRFrame(0, 0, displayWidth, displayHeight, 7);
  uint8_t width = u8g2.getStrWidth(line_1);
  u8g2.drawStr((displayWidth - width) / 2, displayHeight / 2 - 2, line_1);
  width = u8g2.getStrWidth(line_2);
  u8g2.drawStr((displayWidth - width) / 2, displayHeight / 2 + 8, line_2);
  u8g2.sendBuffer();
}

void placeRandomly(upoint_t *point) {
  (*point).x = random(max_x - 2*BALLSIZE) + BALLSIZE;
  (*point).y = random(max_y - 2*BALLSIZE) + BALLSIZE;
}

// used to play the melody asynchronously while the user is playing
void playSound(void) {
  if (currentMelody) {
    uint8_t totalCount = 0;
    for (uint8_t i = 0; 1; i++) {
      uint16_t freq = currentMelody[i][0];
      uint16_t dur = currentMelody[i][1];
      if (melodyIndex == totalCount) {
        if (dur == 0) {
          noTone(BUZZER_PIN);
          currentMelody = NULL;
          melodyIndex = 0;
        } else {
          tone(BUZZER_PIN, freq);
        }
      }
      totalCount += dur;
      if (totalCount > melodyIndex)
        break;
    }
    melodyIndex++;
  }
}

void melodySad(void) {
  // this is played synchronously
  for (uint8_t i = 0; tonesSad[i][1] > 0; i++) {
    tone(BUZZER_PIN, tonesSad[i][0], tonesSad[i][1]*300);
    delay(tonesSad[i][1] * 300 + 50);
  }
}

void melodyFlag(void) {
  currentMelody = tonesFlag;
  melodyIndex = 0;
}

void melodyLevel(void) {
  currentMelody = tonesLevel;
  melodyIndex = 0;
}

void initGame(void) {
  level = 1;
  snakeLen = 1;
  headIndex = 0;
  timer = MAX_TIMER;
  snake[headIndex].x = max_x / 2;
  snake[headIndex].y = max_y / 2;
  placeRandomly(&flag);
}

void gameOver(void) {
  char msg[50];
  sprintf(msg, "score: %d", level);
  showPopup("GAME OVER", msg);
  melodySad();
  initGame();
}

bool checkIfBitten(upoint_t newPos) {
  uint8_t currentIndex = headIndex;
  
  for (int i = 0; i < snakeLen; i++) {
    if (snake[currentIndex].x == newPos.x && snake[currentIndex].y == newPos.y) return true;
    if (currentIndex == 0) currentIndex = MAX_LENGTH;
    currentIndex--;
  }
  return false;
}

bool checkIfGotTheApple(upoint_t newPos) {
  if (newPos.x == flag.x && newPos.y == flag.y) return true;
  return false;
}

void getDir() {
  #ifdef ORN_X_FROM_X
  float xg = acc.readXg();
  #endif
  #ifdef ORN_X_FROM_Y
  float xg = acc.readYg();
  #endif
  #ifdef ORN_X_FROM_Z
  float xg = acc.readZg();
  #endif
  #ifdef ORN_Y_FROM_X
  float yg = acc.readXg();
  #endif
  #ifdef ORN_Y_FROM_Y
  float yg = acc.readYg();
  #endif
  #ifdef ORN_Y_FROM_Z
  float yg = acc.readZg();
  #endif
  #ifdef ORN_X_INV
  xg = -xg;
  #endif
  #ifdef ORN_Y_INV
  yg = -yg;
  #endif

  uint8_t newDir = xg > 0 ? 2 : 0;
  if (fabs(yg) > fabs(xg)) {
    newDir = yg > 0 ? 1 : 3;
  }
  if (abs(dir - newDir) % 2 == 1)
    dir = newDir;
}

upoint_t getNewPos(void) {
  upoint_t newPos;
  getDir();
  upoint_t lastPos = snake[headIndex];

  switch(dir) {
    case 0:
      newPos.x = lastPos.x >= max_x ? 0 : lastPos.x + 1;
      newPos.y = lastPos.y;
      break;
    case 1:
      newPos.x = lastPos.x;
      newPos.y = lastPos.y >= max_y ? 0 : lastPos.y + 1;
      break;
    case 2:
      newPos.x = lastPos.x > 0 ? lastPos.x - 1 : max_x;
      newPos.y = lastPos.y;
      break;
    case 3:
      newPos.x = lastPos.x;
      newPos.y = lastPos.y > 0 ? lastPos.y - 1 : max_y;
      break;
  }
  return newPos;
}

void goToSleep() {
  showPopup("SLEEPING...", "shake to wake");
  startMotionDetection();
  delay(1000);
#ifdef ESP8266
  ESP.deepSleep(0);
#endif
#ifdef __AVR__
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
#endif
}

void setup(void) {
  randomSeed(analogRead(0));
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Marbluino Snake");
#endif
#ifdef ESP8266
  WiFi.mode(WIFI_OFF);
#endif
  
  setupMMA();

  Serial.println("MMA set up!");

  u8g2.begin();
  u8g2.setFont(u8g2_font_baby_tf);
  displayWidth = u8g2.getDisplayWidth();
  displayHeight = u8g2.getDisplayHeight();
  max_x = displayWidth / BALLSIZE - 1;
  max_y = displayHeight / BALLSIZE - 1;
  initGame();
}

void loop(void) {
  upoint_t newPos = getNewPos();
  if (checkIfBitten(newPos)) {
    gameOver();
  } else if (checkIfGotTheApple(newPos)) {
    melodyFlag();
    if (snakeLen < MAX_LENGTH) {
      placeRandomly(&flag);
      level++;
      timer = MAX_TIMER;
    } else {
      gameOver();
    }
  }
  if (snakeLen < level * LEN_PER_LEVEL) snakeLen++;
  headIndex++;
  if (headIndex >= MAX_LENGTH) {
    headIndex = 0;
  }
  snake[headIndex] = newPos;
  drawBoard();

  playSound();
  delay(DELAY);
  if (timer > 0) {
    timer--;
  }
  else {
    if (level <= 1) {
      goToSleep();    
    } else {
      gameOver();
    }
  }
}

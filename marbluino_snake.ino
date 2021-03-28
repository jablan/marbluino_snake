#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

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
#define MAX_LENGTH 20

#define ORIENT_X -1
#define ORIENT_Y 1

#define DEBUG true

U8G2_PCD8544_84X48_F_4W_HW_SPI u8g2(U8G2_R0, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RS_PIN);

typedef struct {
  uint8_t x;
  uint8_t y;
} upoint_t;

uint8_t displayWidth, displayHeight, max_x, max_y, points, timer = MAX_TIMER;
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

// MMA8452Q I2C address is 0x1C(28)
#define MMA_ADDR 0x1C

void mmaRegWrite(byte reg, byte value) {
  Wire.beginTransmission(MMA_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void mmaSetStandbyMode() {
  mmaRegWrite(0x2A, 0x18); //Set the device in 100 Hz ODR, Standby  
}

void mmaSetActiveMode() {
  mmaRegWrite(0x2A, 0x19);  
}

// Causes interrupt when shaken
void mmaSetupMotionDetection() {
  // https://www.nxp.com/docs/en/application-note/AN4070.pdf
  mmaSetStandbyMode();
  mmaRegWrite(0x15, 0x78);
  mmaRegWrite(0x17, 0x1a);
  mmaRegWrite(0x18, 0x10);
  // enable interrupt
  mmaRegWrite(0x2D, 0x04);
  mmaRegWrite(0x2E, 0x04);
  mmaSetActiveMode();
}

void mmaDisableInterrupt() {
  mmaRegWrite(0x2d, 0x00);
}

void setupMMA()
{
  // Initialise I2C communication as MASTER
  Wire.begin();

  mmaSetStandbyMode();
  mmaDisableInterrupt();
  mmaRegWrite(0x0e, 0x00); // set range to +/- 2G
  mmaSetActiveMode();
}

void getOrientation(float xyz_g[3]) {
  unsigned int data[7];

  // Request 7 bytes of data
  Wire.requestFrom(MMA_ADDR, 7);
 
  // Read 7 bytes of data
  // status, xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
  if(Wire.available() == 7) 
  {
    for (int i = 0; i<6; i++) {
      data[i] = Wire.read();
    }
  }

  // Convert the data to 12-bits
  signed short iAccl[3];
  for (int i = 0; i < 3; i++) {
    iAccl[i] = ((data[i*2+1] << 8) | data[i*2+2]) >> 4;
    if (iAccl[i] > 2047)
    {
      iAccl[i] -= 4096;
    }
    xyz_g[i] = (float)iAccl[i] / 1024;
  }
}

void drawBoard(void) {
  static char buf[12];
  u8g2.clearBuffer();
  // draw marble
  uint8_t currentIndex = headIndex;
  for (int i = 0; i < snakeLen; i++) {
    u8g2.drawBox(snake[currentIndex].x * BALLSIZE - BALLSIZE / 2, snake[currentIndex].y * BALLSIZE - BALLSIZE / 2, BALLSIZE, BALLSIZE);
    if (currentIndex == 0) currentIndex = MAX_LENGTH;
    currentIndex--;
  }
  u8g2.drawCircle(flag.x * BALLSIZE, flag.y * BALLSIZE, BALLSIZE/2);
  
  // write points and time
  itoa(points, buf, 10);
  u8g2.drawStr(0, 5, buf);
  itoa(timer/10, buf, 10);
  uint8_t width = u8g2.getStrWidth(buf);
  u8g2.drawStr(max_x-width, 5, buf);
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
//  do {
    (*point).x = random(max_x - 2*BALLSIZE) + BALLSIZE;
    (*point).y = random(max_y - 2*BALLSIZE) + BALLSIZE;
  // ensure not spawning too close to the ball
//  } while (abs((*point).x-ball.x) + abs((*point).y-ball.y) < MIN_DISTANCE);
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
  points = 0;
  snakeLen = 1;
  headIndex = 0;
  timer = MAX_TIMER;
  snake[headIndex].x = max_x / 2;
  snake[headIndex].y = max_y / 2;
  placeRandomly(&flag);
}

void gameOver(void) {
  char msg[50];
  sprintf(msg, "score: %d", points);
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

void getDir(float xyz_g[3]) {
  uint8_t newDir = xyz_g[0] > 0 ? 2 : 0;
  if (fabs(xyz_g[1]) > fabs(xyz_g[0])) {
    newDir = xyz_g[1] > 0 ? 1 : 3;
  }
  if (abs(dir - newDir) % 2 == 1)
    dir = newDir;
}

upoint_t getNewPos(void) {
  float xyz_g[3];
  getOrientation(xyz_g);

  upoint_t newPos;
  getDir(xyz_g);
  upoint_t lastPos = snake[headIndex];

  switch(dir) {
    case 0:
      newPos.x = lastPos.x >= max_x ? 0 : lastPos.x + ORIENT_X;
      newPos.y = lastPos.y;
      break;
    case 1:
      newPos.x = lastPos.x;
      newPos.y = lastPos.y >= max_y ? 0 : lastPos.y + ORIENT_Y;
      break;
    case 2:
      newPos.x = lastPos.x > 0 ? lastPos.x - ORIENT_X : max_x;
      newPos.y = lastPos.y;
      break;
    case 3:
      newPos.x = lastPos.x;
      newPos.y = lastPos.y > 0 ? lastPos.y - ORIENT_Y : max_y;
      break;
  }
  return newPos;
}

void goToSleep() {
  showPopup("SLEEPING...", "shake to wake");
  mmaSetupMotionDetection();
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
#endif
#ifdef ESP8266
  WiFi.mode(WIFI_OFF);
#endif
  
  setupMMA();
 
  u8g2.begin();
  u8g2.setFont(u8g2_font_baby_tf);
  displayWidth = u8g2.getDisplayWidth();
  displayHeight = u8g2.getDisplayHeight();
  max_x = displayWidth / BALLSIZE;
  max_y = displayHeight / BALLSIZE;
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
      snakeLen++;
      points++;
      timer = MAX_TIMER;
    } else {
      gameOver();
    }
  }
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
    if (points == 0) {
      goToSleep();    
    } else {
      gameOver();
    }
  }
}

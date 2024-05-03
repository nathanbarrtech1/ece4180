/////////////////////////////////
/////////// INCLUDES ////////////
/////////////////////////////////

#include <Servo.h>
#include <Encoder.h>
#include "SevSeg.h"
#include <TimerOne.h>
#include <SPFD5408_Adafruit_GFX.h>
#include <SPFD5408_Adafruit_TFTLCD.h>
#include <SPI.h>
#include <SD.h>
#include <SPFD5408_TouchScreen.h>

/////////////////////////////////
/////// SHARED CONSTANTS ////////
/////////////////////////////////

#define NUM_PUZZLES     5

#define VAULT           0
#define LCD             1
#define KNOCK           2
#define LEDS            3
#define ACC             4

#define DEBUG           1
#define SERVOPIN        23

/////////////////////////////////
//////// VAULT CONSTANTS ////////
/////////////////////////////////

#define ButtonPin       17
#define PinA            19
#define PinB            18
#define DEBOUNCE_DELAY  150

/////////////////////////////////
//////// KNOCK CONSTANTS ////////
/////////////////////////////////

#define NUM_OF_KNOCKS 8

/////////////////////////////////
//////// KNOCK CONSTANTS ////////
/////////////////////////////////

#define LCD_PIN 9

/////////////////////////////////
///////// LED CONSTANTS /////////
/////////////////////////////////

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define LCD_RD A0
#define LCD_WR A1
#define LCD_CD A2
#define LCD_CS A3
#define LCD_RESET A4

#define YP A2
#define XM A3
#define YM 8
#define XP 9
#define SD_CS 53

#define COMPLETE A5
#define WIDTH 106
#define HEIGHT 160

#define LED_DEBOUNCE 350

/////////////////////////////////
/////// SHARED VARIABLES ////////
/////////////////////////////////

int valid_puzzles[NUM_PUZZLES] = {false, false, true, false, false};
int solved_puzzles[NUM_PUZZLES] = {false, false, false, false, false};
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, A4);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

/////////////////////////////////
//////// VAULT VARIABLES ////////
/////////////////////////////////

Encoder myEnc(PinA,  PinB);
SevSeg sevseg;
Servo lockServo;

int currButtonVal;
int lastButtonVal;
int rightPos;
int rightNum;


int corrCode[4];
int guessCode[4] = {0, 0, 0, 0};
byte correctNumLEDs[4] =  {27, 29, 37, 35};  //Pin numbers for correct number LEDs (Indicate a correct digit)
byte correctPlaceLEDs[4] = {25, 31, 39, 33}; //Pin numbers for correct place LEDs (Indicate a correct digit in the correct place)

unsigned long buttonTime = 0;
int currNum = 0;
int numGuesses = 0;
int numPresses = 0;
int tempPos = -999;
long oldPosition  = -999;
long newPosition = -999;

/////////////////////////////////
//////// KNOCK VARIABLES ////////
/////////////////////////////////

const int sensorPin = A8;
const int threshold = 300;
byte LED[NUM_OF_KNOCKS] = {25, 31, 39, 33, 27, 29, 37, 35};
int minKnockTimes[NUM_OF_KNOCKS] = {0,      0   , 1000, 2000, 0   , 1000, 3000, 0   };
int maxKnockTimes[NUM_OF_KNOCKS] = {100000, 1000, 3000, 4000, 1000, 3000, 6000, 4000};
const int ledPin[3][3]    = {{5,  6,  7} , {8,  9,  10}, {11, 12, 13}};
int currentKnock = 0;
long int timer;
bool knockValid = false;

/////////////////////////////////
///////// LED VARIABLES /////////
/////////////////////////////////

int ledColors[3][3] =
{
  {BLUE,  RED,    YELLOW},
  {GREEN, BLUE,   RED},
  {WHITE, GREEN,  BLUE}
};

const int LED_PUZZLE[3][3] =
{
  {14, 41, 10},
  {15, 43, 11},
  {16, 45, 12}
};

bool ledOn[3][3] =
{
  {LOW, LOW, LOW},
  {LOW, LOW, LOW},
  {LOW, LOW, LOW}
};

long ledTimer = 0;

void setup()
{
  /////////////////////////////////
  ////////// SHARED SETUP /////////
  /////////////////////////////////
  
  Serial.begin (9600);
  Serial.println("BEGIN!");

  /////////////////////////////////
  ////////// VAULT SETUP //////////
  /////////////////////////////////
  
  pinMode(ButtonPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PinA), updateEncoder, CHANGE);
  
  lockServo.attach(SERVOPIN);
  lockServo.write(90);
  randomSeed(analogRead(A7));
  generateNewCode();

  for (int i = 0 ; i <= 3 ; i++)
  {
    pinMode(correctNumLEDs[i], OUTPUT);
    pinMode(correctPlaceLEDs[i], OUTPUT);
  }

  byte numDigits = 4;
  byte digitPins[] = {22, 34, 38, 44};
  byte segmentPins[] = {26, 42, 36, 28, 24, 30, 40, 32};

  bool resistorsOnSegments = true;
  byte hardwareConfig = COMMON_CATHODE;
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments);
  sevseg.setBrightness(90);

  /////////////////////////////////
  ////////// KNOCK SETUP //////////
  /////////////////////////////////

  pinMode(sensorPin, INPUT);

  pinMode(LCD_PIN, INPUT_PULLUP);

  if (!DEBUG)
    generateKnock();

  /////////////////////////////////
  /////////// TFT SETUP ///////////
  /////////////////////////////////


  /////////////////////////////////
  /////////// LED SETUP ///////////
  /////////////////////////////////

  start();
  LED_PUZZLE_SETUP();

}

void loop()
{
  sevseg.refreshDisplay();
  if (solved_puzzles[LCD] && solved_puzzles[KNOCK] && !solved_puzzles[VAULT])
  {
    valid_puzzles[VAULT] = true;
  }

  if (valid_puzzles[LCD])
  {
    
  }

  ////////////////////////////////
  ///////////// SERVO ////////////
  ////////////////////////////////

  if (checkSolved())
    lockServo.write(130);

  ////////////////////////////////
  ////////// VAULT LOOP //////////
  ////////////////////////////////

  if (valid_puzzles[VAULT])
  {
    if (numPresses == 4) checkNumber();
    else updateNumber();
  }
  else Serial.println("HI");

  ////////////////////////////////
  ////////// KNOCK LOOP //////////
  ////////////////////////////////

  if (valid_puzzles[KNOCK])
  {
    if (currentKnock == 8)
    {
      resetKnocks();
      Celebration();
      valid_puzzles[KNOCK] = false;
      solved_puzzles[KNOCK] = true;
    }
    readAndCheckKnock();
  }

  ////////////////////////////////
  /////////// LED LOOP ///////////
  ////////////////////////////////

  if (valid_puzzles[LEDS] && millis() - ledTimer > LED_DEBOUNCE)
  {
    LED_PUZZLE_ALG();
    IS_LED_PUZZLE_SOLVED();
  }
}
/////////////////////////////////
//////// SHARED FUNCTIONS ///////
/////////////////////////////////

int getCoordinate(TSPoint p, char axis)
{
  if (axis == 'y')
  {
    int x = map(p.y, 112, 800, 0, tft.width()  - 1);

    if       (x >= 2 * tft.width() / 3)   return 0;
    else if  (x >=     tft.width() / 3)   return 1;
    else                                  return 2;
  }

  if (axis == 'x')
  {
    int y = map(p.x,  96, 947, 0, tft.height() - 1);

    if       (y >= 2 * tft.height() / 3)  return 0;
    else if  (y >=     tft.height() / 3)  return 1;
    else                                  return 2;
  }
}

#define BUFFPIXEL 20           //Drawing speed, 20 is meant to be the best but you can use 60 altough it takes a lot of uno's RAM         

//Drawing function, reads the file from the SD card and do the
//conversion and drawing, also it shows messages on the Serial monitor in case of a problem
//No touchy to this function :D

void bmpDraw(char *filename, int x, int y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3 * BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint8_t  lcdidx = 0;
  boolean  first = true;

  if ((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  progmemPrint(PSTR("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');
  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    progmemPrintln(PSTR("File not found"));
    return;
  }

  // Parse BMP header
  if (read16(bmpFile) == 0x4D42) { // BMP signature
    progmemPrint(PSTR("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    progmemPrint(PSTR("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    progmemPrint(PSTR("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if (read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      progmemPrint(PSTR("Bit Depth: ")); Serial.println(bmpDepth);
      if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        progmemPrint(PSTR("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if (bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if ((x + w - 1) >= tft.width())  w = tft.width()  - x;
        if ((y + h - 1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x + w - 1, y + h - 1);

        for (row = 0; row < h; row++) { // For each scanline...
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if (bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col = 0; col < w; col++) { // For each column...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              // Push LCD buffer to the display first
              if (lcdidx > 0) {
                tft.pushColors(lcdbuffer, lcdidx, first);
                lcdidx = 0;
                first  = false;
              }
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            lcdbuffer[lcdidx++] = tft.color565(r, g, b);
          } // end pixel
        } // end scanline
        // Write any remaining data to LCD
        if (lcdidx > 0) {
          tft.pushColors(lcdbuffer, lcdidx, first);
        }
        progmemPrint(PSTR("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if (!goodBmp) progmemPrintln(PSTR("BMP format not recognized."));
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

// Copy string from flash to serial port
// Source string MUST be inside a PSTR() declaration!
void progmemPrint(const char *str) {
  char c;
  while (c = pgm_read_byte(str++)) Serial.print(c);
}

// Same as above, with trailing newline
void progmemPrintln(const char *str) {
  progmemPrint(str);
  Serial.println();
}

/////////////////////////////////
//////// VAULT FUNCTIONS ////////
/////////////////////////////////

void updateEncoder() {
  newPosition = myEnc.read();
}

void generateNewCode()                                        //Function to generate a new random code
{
  Serial.print("Code: ");

  if (DEBUG)
  {
    for (int i = 0 ; i <= 3 ; i++)                              //Loops through the four digits and assigns a random number to each
    {
      corrCode[i] = i;                                   //Generate a random number for each digit
      Serial.print(corrCode[i]);                                    //Display the code on Serial monitor for debugging
    }
  }
  else
  {
    for (int i = 0 ; i <= 3 ; i++)                              //Loops through the four digits and assigns a random number to each
    {
      corrCode[i] = random(0, 9);                                   //Generate a random number for each digit
      Serial.print(corrCode[i]);                                    //Display the code on Serial monitor for debugging
    }
  }
}

void displayNumber()
{
  int num = 0;

  for (int i = 0; i < 4; i++)
  {
    num *= 10;
    if (numPresses == i)
    {
      num += currNum;
    }
    else if (guessCode != 0 && guessCode != 10)
      num += guessCode[i];
  }
  
  sevseg.setNumber(num, 4);
  sevseg.refreshDisplay();
}


void checkNumber()
{
  for (int i = 0; i < 4; i++) Serial.print(guessCode[i]);
  Serial.println();
  rightPos = 0;
  rightNum = 0;
  numGuesses++;
  numPresses = 0;
  bool usedNums[4] = {false, false, false, false};

  for (int i = 0; i < 4; i++)
  {
    if (guessCode[i] == corrCode[i])
    {
      rightPos++;
      usedNums[i] = true;
    }
  }

  checkVaultSolved();

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
    {
      if (guessCode[i] == corrCode[j] && guessCode[i] != corrCode[i] && !usedNums[j])
      {
        rightNum++;
        usedNums[j] = true;
        break;
      }
    }

  updateLEDs(rightNum, rightPos);
  for (int i = 0; i < 4; i++) guessCode[i] = 0;
}

void updateLEDs (int corNum, int corPla)                        //Function to update the LEDs to reflect the guess
{
  for (int i = 0 ; i < 4 ; i++)                                //First turn all LEDs off
  {
    digitalWrite(correctNumLEDs[i], LOW);
    digitalWrite(correctPlaceLEDs[i], LOW);
  }
  for (int j = 0 ; j <= corNum - 1 ; j++)                       //Turn on the number of correct digits in wrong place LEDs
  {
    digitalWrite(correctNumLEDs[j], HIGH);
  }
  for (int k = 0 ; k <= corPla - 1 ; k++)                       //Turn on the number of correct digits in the correct place LEDs
  {
    digitalWrite(correctPlaceLEDs[k], HIGH);
  }
}

void updateNumber()
{
  while (!buttonPressed())
  {
    byte incSize = 3;
    if (newPosition != oldPosition) {
      if (newPosition - oldPosition < -1 * incSize)           currNum--;
      else if (newPosition - oldPosition > incSize) currNum++;
      else return;
      oldPosition = newPosition;

      if (currNum > 9) currNum = 0;
      if (currNum < 0) currNum = 9;
    }
    displayNumber();
  }
  enterNumber();
}

bool buttonPressed() {
  currButtonVal = digitalRead(ButtonPin);
  if (!currButtonVal && currButtonVal != lastButtonVal && (buttonTime - millis()) > DEBOUNCE_DELAY)
  {
    buttonTime = millis();
    return true;
  }
  lastButtonVal = currButtonVal;
  return false;
}

void enterNumber()
{
  guessCode[numPresses] = currNum;
  currNum = 0;
  numPresses++;
  lastButtonVal = currButtonVal;
}

void checkVaultSolved()
{
  if (rightPos == 4)
  {
    sevseg.blank();
    sevseg.refreshDisplay();
    sevseg.setNumber(numGuesses, 0);
    
    valid_puzzles[VAULT] = false;
    solved_puzzles[VAULT] = true;
    Celebration();
  }
  else sevseg.setNumber(0, 4);
}

bool checkSolved()
{
  for (int i = 0; i < NUM_PUZZLES; i++)
    if (!solved_puzzles[i])
      return false;

  return true;
}

/////////////////////////////////
//////// KNOCK FUNCTIONS ////////
/////////////////////////////////

void generateKnock()
{
  bool b = false;
  for (int i = 1; i < NUM_OF_KNOCKS; i++)
  {
    int temp;

    if (!b)
      temp = random(4);
    else
      temp = random(3);

    if (temp == 3)
      b = true;

    minKnockTimes[i] = temp * 1000;
    maxKnockTimes[i] = (temp + 2) * 1000;

    Serial.print(temp);
    Serial.print(", ");
    Serial.println(minKnockTimes[i]);
  }
}

void readAndCheckKnock()
{
  int val = analogRead(sensorPin);

  if (val >= threshold)
  {
    if (millis() - timer >= minKnockTimes[currentKnock] && millis() - timer <= maxKnockTimes[currentKnock])
    {
      Serial.print("SUCCESS ");
      Serial.println(currentKnock);
      digitalWrite(LED[currentKnock], HIGH);
      currentKnock++;
      delay(300);
    }

    else if (millis() - timer < minKnockTimes[currentKnock])
    {
      resetKnocks();
      delay(100);
      for (int i = 0; i < 4; i++)
        digitalWrite(correctPlaceLEDs[i], HIGH);
      delay(1000);
      resetKnocks();
    }

    timer = millis();
  }

  if (millis() - timer > maxKnockTimes[currentKnock])
  {
    resetKnocks();
    delay(100);
    for (int i = 0; i < 4; i++)
      digitalWrite(correctNumLEDs[i], HIGH);
    delay(1000);
    resetKnocks();
  }
}

void resetKnocks()
{
  currentKnock = 0;
  for (int i = 0; i < NUM_OF_KNOCKS; i++)
  {
    digitalWrite(LED[i], LOW);
  }
}

void Celebration()
{
  int delayTime = 150;

  for (int i = 0; i < 4; i++)
  {
    if (i % 2 == 0)
    {
      digitalWrite(correctNumLEDs[i], HIGH);
      delay(delayTime);
      digitalWrite(correctNumLEDs[i], LOW);
      delay(delayTime);
      digitalWrite(correctPlaceLEDs[i], HIGH);
      delay(delayTime);
      digitalWrite(correctPlaceLEDs[i], LOW);
      delay(delayTime);
    }

    else
    {
      digitalWrite(correctPlaceLEDs[i], HIGH);
      delay(delayTime);
      digitalWrite(correctPlaceLEDs[i], LOW);
      delay(delayTime);
      digitalWrite(correctNumLEDs[i], HIGH);
      delay(delayTime);
      digitalWrite(correctNumLEDs[i], LOW);
      delay(delayTime);
    }
  }

  for (int i = 0; i < 4; i++)
  {
    digitalWrite(correctPlaceLEDs[i], HIGH);
    delay(delayTime);
    digitalWrite(correctPlaceLEDs[i], LOW);
    delay(delayTime);
  }

  for (int i = 3; i >= 0; i--)
  {
    digitalWrite(correctNumLEDs[i], HIGH);
    delay(delayTime);
    digitalWrite(correctNumLEDs[i], LOW);
    delay(delayTime);
  }
}
/////////////////////////////////
///////// LED FUNCTIONS /////////
/////////////////////////////////

void LED_PUZZLE_ALG()
{

  TSPoint p;
  do
  {
    p = ts.getPoint();
    pinMode(XM, OUTPUT);  //.kbv ALWAYS restore pinMode()
    pinMode(YP, OUTPUT);
  }
  while (p.z < ts.pressureThreshhold);



  int ledRow = getCoordinate(p, 'x');
  int ledCol = 2 - getCoordinate(p, 'y');

  for (int k = -1; k <= 1; k++)
    for (int l = -1; l <= 1; l++)
      if (ledRow + k >= 0 && ledRow + k < 3 && ledCol + l >= 0 && ledCol + l < 3 && (k == 0 || l == 0))
      {
        ledOn[ledRow + k][ledCol + l] = !ledOn[ledRow + k][ledCol + l];
        if (k == 0 && l == 0)
          Serial.println(ledOn[ledRow + k][ledCol + l]);

        digitalWrite(LED_PUZZLE[ledRow + k][ledCol + l],  ledOn[ledRow + k][ledCol + l]);
      }

  ledTimer = millis();

}
void IS_LED_PUZZLE_SOLVED()
{
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
    {
      if (!ledOn[i][j]) return;
    }

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
    {
      ledOn[i][j] = false;
      digitalWrite(LED_PUZZLE[i][j], LOW);
    }
    
  valid_puzzles[LEDS] = false;
  solved_puzzles[LEDS] = true;
  
  ledCelebration();
  ledCelebration();
  ledCelebration();
}

void start()
{
  pinMode(COMPLETE, OUTPUT);
  Serial.begin(9600);
  tft.reset();
  uint16_t identifier = tft.readID();
  pinMode(10, OUTPUT);
  tft.begin(identifier);
  if (!SD.begin(SD_CS)) {
    progmemPrintln(PSTR("failed!"));
    return;
  }
  tft.fillScreen(BLACK);
}

void LED_PUZZLE_SETUP()
{
  tft.setRotation(1);
  randomSeed(micros());
  long rand1[2];
  long rand2[2];

  bool rand1CC = false;
  bool rand2CC = false;

  while (!(rand1CC && rand2CC))
  {
    long temprand1[2] = {random(3), random(3)};
    long temprand2[2] = {random(3), random(3)};

    *rand1 = *temprand1;
    *rand2 = *temprand2;

    rand1CC = (rand1[0] + rand1[1]) % 2 == 0 && rand1[0] != 1 && rand1[1] != 1;
    rand2CC = (rand2[0] + rand2[1]) % 2 == 0 && rand2[0] != 1 && rand2[1] != 1;
  }
  delay(1000);

  digitalWrite(LED_PUZZLE[rand1[0]][rand1[1]], HIGH);
  digitalWrite(LED_PUZZLE[rand2[0]][rand2[1]], HIGH);

  ledOn[rand1[0]][rand1[1]] = HIGH;
  ledOn[rand2[0]][rand2[1]] = HIGH;
}

void ledCelebration()
{
  digitalWrite(ledPin[2][0], HIGH);
  delay(100);
  digitalWrite(ledPin[2][0], LOW);
  digitalWrite(ledPin[1][0], HIGH);
  digitalWrite(ledPin[2][1], HIGH);
  delay(100);
  digitalWrite(ledPin[1][0], LOW);
  digitalWrite(ledPin[2][1], LOW);
  digitalWrite(ledPin[0][0], HIGH);
  digitalWrite(ledPin[1][1], HIGH);
  digitalWrite(ledPin[2][2], HIGH);
  delay(100);
  digitalWrite(ledPin[0][0], LOW);
  digitalWrite(ledPin[1][1], LOW);
  digitalWrite(ledPin[2][2], LOW);
  digitalWrite(ledPin[1][2], HIGH);
  digitalWrite(ledPin[0][1], HIGH);
  delay(100);
  digitalWrite(ledPin[1][2], LOW);
  digitalWrite(ledPin[0][1], LOW);
  digitalWrite(ledPin[0][2], HIGH);
  delay(100);
  digitalWrite(ledPin[0][2], LOW);

  delay(300);

  digitalWrite(ledPin[0][2], HIGH);
  delay(100);
  digitalWrite(ledPin[0][2], LOW);
  digitalWrite(ledPin[1][2], HIGH);
  digitalWrite(ledPin[0][1], HIGH);
  delay(100);
  digitalWrite(ledPin[1][2], LOW);
  digitalWrite(ledPin[0][1], LOW);
  digitalWrite(ledPin[0][0], HIGH);
  digitalWrite(ledPin[1][1], HIGH);
  digitalWrite(ledPin[2][2], HIGH);
  delay(100);
  digitalWrite(ledPin[0][0], LOW);
  digitalWrite(ledPin[1][1], LOW);
  digitalWrite(ledPin[2][2], LOW);
  digitalWrite(ledPin[1][0], HIGH);
  digitalWrite(ledPin[2][1], HIGH);
  delay(100);
  digitalWrite(ledPin[1][0], LOW);
  digitalWrite(ledPin[2][1], LOW);
  digitalWrite(ledPin[2][0], HIGH);
  delay(100);
  digitalWrite(ledPin[2][0], LOW);

  delay(300);

  digitalWrite(ledPin[2][0], HIGH);
  delay(100);
  digitalWrite(ledPin[2][0], LOW);
  digitalWrite(ledPin[1][0], HIGH);
  digitalWrite(ledPin[2][1], HIGH);
  delay(100);
  digitalWrite(ledPin[1][0], LOW);
  digitalWrite(ledPin[2][1], LOW);
  digitalWrite(ledPin[0][0], HIGH);
  digitalWrite(ledPin[1][1], HIGH);
  digitalWrite(ledPin[2][2], HIGH);
  delay(100);
  digitalWrite(ledPin[0][0], LOW);
  digitalWrite(ledPin[1][1], LOW);
  digitalWrite(ledPin[2][2], LOW);
  digitalWrite(ledPin[1][2], HIGH);
  digitalWrite(ledPin[0][1], HIGH);
  delay(100);
  digitalWrite(ledPin[1][2], LOW);
  digitalWrite(ledPin[0][1], LOW);
  digitalWrite(ledPin[0][2], HIGH);
  delay(100);
  digitalWrite(ledPin[0][2], LOW);

  delay(300);
}

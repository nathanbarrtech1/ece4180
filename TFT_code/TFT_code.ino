/*This code is to use with 2.4" TFT LCD touch screen shield, it reads bmp images stored on SD card
  and shows them on the screen
  Refer to SurtrTech.com for more details
*/

#include <SPFD5408_Adafruit_GFX.h>
#include <SPFD5408_Adafruit_TFTLCD.h>
#include <SPI.h>
#include <SD.h>
#include <SPFD5408_TouchScreen.h>

// LCD PINS
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

// OUTPUT PIN
#define COMPLETE A5
#define WIDTH 106
#define HEIGHT 160

#define DEBUG 1

char corrImgArray[3][3][8] =
{
  {"00.bmp", "10.bmp", "20.bmp"},
  {"01.bmp", "11.bmp", "21.bmp"},
  {"02.bmp", "12.bmp", "22.bmp"}
};

char currImgArray[3][3][8] =
{
  {"00.bmp", "10.bmp", "20.bmp"},
  {"01.bmp", "11.bmp", "21.bmp"},
  {"02.bmp", "12.bmp", "22.bmp"}
};

byte state = 0;
bool notDone = true;

char blank[8];

int imgSel;
char imgSelect[3][4] = {"Lio", "Spi", "Shi"};

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, A4);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

void setup()
{
  start();
  tft.setRotation(2);
  
  int n = 0;

  TSPoint p;
  do
  {
    p = ts.getPoint();
    pinMode(XM, OUTPUT);  //.kbv ALWAYS restore pinMode()
    pinMode(YP, OUTPUT);
  }
  while (p.z < ts.pressureThreshhold);

  randomSeed(millis());

  imgSel = random(3);
  
  if (DEBUG)
  {
    swapArrElements(0, 1, 1, 1, currImgArray);
    memcpy(blank, currImgArray[1][1], sizeof(currImgArray[1][1]));
  }
  else
  {
    while (n < 200 || !solvable(currImgArray))
    {
      swapArrElements(random(3), random(3), random(3), random(3), currImgArray);
      n++;
    }
    memcpy(blank, currImgArray[random(0, 2)][random(0, 2)], sizeof(currImgArray[1][1]));
  }
  drawImage(currImgArray);
  delay(500);
}

void loop()
{
  LCD();
}

void LCD()
{
  if (notDone)
  {
    digitalWrite(A5, LOW);
    if (solved(corrImgArray, currImgArray))
    {
      for (int i = 0; i < 3; i ++)
        for (int j = 0; j < 3; j ++)
        {
          if (currImgArray[i][j][0] == blank[0] && currImgArray[i][j][1] == blank[1])
          {
            char temp[8];
            memcpy(blank, temp, sizeof(blank));
            drawTile(i, j, currImgArray);
            notDone = false;
            digitalWrite(A5, HIGH);
            break;
          }
          if (!notDone) break;
        }
    }

    TSPoint p = ts.getPoint();
    pinMode(XM, OUTPUT);  //.kbv ALWAYS restore pinMode()
    pinMode(YP, OUTPUT);

    static int i1, j1, i2, j2;
    static bool onP2 = false;

    if (!onP2 && p.z > ts.pressureThreshhold)
    {
      i1 = 2 - getCoordinate(p, 1, 'x');
      j1 = 2 - getCoordinate(p, 1, 'y');

      if (currImgArray[i1][j1][0] != blank[0] || currImgArray[i1][j1][1] != blank[1])
      {
        for (int i = 0; i < 3; i++)
          tft.drawRect(1 + WIDTH * j1 + i, HEIGHT * i1 + i, WIDTH - 2 * i, HEIGHT - 2 * i, 0xFFE0);
        onP2 = true;
        delay(400);
      }
    }

    else if (onP2 && p.z > ts.pressureThreshhold)
    {
      i2 = 2 - getCoordinate(p, 2, 'x');
      j2 = 2 - getCoordinate(p, 2, 'y');

      if (((abs(i2 - i1) == 1 && j1 == j2) || (abs(j2 - j1) == 1 && i1 == i2)) && (currImgArray[i2][j2][0] == blank[0] && currImgArray[i2][j2][1] == blank[1]))
      {
        swapArrElements(i1, j1, i2, j2, currImgArray);

        drawTile(i1, j1, currImgArray);
        drawTile(i2, j2, currImgArray);
      }
      else
        drawTile(i1, j1, currImgArray);

      onP2 = false;
      delay(400);
    }

    delay (100);
  }

  if (!notDone)
  {
    TSPoint p = ts.getPoint();
    pinMode(XM, OUTPUT);  //.kbv ALWAYS restore pinMode()
    pinMode(YP, OUTPUT);

    if (p.z > ts.pressureThreshhold)
    {
      if (state == 0)
      {

        state++;
      }

      else if (state == 1)
      {

        state++;
      }

      else if (state == 2)
      {

        state = 0;
      }
    }
  }
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
  tft.fillScreen(0x0000);
}
bool solved(char corrArr[3][3][8], char guessArr[3][3][8])
{
  for (int i = 0; i < 3; i ++)
    for (int j = 0; j < 3; j ++)
    {
      if (corrArr[i][j][0] != guessArr[i][j][0]) return false;
      if (corrArr[i][j][1] != guessArr[i][j][1]) return false;
    }
  return true;
}

void swapArrElements(int i1, int j1, int i2, int j2, char arr[][3][8])
{
  char temp[8];
  memcpy(temp,        arr[i1][j1], sizeof(char[8]));
  memcpy(arr[i1][j1], arr[i2][j2], sizeof(char[8]));
  memcpy(arr[i2][j2], temp,        sizeof(char[8]));
}

bool solvable(char arr[][3][8])
{
  int inv_count = 0;

  for (int i = 0; i < 9 - 1; i++)
  {
    int arrI = arr[i / 3][i % 3][0] * 3 + arr[i / 3][i % 3][1];
    for (int j = i + 1; j < 9; j++)
    {
      int arrJ = arr[j / 3][j % 3][0] * 3 + arr[j / 3][j % 3][1];

      if (arrJ && arrI &&  arrI > arrJ)
        inv_count++;
    }
  }

  return (inv_count % 2 == 0);
}

int getCoordinate(TSPoint p, int n, char axis)
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

void printCoordinates(int i, int j, int n, char arr[][3][8])
{
  Serial.print("i");
  Serial.print(n);
  Serial.print(" = ");
  Serial.print(i);
  Serial.print(", j");
  Serial.print(n);
  Serial.print(" = ");
  Serial.print(j);
  Serial.print(", file = ");
  Serial.println(arr[i][j]);
}

void printCoordinates(int i, int j, int x, int y, int n, char arr[][3][8])
{
  printCoordinates(i, j, n, arr);

  Serial.print("x = ");
  Serial.print(x);
  Serial.print(", y = ");
  Serial.println(y);
}

void drawImage(char arr[][3][8])
{
  tft.fillScreen(0x0000);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      drawTile(i, j, arr);
}

void drawTile(int i, int j, char arr[][3][8])
{
  if (arr[i][j][0] != blank[0] || arr[i][j][1] != blank[1])
  {
    char temp[19];
    strcpy(temp, imgSelect[imgSel]);
    strcat(temp, "/");
    strcat(temp, arr[i][j]);
    bmpDraw(temp, 1 + WIDTH * j, HEIGHT * i);
  }
    
  else
    tft.fillRect(1 + WIDTH * j, HEIGHT * i, WIDTH, HEIGHT, 0x0000);
}

void printArray(char arr[][3][8])
{
  for (int i = 0; i < 3; i++)
  {
    Serial.print("{");

    for (int j = 0; j < 3; j++)
    {
      if (j != 0)
        Serial.print(", ");
      Serial.print(arr[i][j]);
    }
    Serial.println("}");
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

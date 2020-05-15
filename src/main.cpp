/**
 * @author: Fran Acién and David Arias, with the help of Pablo Alvarez and Dani Kholer
 *  SSTV emitter using arduino DUE
 *
**/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <AD9850.h>
#include <JPEGDecoder.h>
#include <Adafruit_VC0706.h>
#include <DueTimer.h>
#include <Adafruit_GPS.h>

// Scottie 1 properties
#define COLORCORRECTION 3.1372549

// Scottie 1 mode
#define COLORSCANTIMEPERLINE 138.240        //ms
#define COLORSCANPERPIXEL 432               //microseconds
#define SEPARATORPULSETIME 1.5              //ms
#define SEPARATORPULSEFREQ 1500             //ms
#define SYNCPULSETIME 9                     //ms
#define SYNCPULSEFREQ 1200                  //Hz

// AD9850 consts
#define AD9850_CLK_PIN 51         //Working clock output pin
#define AD9850_FQ_UPDATE_PIN 49   //Frequency update
#define AD9850_DATA_PIN 47        //Serial data output pin
#define AD9850_RST_PIN 45         //Reset output pin

// Sd consts
#define SD_SLAVE_PIN 53
#define SD_CLOCK_PIN 13
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 12

// Other stuff
#define BUILT_IN_PIN 13

volatile uint8_t phase = 0;

char pic_filename[13];
char pic_decoded_filename[13];

uint8_t frameBuf[81920]; //320*256

volatile byte buffE[320]; // Buffer conintating Red values after torch
volatile byte buffR[320]; // Buffer conintating Red values of the line
volatile byte buffG[320]; // Buffer conintating Green values of the line
volatile byte buffB[320]; // Buffer conintating Blue values of the line

volatile byte sEm = 0;    // State of Emition
                    // 0 not emitting
                    // 1 emitting line (NOT HEADER OR VOX)
                    // 2 Change Color

volatile byte sCol = 0;   // Transmitting color Green
                    // Transmitting color Blue
                    // Transmitting color Red

volatile int tp = 0;     // Index of pixel while transmitting with timer
volatile int line;

// Camera stuff
Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);


uint16_t playPixel(long pixel);
uint16_t scottie_freq(uint8_t c);
void vox_tone();
void scottie1_calibrationHeader();
void transmit_micro(int freq, float duration);
void transmit_mili(int freq, float duration);
void scottie1_transmit_file(char* filename);
void shot_pic();
void jpeg_decode(char* filename, char* fileout);
//void writeHeader(File* dst, nmea_float_t latitude, char lat, nmea_float_t longitude, char lon, nmea_float_t altitude);    //Write 16 lines with values
void writeHeader(File* dst);

char charId[13] = "EA4RCT-SSTV-"; // ***** INFORMATION HEADER: MAX 12 CAHARCTERS *****
volatile long syncTime;

//FONTS
const uint8_t b_fonts[43][11] = {
        {0x00, 0x18, 0x24, 0x62, 0x62, 0x62, 0x7E, 0x62, 0x62, 0x62, 0x00}, //00: A
        {0x00, 0x7C, 0x32, 0x32, 0x32, 0x3C, 0x32, 0x32, 0x32, 0x7C, 0x00}, //01: B
        {0x00, 0x3C, 0x62, 0x62, 0x60, 0x60, 0x60, 0x62, 0x62, 0x3C, 0x00}, //02: C
        {0x00, 0x7C, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x7C, 0x00}, //03: D
        {0x00, 0x7E, 0x60, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x7E, 0x00}, //04: E
        {0x00, 0x7E, 0x60, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x60, 0x00}, //05: F
        {0x00, 0x3C, 0x62, 0x62, 0x60, 0x60, 0x66, 0x62, 0x62, 0x3C, 0x00}, //06: G
        {0x00, 0x62, 0x62, 0x62, 0x62, 0x7E, 0x62, 0x62, 0x62, 0x62, 0x00}, //07: H
        {0x00, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00}, //08: I
        {0x00, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x4C, 0x4C, 0x4C, 0x38, 0x00}, //09: J
        {0x00, 0x62, 0x64, 0x68, 0x70, 0x68, 0x64, 0x62, 0x62, 0x62, 0x00}, //10: K
        {0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00}, //11: L
        {0x00, 0x42, 0x62, 0x76, 0x6A, 0x62, 0x62, 0x62, 0x62, 0x62, 0x00}, //12: M
        {0x00, 0x42, 0x62, 0x72, 0x6A, 0x66, 0x62, 0x62, 0x62, 0x62, 0x00}, //13: N
        {0x00, 0x3C, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x3C, 0x00}, //14: O
        {0x00, 0x7C, 0x62, 0x62, 0x62, 0x7C, 0x60, 0x60, 0x60, 0x60, 0x00}, //15: P
        {0x00, 0x3C, 0x62, 0x62, 0x62, 0x62, 0x62, 0x6A, 0x6A, 0x3C, 0x08}, //16: Q
        {0x00, 0x7C, 0x62, 0x62, 0x62, 0x7C, 0x68, 0x64, 0x62, 0x62, 0x00}, //17: R
        {0x00, 0x3C, 0x62, 0x60, 0x60, 0x3C, 0x06, 0x06, 0x46, 0x3C, 0x00}, //18: S
        {0x00, 0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}, //19: T
        {0x00, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x3C, 0x00}, //20: U
        {0x00, 0x62, 0x62, 0x62, 0x62, 0x62, 0x62, 0x22, 0x14, 0x08, 0x00}, //21: V
        {0x00, 0x62, 0x62, 0x62, 0x62, 0x62, 0x6A, 0x76, 0x62, 0x42, 0x00}, //22: W
        {0x00, 0x42, 0x62, 0x74, 0x38, 0x1C, 0x2E, 0x46, 0x42, 0x42, 0x00}, //23: X
        {0x00, 0x42, 0x62, 0x74, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}, //24: Y
        {0x00, 0x7E, 0x06, 0x0E, 0x0C, 0x18, 0x30, 0x70, 0x60, 0x7E, 0x00}, //25: Z
        {0x00, 0x3C, 0x62, 0x62, 0x66, 0x6A, 0x72, 0x62, 0x62, 0x3C, 0x00}, //26: 0
        {0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}, //27: 1
        {0x00, 0x3C, 0x46, 0x06, 0x06, 0x1C, 0x20, 0x60, 0x60, 0x7E, 0x00}, //28: 2
        {0x00, 0x3C, 0x46, 0x06, 0x06, 0x1C, 0x06, 0x06, 0x46, 0x3C, 0x00}, //29: 3
        {0x00, 0x0C, 0x1C, 0x2C, 0x4C, 0x4C, 0x7E, 0x0C, 0x0C, 0x0C, 0x00}, //30: 4
        {0x00, 0x7E, 0x60, 0x60, 0x60, 0x7C, 0x06, 0x06, 0x46, 0x3C, 0x00}, //31: 5
        {0x00, 0x3C, 0x62, 0x60, 0x60, 0x7C, 0x62, 0x62, 0x62, 0x3C, 0x00}, //32: 6
        {0x00, 0x7E, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00}, //33: 7
        {0x00, 0x3C, 0x62, 0x62, 0x62, 0x3C, 0x62, 0x62, 0x62, 0x3C, 0x00}, //34: 8
        {0x00, 0x3C, 0x46, 0x46, 0x46, 0x3E, 0x06, 0x06, 0x46, 0x3C, 0x00}, //35: 9
        {0x00, 0x00, 0x02, 0x06, 0x0E, 0x1C, 0x38, 0x70, 0x60, 0x40, 0x00}, //36: /
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x00, 0x00, 0x00}, //37: -
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00}, //38: .
        {0x00, 0x3C, 0x46, 0x06, 0x06, 0x0C, 0x10, 0x00, 0x30, 0x30, 0x00}, //39: ?
        {0x00, 0x18, 0x18, 0x18, 0x18, 0x10, 0x10, 0x00, 0x18, 0x18, 0x00}, //40: !
        {0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00}, //41: :
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  //42: space
};

// Nibble font table
const uint8_t l_fonts[23][5] = {
  { 0xE2, 0xA6, 0xA2, 0xA2, 0xE2 }, // 0: 01
  { 0xEE, 0x22, 0xE6, 0x82, 0xEE }, // 1: 23
  { 0xAE, 0xA8, 0xEE, 0x22, 0x2E }, // 2: 45
  { 0x8E, 0x82, 0xE2, 0xA2, 0xE2 }, // 3: 67
  { 0xEE, 0xAA, 0xEE, 0xA2, 0xE2 }, // 4: 89
  { 0x00, 0x22, 0x00, 0x22, 0x04 }, // 5: :;
  { 0x20, 0x4E, 0x80, 0x4E, 0x20 }, // 6: <=
  { 0x8E, 0x42, 0x26, 0x40, 0x84 }, // 7: >?
  { 0x64, 0x9A, 0xBE, 0x8A, 0x7A }, // 8: @A
  { 0xC6, 0xA8, 0xC8, 0xA8, 0xC6 }, // 9: BC
  { 0xCE, 0xA8, 0xAC, 0xA8, 0xCE }, // 10: DE
  { 0xE6, 0x88, 0xCE, 0x8A, 0x86 }, // 11: FG
  { 0xA4, 0xA4, 0xE4, 0xA4, 0xA4 }, // 12: HI
  { 0x69, 0x2A, 0x2C, 0x2A, 0x49 }, // 13: JK
  { 0x8A, 0x8E, 0x8E, 0x8A, 0xEA }, // 14: LM
  { 0x04, 0x9A, 0xDA, 0xBA, 0x94 }, // 15: NO
  { 0xC4, 0xAA, 0xCA, 0x8E, 0x86 }, // 16: PQ
  { 0xC6, 0xA8, 0xC4, 0xA2, 0xAC }, // 17: RS
  { 0xE0, 0x4A, 0x4A, 0x4A, 0x44 }, // 18: TU
  { 0x09, 0xA9, 0xA9, 0x6F, 0x26 }, // 19: vW (sort of..)
  { 0x0A, 0xAA, 0x46, 0xA2, 0x04 }, // 20: XY
  { 0xE6, 0x24, 0x44, 0x84, 0xE6 }, // 21: Z[
  { 0x00, 0x00, 0x00, 0x00, 0x00 }  // 22: SPACE
};

void timer1_interrupt(){
  if (sEm == 1){
    if(tp < 320){  // Transmitting pixels
      if(sCol == 0){  // Transmitting color Green
        DDS.setfreq(1500 + 3.13 * buffG[tp], phase);
      } else if(sCol == 1){ // Transmitting color Blue
        DDS.setfreq(1500 + 3.13 * buffB[tp], phase);
      } else if(sCol == 2){ // Transmitting color Red
        DDS.setfreq(1500 + 3.13 * buffE[tp], phase);
      }
    } else if(tp == 320){
      if(sCol == 0){  // Separator pulse after transmit Green
        DDS.setfreq(1500, phase);
      } else if(sCol == 1){ // Sync porch
        DDS.setfreq(1200, phase);
      } else if(sCol == 2){ // // Separator pulse after transmit Red
        DDS.setfreq(1500, phase);
      }
      syncTime = micros();
      sEm = 2;    // State when change color
    }
    tp++;
  }
}

void setup() {
  delay(5000);
  pinMode(BUILT_IN_PIN, OUTPUT);
  pinMode(SD_SLAVE_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Starting");

  // AD9850 initilize
  DDS.begin(AD9850_CLK_PIN, AD9850_FQ_UPDATE_PIN, AD9850_DATA_PIN, AD9850_RST_PIN);

  // Sd initialize
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_SLAVE_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // Setup Timer with the emision interval
  Timer1.attachInterrupt(timer1_interrupt).start(430); // ***** 354(uS/px) +/- SLANT ADJUST *****
  delay(100);

  shot_pic();

  Serial.print("Picture taken saved on:");
  Serial.println(pic_filename);

  strcpy(pic_decoded_filename, pic_filename);
  pic_decoded_filename[8] = 'B';
  pic_decoded_filename[9] = 'I';
  pic_decoded_filename[10] = 'N';

  Serial.print("Writting on: ");
  Serial.println(pic_decoded_filename);

  jpeg_decode(pic_filename, pic_decoded_filename);

  scottie1_transmit_file(pic_decoded_filename);
}

void loop() {
  // put your main code here, to run repeatedly:


}

/**
 * Get output frequency given a color component (R, G, B) from 0 to 255
 * @param uint8_t c - single color component from 0 to 255
 * @return uint16_t - scottie1 frequency
**/
uint16_t scottie_freq(uint8_t c){
  return 1500 + (c * COLORCORRECTION);
}

void vox_tone(){
  /** VOX TONE (OPTIONAL) **/
  transmit_mili(1900, 100);
  transmit_mili(1500, 100);
  transmit_mili(1900, 100);
  transmit_mili(1500, 100);
  transmit_mili(2300, 100);
  transmit_mili(1500, 100);
  transmit_mili(2300, 100);
  transmit_mili(1500, 100);
}

void scottie1_calibrationHeader(){
  /** CALIBRATION HEADER **/
  transmit_mili(1900, 300);
  transmit_mili(1200, 10);
  transmit_mili(1900, 300);
  transmit_mili(1200, 30);
  transmit_mili(1300, 30);    // 0
  transmit_mili(1300, 30);    // 0
  transmit_mili(1100, 30);    // 1
  transmit_mili(1100, 30);    // 1
  transmit_mili(1100, 30);    // 1
  transmit_mili(1100, 30);    // 1
  transmit_mili(1300, 30);    // 0
  transmit_mili(1300, 30);    // Even parity
  transmit_mili(1200, 30);    // VIS stop bit
}

/**
 * Set a frequency for a certain amount of time
 * @param int freq - Frequency
 * @param float duration - duration in microseconds
 */
void transmit_micro(int freq, float duration){
  DDS.setfreq(freq, phase);
  delayMicroseconds(duration);
}

/**
 * Set a frequency for a certain amount of time
 * @param int freq - Frequency
 * @param float duration - duration in milliseconds
 */
void transmit_mili(int freq, float duration){
  DDS.setfreq(freq, phase);
  delay(duration);
}

void scottie1_transmit_file(char* filename){
  /*
  Be aware that you have to read variables on sync torch due its 9 ms instead 1.5 ms of the sync Pulse
  */

  bool head;
  Serial.println("Transmitting picture");

  File myFile = SD.open(filename);
  if (myFile) {
    head = true;

    /** TRANSMIT EACH LINE **/
    while(myFile.available() || line == 255){
      if(head == true){ // Header
        /** VOX TONE (OPTIONAL) **/
        vox_tone();

        /** CALIBRATION HEADER **/
        scottie1_calibrationHeader();

        // Configure syncTime
        syncTime = micros();

        // Read line and store color values in the buffer
        for(uint16_t i = 0; i < 320; i++){
          buffR[i] = myFile.read();
          buffG[i] = myFile.read();
          buffB[i] = myFile.read();
        }

        //Serial.println("++");
        //Serial.println(micros() - syncTime); //Cheak reading time

        while(micros() - syncTime < 9000 - 10){}

        // Separator pulse
        DDS.setfreq(1500, phase);
        syncTime = micros();  // Configure syncTime

        line = 0;
        head = false;
      }

      while(micros() - syncTime < 1500 - 10){} // Separator pulse

      // Green Scan
      tp = 0; sCol = 0; sEm = 1;
      while(sEm == 1){};

      // Separator Pulse
      DDS.setfreq(1500, phase);
      while(micros() - syncTime < 1500 - 10){}

      // Blue Scan
      tp = 0; sCol = 1; sEm = 1;
      while(sEm == 1){};

      //Evacuate
      for(uint16_t i = 0; i < 320; i++){
        buffE[i] = buffR[i];
      }

      if(line != 255){
        // Read line and store color values in the buffer
        for(uint16_t i = 0; i < 320; i++){
          buffR[i] = myFile.read();
          buffG[i] = myFile.read();
          buffB[i] = myFile.read();
        }
      }

      //Serial.println("--");
      //Serial.println(micros() - syncTime); //Cheak reading time

      //Sync pulse
      while(micros() - syncTime < 9000 - 10){}

      // Sync porch
      DDS.setfreq(1500, phase);
      syncTime = micros();
      while(micros() - syncTime < 1500 - 10){}

      // Red Scan
      tp = 0; sCol = 2; sEm = 1;
      while(sEm == 1){};

      line++;
      if(line == 256){
        Serial.println("Finish");
        DDS.setfreq(2, phase);
        DDS.down();
        sEm = 0;
      }
      else {
        // Separator pulse
        DDS.setfreq(1500, phase);
        syncTime = micros();
        sEm = 2;
      }
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void jpeg_decode(char* filename, char* fileout){
  uint8 *pImg;
  int x,y,bx,by;
  byte sortBuf[15360]; //320(px)*16(lines)*3(bytes) // Header buffer
  int i,j,k;
  int pxSkip;

  // Open the file for writing
  File imgFile = SD.open(fileout, FILE_WRITE);

  for(i = 0; i < 15360; i++){ // Cleaning Header Buffer array
    sortBuf[i] = 0xFF;
  }

  for(i = 0; i < 12; i++){
    byte fontNumber;
    char ch;
    ch = charId[i];
    for(y = 0; y < 11; y++){
      for(x = 0; x < 8; x++){
        pxSkip = 16 + (320 * (y + 3)) + (3 * 8 * i) + (3 * x); //Width: x3

        uint8_t mask;
        mask = pow(2, 7 - x);

        if(ch >= 65 && ch <= 90){ // A to Z
                fontNumber = ch - 65;
        }
        else if(ch >= 48 && ch <= 57){ //0 to 9
                fontNumber = ch - 22;
        }
        else if(ch == '/'){fontNumber = 36;}
        else if(ch == '-'){fontNumber = 37;}
        else if(ch == '.'){fontNumber = 38;}
        else if(ch == '?'){fontNumber = 39;}
        else if(ch == '!'){fontNumber = 40;}
        else if(ch == ':'){fontNumber = 41;}
        else if(ch == ' '){fontNumber = 42;}
        else              {fontNumber = 42;}

        if((b_fonts[fontNumber][y] & mask) != 0){
          for(j = 0; j < 9; j++){
                  sortBuf[(3 * pxSkip) + j] = 0x00;
          }
        }
      }
    }
  }

  for(k = 0; k < 15360; k++){  // Adding header to the binary file
    imgFile.write(sortBuf[k]);
  }

  writeHeader(&imgFile);  //Writing first 10560 bytes (11*320*3)

  // Decoding start
  JpegDec.decode(filename,0);
  // Image Information
  Serial.print("Width     :");
  Serial.println(JpegDec.width);
  Serial.print("Height    :");
  Serial.println(JpegDec.height);
  Serial.print("Components:");
  Serial.println(JpegDec.comps);
  Serial.print("MCU / row :");
  Serial.println(JpegDec.MCUSPerRow);
  Serial.print("MCU / col :");
  Serial.println(JpegDec.MCUSPerCol);
  Serial.print("Scan type :");
  Serial.println(JpegDec.scanType);
  Serial.print("MCU width :");
  Serial.println(JpegDec.MCUWidth);
  Serial.print("MCU height:");
  Serial.println(JpegDec.MCUHeight);
  Serial.println("");

  Serial.println("Writting bin to SD");

  i = 0;
  j = 0;
  while(JpegDec.read()){
    pImg = JpegDec.pImage ;
    for(by=0; by<JpegDec.MCUHeight; by++){
      for(bx=0; bx<JpegDec.MCUWidth; bx++){
        x = JpegDec.MCUx * JpegDec.MCUWidth + bx;
        y = JpegDec.MCUy * JpegDec.MCUHeight + by;
        if(x<JpegDec.width && y<JpegDec.height){
          if(JpegDec.comps == 1){ // Grayscale
            //sprintf(str,"%u", pImg[0]);
            imgFile.write(pImg, 1);
          }else{ // RGB
            // When saving to the SD, write 16 lines on one time
            // First we write on the array 16 lines and then we save to SD
            pxSkip = ((y - (16 * j)) * 320) + x;
            sortBuf[(3 * pxSkip) + 0] = pImg[0];
            sortBuf[(3 * pxSkip) + 1] = pImg[1];
            sortBuf[(3 * pxSkip) + 2] = pImg[2];

            i++;
            if(i == 5120){ //320(px)x16(lines)
              for(k = 0; k < 15360; k++){
                imgFile.write(sortBuf[k]);
              }
              i = 0;
              j++; //15(sections)
            }
          }
        }
        pImg += JpegDec.comps ;
      }
    }
  }

  Serial.println("Bin has been written on SD");
  imgFile.close();
}

void shot_pic(){
  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }

  for (int i = 0; i <= 10; i++){
    cam.setImageSize(VC0706_320x240);
  }

  Serial.println("Snap in 3 secs...");
  delay(3000);

  if (! cam.takePicture())
    Serial.println("Failed to snap!");
  else
    Serial.println("Picture taken!");

  // Create an image with the name IMAGExx.JPG`
  strcpy(pic_filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    pic_filename[5] = '0' + i/10;
    pic_filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(pic_filename)) {
      break;
    }
  }

  // Open the file for writing
  File imgFile = SD.open(pic_filename, FILE_WRITE);

  // Get the size of the image (frame) taken
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");
}

/**     Write on a file with 11 lines the values of the GPS
 * @param dst Given an opened File stream then write data to dst.
 * @param latitude Floating point latitude value in degrees/min as received from the GPS (DDMM.MMMM)
 * @param lat N/S
 * @param longitude Floating point longitude value in degrees/min as received from the GPS (DDMM.MMMM)
 * @param lon E/W
 * @param altitude Altitude in meters above MSL
 */

//void writeHeader(File* dst, nmea_float_t latitude, char lat, nmea_float_t longitude, char lon, nmea_float_t altitude){    //Write 16 lines with values
void writeHeader(File* dst){
  int x,y;
  byte sortBuf[10560]; //320(px)*11(lines)*3(bytes) // Header buffer
  int i,j,k;
  int pxSkip;

  char res[51] = "LAT: 1234.1234N     LONG: 1234.1234W     ALT:10000";

  for(i = 0; i < 10560; i++){ // Cleaning Header Buffer array
    sortBuf[i] = 0xFF;
  }

  for(i = 0; i < sizeof(res); i++){
    byte fontNumber;
    char ch;
    ch = res[i];
    for(y = 0; y < 5; y++){
      for(x = 0; x < 4; x++){
        //pxSkip = HORIZONTALOFFSET + VERSTICALOFFSET + (BITSPERWORD * i);
        //pxSkip = 16 + (320 * (y + 3)) + (4 * 2 * i) + (2 * x); Width: x2
        pxSkip = 16 + (320 * (y + 3)) + (4 * i) + x;

        // If ch is pair mask is: 11110000, if no 00001111
        uint8_t sl = (ch % 2)? 3 : 7 ;
        uint8_t mask = pow(2, sl - x);

        if(ch >= 48 && ch <=91){
          fontNumber = (ch-48)/2;
        }
        else {
          fontNumber = 22;
        }

        if((l_fonts[fontNumber][y] & mask) != 0){
          for(j = 0; j < 3; j++){
                  sortBuf[(3 * pxSkip) + j] = 0x00;
          }
        }
      }
    }
  }

  for(k = 0; k < 10560; k++){  // Adding header to the binary file
    dst->write(sortBuf[k]);
  }
}

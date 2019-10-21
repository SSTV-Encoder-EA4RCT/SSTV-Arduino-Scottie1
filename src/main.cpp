#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <AD9850>
#include <JPEGDecoder>
#include <Adafruit_VC0706.h>

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
#define AD9850_CLK_PIN 99         //Working clock output pin
#define AD9850_FQ_UPDATE_PIN 99   //Frequency update
#define AD9850_DATA_PIN 99        //Serial data output pin
#define AD9850_RST_PIN 99         //Reset output pin

// Sd consts
#define SD_SLAVE_PIN 53
#define SD_CLOCK_PIN 13
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 12

uint8_t phase = 0;

char pic_filename[13];

uint8_t frameBuf[81920]; //320*256

byte buffR[320]; // Buffer conintating Red values of the line
byte buffG[320]; // Buffer conintating Green values of the line
byte buffB[320]; // Buffer conintating Blue values of the line

// Camera stuff
SoftwareSerial cameraconnection = SoftwareSerial(2, 3);
Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);


uint16_t playPixel(long pixel);
uint16_t scottie_freq(uint8_t c);
void vox_tone();
void scottie1_calibrationHeader();
void transmit_micro(int freq, float duration);
void transmit_mili(int freq, float duration);
void scottie1_transmit_file(String filename);
void shot_pic();
void jpeg_decode();

void setup() {
  Serial.begin(9600);

  // AD9850 initilize
  DDS.begin(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN);

  // Sd initialize
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDSLAVE)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  shot_pic();
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

void scottie1_transmit_file(String filename){
  File myFile = SD.open(path);
  if (myFile) {
    Serial.println("test.txt:");

    /** VOX TONE (OPTIONAL) **/
    vox_tone();

    /** CALIBRATION HEADER **/
    scottie1_calibrationHeader();

    /** STARTING SYNC PULSE (FIRST LINE ONLY)  **/
    transmit_micro(1200, 9000);

    int line = 0;
    /** TRANSMIT EACH LINE **/
    while(myFile.available()){
      // Read line and store color values in the buffer
      for(uint16_t i = 0; i < 320; i++){
        buffR[i] = myFile.read();
        buffG[i] = myFile.read();
        buffB[i] = myFile.read();
      }

      // Separator pulse
      transmit_micro(1500, 1500);

      // Green Scan
      for(uint16_t i = 0; i < 320; i++){
        transmit_micro(scottie_freq(buffG[i]), 432);    // .4320ms/pixel
      }

      // Separator Pulse
      transmit_micro(1500, 1500);

      // Blue Scan
      for(uint16_t i = 0; i < 320; i++){
        transmit_micro(scottie_freq(buffB[i]), 432);    // .4320ms/pixel
      }

      // Sync Pulse
      transmit_micro(1200, 9000);

      // Sync porch
      transmit_micro(1500, 1500);

      // Red Scan
      for(uint16_t i = 0; i < 320; i++){
        transmit_micro(scottie_freq(buffR[i]), 432);    // .4320ms/pixel
      }
    }

    Serial.println("Finish");

    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void jpeg_decode(String filename){
  char str[100];
  uint8 *pImg;
  int x,y,bx,by;

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

  while(JpegDec.read()){
      pImg = JpegDec.pImage ;
      for(by=0; by<JpegDec.MCUHeight; by++){
          for(bx=0; bx<JpegDec.MCUWidth; bx++){
              x = JpegDec.MCUx * JpegDec.MCUWidth + bx;
              y = JpegDec.MCUy * JpegDec.MCUHeight + by;
              if(x<JpegDec.width && y<JpegDec.height){
                  if(JpegDec.comps == 1){ // Grayscale
                      sprintf(str,"%u", pImg[0]);
                  }else{ // RGB
                      sprintf(str,"%u%u%u", pImg[0], pImg[1], pImg[2]);
                  }
              }
              pImg += JpegDec.comps ;
          }
      }
  }
}

void shot_pic(){
  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }

  cam.setImageSize(VC0706_320x240);

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

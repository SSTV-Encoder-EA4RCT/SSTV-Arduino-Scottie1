#include <Arduino.h>
#include <AD9850>
#include <JPEGDecoder>

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

byte buffR[320]; // Buffer conintating Red values of the line
byte buffG[320]; // Buffer conintating Green values of the line
byte buffB[320]; // Buffer conintating Blue values of the line

uint16_t playPixel(long pixel);
uint16_t scottie_freq(uint8_t c);
void vox_tone();
void scottie1_calibrationHeader();
void transmit_micro(int freq, float duration);
void transmit_mili(int freq, float duration);
void scottie1_transmit_file(String filename);

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

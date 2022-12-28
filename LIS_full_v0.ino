/*********
  Adapted in part from the code provided by Rui Santos
  at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card

   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory

#include <Wire.h>

#include "TSYS01.h"

#include "MS5837.h"

#include <Ezo_i2c.h>

#include <Ezo_i2c_util.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS myGNSS;

// define the number of bytes you want to access
#define EEPROM_SIZE 1

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define MCPAddress  0x27 //I2C Address

#define IO_DIR_REG 0x00           // The Output Register
#define GPIO_REG 0x09             // The GPIO Register
#define GPPU 0x06
#define IOCON 0x05                // Settings
#define SEQOP_REQ 0b00100000      // Disable address increment

#define I2C_SDA           13
#define I2C_SCL           12

#define packetLength 100

#define SAMPLING_CYCLES 500

Ezo_board EC = Ezo_board(100, "EC");

MS5837 sensor2;

TSYS01 sensor1;

int pictureNumber = 0;
int experimentNumber = 0;

int error;
uint8_t button;
bool blink_on;
int sampling_elapsed;
int cycles_until_sampling;
float calib_depth;

uint8_t *myBuffer;

int mode;
int param;
int cam_on;
bool init_loop = 0;
bool init_GPS = 0;
unsigned long runtime;

void setup() {
  delay(500); //try and fix the bug here by pulling io12 to ground
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  //Serial.println();
  //while(true){
  // Serial.println("I'm here");
  //  delay(500);
  //}

  Wire.begin(I2C_SDA, I2C_SCL);

  Wire.beginTransmission(MCPAddress); // Check if connection succesfull
  error = Wire.endTransmission();
  if(error == 0){
    Serial.println("Success IO expander");
  }else{
    Serial.println("Failure: IO expander ");
    Serial.print(error);
  }
  Serial.println("Setting Outputs IO ex");
  writeBlockData(IO_DIR_REG,0x02);
  Serial.println("Enabling pullup on pin 0 and 1");
  writeBlockData(GPPU,0x05);
  Serial.println("Writing LOW IO exp");
  writeBlockData(GPIO_REG,0x00);

  sensor1.init();

  while (!sensor2.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor2.setModel(MS5837::MS5837_30BA);
  sensor2.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  //____________calibration of the depth sensor in the air______________

  sensor2.read();
  calib_depth = sensor2.depth();
  Serial.print("calibration depth is:");
  Serial.println(calib_depth);

  //____________end of depth calibration________________________________
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  
  s->set_brightness(s, 2);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 300);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 1);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    
  //Serial.println("Starting SD Card");
  if(!SD_MMC.begin("/sdcard", true)){
    Serial.println("SD Card Mount Failed");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }
    
    //________________________reading config from SD file, config.txt___________
  /* Format of the text file: description of the parameters followed by a double point and then their value. Then disclaimer */
  fs::FS &fs = SD_MMC; 
  File settings = fs.open("/config.txt",FILE_READ);
  if(!settings){
    Serial.println("Failed to open settings file");
  }
  else{
    char ch;
    char var1[10];
    char var2[10];
    char var3[10];
    int p =0;
    while (settings.available()){
      ch = settings.read();
      if (ch == ':'){
        while (settings.available() && (!(ch == ','))){
          ch = settings.read();
          var1[p] = ch;
          p++;          
        }
        var1[p+1] = '\0';
        p = 0;
        mode = strtol(var1,NULL,10);
        ch =  '\0';
        while (settings.available() && (!(ch == ','))){
          ch = settings.read();
          var2[p] = ch;
          p++;
        }
        var2[p+1] = '\0';
        p = 0;
        param = strtol(var2,NULL,10); 
        ch =  '\0';
        while (settings.available() && (!(ch == ','))){
          ch = settings.read();
          var3[p] = ch;
          p++;
        }
        var3[p+1] = '\0';
        cam_on = strtol(var3,NULL,10);
        break;
        
      }
    }    
  }
  settings.close();
  Serial.print("Settings read: mode: ");
  Serial.print(mode);
  Serial.print(", param: ");
  Serial.print(param);
  Serial.print(", cam_on: ");
  Serial.println(cam_on);
  cycles_until_sampling = param;
  camera_fb_t * fb = NULL;
  
  // Take Picture with Camera
  pinMode(4, OUTPUT);
  if (cam_on){
    digitalWrite(4, HIGH);
    fb = esp_camera_fb_get();  
    if(!fb) {
      Serial.println("Camera capture failed");
      return;
    }
    for (int i = 0; i <= 20; i++) {
      delay(50);
      esp_camera_fb_return(fb);
      fb = esp_camera_fb_get();
    }
    // initialize EEPROM with predefined size
    EEPROM.begin(EEPROM_SIZE);
    //experimentNumber = EEPROM.read(0) + 1;
    pictureNumber = pictureNumber+1;
  
    // Path where new picture will be saved in SD Card
    String path = "/experiment" + String(experimentNumber) + "_picture" + String(pictureNumber) +".jpg";

    Serial.printf("Picture file name: %s\n", path.c_str());
    
    File file = fs.open(path.c_str(), FILE_WRITE);
    if(!file){
      Serial.println("Failed to open file in writing mode");
    } 
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.printf("Saved file to path: %s\n", path.c_str());
      //EEPROM.write(0, experimentNumber);
      //EEPROM.commit();
    }
    file.close();
    esp_camera_fb_return(fb); 
    
    // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
    digitalWrite(4, LOW);
    //rtc_gpio_hold_en(GPIO_NUM_4);
  }
  fs.remove("/experiment" + String(experimentNumber) + "_logs.txt");

  //________________________GNSS module initialization____________________

  myGNSS.setFileBufferSize(301); // setFileBufferSize must be called _before_ .begin

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address"));
  }
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setNavigationFrequency(1); //Produce one navigation solution per second
  myGNSS.setAutoPVTcallbackPtr(&logPVTdata); // Enable automatic NAV PVT messages with callback to logPVTdata
  myGNSS.logNAVPVT(); // Enable NAV PVT data logging
  //myBuffer = new uint8_t[packetLength]; // Create our own buffer to hold the data while we write it to SD card

  //________________________end of GNSS module initialization_________________

  delay(100);

  Serial.println("setup and calibration finished, starting continuous sampling");
  //esp_deep_sleep_start();
}

void loop() {
  //Serial.println("Reading values IOexp");
  button = readBlockData(GPIO_REG,1)&0x02;
  //Serial.print("Button is:");
  //Serial.println(button);
  switch(mode){
    case 0:
      break;

    case 1:
      //duration mode
      //Serial.println("in mode 1 switch case");
      if (cycles_until_sampling == 0){
        sampling_elapsed = SAMPLING_CYCLES;
      }
      break;
    case 2:
      //depth mode
      if ((sensor2.depth()-calib_depth)> param && param ){
        sampling_elapsed = SAMPLING_CYCLES;
        param = 0;
      }
      break;
  }
  cycles_until_sampling--;
  cycles_until_sampling = max(cycles_until_sampling,-1); //cap the cycle counter to -1 to prevent ovf
  if ((button > 0) || (sampling_elapsed > 0)){
    Serial.println("sampling ON");
    writeBlockData(GPIO_REG,0x01);
  }
  else{
    if (blink_on){
      writeBlockData(GPIO_REG,0b11111100);
      blink_on = false;
    }
    else{
      writeBlockData(GPIO_REG,0b11111000);
      blink_on = true;
    }
    
  }
  sampling_elapsed--;
  sampling_elapsed = max(sampling_elapsed,0); //cap the cycle counter to 0 to prevent ovf
  
  sensor1.read();

  Serial.print("Temperature acc: ");
  
  Serial.print(sensor1.temperature()); 
  
  Serial.println(" deg C");

  //EC.send_read_cmd();

  delay(600);
  
  //receive_and_print_reading(EC); 

  // Update pressure and temperature readings
  sensor2.read();

  //Serial.print("Pressure: "); 
  //Serial.print(sensor2.pressure()); 
  //Serial.println(" mbar");
  
  //Serial.print("Temperature inacc: "); 
  //Serial.print(sensor2.temperature()); 
  //Serial.println(" deg C");
  
  Serial.print("Depth: "); 
  Serial.print(sensor2.depth()-calib_depth); 
  Serial.println(" m");
  
  //Serial.print("Altitude: "); 
  //Serial.print(sensor2.altitude()); 
  //Serial.println(" m above mean sea level");

//_________________________process GNSS module data________________________
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();
//_________________________end process GNSS module data____________________
  

  fs::FS &fs = SD_MMC; 
  String path2 = "/experiment" + String(experimentNumber) + "_logs.txt"; 
  //Serial.printf("logs file name: %s\n", path2.c_str());
  char convert_str[7];
  File file2 = fs.open(path2.c_str(), FILE_WRITE);
  if (init_loop){
    file2.seek(file2.size());
  }
  else{init_loop = 1;}
  runtime = millis();
  dtostrf(runtime/1000.0,5,2,convert_str);
  file2.write((const uint8_t*)convert_str,7);
  file2.write(0x2C);
  dtostrf(sensor1.temperature(),5,2,convert_str);
  file2.write((const uint8_t*)convert_str,7);
  file2.write(0x2C);
  dtostrf(sensor2.temperature(),5,2,convert_str);
  file2.write((const uint8_t*)convert_str,7);
  file2.write(0x2C);
  dtostrf(sensor2.depth()-calib_depth,5,2,convert_str);
  file2.write((const uint8_t*)convert_str,7);
  file2.write(0x2C);
  dtostrf(EC.get_last_received_reading(),5,2,convert_str);
  file2.write((const uint8_t*)convert_str,7);
  file2.write(0x2C);
  dtostrf((float)(sampling_elapsed > 0),1,0,convert_str);  
  file2.write((const uint8_t*)convert_str,2);
  file2.write(0x0A);
  file2.close();   

//____________________continuous camera captures______________________  
  if (cam_on){
    camera_fb_t * fb = NULL;
    digitalWrite(4, HIGH);
    fb = esp_camera_fb_get();  
    if(!fb) {
      Serial.println("Camera capture failed");
    }
    pictureNumber = pictureNumber+1;
    // Path where new picture will be saved in SD Card
    String path_cam_cont  = "/experiment" + String(experimentNumber) + "_picture" + String(pictureNumber) +".jpg";
    //fs::FS &fs = SD_MMC; 
    Serial.printf("Picture file name: %s\n", path_cam_cont.c_str());
    File file4 = fs.open(path_cam_cont.c_str(), FILE_WRITE);
    file4.write(fb->buf, fb->len); // payload (image), payload length
    file4.close();
    esp_camera_fb_return(fb); 
    digitalWrite(4, LOW);
  }
  //_________________________end cont. camera captures_______________________

}

int writeBlockData(uint8_t cmd, uint8_t val)
{
  Wire.beginTransmission(MCPAddress);
  Wire.write(cmd);
  Wire.write(val);
  delay(10);
  return Wire.endTransmission();
}

uint8_t readBlockData(uint8_t reg, int nb_byte)
{
  uint8_t v1;

  Wire.beginTransmission(MCPAddress);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MCPAddress,nb_byte);

  v1=Wire.read();
  Wire.endTransmission();

  delay(10);
  return v1;
}

void logPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct){
  char buffer_PVT[33];
  fs::FS &fs = SD_MMC; 
  String path2 = "/experiment" + String(experimentNumber) + "_GPS_logs.txt"; 
  //Serial.printf("logs file name: %s\n", path2.c_str());
  Serial.println("Ublox logging callback now");
  File file3 = fs.open(path2.c_str(), FILE_WRITE);
  if (init_GPS){
    file3.seek(file3.size());
  }
  else {init_GPS = 1;}
  runtime = millis();
  dtostrf(runtime/1000.0,5,2,buffer_PVT);
  file3.write((const uint8_t*)buffer_PVT,9);
  file3.write(0x2C);
  uint8_t hms = ubxDataStruct->hour; // Print the hours
  dtostrf((float)hms,2,0,buffer_PVT);
  file3.write((const uint8_t*)buffer_PVT,2);
  file3.write(0x3A);
  hms = ubxDataStruct->min; // Print the minutes
  dtostrf((float)hms,2,0,buffer_PVT);
  file3.write((const uint8_t*)buffer_PVT,2);
  file3.write(0x3A);
  
  hms = ubxDataStruct->sec; // Print the seconds
  dtostrf((float)hms,2,0,buffer_PVT);
  file3.write((const uint8_t*)buffer_PVT,3);
  file3.write(0x2C);

  
  long latitude = ubxDataStruct->lat; // Print the latitude
  dtostrf((float)latitude,20,1,buffer_PVT);
  file3.write((const uint8_t*)buffer_PVT,20);
  file3.write(0x2C);

  long longitude = ubxDataStruct->lon; // Print the longitude
  dtostrf((float)longitude,20,1,buffer_PVT);
  file3.write((const uint8_t*)buffer_PVT,20);
  file3.write(0x2C);

  long altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
  dtostrf((float)altitude,5,0,buffer_PVT);
  file3.write((const uint8_t*)buffer_PVT,5);
  file3.write(0x0A);  
  
  file3.close();
}
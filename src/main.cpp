// by SATANASSI Alessandro

#include <Arduino.h>


#include "TinyUSB_Devices.h"

#if defined(DEVICE_LILYGO_T_DONGLE_S3)
  #include "logo.h"
  #include "pin_config_DONGLE.h"
  #include <Adafruit_ST7735.h>
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
  
  #define WHITE            0xFFFF
  #define BLACK            0x0000
  #define BLUE             0xF800
  #define BRED             0XF81F
  #define GRED             0XFFE0
  #define GBLUE            0X07FF
  #define RED              0x001F
  #define MAGENTA          0xF81F
  #define GREEN            0x07E0
  #define CYAN             0x7FFF
  #define YELLOW           0xFFE0
  #define BROWN            0XBC40
  #define BRRED            0XFC07
  #define GRAY             0X8430
#endif // DEVICE_LILYGO_T_DONGLE_S3

#define OPENFIRE_DONGLE_VERSION 6.9
#define OPENFIRE_DONGLE_CODENAME "Sessantanove"

// ================== GESTIONE DUAL CORE ============================
#if defined(DUAL_CORE) && defined(ESP_PLATFORM) && false
        void setup1();
        void loop1();
        TaskHandle_t task_loop1;
        void esploop1(void* pvParameters) {
            setup1();
            for (;;) loop1();
        }
#endif //DUAL_CORE
// ========================= FINE GESTIONE DUAL CORE ======================================

// The main show!
void setup() {
  // =========================== X GESTIONE DUAL CORE =============================== 
  #if defined(DUAL_CORE) && defined(ESP_PLATFORM) && false
    xTaskCreatePinnedToCore(
    esploop1,               /* Task function. */
    "loop1",                /* name of task. */
    10000,                  /* Stack size of task */
    NULL,                   /* parameter of the task */
    1,                      /* priority of the task */
    &task_loop1,            /* Task handle to keep track of created task */
    !ARDUINO_RUNNING_CORE); /* pin task to core 0 */
  #endif
  // ======================== FINE X GESTIONE DUAL CORE =================================       
 
  #if defined(DEVICE_LILYGO_T_DONGLE_S3)
    pinMode(TFT_LEDA_PIN, OUTPUT);
    digitalWrite(TFT_LEDA_PIN, 0); // accende retroilluminazione del display
  #endif // DEVICE_LILYGO_T_DONGLE_S3
  
  #if defined(DEVICE_LILYGO_T_DONGLE_S3)
    tft.initR(INITR_MINI160x80_PLUGIN);  // Init ST7735S mini display
    tft.setRotation(3);
    
    /*
    // logo OpenFire
    tft.fillScreen(BLACK);
    tft.drawBitmap(40, 0, customSplashBanner, CUSTSPLASHBANN_WIDTH, CUSTSPLASHBANN_HEIGHT, BLUE); // logo tondo
    tft.drawBitmap(56, 23, customSplash, CUSTSPLASH_WIDTH, CUSTSPLASH_HEIGHT, RED); // scritta "OpenFire"
    //tft.drawXBitmap();
    delay (1000);
    tft.fillScreen(BLACK);
    tft.drawRGBBitmap(50,17,(uint16_t *)logo_rgb,LOGO_RGB_WIDTH,LOGO_RGB_HEIGHT);
    delay (1000);
    tft.fillScreen(BLACK);
    tft.drawRGBBitmap(55,20,(uint16_t *)logo_rgb_alpha,LOGO_RGB_ALPHA_WIDTH,LOGO_RGB_ALPHA_HEIGHT);
    delay (1000);
    */
    tft.fillScreen(BLACK);
    tft.drawRGBBitmap(10,1,(uint16_t *)logo_rgb_alpha_open,LOGO_RGB_ALPHA_OPEN_WIDTH,LOGO_RGB_ALPHA_OPEN_HEIGHT);
    delay (3000);

    tft.fillScreen(BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextColor(RED);
    tft.println("..SEARCHING..");
    //tft.drawRGBBitmap(0,20,(uint16_t *)logo_rgb,LOGO_RGB_WIDTH,LOGO_RGB_HEIGHT);
    //tft.drawRGBBitmap(5,25,(uint16_t *)logo_rgb_alpha,LOGO_RGB_ALPHA_WIDTH,LOGO_RGB_ALPHA_HEIGHT);
    tft.drawBitmap(10, 25, customSplash, CUSTSPLASH_WIDTH, CUSTSPLASH_HEIGHT, BLUE);
    tft.setTextSize(2);
    tft.setCursor(65, 30);
    tft.setTextColor(GRAY);
    tft.println("Channel");
    tft.setTextSize(2);
    //tft.setCursor(100, 60);
    tft.setTextColor(WHITE);
    //tft.printf("Player: %2d", 1);
    //tft.println("1");
    //tft.fillRect(100,60,50,20,0/*BLACK*/);

  #endif // DEVICE_LILYGO_T_DONGLE_S3

  // ====== gestione connessione wireless ====================
  SerialWireless.begin();
  SerialWireless.connection_dongle();
  // ====== fine gestione wireless .. va avanti solo dopo che si è accoppiato il dispositivo =======

  // ====== connessione USB ====== imposta VID e PID come quello che gli passa la pistola ===============
  if (!TinyUSBDevice.isInitialized()) { // aggiunto ..funzionava lo stesso, ma così è più sicuro .. sicuramente serve per Esp32 con libreria non integrfata nel core
    TinyUSBDevice.begin(0);
  }
      
  TinyUSBDevice.setManufacturerDescriptor(usb_data_wireless.deviceManufacturer);
  TinyUSBDevice.setProductDescriptor(usb_data_wireless.deviceName);
  TinyUSBDevice.setID(usb_data_wireless.deviceVID, usb_data_wireless.devicePID);

  // Initializing the USB devices chunk.
  TinyUSBDevices.begin(1); 
  // ====== fine connessione USB ==========================================================================
  
  #if defined(DEVICE_LILYGO_T_DONGLE_S3)
    
    tft.fillScreen(BLACK);
    /*
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextColor(RED);
    tft.println("CONNESSO");
    */
    tft.drawBitmap(40, 0, customSplashBanner, CUSTSPLASHBANN_WIDTH, CUSTSPLASHBANN_HEIGHT, BLUE);
    tft.setTextSize(2);
    tft.setCursor(0, 20);
    tft.setTextColor(RED);
    tft.println(usb_data_wireless.deviceName);
    tft.setTextColor(RED);
    tft.setCursor(0, 40);
    tft.printf("Player: %d", usb_data_wireless.devicePlayer);
    tft.setTextSize(2);
    tft.setCursor(0, 60);
    tft.setTextColor(GRAY);
    tft.printf("Channel: %d", usb_data_wireless.channel);

  #endif // DEVICE_LILYGO_T_DONGLE_S3
}

#define FIFO_SIZE_READ_SER 200  // l'originale era 32
#define TIME_OUT_AVALAIBLE 3

int rx_avalaible = 0;
unsigned long startTime = 0; // = millis();

void loop()
{
   
  SerialWireless.lenBufferSerialWrite = Serial.available();
  if (SerialWireless.lenBufferSerialWrite) {
    if (SerialWireless.lenBufferSerialWrite > FIFO_SIZE_WRITE_SERIAL) SerialWireless.lenBufferSerialWrite = FIFO_SIZE_WRITE_SERIAL;
    Serial.readBytes(SerialWireless.bufferSerialWrite, SerialWireless.lenBufferSerialWrite);
    SerialWireless.flush();
  }


#ifdef COMMENTO  
  if (Serial.available() > rx_avalaible) {
    startTime = millis();
    rx_avalaible = Serial.available();
  }     
  if (rx_avalaible && (millis() > startTime + TIME_OUT_AVALAIBLE)) { 
    if (rx_avalaible > FIFO_SIZE_READ_SER) rx_avalaible= FIFO_SIZE_READ_SER;
    Serial.readBytes(SerialWireless.bufferSerialWrite, rx_avalaible);
    SerialWireless.lenBufferSerialWrite = rx_avalaible;
    SerialWireless.flush();
    rx_avalaible = 0;
  } 
#endif //COMMENTO
}
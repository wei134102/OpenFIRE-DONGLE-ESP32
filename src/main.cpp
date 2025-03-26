// by SATANASSI Alessandro

#include <Arduino.h>

#include "TinyUSB_Devices.h"

#if defined(DEVICE_LILYGO_T_DONGLE_S3)
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

#define DEVICE_VID 0xF143
#define DEVICE_PID 0x1998

#define MANUFACTURER_NAME "OpenFIRE_DONGLE"
#define DEVICE_NAME "FIRECon_DONGLE"



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
          
  if (!TinyUSBDevice.isInitialized()) { // aggiunto ..funzionava lo stesso, ma così è più sicuro .. sicuramente serve per Esp32 con libreria non integrfata nel core
    TinyUSBDevice.begin(0);
  }
  TinyUSBDevice.setManufacturerDescriptor(MANUFACTURER_NAME);
  TinyUSBDevice.setProductDescriptor(DEVICE_NAME);
  TinyUSBDevice.setID(DEVICE_VID, DEVICE_PID);   
  
  // Initializing the USB devices chunk.
  TinyUSBDevices.begin(1); 
  SerialWireless.begin();
  SerialWireless.connection_dongle();
  
  #if defined(DEVICE_LILYGO_T_DONGLE_S3)
    pinMode(TFT_LEDA_PIN, OUTPUT);
    digitalWrite(TFT_LEDA_PIN, 0); // accende retroilluminazione del display
  #endif // DEVICE_LILYGO_T_DONGLE_S3
  
  #if defined(DEVICE_LILYGO_T_DONGLE_S3)
    tft.initR(INITR_MINI160x80_PLUGIN);  // Init ST7735S mini display
    tft.setRotation(3);
    tft.fillScreen(BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextColor(RED);
    tft.println("TEST DI PROVA");
    delay (1000);
    tft.setCursor(0, 20);
    tft.setTextColor(GRAY);
    tft.println("Displ. ST7735");
    tft.setTextColor(BLUE);
    tft.setCursor(15, 40);
    tft.println("160x80 RGB");
    delay (500);
    tft.setTextSize(1);
    tft.setCursor(30, 70);
    tft.setTextColor(GREEN);
    tft.println("WWW.ADRIROBOT.IT");
    delay (1000);
  #endif // DEVICE_LILYGO_T_DONGLE_S3
}

#define FIFO_SIZE_READ_SER 200  // l'originale era 32
#define TIME_OUT_AVALAIBLE 3

int rx_avalaible = 0;
unsigned long startTime = 0; // = millis();

void loop()
{
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
}
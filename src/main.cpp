// by SATANASSI Alessandro

#include <Arduino.h>


#include "TinyUSB_Devices.h"

#if defined(USES_OLED_DISPLAY)
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #include <WiFi.h>
  
  #define SCREEN_WIDTH 128 // OLED显示屏宽度，单位：像素
  #define SCREEN_HEIGHT 64 // OLED显示屏高度，单位：像素
  #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

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
  
  #if defined(USES_OLED_DISPLAY)
    // 初始化OLED显示屏
    Wire.begin(OLED_SDA, OLED_SCL);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 地址0x3C适用于128x64
      Serial.println(F("SSD1306初始化失败"));
    }
    
    // 初始化WiFi以获取MAC地址
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    delay(100);
    
    // 获取MAC地址
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA); // 直接从ESP-IDF获取MAC地址
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("OpenFIRE DONGLE"));
    display.println(F("Version: "));
    display.print(OPENFIRE_DONGLE_VERSION);
    display.print(" ");
    display.println(OPENFIRE_DONGLE_CODENAME);
    display.println(F("MAC Address:"));
    display.println(macStr);
    display.display();
    delay(3000);
    
    // 显示搜索信息
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Searching..."));
    display.println(F("Waiting for device"));
    display.println(F("MAC: "));
    display.println(macStr);
    display.display();
  #endif
 
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
  
  #if defined(USES_OLED_DISPLAY)
    // 显示正在配对
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Pairing..."));
    display.println(F("Scanning channels"));
    display.display();
  #endif
  
  SerialWireless.connection_dongle();
  
  #if defined(USES_OLED_DISPLAY)
    // 显示配对成功
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Pairing Success!"));
    display.println(F("Channel: "));
    display.println(usb_data_wireless.channel);
    display.display();
    delay(1000);
  #endif
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
  Serial.begin(9600);
  Serial.setTimeout(0);
  Serial.setTxTimeoutMs(0);
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
  
  #if defined(USES_OLED_DISPLAY)
    // 显示连接信息
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Connected!"));
    display.println(F("Device: "));
    display.println(usb_data_wireless.deviceName);
    display.println(F("Player: "));
    display.println(usb_data_wireless.devicePlayer);
    display.println(F("Channel: "));
    display.println(usb_data_wireless.channel);
    display.println(F("MAC: "));
    display.println(macStr); // 使用之前声明的macStr变量
    display.display();
  #endif
}

#define FIFO_SIZE_READ_SER 200  // l'originale era 32
#define TIME_OUT_AVALAIBLE 2
#define TIME_OUT_SERIAL_MICRO 1000 // 1000 microsecondi = 1 millisecondo

int rx_avalaible = 0;
unsigned long startTime = 0; // = millis();

uint64_t timer_serial_micro = 0;

//uint16_t len_aux;
uint8_t buffer_aux[FIFO_SIZE_READ_SER];
void loop()
{
  
  //micros();
  //esp_timer_get_time();
  if (esp_timer_get_time() - timer_serial_micro > TIME_OUT_SERIAL_MICRO) // controlla ogni millisecondo più o meno a 9600 bps
  {
  if (Serial.available() > rx_avalaible) {
    startTime = millis();
    rx_avalaible = Serial.available();
  }     
  if (rx_avalaible && (millis() > startTime + TIME_OUT_AVALAIBLE)) { 
    if (rx_avalaible > FIFO_SIZE_READ_SER) rx_avalaible = FIFO_SIZE_READ_SER;
    Serial.readBytes(buffer_aux, rx_avalaible);
    SerialWireless.write(buffer_aux, rx_avalaible);
    //SerialWireless.lenBufferSerialWrite = rx_avalaible;
    //SerialWireless.flush();
    rx_avalaible = 0;
  } 
  timer_serial_micro = esp_timer_get_time();
}


  /*
  len_aux = Serial.available();
  if (len_aux) {
    if (len_aux > FIFO_SIZE_WRITE_SERIAL) len_aux = FIFO_SIZE_WRITE_SERIAL;
    Serial.readBytes(buffer_aux, len_aux);
    SerialWireless.write(buffer_aux,len_aux);
  }
  */

  /*
  SerialWireless.lenBufferSerialWrite = Serial.available();
  if (SerialWireless.lenBufferSerialWrite) {
    if (SerialWireless.lenBufferSerialWrite > FIFO_SIZE_WRITE_SERIAL) SerialWireless.lenBufferSerialWrite = FIFO_SIZE_WRITE_SERIAL;
    Serial.readBytes(SerialWireless.bufferSerialWrite, SerialWireless.lenBufferSerialWrite);
    SerialWireless.flush();
  }
  */

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
// by SATANASSI Alessandro

#include <Arduino.h>


#include "TinyUSB_Devices.h"
#include "pin_config_DONGLE.h"

// 声明外部变量
extern uint8_t espnow_wifi_channel;
extern USB_Data_GUN_Wireless usb_data_wireless;

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
  // 初始化BOOT按键引脚
  pinMode(BTN_PIN, INPUT_PULLUP);
  
  // 声明变量（函数作用域）
  uint8_t mac[6];
  char macStr[18];
  
  // 获取MAC地址用于后续显示
  WiFi.mode(WIFI_MODE_STA);
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  WiFi.disconnect();
  
  // 频段切换功能
  // 初始化OLED显示
  #if defined(USES_OLED_DISPLAY)
    Wire.begin(OLED_SDA, OLED_SCL);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println(F("SSD1306初始化失败"));
    }
  #endif
  
  #if defined(DEVICE_LILYGO_T_DONGLE_S3)
    // 初始化TFT显示
    pinMode(TFT_LEDA_PIN, OUTPUT);
    digitalWrite(TFT_LEDA_PIN, 0); // 开启背光
    tft.initR(INITR_MINI160x80_PLUGIN);
    tft.setRotation(3);
    tft.fillScreen(BLACK);
  #endif
  
  // 频段切换倒计时
  unsigned long start_time = millis();
  uint8_t current_channel = 8; // 默认从8开始
  bool button_pressed = false;
  
  while (millis() - start_time < 5000) {
    unsigned long remaining_time = 5000 - (millis() - start_time);
    int countdown = remaining_time / 1000 + 1;
    
    // 检测按键
    if (digitalRead(BTN_PIN) == LOW && !button_pressed) {
      // 按键按下，频道+1
      current_channel++;
      if (current_channel > 12) {
        current_channel = 1;
      }
      button_pressed = true;
      start_time = millis(); // 重置5秒倒计时
    } else if (digitalRead(BTN_PIN) == HIGH) {
      button_pressed = false;
    }
    
    // 在OLED上显示
    #if defined(USES_OLED_DISPLAY)
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("OpenFIRE DONGLE"));
      display.println(F("Press BOOT to change"));
      display.print(F("Channel: "));
      display.println(current_channel);
      display.print(F("Time left: "));
      display.print(countdown);
      display.println("s");
      display.display();
    #endif
    
    // 在TFT上显示
    #if defined(DEVICE_LILYGO_T_DONGLE_S3)
      tft.fillScreen(BLACK);
      tft.setTextSize(2);
      tft.setCursor(0, 0);
      tft.setTextColor(RED);
      tft.println("CHANGE CHANNEL");
      tft.setTextSize(2);
      tft.setCursor(0, 20);
      tft.setTextColor(GRAY);
      tft.print("Press BOOT");
      tft.setTextSize(3);
      tft.setCursor(20, 40);
      tft.setTextColor(WHITE);
      tft.print(current_channel);
      tft.setTextSize(1);
      tft.setCursor(80, 60);
      tft.setTextColor(GRAY);
      tft.print("Time: ");
      tft.print(countdown);
      tft.print("s");
    #endif
    
    delay(50); // 防抖动
  }
  
  // 更新全局频道变量和usb_data_wireless结构体
  espnow_wifi_channel = current_channel;
  usb_data_wireless.channel = current_channel;
  
  // 添加调试信息
  Serial.begin(9600);
  Serial.print("Selected channel: ");
  Serial.println(current_channel);
  Serial.print("espnow_wifi_channel: ");
  Serial.println(espnow_wifi_channel);
  Serial.print("usb_data_wireless.channel: ");
  Serial.println(usb_data_wireless.channel);
  
  // 显示最终选择的频道
  #if defined(USES_OLED_DISPLAY)
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Selected Channel:"));
    display.println(current_channel);
    display.display();
    delay(1000);
  #endif
  
  #if defined(DEVICE_LILYGO_T_DONGLE_S3)
    tft.fillScreen(BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 20);
    tft.setTextColor(GREEN);
    tft.println("Channel:");
    tft.setTextSize(3);
    tft.setCursor(20, 40);
    tft.setTextColor(WHITE);
    tft.print(current_channel);
    delay(1000);
  #endif
  
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
    tft.fillScreen(BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextColor(RED);
    tft.println("..SEARCHING..");
    tft.drawBitmap(10, 25, customSplash, CUSTSPLASH_WIDTH, CUSTSPLASH_HEIGHT, BLUE);
    tft.setTextSize(2);
    tft.setCursor(65, 30);
    tft.setTextColor(GRAY);
    tft.println("Channel");
  #endif // DEVICE_LILYGO_T_DONGLE_S3

  // ====== gestione connessione wireless ====================
  SerialWireless.begin();
  
  // 确保应用正确的频道设置
  esp_wifi_set_channel(espnow_wifi_channel, WIFI_SECOND_CHAN_NONE);
  Serial.print("Re-set channel after begin: ");
  Serial.println(espnow_wifi_channel);
  
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
    display.print(F("Device: "));
    display.println(usb_data_wireless.deviceName);
    display.print(F("Player: "));
    display.println(usb_data_wireless.devicePlayer);
    display.print(F("Channel: "));
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
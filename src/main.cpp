// by SATANASSI Alessandro

#include <Arduino.h>
#include <SPI.h>

#include "TinyUSB_Devices.h"

#if defined(USES_OLED_DISPLAY)
	#include "OpenFIRE_logo.h"
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #include <WiFi.h>
  
  #define SCREEN_WIDTH 128 // OLED显示屏宽度，单位：像素
  #define SCREEN_HEIGHT 64 // OLED显示屏高度，单位：像素
  #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

#ifdef USES_DISPLAY
  #include "OpenFIRE_logo.h"
  #ifdef USE_LOVYAN_GFX
    #define LGFX_USE_V1
    #include <LovyanGFX.hpp>
    #include "LGFX_096_ST7735S_80x160.hpp"

    LGFX tft;

    #define WHITE            TFT_WHITE
    #define BLACK            TFT_BLACK
    #define BLUE             TFT_BLUE
    #define RED              TFT_RED
    #define MAGENTA          TFT_MAGENTA
    #define GREEN            TFT_GREEN
    #define CYAN             TFT_CYAN
    #define YELLOW           TFT_YELLOW
    #define BROWN            TFT_BROWN
    #define GRAY             TFT_LIGHTGRAY
    #define ORANGE           TFT_ORANGE
  #else
    #include <Adafruit_ST7735.h>
    Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

    #define WHITE            ST7735_WHITE
    #define BLACK            ST7735_BLACK
    #define BLUE             ST7735_BLUE
    #define RED              ST7735_RED
    #define MAGENTA          ST7735_MAGENTA
    #define GREEN            ST7735_GREEN
    #define CYAN             ST7735_CYAN
    #define YELLOW           ST7735_YELLOW
    #define BROWN            0XBC40
    #define GRAY             0X8430
    #define ORANGE           ST7735_ORANGE
  
  #endif // USE_LOVYAN_GFX

#endif // USES_DISPLAY

#include "OpenFIRE-DONGLE-version.h"

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

  #ifdef USES_DISPLAY
    #ifdef USE_LOVYAN_GFX
      tft.init();
      tft.setSwapBytes(true);
      tft.setColorDepth(16);
      //tft.setBrightness(255); 
    #else
      tft.initR(INITR_MINI160x80_PLUGIN);  // Init ST7735S mini display
      pinMode(TFT_PIN_BL, OUTPUT);
      digitalWrite(TFT_PIN_BL, 0); // accende retroilluminazione del display
    #endif // USE_LOVYAN_GFX
    
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
    #ifdef USE_LOVYAN_GFX
      tft.pushImage(10,1,LOGO_RGB_ALPHA_OPEN_WIDTH, LOGO_RGB_ALPHA_OPEN_HEIGHT, (uint16_t *)logo_rgb_alpha_open);
    #else
      tft.drawRGBBitmap(10,1,(uint16_t *)logo_rgb_alpha_open,LOGO_RGB_ALPHA_OPEN_WIDTH,LOGO_RGB_ALPHA_OPEN_HEIGHT);
    #endif //USE_LOVYAN_GFX
    
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
  #endif //USES_DISPLAY

	// 初始化OLED显示
	#if defined(USES_OLED_DISPLAY)
    Serial.println("===== OLED初始化调试 ======");
    Serial.print("当前使用的SDA引脚: ");
    Serial.println(OLED_SDA);
    Serial.print("当前使用的SCL引脚: ");
    Serial.println(OLED_SCL);

    // 扫描I2C总线，检查是否有设备
    Serial.println("扫描I2C设备...");
    byte error, address;
    int nDevices = 0;
    Wire.begin(OLED_SDA, OLED_SCL);

	for(address = 1; address < 127; address++ ) {
	  // 开始传输到指定地址
	  Wire.beginTransmission(address);
	  error = Wire.endTransmission();
	  
	  if (error == 0) {
		Serial.print("找到I2C设备，地址: 0x");
		if (address<16) Serial.print("0");
		Serial.println(address,HEX);
		nDevices++;
	    }
	  }

	if (nDevices == 0) {
	  Serial.println("未找到任何I2C设备!");
	}

	// 初始化Wire库，使用标准速度（100kHz）
	Wire.begin(OLED_SDA, OLED_SCL);

	// 尝试用默认地址初始化
	Serial.println("尝试初始化OLED...");
	if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
	  Serial.println("使用地址0x3C初始化SSD1306失败");
	  // 尝试备用地址
	  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
		Serial.println("使用地址0x3D初始化SSD1306也失败");
	  }
	} else {
	  Serial.println("SSD1306初始化成功!");
	  // 测试显示
	  display.clearDisplay();
	  display.setTextSize(1);
	  display.setTextColor(SSD1306_WHITE);
	  display.setCursor(0, 0);
	  display.println("OpenFIRE DONGLE");
	  display.println("OLED test sucessed");
	  display.display();
	}
	Serial.println("===== OLED初始化完成 ======");
	#endif

	// 添加TFT显示内容的OLED版本
	#if defined(USES_OLED_DISPLAY)
	// 清屏并显示搜索状态
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0, 0);
	display.println("SEARCHING");
	
	// 显示通道信息
	display.setTextSize(1);
	display.setCursor(0, 20);
	display.println("Channel:");
	
	// 显示OpenFIRE标识
	display.setCursor(0, 35);
	display.println("OpenFIRE");
	display.println("DONGLE");
	
	// 更新显示
	display.display();
	#endif



  // ====== gestione connessione wireless ====================
  SerialWireless.init_wireless();
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
  Serial.begin(9600);
  Serial.setTimeout(0);
  ////////////////////Serial.setTxTimeoutMs(0);
  // ====== fine connessione USB ==========================================================================
  
  #ifdef USES_DISPLAY   
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

  #endif // USES_DISPLAY

  // 添加连接成功后OLED显示内容
  #if defined(USES_OLED_DISPLAY)
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // 显示设备名称
    display.setCursor(0, 0);
    display.println(usb_data_wireless.deviceName);
    
    // 显示玩家信息
    display.setCursor(0, 20);
    display.printf("Player: %d", usb_data_wireless.devicePlayer);
    
    // 显示通道信息
    display.setCursor(0, 35);
    display.printf("Channel: %d", usb_data_wireless.channel);
    
    // 显示连接状态
    display.setCursor(0, 50);
    display.println("CONNECTED");
    
    display.display();
  #endif // USES_OLED_DISPLAY
}

#define FIFO_SIZE_READ_SER 200  // l'originale era 32
#define TIME_OUT_AVALAIBLE 2
#define TIME_OUT_SERIAL_MICRO 1000 // 1000 microsecondi = 1 millisecondo

int rx_avalaible = 0;
unsigned long startTime = 0; // = millis();

uint64_t timer_serial_micro = 0;

#ifdef DEBUG_OPENFIRE
#define TEMPO_OGNI_VISUALIZZAZIONE 1000000 // microsecondi = 1 secondo
uint64_t timer_ultimo_pacchetto_scompattato = 0;
uint64_t timer_ultimo_pacchetto_ricevuto_da_callback = 0;

uint64_t tempo_ultimo_pacchetto_scompattato = 0;
uint64_t tempo_ultimo_pacchetto_ricevuto_da_callback = 0;


uint64_t timer_ultimo_dato_visualizzato = 0;
uint64_t timer_tempo_minimo_pacchetto_ricevuto_da_callback = __UINT64_MAX__;
uint64_t timer_tempo_minimo_pacchetto_scompattato = __UINT64_MAX__;
uint64_t timer_tempo_massimo_pacchetto_ricevuto_da_callback = 0;
uint64_t timer_tempo_massimo_pacchetto_scompattato = 0;
uint16_t pacchetti_scompattati = 0;
uint16_t pacchetti_ricevuti_da_callback = 0;
#endif


//uint16_t len_aux;
uint8_t buffer_aux[FIFO_SIZE_READ_SER];
void loop()
{
  
#ifdef DEBUG_OPENFIRE
if (esp_timer_get_time() - timer_ultimo_dato_visualizzato > TEMPO_OGNI_VISUALIZZAZIONE) {
  Serial.print("tempo_minimo_pacchetto_ricevuto_da_callback: "), Serial.println(timer_tempo_minimo_pacchetto_ricevuto_da_callback);
  Serial.print("tempo_minimo_pacchetto_scompattato: "), Serial.println(timer_tempo_minimo_pacchetto_scompattato);

  Serial.print("tempo_massimo_pacchetto_ricevuto_da_callback: "), Serial.println(timer_tempo_massimo_pacchetto_ricevuto_da_callback);
  Serial.print("tempo_massimo_pacchetto_scompattato: "), Serial.println(timer_tempo_massimo_pacchetto_scompattato);

  Serial.print("pacchetti_scompattati: "), Serial.println(pacchetti_scompattati);
  Serial.print("pacchetti_ricevuti_da_callback: "), Serial.println(pacchetti_ricevuti_da_callback);

  Serial.println("================================");

  timer_ultimo_dato_visualizzato = esp_timer_get_time();
  
  //timer_ultimo_pacchetto_scompattato = 0;
  //timer_ultimo_pacchetto_ricevuto_da_callback = 0;
  
  timer_tempo_minimo_pacchetto_ricevuto_da_callback = __UINT64_MAX__;
  timer_tempo_minimo_pacchetto_scompattato = __UINT64_MAX__;
  
  timer_tempo_massimo_pacchetto_ricevuto_da_callback = 0;
  timer_tempo_massimo_pacchetto_scompattato = 0;
  
  pacchetti_scompattati = 0;
  pacchetti_ricevuti_da_callback = 0;
}
#endif
vTaskDelay(pdMS_TO_TICKS(1));
rx_avalaible = Serial.available();
if (rx_avalaible > FIFO_SIZE_READ_SER) rx_avalaible = FIFO_SIZE_READ_SER;
if (rx_avalaible)
{
    Serial.readBytes(buffer_aux, rx_avalaible);
    SerialWireless.write(buffer_aux, rx_avalaible);
    SerialWireless.flush(); // provare a togliere
}
//yield();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMMENTO
  
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
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////


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
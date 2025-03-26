
// defined(ARDUINO_ARCH_RP2040)
// defined(ARDUINO_ARCH_ESP32)



#include <Arduino.h>

//const uint8_t peerAddress[6] = {0x24, 0x58, 0x7C, 0xDA, 0x38, 0xA0};

#include <SPI.h>
#include "TinyUSB_Devices.h"
#include <atomic>
//#include <FS.h>
//#include <SPIFFS.h>


////////#include <USB.h>

//ESPUSB USB_NUOVO;
//#define  TinyUSBDevice USB



//#include <Adafruit_TinyUSB.h>
/*
// ====== personalizzazione di TinyUSB ==================
#ifdef CFG_TUSB_CONFIG_FILE
    #include CFG_TUSB_CONFIG_FILE
#else
    #include "tusb_config.h"
#endif
// ======================================================
*/

/*
#include "pin_config_DONGLE.h"
#if defined(ARDUINO_ARCH_ESP32)
  //#include <esp_now.h>
  #include <ESP32_NOW.h>
  #include <ESP32_NOW_Serial.h>
  #include <WiFi.h> // serve per leggere max addres
  #include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#elif defined(ARDUINO_ARCH_RP2040)
  // vediamo
#endif
*/


//#include <nRF24L01.h>
//#include <RF24.h>
#define RF24_CE_PIN 15
#define RF24_CSN_PIN 17
#define RF24_SCK_PIN
#define RF24_TX_MOSI_PIN
#define RF24_RX_MISO_PIN
#define RF24_SPI_NUM spi1 // spi0 or spi1 bus


//#define  TinyUSBDevice USB

// instantiate an object for the nRF24L01 transceiver
//RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

/*
RF24_SPI_NUM.setSCK(RF24_SCK_PIN);
RF24_SPI_NUM.setCS(RF24_CSN_PIN);
RF24_SPI_NUM.setRX(RF24_RX_MISO_PIN);
RF24_SPI_NUM.setTX(RF24_TX_MOSI_PIN);
*/

////////SPI spi;

/*
// again please review the GPIO pins' "Function Select Table" in the Pico SDK docs
    spi.begin(RF24_SPI_N, RF24_SCK_PIN, RF24_TX_MOSI_PIN, RF24_RX_MISO_PIN); // spi0 or spi1 bus, SCK, TX (MOSI), RX (MISO)

    if (!radio.begin(&spi)) {
        printf("Radio hardware is not responding!\n");
    }
*/


//#include <USB.h>
//#include <USBCDC.h>
//#include "TFT_eSPI.h"
//#include "logo.h"
//#include "OneButton.h"

//#define FASTLED_USES_ESP32S3_I2S
//#include <FastLED.h>


//#define USE_TFT_ESPI
//#define USE_ARDUINO_GFX
#ifdef USES_DISPLAY
#define USE_ADAFRUI_ST7735
#endif
//#define NO_DISPLAY

#if defined(USE_TFT_ESPI)
  #include <TFT_eSPI.h>
  //#define LV_USE_TFT_ESPI
  TFT_eSPI tft = TFT_eSPI();
#elif defined(USE_ARDUINO_GFX)
  #include <Arduino_GFX_Library.h>
  Arduino_DataBus *bus = new Arduino_ESP32SPIDMA(TFT_DC /*dc*/, TFT_CS /*cs*/, TFT_SCLK /*sck*/, TFT_MOSI /*mosi*/, TFT_MISO /*miso*/, FSPI /*HSPI*/ /* spi_num*/, true /*is_shared_interface*/ );
  Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST /* RST */, (uint8_t) 3 /* Rotation */, true /* IPS */, (int16_t) TFT_WIDTH /* L */, (int16_t) TFT_HEIGHT /* H */, 26, 0,0,1);
#elif defined(USE_ADAFRUI_ST7735)
    #include <Adafruit_ST7735.h>
    Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
#else // NO_DISPLAY
    // NON HAI DEFINITO ALCUN DRIVER
#endif

//CRGB leds;
//OneButton button(BTN_PIN, true);
//uint8_t btn_press = 0;

#define PRINT_STR(str, x, y)        \
    do {                            \
        Serial.println(str);        \
        tft.drawString(str, x, y);  \
        y += 8;                     \
    } while (0);





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

#define OPENFIRE_DONGLE_VERSION 6.9
#define OPENFIRE_DONGLE_CODENAME "Sessantanove"

// SERIAL_TIMEOUT -- L'ORIGINALE ERA 0 ... (IN MILLISECONDI, TROPPO BREVE PER BLUETOOTH .. E' IL TIMEOUT PER LE OPERAZIONI DI LETTURA)
// 0 INDICA ATTESA INFINITA
#define SERIAL_TIMEOUT 0 //1000 DI DEFAULT .. SECONDO ME PUÒ STARE ANCHE A 0

// For custom builders, remember to check (COMPILING.md) for IDE instructions!
// ISSUERS: REMEMBER TO SPECIFY YOUR USING A CUSTOM BUILD & WHAT CHANGES ARE MADE TO THE SKETCH; OTHERWISE YOUR ISSUE MAY BE CLOSED!

// definisce codici di comunicazione tra i due core tramite la FIFO predisposta
#define CORE_ATTIVA_BT 69
#define CORE_BT_ATTIVATO 71
#define CORE_USB_ATTIVATO 66
#define CORE_SETUP_CORE_PRINCIPALE_FINITO 50

#define DEVICE_VID 0xF143
#define DEVICE_PID 0x1998


#define MANUFACTURER_NAME "OpenFIRE_DONGLE"
#define DEVICE_NAME "FIRECon_DONGLE"

Stream* Serial_OpenFIRE_Stream;

// DEFINIZIONE DELLE PORTE SERIALI - MASSIMO DUE PORTE CONTEMPORANEAMENTE SI POSSONO GESTIRE

//Adafruit_USBD_CDC USBserial1;
//USBCDC USBserial1;
//USBCDC USBserial2;
//USBCDC USBserial3;
//USBCDC USBserial4;

//-----------------------------------------------------------------------------------------------------
//Adafruit_USBD_Device TinyUSBDevice1;
//Adafruit_USBD_Device TinyUSBDevice2;



void TinyUSBInit_DONGLE()
{
    
   

    if (!TinyUSBDevice.isInitialized()) { // aggiunto ..funzionava lo stesso, ma così è più sicuro .. sicuramente serve per Esp32 con libreria non integrfata nel core
        TinyUSBDevice.begin(0);
    }
    
    //TinyUSBDevice.clearConfiguration(); // cancella anche serial
    
    TinyUSBDevice.setManufacturerDescriptor(MANUFACTURER_NAME);
    TinyUSBDevice.setProductDescriptor(DEVICE_NAME);
    TinyUSBDevice.setID(DEVICE_VID, DEVICE_PID);

    //TinyUSBDevice.addInterface(&TinyUSBDevice1);
    //TinyUSBDevice.addInterface(&TinyUSBDevice2);
    


    //TinyUSB_Device_Init(0);

    //tud_init(0);
    //tud_init(1);
    
    #ifdef COMMENTO
    USB.manufacturerName(MANUFACTURER_NAME);
    USB.productName(DEVICE_NAME);
    USB.PID(DEVICE_PID);
    USB.VID(DEVICE_VID);
    #endif // COMMENTO


    //USB_NUOVO.manufacturerName("PROVA NUOVO");
    //USB_NUOVO.productName("PRODOTTO NUOVO");
    //USB_NUOVO.PID(69);
    //USB_NUOVO.VID(96);


    //USB.begin();
    //USB_NUOVO.begin();
    

    
}



//-----------------------------------------------------------------------------------------------------
// #define LV_USE_ST7735  

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


#if defined(ARDUINO_ARCH_ESP32)

//#define MAX_PEERS 8 // 4 pistole e 4 pedaliere .. per giocare in 4 
//uint8_t peerAddress[] = {0x18, 0xFE, 0x34, 0xAB, 0xCD, 0xEF};  // indirizzo dell'altro disposiotivo con cui stiamo comunicando

//uint8_t macAddress_key[] = {0xF0, 0x9E, 0x9E, 0x28, 0x9E, 0xB8};  // indirizzo della nostra scheda DONGLE

// Dichiarazione di un array per memorizzare gli indirizzi MAC dei peer
//uint8_t peerAddresses[MAX_PEERS][ESP_NOW_ETH_ALEN];
//int numPeers = 0;

// ... (resto del tuo codice)
#ifdef COMMENTO
// Struttura per tenere traccia dei peer e dei loro stati
struct Peer {
  uint8_t address[ESP_NOW_ETH_ALEN];
  uint8_t retries;
  unsigned long lastSent;
};
Peer peers[MAX_PEERS];
int numPeers = 0;
#endif // COMMENTO

#endif 

#ifdef COMMENTO
// Callback per la ricezione dei dati
void OnDataRecv_ERR(const uint8_t * mac, const uint8_t *incomingData, uint8_t len) {
  // ... (elaborazione dei dati ricevuti)

  // Aggiorna lo stato del peer
  for (int i = 0; i < numPeers; i++) {
    if (memcmp(peers[i].address, mac, ESP_NOW_ETH_ALEN) == 0) {
      peers[i].retries = 0; // Reset dei tentativi
      peers[i].lastSent = millis(); // Aggiorna l'ultimo invio
      break;
    }
  }
}

// Callback per la spedizione
void OnDataSent_ERR(const uint8_t *mac_addr, esp_now_send_status_t status) {
  for (int i = 0; i < numPeers; i++) {
    if (memcmp(peers[i].address, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
      if (status != ESP_NOW_SEND_SUCCESS) {
        peers[i].retries++;
        if (peers[i].retries > MAX_RETRIES) {
          // Rimuovi il peer
          // ... (implementa la logica per rimuovere il peer)
        } else {
          // Ritrasmetti il pacchetto dopo un ritardo
          unsigned long now = millis();
          if (now - peers[i].lastSent > RETRANSMISSION_DELAY) {
            peers[i].lastSent = now;
            // Ritrasmetti il pacchetto
            esp_now_send(mac_addr, data, len);
          }
        }
      }
      break;
    }
  }
}

// ... (resto del tuo codice)

// Callback per la ricezione dei dati
void OnDataRecv_TUTTI(const uint8_t * mac, const uint8_t *incomingData, uint8_t len) {
  // Memorizza l'indirizzo MAC del peer
  memcpy(peerAddresses[numPeers], mac, ESP_NOW_ETH_ALEN);
  numPeers++;

  // ... (elaborazione dei dati ricevuti)

  // Invia i dati a tutti i peer
  for (int i = 0; i < numPeers; i++) {
    esp_now_send(peerAddresses[i], data, len);
  }
}

// Funzione per inviare dati a tutti i peer
void broadcastData(uint8_t *data, uint8_t len) {
  for (int i = 0; i < numPeers; i++) {
    esp_now_send(peerAddresses[i], data, len);
  }
}

#endif //COMMENTO

#if defined(ARDUINO_ARCH_ESP32)


#ifdef COMMENTO
// Callback per la ricezione dei dati
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.print("Received data from: ");
  //printMac(mac);
  Serial.println("");

  // Stampa i dati ricevuti
  for (int i = 0; i < len; i++) {
    Serial.print(incomingData[i]);
    Serial.print(" ");
  }
  Serial.println("");

  // Invia una risposta
  uint8_t response[] = {0xFA, 0xFB, 0xFC};
  esp_now_send(mac, (uint8_t *)response, sizeof(response));
}

// Callback per la spedizione
void OnDataSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last packet send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

int findBestChannel() {
  int bestChannel = 1;
  int minRSSI = -100; // Valore iniziale molto basso

  WiFi.mode(WIFI_STA); // Imposta la modalità stazione
  WiFi.disconnect(); // Disconnetti dalla rete Wi-Fi

  for (int channel = 1; channel <= 13; channel++) {
    WiFi.channel(channel);
    delay(2000); // Attendi la stabilizzazione del canale

    int rssi = WiFi.RSSI();
    Serial.print("Canale ");
    Serial.print(channel);
    Serial.print(": RSSI = ");
    Serial.println(rssi);

    if (rssi > minRSSI) {
      bestChannel = channel;
      minRSSI = rssi;
    }
  }

  return bestChannel;
}

#endif // COMMENTO


#endif

#ifdef COMMENTO
class ESP_NOW_Serial_Class_OpenFIRE : public ESP_NOW_Serial_Class {
  public:
  // Constructor of the class
  ESP_NOW_Serial_Class_OpenFIRE(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface = WIFI_IF_STA, const uint8_t *lmk = NULL, bool remove_on_fail = false) : ESP_NOW_Serial_Class(mac_addr, channel, iface, lmk, remove_on_fail) {}

  // Destructor of the class
  ~ESP_NOW_Serial_Class_OpenFIRE() {}


};

#define ESPNOW_WIFI_CHANNEL 4

const uint8_t peerAddress[6] = {0xF8, 0xB3, 0xB7, 0x32, 0xBE, 0x64}; // altra periferica con cui si vuole comunicare indirizzo del ESP32

ESP_NOW_Serial_Class_OpenFIRE ESP32_NOW_OpenFIRE(peerAddress,ESPNOW_WIFI_CHANNEL,WIFI_IF_STA,NULL,false);
  
uint32_t msg_count = 0;

#define MAX_BUFF_READ 128

uint8_t data_read[MAX_BUFF_READ];

#endif // COMMENTO









//startIrCamTimer(IRCamUpdateRate);

// number of times the IR camera will update per second
//constexpr unsigned int IRCamUpdateRate = 209; // 209 hz
constexpr unsigned int IRCamUpdateRate = 1;     // 1 hz (ogni 1 secondo)
volatile bool irPosUpdateTick = false;
hw_timer_t *My_timer = NULL;

void ARDUINO_ISR_ATTR esp32s3pwmIrq(void) ///sistemare
{
    // pwm_hw->intr = 0xff;
    irPosUpdateTick = true;
}


void startIrCamTimer(int frequencyHz)
{

  My_timer = timerBegin(1000000);      
  timerAttachInterrupt(My_timer, &esp32s3pwmIrq);
  timerAlarm(My_timer, (uint64_t) 1000000 / frequencyHz, true, 0);

}



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
    
    startIrCamTimer(IRCamUpdateRate);
    
    TinyUSBInit_DONGLE();
    
           
    // Initializing the USB devices chunk.
    TinyUSBDevices.begin(1); 
    SerialWireless.begin();
    SerialWireless.connection_dongle();


    // wait until device mounted
    #define MILLIS_TIMEOUT  5000 //5 secondi
           
    unsigned long lastMillis = millis ();

    //while ((millis () - lastMillis <= MILLIS_TIMEOUT) && (!TinyUSBDevice.mounted())) { yield(); }
    
    //while(!TinyUSBDevice.mounted()) { yield(); }

    
    //Serial.begin(9600);   // 9600 = 1ms data transfer rates, default for MAMEHOOKER COM devices.
    //Serial.setTimeout(SERIAL_TIMEOUT);
    ///////////////////////////Serial_OpenFIRE_Stream = & Serial;

    
   
    


    // Registra la callback per la ricezione dei dati
    //esp_now_register_recv_cb(OnDataRecv);

    //esp_now_register_send_cb(OnDataSend);

/*
    // INVIA DATI
while (1) {
    static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 2000) {
    uint8_t data[] = {0x01, 0x02, 0x03};
    esp_now_send(peerAddress, (uint8_t *)data, sizeof(data));
    lastSendTime = millis();
  }
}
*/

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //#define Serial (*Serial_OpenFIRE_Stream)
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    #if defined(DEVICE_LILYGO_T_DONGLE_S3)
    pinMode(TFT_LEDA_PIN, OUTPUT);
    digitalWrite(TFT_LEDA_PIN, 0); // accende retroilluminazione del display
    #endif



/*
uint8_t i =0;
for(i=0;i<1;i++)  {
    digitalWrite(TFT_LEDA_PIN, 1); // spegne retroillumionazione
    delay(2000);
    digitalWrite(TFT_LEDA_PIN, 0); // accende retroilluminazione
    delay(2000);
}
digitalWrite(TFT_LEDA_PIN, 1);
*/

#if defined(USE_TFT_ESPI)
   // INIZIALIZZAZIONE DEL DISPLAY //////
    //pinMode(TFT_LEDA_PIN, OUTPUT);
    // Initialise TFT
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_DARKGREY);
    //digitalWrite(TFT_LEDA_PIN, 0);
    tft.setTextFont(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    //tft.pushImage(0, 0, 160, 80, (uint16_t *)gImage_logo);
    tft.setCursor(40, 5);
    tft.println(F("Arduino"));
    tft.drawString("cIAO", 30, 40);
#elif defined(USE_ARDUINO_GFX)
  #include <Arduino_GFX_Library.h>
  Arduino_DataBus *bus = new Arduino_ESP32SPIDMA(TFT_DC /*dc*/, TFT_CS /*cs*/, TFT_SCLK /*sck*/, TFT_MOSI /*mosi*/, TFT_MISO /*miso*/, HSPI /* spi_num*/, true /*is_shared_interface*/ );
  Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST /* RST */, (uint8_t) 3 /* Rotation */, true /* IPS */, (int16_t) TFT_WIDTH /* L */, (int16_t) TFT_HEIGHT /* H */, 26, 0,0,1);

  gfx->begin();
  gfx->fillScreen(BLUE);
  gfx->println("Hello World!");
  gfx->setCursor(10, 10);
  gfx->setTextColor(RED);
  gfx->println("Hello World!");
  gfx->drawRect(16, 16, 120, 90, WHITE);
  gfx->setTextBound(18, 18, 116, 86);
  // gfx->setTextWrap(false);
  // gfx->setTextSize(3, 3, 1);
  gfx->setCursor(30, 18);
  gfx->setTextColor(WHITE);
  gfx->println("Arduino has over the years released over 100 hardware products: boards, shields, carriers, kits and other accessories. In this page, you will find an overview of all active Arduino hardware, including the Nano, MKR and Classic families.");
#elif defined(USE_ADAFRUI_ST7735)
  //tft.initR(INITR_MINI160x80);  // Init ST7735S mini display
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
  ///tft.println(WiFi.macAddress());
  delay (500);
  tft.setTextSize(1);
  tft.setCursor(30, 70);
  tft.setTextColor(GREEN);
  tft.println("WWW.ADRIROBOT.IT");
  delay (1000);
#else
    // NON HAI DEFINITO ALCUN DRIVER
#endif

#ifndef COMMENTO 


#if defined(ARDUINO_ARCH_ESP32)


#ifdef COMMENTO
  
     // leggi mac addres
    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin();
    /*
    Serial.println(WiFi.macAddress());
    uint8_t newMac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    WiFi.macAddress(newMac);
    Serial.println(WiFi.macAddress());
    WiFi.disconnect();
    */
    // ======================
    

    // Inizializza ESP-NOW
    if (esp_now_init() != ESP_OK) {
        //Serial.println("Error initializing ESP-NOW");
        tft.fillScreen(BLACK);
        tft.setTextSize(2);
        tft.setCursor(0, 0);
        tft.setTextColor(RED);
        tft.println("Error initializing ESP-NOW");
        delay (1000);
    }
    else {
        //Serial.println("ESP-NOW Inizializzato");
        tft.fillScreen(BLACK);
        tft.setTextSize(2);
        tft.setCursor(0, 0);
        tft.setTextColor(RED);
        tft.println("ESP-NOW Inizializzato");
        delay (1000);
    }




#endif // COMMENTO

#endif

#ifdef COMMENTO
    // Imposta l'indirizzo del peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;


// Aggiungi il peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //Serial.println("Failed to add peer");
    tft.setTextSize(2);
    tft.setCursor(0, 35);
    tft.setTextColor(RED);
    tft.println("Failed to add peer");
    delay (1000);
  }
  else {
Serial.println("add peer OK");
    tft.setTextSize(2);
    tft.setCursor(0, 35);
    tft.setTextColor(RED);
    tft.println("add peer OK");
    delay (1000);
  }

#endif // COMMENTO

#endif //COMMENTO



#ifdef COMMENTO

//while (1) findBestChannel();



    // BGR ordering is typical
    //FastLED.addLeds<APA102, LED_DI_PIN, LED_CI_PIN, BGR>(&leds, 1);
    //while (1) {
    //    static uint8_t hue = 0;
    //    leds = CHSV(hue++, 0XFF, 100);
    //    FastLED.show();
    //    delay(50);
    //}
    
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
    ///WiFi.begin();

    while (!WiFi.STA.started()) {
    delay(100);
  }
    tft.fillScreen(BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextColor(RED);
    tft.println("WIFI Attivo");
    delay (1000);

if (!ESP32_NOW_OpenFIRE.begin(0)) {
   //Serial.println("FAILED a inizializzare ESP NOW SERIAL");
    tft.setTextSize(2);
    tft.setCursor(0, 35);
    tft.setTextColor(RED);
    tft.println("Failed Esp Now");
    delay (1000);
  }
  else {
  //Serial.println("ESP NOW Serial inizializzato regolarmente");
    tft.setTextSize(2);
    tft.setCursor(0, 35);
    tft.setTextColor(RED);
    tft.println("ESP NOW OK");
    delay (1000);
  }

#endif //COMMENTO


/*
rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);  // Blue
delay(2000);
rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Green
delay(2000);
rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
delay(2000);
*/


}










#ifdef COMMENDO
// Callback per la ricezione dei dati
void tud_cdc_rx_cb(uint8_t* buf, uint8_t count) {
  // Processa i dati ricevuti
  for (int i = 0; i < count; i++) {
    Serial.print(buf[i]);
  }
}

void setup() {
  Serial.begin(115200);

  // Inizializza TinyUSB
  tud_init();

  // Registra la callback per la ricezione dei dati
  tud_cdc_set_rx_cb(tud_cdc_rx_cb);
}

void loop() {
  tud_task();
}

#endif // COMMENTO

#ifdef COMMENTO

extern USBCDC USBserial0;

#define Serial USBserial0

enum PACKET_TX {
  SERIAL_TX = 0,
  KEYBOARD_TX,
  MOUSE_TX,
  GAMEPADE_TX
};

#endif //COMMENTO

    //////////////////////////uint8_t *buffer;
#define FIFO_SIZE_READ_SER 200  // l'originale era 32

#ifdef COMMENTO
#define BUFFER_SIZE 128 //buffer scrittura
    // ================ PER SCRITTURA DEI DATI ============================
    
  uint8_t buffer[BUFFER_SIZE];

    volatile uint16_t writeIndex = 0;
    volatile uint16_t readIndex = 0;
    volatile uint16_t _writeLen = 0;

    std::atomic<uint16_t> _writeIndex_atomic; //.store(0); //(<uint16_t>0);
    std::atomic<uint16_t> _readIndex_atomic; //(0);
    std::atomic<uint16_t> _writeLen_atomic; //(0);

// ============================================================================

// ===================== PER LETTURA DEI DATI

    volatile uint16_t _writer = 0;
    volatile uint16_t _reader = 0;
    volatile uint16_t _readLen = 0;

    std::atomic<uint16_t> _writer_atomic;//(0);
    std::atomic<uint16_t> _reader_atomic;//(0);
    std::atomic<uint16_t> _readLen_atomic;//(0);
#endif //COMMENTO

    //uint8_t _queue[FIFO_SIZE_READ_SER];
    /////////////////////////uint8_t *_queue;

    bool inizializzato = false;
    //uint16_t sendSize = 0;
    int rx_avalaible = 0;
    unsigned long startTime = 0; // = millis();

    //extern ESP_NOW_Serial_Class_OpenFIRE ESP32_NOW_Serial_OpenFIRE;
    //extern SerialTransfer OpenFIRE_serialTransfer;
    //extern SerialWireless_ SerialWireless;
    //extern Adafruit_USBD_HID usbHid;

// Main core events hub
// splits off into subsequent ExecModes depending on circumstances
void loop()
{
if (!inizializzato) {
//PER LETTURA DATI
    #ifdef COMMENTO
    _writer_atomic.store(0);//(0);
    _reader_atomic.store(0);//(0);
    _readLen_atomic.store(0);//(0);

    // PER SCRITTURA DATI
    _writeIndex_atomic.store(0); //.store(0); //(<uint16_t>0);
    _readIndex_atomic.store(0); //(0);
    _writeLen_atomic.store(0); //(0);

    _writer = 0;
    _reader = 0;

    writeIndex = 0;
    readIndex = 0;
    _writeLen = 0;
    _readLen = 0;
    #endif //COMMENTO
  //Serial.println(WiFi.macAddress());
   inizializzato = true;
}

// =================================================================
/*
if (ESP32_NOW_Serial_OpenFIRE.available())
{
  uint8_t dato;
  //dato = ESP32_NOW_Serial_OpenFIRE.read();
  ESP32_NOW_Serial_OpenFIRE.read(&dato, 1);
  Serial.write(&dato,1);
}
*/


//SerialWireless.checkForRxPacket();
//uint8_t numSerialAvaliable = SerialWireless.available();
//if (numSerialAvaliable)  


/*
while (SerialWireless.available()) 
{
  Serial.write(SerialWireless.read());
}
*/


#ifdef COMMENTO

while (SerialWireless.availableBin()) 
{
  uint8_t dato;
  /*
  dato = ESP32_NOW_Serial_OpenFIRE.read();
  ESP32_NOW_Serial_OpenFIRE.read(&dato, 1);
  */
  dato = (uint8_t) SerialWireless.readBin();
  SerialWireless.packet.parse(dato, true);
  //if (dato==START_BYTE) Serial.println(" === START_BYTE ===");

#define PACKET_CONTINUE             3
#define PACKET_NEW_DATA             2
#define PACKET_NO_DATA              1
#define PACKET_CRC_ERROR            0
#define PACKET_PAYLOAD_ERROR       -1
#define PACKET_STOP_BYTE_ERROR     -2
#define PACKET_STALE_PACKET_ERROR  -3

switch (SerialWireless.packet.status)
{
case PACKET_CONTINUE:
  /* code */
  //Serial.println("PACKET_CONTINUE");
  break;
case PACKET_NEW_DATA :
  /* code */
  //Serial.println("PACKET_NEW_DATA");
  Serial.write(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE],SerialWireless.packet.bytesRead);
  //Serial.println();
  //Serial.print("PACCHHETTO IN NUMERI: ");
  /*
  for (uint8_t i =0 ; i<SerialWireless.packet.bytesRead;i++)
  {
     Serial.print(SerialWireless.packet.rxBuff[PREAMBLE_SIZE+i]);
     Serial.print(" ");
  }
  Serial.println();
  */
  break;
case PACKET_NO_DATA :
  /* code */
  //Serial.println("PACKET_NO_DATA");
  break;
case PACKET_CRC_ERROR:
  /* code */
  //Serial.println("PACKET_CRC_ERROR");
  break;
case PACKET_PAYLOAD_ERROR :
  /* code */
  //Serial.println("PACKET_PAYLOAD_ERROR");
  break;
case PACKET_STOP_BYTE_ERROR:
  /* code */
  //Serial.println("PACKET_STOP_BYTE_ERROR");
  break;
case PACKET_STALE_PACKET_ERROR:
  /* code */
  //Serial.println("PACKET_STALE_PACKET_ERROR");
  break;
default:
  //Serial.println("PACKET_ALTRO");
  break;
}

/*
  if (SerialWireless.packet.status==NEW_DATA) Serial.write(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE],SerialWireless.packet.bytesRead);
  //Serial.write(&dato,1);
*/
}

#endif //COMMENTO

// ==================================================================

#ifdef COMMENTO
if (OpenFIRE_serialTransfer.tick()) {

    if (TinyUSBDevice.suspended())  { TinyUSBDevice.remoteWakeup(); }
    while(!usbHid.ready()) yield();

    switch (OpenFIRE_serialTransfer.currentPacketID()) {
    case MOUSE_TX:
        /* code */
        Serial.println("Pacchetto MOUSE");      
        usbHid.sendReport(HID_RID_MOUSE, OpenFIRE_serialTransfer.packet.rxBuff, sizeof(hid_abs_mouse_report_t));
        break;
    case GAMEPADE_TX:
        /* code */
        Serial.println("Pacchetto GAMEPAD");
        usbHid.sendReport(HID_RID_GAMEPAD, OpenFIRE_serialTransfer.packet.rxBuff, sizeof(hid_gamepad16_report_t));
        break;
    case KEYBOARD_TX:
        /* code */
        Serial.println("Pacchetto KEYBOARD");
        usbHid.sendReport(HID_RID_KEYBOARD, OpenFIRE_serialTransfer.packet.rxBuff, sizeof(hid_keyboard_report_t));
        break;
    case SERIAL_TX:
        Serial.println("Pacchetto SERIALE");
        Serial.write(OpenFIRE_serialTransfer.packet.rxBuff, OpenFIRE_serialTransfer.bytesRead);
        //Serial.printf("message: %s Num: %d\n", (char *)&data_read[1], msg_count++);
        break;
    default:
        break;
  
  }  
}   
#endif // COMMENTO


/*
uint16_t numSerialAvailable = Serial.available();
for (uint16_t i =0; i<numSerialAvailable; i++) 
{
  uint8_t dato = (uint8_t) Serial.read();
  SerialWireless.write(dato);
  Serial.write(dato);
}
//Serial.readBytes(_queue, rx_avalaible);
*/


#ifndef COMMENTO

    #define TIME_OUT_AVALAIBLE 3
    
    if (Serial.available() > rx_avalaible)
    {
       startTime = millis();
       rx_avalaible = Serial.available();
      } 
    
    if (rx_avalaible && (millis() > startTime + TIME_OUT_AVALAIBLE)) {
      /////////////rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);  // Blue
      //rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
      //rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Green
      
      /////////////////Serial.println("Pacchetto SERIALE in arrivo da PC");
      
      if (rx_avalaible > FIFO_SIZE_READ_SER) rx_avalaible= FIFO_SIZE_READ_SER;

      Serial.readBytes(SerialWireless.bufferSerialWrite, rx_avalaible);
      //Serial.write(SerialWireless.bufferSerialWrite, rx_avalaible);
      SerialWireless.lenBufferSerialWrite = rx_avalaible;
      //SerialWireless.printf("Topo gigio come va - millis: %d \n", millis());
      SerialWireless.flush();
      //_queue[rx_avalaible]='\0';
      //Serial.println((char *)_queue);
      //sendSize = 0;

      //sendSize = OpenFIRE_serialTransfer.txObj(_queue, sendSize, rx_avalaible);
      //OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
      rx_avalaible = 0;
    } 

#endif //COMMENTO



  // per inviare
  if (irPosUpdateTick && false) {
      //rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);  // Blue
      //rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
      rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Green

    Serial.println("Pacchetto SERIALE inviato a pistola '.' ");

    //sendSize = 0;
    //sendSize = OpenFIRE_serialTransfer.txObj(".", sendSize, 1);
    //OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
    
    irPosUpdateTick = false;
  } 


  #ifdef COMMENTO
  // Broadcast a message to all devices within the network
  //uint8_t data_read[64];
  //char data[64];
  //snprintf(data, sizeof(data), "Hello, World! da snprintf #%lu \r\n", msg_count++);

  //Serial.printf("Broadcasting message: %s\n", data);
  Serial.printf("Nessun messaggio. Num: %d\n", msg_count++);
  //delay(1000);

  int num_avaliable = ESP32_NOW_OpenFIRE.available();
  //int len = num_avaliable;
  if (num_avaliable) {
    Serial.printf("ARRIVATO UN MESSAGGIO DI NUMERO BYTE: %d \n", num_avaliable);
    //if (num_avaliable > MAX_BUFF_READ) num_avaliable=MAX_BUFF_READ;
  // =================== inserisce nel buffer locale ===============================  
    
  //uint16_t sentBytes = 0;

   
        // Calcola lo spazio disponibile nel buffer circolare
        //uint16_t spaceAvailable = BUFFER_SIZE - _writeLen; //(writeIndex >= readIndex) ? (BUFFER_SIZE - writeIndex + readIndex) : (readIndex - writeIndex - 1); // non ci va il -1 ?????
        uint16_t spaceAvailable = BUFFER_SIZE - _writeLen_atomic.load(); //(writeIndex >= readIndex) ? (BUFFER_SIZE - writeIndex + readIndex) : (readIndex - writeIndex - 1); // non ci va il -1 ?????
        //if (num_avaliable > spaceAvailable) num_avaliable=spaceAvailable;

        // Controlla se c'è spazio disponibile
        if (spaceAvailable >0) {
            if (num_avaliable > spaceAvailable) num_avaliable=spaceAvailable;

            // Calcola quanti byte possono essere scritti
            uint16_t bytesToWrite = num_avaliable; //min(spaceAvailable, len - sentBytes);
                   
            // Scrive i dati nel buffer circolare
            //uint16_t bytesToWriteEnd = min(bytesToWrite, (BUFFER_SIZE - writeIndex));
            uint16_t bytesToWriteEnd;
            if ((BUFFER_SIZE - writeIndex) <= bytesToWrite) bytesToWriteEnd = bytesToWrite;
              else bytesToWriteEnd = BUFFER_SIZE - writeIndex;



            //memcpy(&buffer[writeIndex], &p[sentBytes], bytesToWriteEnd);
            ESP32_NOW_OpenFIRE.read(&buffer[writeIndex], bytesToWriteEnd);
            if (bytesToWrite > bytesToWriteEnd) {
            
                //memcpy(&buffer[0], &p[sentBytes + bytesToWriteEnd], bytesToWrite - bytesToWriteEnd);
                ESP32_NOW_OpenFIRE.read(&buffer[0], bytesToWrite - bytesToWriteEnd);

            }
            //////////////////////////// writeIndex = (writeIndex + bytesToWrite) % BUFFER_SIZE;
            writeIndex += bytesToWrite; 
            if (writeIndex >= BUFFER_SIZE) { writeIndex -= BUFFER_SIZE; } // metodo forse più performante di quello sopra con l'operatore %
            _writeLen_atomic.fetch_add(bytesToWrite);           
}
    
  // ================================================================================  
    
    //if (num_avaliable > MAX_BUFF_READ) num_avaliable=MAX_BUFF_READ;
    
    //msg_count++;  
    //ESP32_NOW_OpenFIRE.read(data_read, num_avaliable);

    
    uint8_t tipo_packet = data_read[0];

    switch (tipo_packet) {
        case SERIAL_TX:
        Serial.println("Pacchetto SERIALE");
        Serial.printf("message: %s Num: %d\n", (char *)&data_read[1], msg_count++);
        break;
        case KEYBOARD_TX:
        /* code */
        break;
        case MOUSE_TX:
        /* code */
        break;
        case GAMEPADE_TX:
        /* code */
        break;
        default:
        break;
    }
    
    //Serial.printf("message: %s Num: %d\n", (char *)data_read, msg_count++);
  
  /*
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.setTextColor(RED);
  tft.println((char *)data_read);
  tft.setCursor(0, 35);
  tft.setTextColor(RED);
  tft.println(msg_count++);
  delay(1000);
  */
  //ESP32_NOW_OpenFIRE.write((uint8_t *) data, sizeof(data),1000);
  //ESP32_NOW_OpenFIRE.printf("Hello, World! #%lu \r\n", msg_count++);

  


  }
  
  #endif // COMMENTO






#ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

#ifdef COMMENTO
  uint8_t buf[64];
  uint32_t count = 0;
  while (SerialTinyUSB.available()) {
    buf[count++] = (uint8_t) toupper(SerialTinyUSB.read());
  }

  if (count) {
    SerialTinyUSB.write(buf, count);
  }
#endif // COMMENTO

}

#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_ESP32)) && defined(DUAL_CORE) && false
// Second core setup
// does... nothing, since timing is kinda important.
void setup1()
{  
    // i sleep
}

// Second core main loop
// currently handles all button & serial processing when Core 0 is in ExecRunMode()
void loop1()
{

    // i sleep

}
#endif // ARDUINO_ARCH_RP2040 || DUAL_CORE
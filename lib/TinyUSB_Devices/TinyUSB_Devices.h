/*
 * This module simulates the standard Arduino "Mouse.h" and
 * "Keyboard.h" API for use with the TinyUSB HID API. Instead of doing
 *  #include <HID.h>
 *  #include <Mouse.h>
 *  #include <Keyboard.h>
 *  
 *  Simply do
 *  
 *  #include <TinyUSB_Mouse_Keyboard.h>
 *  
 *  and this module will automatically select whether or not to use the
 *  standard Arduino mouse and keyboard API or the TinyUSB API. We had to
 *  combine them into a single library because of the way TinyUSB handles
 *  descriptors.
 *  
 *  For details on Arduino Mouse.h see
 *   https://www.arduino.cc/reference/en/language/functions/usb/mouse/
 *  For details on Arduino Keyboard.h see
 *   https://www.arduino.cc/reference/en/language/functions/usb/keyboard/
 *
 *  NOTE: This code is derived from the standard Arduino Mouse.h, Mouse.cpp,
 *    Keyboard.h, and Keyboard.cpp code. The copyright on that original code
 *    is as follows.
 *   
 *  Copyright (c) 2015, Arduino LLC
 *  Original code (pre-library): Copyright (c) 2011, Peter Barrett
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#pragma once


#include <Arduino.h>



#ifdef USE_TINYUSB
  //#define CFG_TUD_ENABLED
     //#define CFG_TUD_CDC
     //#define CFG_TUD_HID
  //#define CFG_TUH_ENABLED 1
     //#define CFG_TUH_CDC
  #include <Adafruit_TinyUSB.h>
  
#elif defined(CFG_TUSB_MCU)
  #error Incompatible USB stack. Use Adafruit TinyUSB.
#else
  #include <HID.h>
#endif

#include "tusb_gamepad16.h"

/*
#include "OpenFIRE_Packet.h"

#if defined(ARDUINO_ARCH_ESP32)
  #include <esp_now.h>
  //#include <ESP32_NOW.h>
  //#include <ESP32_NOW_Serial.h>
  #include <WiFi.h> // serve per leggere max addres
  #include <esp_mac.h>  // For the MAC2STR and MACSTR macros
  #include "MacAddress.h"
  #include "esp_wifi.h"
  #include "freertos/semphr.h"
#elif defined(ARDUINO_ARCH_RP2040)
  // vediamo
#endif
*/

#ifdef OPENFIRE_WIRELESS_ENABLE
  #include "OpenFIRE_Wireless.h"
#endif //OPENFIRE_WIRELESS_ENABLE

//#include "tusb_gamepad16.h"

//SerialTransfer OpenFIRE_transfer_ab;



//#include <HID.h>
//#include <Mouse.h>
//#include <Keyboard.h>
//#include <TinyUSB_Mouse_and_Keyboard.h>


/*****************************
 *   GLOBAL SECTION
 *****************************/

class TinyUSBDevices_ {
public:
  TinyUSBDevices_(void);
  void begin(byte polRate);
  void beginBT(const char *localName, const char *hidName);
  bool onBattery = false; // se connessione usb è false, se con latro tipo di connessione wireless è true
  uint8_t wireless_mode = 0; // 0 = nessuna connessione wireless altro valore connessione // qualòsiasi altro valore deve essere diverso da zeo
};
extern TinyUSBDevices_ TinyUSBDevices;

/*****************************
 *   ESP_NOW
 *****************************/

/*
 class ESP_NOW_Serial_Class_OpenFIRE : public ESP_NOW_Serial_Class {
  public:
  // Constructor of the class
  ESP_NOW_Serial_Class_OpenFIRE(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface = WIFI_IF_STA, const uint8_t *lmk = NULL, bool remove_on_fail = false) : ESP_NOW_Serial_Class(mac_addr, channel, iface, lmk, remove_on_fail) {}

  // Destructor of the class
  ~ESP_NOW_Serial_Class_OpenFIRE() {}


};
*/
/*
enum PACKET_TX {
  SERIAL_TX = 1,
  KEYBOARD_TX,
  MOUSE_TX,
  GAMEPADE_TX
};
*/

/*
enum WIRELESS_MODE {
  NONE =  0,
  ENABLE_BLUETOOTH_TO_PC,         //1
  ENABLE_BLUETOOTH_TO_DONGLE,     //2
  ENABLE_ESP_NOW_TO_DONGLE,       //3
  ENABLE_WIFI_TO_DONGLE,          //4
  ENABLE_NRF_24L01PLUS_TO_DONGLE  //5
};
*/

enum HID_RID_e{
  HID_RID_KEYBOARD = 1,
  //HID_RID_CONSUMER, //2
  HID_RID_MOUSE, //2  3
  HID_RID_GAMEPAD // 3  4
};

uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_RID_e::HID_RID_KEYBOARD)),
  //TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(HID_RID_e::HID_RID_CONSUMER)),
  TUD_HID_REPORT_DESC_ABSMOUSE(HID_REPORT_ID(HID_RID_e::HID_RID_MOUSE)),
  TUD_HID_REPORT_DESC_GAMEPAD16(HID_REPORT_ID(HID_RID_e::HID_RID_GAMEPAD))
};


#ifdef COMMENTO
/*****************************
 *   SERIAL WIRELESS SECTION
 *****************************/
#ifndef _SERIAL_STREAM_H_
#define _SERIAL_STREAM_H_


//#define ESPNOW_WIFI_CHANNEL 4

class SerialWireless_ : public Stream       //, public Packet
  {
    
  public:
    
  // espo32s3 n16r8 workkit
  //const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE8, 0x0F, 0xB8}; // altra periferica con cui si vuole comunicare indirizzo del ESP32
  
  //ESP_NOW_Serial_Class_OpenFIRE ESP32_NOW_Serial_OpenFIRE(peerAddress,ESPNOW_WIFI_CHANNEL,WIFI_IF_STA,NULL,false);
  //ESP_NOW_Serial_Class_OpenFIRE ESP32_NOW_Serial_OpenFIRE;

  //SerialTransfer OpenFIRE_serialTransfer;
  //configST myConfig;

  //SemaphoreHandle_t tx_sem = NULL;
  SemaphoreHandle_t tx_sem = NULL;

  // ======= per FIFO SERIAL ===============
  // ===== per write === buffer lineare ====
  #define FIFO_SIZE_WRITE_SERIAL 200
  #define TIME_OUT_SERIAL_WRITE 3
  uint8_t bufferSerialWrite[FIFO_SIZE_WRITE_SERIAL];
  unsigned long startTimeSerialWrite = 0; // = millis();
  volatile uint16_t lenBufferSerialWrite = 0;
  // ====== per read ====== buffer circolare =====
  #define FIFO_SIZE_READ_SERIAL 200
  uint8_t bufferSerialRead[FIFO_SIZE_READ_SERIAL];
  volatile uint16_t lenBufferSerialRead = 0;
  volatile uint16_t _writerSerialRead = 0;
  volatile uint16_t _readerSerialRead = 0;
  bool _overflow_bufferSerialRead = false; 
  void write_on_rx_serialBuffer(const uint8_t *data, int len);
  // ============================================

  Packet  packet;
  //uint8_t buffer_espnow[ESP_NOW_MAX_DATA_LEN]; // buffer di utilità per inviare con ESP_NOW


  // per buffer lettura
  volatile uint16_t _readLen = 0;
  volatile uint16_t _writer = 0;
  volatile uint16_t _reader = 0;
  #define FIFO_SIZE_READ 1024 // buffer lettura
  uint8_t _queue[FIFO_SIZE_READ];
  bool _overflow_read = false; 
  // fine per buffer lettura

  // per buffer scrittura 
  volatile uint16_t writeIndex = 0;
  volatile uint16_t readIndex = 0;
  volatile uint16_t _writeLen = 0;
  #define BUFFER_SIZE 1024 //buffer scrittura
  uint8_t buffer[BUFFER_SIZE];
  bool _overflow_write = false; 
  /*
  volatile uint16_t _readLen = 0;
  volatile uint16_t _writer = 0;
  volatile uint16_t _reader = 0;
  #define FIFO_SIZE_READ 256
  uint8_t _queue[FIFO_SIZE_READ];
  bool _overflow = false; 
  */
  // fine per buffer scrittura
  




  SerialWireless_() : Stream() {
    //ESP_NOW_Serial_Class_OpenFIRE ESP32_NOW_Serial_OpenFIRE(peerAddress,ESPNOW_WIFI_CHANNEL,WIFI_IF_STA,NULL,false);

  }
  ~SerialWireless_() {}
  
  void begin();
  
  // overrade da ::Stream
  int peek() override;
  int read() override;
  int available() override;
  // fine override da ::stream
  
  // overraid da ::Print
  int availableForWrite() override;
  void flush() override; 
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *data, size_t len) override;
  using Print::write;
  // ==== fine override da :: Print

  // inserire da me per gestione buffer uscita
  size_t writeBin(uint8_t c);
  size_t writeBin(const uint8_t *data, size_t len);
  void flushBin();
  int availableForWriteBin();
  // inserire da me per gestione buffer ingresso
  int peekBin();
  int readBin();
  int availableBin();

  //inserito per gestire Packet
  volatile uint16_t numAvailablePacket = 0;
  int availablePacket();

        
  // ======== generiche ============
  void SendData();  // utilizziamo anche flush
  void SendPacket(); // non penso lo utilizzeremo

  // ======== specifiche ===========
  void SendPacketKeyboard();
  void SendPacketMouse();
  void SendPacketGamepad();
  void SendPacketSerial();

  void ReadPacketKeyboard();
  void ReadPacketMouse();
  void ReadPacketGamepad();
  void ReadPacketSerial();
  // ===============================

  bool checkForTxData(); // non implementato
  bool checkForRxPacket(); // andrà nel ciclo principale
  size_t tryToSend(); // non impelementato

  // ===============================


  
private:

  //static void _esp_now_tx_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
  //static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  // per utilità
  uint16_t sendSize = 0;
  /*
  // per buffer lettura
  volatile uint16_t _readLen = 0;
  volatile uint16_t _writer = 0;
  volatile uint16_t _reader = 0;
  #define FIFO_SIZE_READ 256
  uint8_t _queue[FIFO_SIZE_READ];
  // fine per buffer lettura
  */
  
  //bool _overflow = false; 

  };
extern SerialWireless_ SerialWireless;

#endif // _SERIAL_STREAM_H_
#endif // COMMENTO





/*****************************
 *   MOUSE SECTION
 *****************************/ 
#ifndef _ABSMOUSE5_H_
#define _ABSMOUSE5_H_

/*
#include <stdint.h>

#define MOUSE_LEFT 0x01
#define MOUSE_RIGHT 0x02
#define MOUSE_MIDDLE 0x04
#define MOUSE_BUTTON4 0x08
#define MOUSE_BUTTON5 0x10
*/

#ifdef COMMENTO
#define TUD_HID_REPORT_DESC_ABSMOUSE5(...) \
	0x05, 0x01, \
	0x09, 0x02, \
	0xA1, 0x01, \
	__VA_ARGS__ \
	0x09, 0x01, \
	0xA1, 0x00, \
	0x05, 0x09, \
	0x19, 0x01, \
	0x29, 0x05, \
	0x15, 0x00, \
	0x25, 0x01, \
	0x95, 0x05, \
	0x75, 0x01, \
	0x81, 0x02, \
	0x95, 0x01, \
	0x75, 0x03, \
	0x81, 0x03, \
	0x05, 0x01, \
	0x09, 0x30, \
	0x09, 0x31, \
	0x16, 0x00, 0x00, \
	0x26, 0xFF, 0x7F, \
	0x36, 0x00, 0x00, \
	0x46, 0xFF, 0x7F, \
	0x75, 0x10, \
	0x95, 0x02, \
	0x81, 0x02, \
	0xC0, \
	0xC0
#endif // COMMENTO

#ifdef COMMENTO
// Absolute Mouse Report Descriptor Template
#define TUD_HID_REPORT_DESC_ABSMOUSE5(...) \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP      )                   ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_MOUSE     )                   ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION  )                   ,\
    /* Report ID if any */\
    __VA_ARGS__ \
    HID_USAGE      ( HID_USAGE_DESKTOP_POINTER )                   ,\
    HID_COLLECTION ( HID_COLLECTION_PHYSICAL   )                   ,\
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_BUTTON  )                   ,\
        HID_USAGE_MIN   ( 1                                      ) ,\
        HID_USAGE_MAX   ( 5                                      ) ,\
        HID_LOGICAL_MIN ( 0                                      ) ,\
        HID_LOGICAL_MAX ( 1                                      ) ,\
        /* Left, Right, Middle, Backward, Forward buttons */ \
        HID_REPORT_COUNT( 5                                      ) ,\
        HID_REPORT_SIZE ( 1                                      ) ,\
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
        /* 3 bit padding */ \
        HID_REPORT_COUNT( 1                                      ) ,\
        HID_REPORT_SIZE ( 3                                      ) ,\
        HID_INPUT       ( HID_CONSTANT                           ) ,\
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_DESKTOP )                   ,\
        /* X, Y absolute position [0, 32767] */ \
        HID_USAGE       ( HID_USAGE_DESKTOP_X                    ) ,\
        HID_USAGE       ( HID_USAGE_DESKTOP_Y                    ) ,\
        HID_LOGICAL_MIN  ( 0x00                                ) ,\
        HID_LOGICAL_MAX_N( 0x7FFF, 2                           ) ,\
        HID_REPORT_SIZE  ( 16                                  ) ,\
        HID_REPORT_COUNT ( 2                                   ) ,\
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
        /* Vertical wheel scroll [-127, 127] */ \
        HID_USAGE       ( HID_USAGE_DESKTOP_WHEEL                )  ,\
        HID_LOGICAL_MIN ( 0x81                                   )  ,\
        HID_LOGICAL_MAX ( 0x7f                                   )  ,\
        HID_REPORT_COUNT( 1                                      )  ,\
        HID_REPORT_SIZE ( 8                                      )  ,\
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE )  ,\
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_CONSUMER ), \
       /* Horizontal wheel scroll [-127, 127] */ \
        HID_USAGE_N     ( HID_USAGE_CONSUMER_AC_PAN, 2           ), \
        HID_LOGICAL_MIN ( 0x81                                   ), \
        HID_LOGICAL_MAX ( 0x7f                                   ), \
        HID_REPORT_COUNT( 1                                      ), \
        HID_REPORT_SIZE ( 8                                      ), \
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ), \
    HID_COLLECTION_END                                            , \
  HID_COLLECTION_END \

#endif //COMMENTO

#ifdef COMMENTO
/// Standard HID Boot Protocol Mouse Report.

// Absolute Mouse: same as the Standard (relative) Mouse Report but
// with int16_t instead of int8_t for X and Y coordinates.
typedef struct TU_ATTR_PACKED
{
    uint8_t buttons; /**< buttons mask for currently pressed buttons in the mouse. */
    int16_t x;       /**< Current x position of the mouse. */
    int16_t y;       /**< Current y position of the mouse. */
    int8_t wheel;    /**< Current delta wheel movement on the mouse. */
    int8_t pan;      // using AC Pan
} hid_abs_mouse_report_t;


/// Standard Mouse Buttons Bitmap
typedef enum
{
  MOUSE_BUTTON_LEFT     = TU_BIT(0), ///< Left button
  MOUSE_BUTTON_RIGHT    = TU_BIT(1), ///< Right button
  MOUSE_BUTTON_MIDDLE   = TU_BIT(2), ///< Middle button
  MOUSE_BUTTON_BACKWARD = TU_BIT(3), ///< Backward button,
  MOUSE_BUTTON_FORWARD  = TU_BIT(4), ///< Forward button,
}hid_mouse_button_bm_t;

#endif // COMMENTO


// 5 button absolute mouse
class AbsMouse5_
{
private:
	
  hid_abs_mouse_report_t absmouse5Report = {0,0,0,0,0};
  //const uint8_t _reportId; // non serve a nulla si puo' togliere
	//uint8_t _buttons = 0;
	//int16_t _x = 0;
	//int16_t _y = 0;
  //int8_t _wheel = 0;
  //int8_t _pan = 0;
	//bool _autoReport; // anche questo non serve a nulla, tanto è sempre attivo

public:
	//AbsMouse5_(uint8_t reportId = 1);  // si puo' togliere non serve a nulla
  AbsMouse5_();  // si puo' togliere non serve a nulla
	//void init(bool autoReport = true); // si può togliere non serve a nulla
	void report_absmouse5(void);
	//void move(uint16_t x, uint16_t y);
  void move(int16_t x, int16_t y);
  void move_wheel_pan(int8_t wheel, int8_t pan); // NON INDISPENSABILE AGGIUNTA DA ME
  void press(uint8_t b = MOUSE_BUTTON_LEFT); // solo un bottone per volta
	void release(uint8_t b = MOUSE_BUTTON_LEFT); // solo un bottone per volta
	void releaseAll() { release(0x1f); }
  void click(uint8_t b = MOUSE_BUTTON_LEFT); // clicca tutti i pulsanti come impostati e poi li rilascia subito - valutare se tenerlo // NON INDISPENSABILE AGGIUNTA DA ME
  void buttons(uint8_t b); // imposta tutti i bottoni insieme senza farlo una per volta o li disabilita - valutare se tenerlo // NON INDISPENSABILE AGGIUNTA DA ME
  bool isPressed(uint8_t b = MOUSE_BUTTON_LEFT);   // NON INDISPENSABILE AGGIUNTA DA ME
};

// global singleton
extern AbsMouse5_ AbsMouse5;

#endif // _ABSMOUSE5_H_

/******************************
 *    KEYBOARD SECTION
 ******************************/
#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_


#ifdef COMMENTO
  // Keyboard Report Descriptor Template
#define TUD_HID_REPORT_DESC_KEYBOARD(...) \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                    ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_KEYBOARD )                    ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                    ,\
    /* Report ID if any */\
    __VA_ARGS__ \
    /* 8 bits Modifier Keys (Shift, Control, Alt) */ \
    HID_USAGE_PAGE ( HID_USAGE_PAGE_KEYBOARD )                     ,\
      HID_USAGE_MIN    ( 224                                    )  ,\
      HID_USAGE_MAX    ( 231                                    )  ,\
      HID_LOGICAL_MIN  ( 0                                      )  ,\
      HID_LOGICAL_MAX  ( 1                                      )  ,\
      HID_REPORT_COUNT ( 8                                      )  ,\
      HID_REPORT_SIZE  ( 1                                      )  ,\
      HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE )  ,\
      /* 8 bit reserved */ \
      HID_REPORT_COUNT ( 1                                      )  ,\
      HID_REPORT_SIZE  ( 8                                      )  ,\
      HID_INPUT        ( HID_CONSTANT                           )  ,\
    /* Output 5-bit LED Indicator Kana | Compose | ScrollLock | CapsLock | NumLock */ \
    HID_USAGE_PAGE  ( HID_USAGE_PAGE_LED                   )       ,\
      HID_USAGE_MIN    ( 1                                       ) ,\
      HID_USAGE_MAX    ( 5                                       ) ,\
      HID_REPORT_COUNT ( 5                                       ) ,\
      HID_REPORT_SIZE  ( 1                                       ) ,\
      HID_OUTPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE  ) ,\
      /* led padding */ \
      HID_REPORT_COUNT ( 1                                       ) ,\
      HID_REPORT_SIZE  ( 3                                       ) ,\
      HID_OUTPUT       ( HID_CONSTANT                            ) ,\
    /* 6-byte Keycodes */ \
    HID_USAGE_PAGE ( HID_USAGE_PAGE_KEYBOARD )                     ,\
      HID_USAGE_MIN    ( 0                                   )     ,\
      HID_USAGE_MAX_N  ( 255, 2                              )     ,\
      HID_LOGICAL_MIN  ( 0                                   )     ,\
      HID_LOGICAL_MAX_N( 255, 2                              )     ,\
      HID_REPORT_COUNT ( 6                                   )     ,\
      HID_REPORT_SIZE  ( 8                                   )     ,\
      HID_INPUT        ( HID_DATA | HID_ARRAY | HID_ABSOLUTE )     ,\
  HID_COLLECTION_END \
 
 #endif //COMMENTO
  
  //  Keyboard codes
  //  Note these are different in some respects to the TinyUSB codes but 
  //  are compatible with Arduino Keyboard.h API
  
  /*
  #define KEY_LEFT_CTRL   0x80
  #define KEY_LEFT_SHIFT    0x81
  #define KEY_LEFT_ALT    0x82
  #define KEY_LEFT_GUI    0x83
  #define KEY_RIGHT_CTRL    0x84
  #define KEY_RIGHT_SHIFT   0x85
  #define KEY_RIGHT_ALT   0x86
  #define KEY_RIGHT_GUI   0x87
  
  #define KEY_UP_ARROW    0xDA
  #define KEY_DOWN_ARROW    0xD9
  #define KEY_LEFT_ARROW    0xD8
  #define KEY_RIGHT_ARROW   0xD7
  #define KEY_BACKSPACE   0xB2
  #define KEY_TAB       0xB3
  #define KEY_RETURN      0xB0
  #define KEY_ESC       0xB1
  #define KEY_INSERT      0xD1
  #define KEY_DELETE      0xD4
  #define KEY_PAGE_UP     0xD3
  #define KEY_PAGE_DOWN   0xD6
  #define KEY_HOME      0xD2
  #define KEY_END       0xD5
  #define KEY_CAPS_LOCK   0xC1
  #define KEY_F1        0xC2
  #define KEY_F2        0xC3
  #define KEY_F3        0xC4
  #define KEY_F4        0xC5
  #define KEY_F5        0xC6
  #define KEY_F6        0xC7
  #define KEY_F7        0xC8
  #define KEY_F8        0xC9
  #define KEY_F9        0xCA
  #define KEY_F10       0xCB
  #define KEY_F11       0xCC
  #define KEY_F12       0xCD
  #define KEY_F13       0xF0
  #define KEY_F14       0xF1
  #define KEY_F15       0xF2
  #define KEY_F16       0xF3
  #define KEY_F17       0xF4
  #define KEY_F18       0xF5
  #define KEY_F19       0xF6
  #define KEY_F20       0xF7
  #define KEY_F21       0xF8
  #define KEY_F22       0xF9
  #define KEY_F23       0xFA
  #define KEY_F24       0xFB
  */
  //  Low level key report: up to 6 keys and shift, ctrl etc at once
  /*
  typedef struct
  {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
  } KeyReport;
  */
  /*
   * This class contains the exact same methods as the Arduino Keyboard.h class.
   */
#ifdef COMMENTO  
/// Standard HID Boot Protocol Keyboard Report.
typedef struct TU_ATTR_PACKED
{
  uint8_t modifier;   /**< Keyboard modifier (KEYBOARD_MODIFIER_* masks). */
  uint8_t reserved;   /**< Reserved for OEM use, always set to 0. */
  uint8_t keycode[6]; /**< Key codes of the currently pressed keys. */
} hid_keyboard_report_t;
#endif //COMMENTO

#define SHIFT 0x80 
#define ALT_GR 0xc0 
#define ISO_KEY 0x64
#define ISO_REPLACEMENT 0x32


//extern const uint8_t KeyboardLayout_it_IT[128] PROGMEM =
const uint8_t KeyboardLayout_it_IT[128] =
{
	0x00,          // NUL
	0x00,          // SOH
	0x00,          // STX
	0x00,          // ETX
	0x00,          // EOT
	0x00,          // ENQ
	0x00,          // ACK
	0x00,          // BEL
	0x2a,          // BS  Backspace
	0x2b,          // TAB Tab
	0x28,          // LF  Enter
	0x00,          // VT
	0x00,          // FF
	0x00,          // CR
	0x00,          // SO
	0x00,          // SI
	0x00,          // DEL
	0x00,          // DC1
	0x00,          // DC2
	0x00,          // DC3
	0x00,          // DC4
	0x00,          // NAK
	0x00,          // SYN
	0x00,          // ETB
	0x00,          // CAN
	0x00,          // EM
	0x00,          // SUB
	0x00,          // ESC
	0x00,          // FS
	0x00,          // GS
	0x00,          // RS
	0x00,          // US

	0x2c,          // ' '
	0x1e|SHIFT,    // !
	0x1f|SHIFT,    // "
	0x34|ALT_GR,   // #
	0x21|SHIFT,    // $
	0x22|SHIFT,    // %
	0x23|SHIFT,    // &
	0x2d,          // '
	0x25|SHIFT,    // (
	0x26|SHIFT,    // )
	0x30|SHIFT,    // *
	0x30,          // +
	0x36,          // ,
	0x38,          // -
	0x37,          // .
	0x24|SHIFT,    // /
	0x27,          // 0
	0x1e,          // 1
	0x1f,          // 2
	0x20,          // 3
	0x21,          // 4
	0x22,          // 5
	0x23,          // 6
	0x24,          // 7
	0x25,          // 8
	0x26,          // 9
	0x37|SHIFT,    // :
	0x36|SHIFT,    // ;
	0x32,          // <
	0x27|SHIFT,    // =
	0x32|SHIFT,    // >
	0x2d|SHIFT,    // ?
	0x33|ALT_GR,   // @
	0x04|SHIFT,    // A
	0x05|SHIFT,    // B
	0x06|SHIFT,    // C
	0x07|SHIFT,    // D
	0x08|SHIFT,    // E
	0x09|SHIFT,    // F
	0x0a|SHIFT,    // G
	0x0b|SHIFT,    // H
	0x0c|SHIFT,    // I
	0x0d|SHIFT,    // J
	0x0e|SHIFT,    // K
	0x0f|SHIFT,    // L
	0x10|SHIFT,    // M
	0x11|SHIFT,    // N
	0x12|SHIFT,    // O
	0x13|SHIFT,    // P
	0x14|SHIFT,    // Q
	0x15|SHIFT,    // R
	0x16|SHIFT,    // S
	0x17|SHIFT,    // T
	0x18|SHIFT,    // U
	0x19|SHIFT,    // V
	0x1a|SHIFT,    // W
	0x1b|SHIFT,    // X
	0x1c|SHIFT,    // Y
	0x1d|SHIFT,    // Z
	0x2f|ALT_GR,   // [
	0x35,          // bslash
	0x30|ALT_GR,   // ]
	0x2e|SHIFT,    // ^
	0x38|SHIFT,    // _
	0x00,          // `  not in this layout
	0x04,          // a
	0x05,          // b
	0x06,          // c
	0x07,          // d
	0x08,          // e
	0x09,          // f
	0x0a,          // g
	0x0b,          // h
	0x0c,          // i
	0x0d,          // j
	0x0e,          // k
	0x0f,          // l
	0x10,          // m
	0x11,          // n
	0x12,          // o
	0x13,          // p
	0x14,          // q
	0x15,          // r
	0x16,          // s
	0x17,          // t
	0x18,          // u
	0x19,          // v
	0x1a,          // w
	0x1b,          // x
	0x1c,          // y
	0x1d,          // z
	0x00,          // {  not supported (requires AltGr+Shift)
	0x35|SHIFT,    // |
	0x00,          // }  not supported (requires AltGr+Shift)
	0x00,          // ~  not in this layout
	0x00           // DEL
};


  class Keyboard_ : public Print
  {
  private:
    hid_keyboard_report_t _keyReport = {0,0,{0,0,0,0,0,0}};
    
    //void sendReport(hid_keyboard_report_t* keys);
    void report_keyboard(void);

  public:
    Keyboard_(void);
    size_t write(uint8_t k); // mai usata pa serve per print
    size_t write(const uint8_t *buffer, size_t size); // mai usata ma serve per print
    size_t press(uint8_t k);
    size_t release(uint8_t k); // usata ma non ne vedo il senso
    void releaseAll(void); // usata
  };
extern Keyboard_ Keyboard;

#endif // _KEYBOARD_H_

/*****************************
 *   GAMEPAD SECTION 16
 *****************************/
#ifndef _GAMEPAD_H_
#define _GAMEPAD_H_



//#include "tusb_gamepad16.h"

// incluso in "tusb_gamapad16.h" della dir include
#ifdef COMMENTO
// Gamepad 16 Report Descriptor Template // //NON E' ANCORA DEFINITA IN TINYUSB
// with 32 buttons, 2 joysticks and 1 hat/dpad with following layout
// | X | Y | Z | Rz | Rx | Ry (2 byte each) | hat/DPAD (1 byte) | Button Map (4 bytes) |
#define TUD_HID_REPORT_DESC_GAMEPAD16(...) \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_GAMEPAD  )                 ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
    /* Report ID if any */\
    __VA_ARGS__ \
    /* 16 bit X, Y, Z, Rz, Rx, Ry (min -32767, max 32767 ) */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Z                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RZ                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
    HID_LOGICAL_MIN_N  ( -32767, 2                              ) ,\
    HID_LOGICAL_MAX_N  ( 32767, 2                               ) ,\
    HID_REPORT_COUNT   ( 6                                      ) ,\
    HID_REPORT_SIZE    ( 16                                     ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 8 bit DPad/Hat Button Map  */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
    HID_LOGICAL_MIN    ( 1                                      ) ,\
    HID_LOGICAL_MAX    ( 8                                      ) ,\
    HID_PHYSICAL_MIN   ( 0                                      ) ,\
    HID_PHYSICAL_MAX_N ( 315, 2                                 ) ,\
    HID_REPORT_COUNT   ( 1                                      ) ,\
    HID_REPORT_SIZE    ( 8                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 32 bit Button Map */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
    HID_USAGE_MIN      ( 1                                      ) ,\
    HID_USAGE_MAX      ( 32                                     ) ,\
    HID_LOGICAL_MIN    ( 0                                      ) ,\
    HID_LOGICAL_MAX    ( 1                                      ) ,\
    HID_REPORT_COUNT   ( 32                                     ) ,\
    HID_REPORT_SIZE    ( 1                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END \

#endif //COMMENTO

#ifdef COMMENTO // PRECEDENTE IMPOSTAZIONE
#define TUD_HID_REPORT_DESC_GAMEPAD16(...) \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_GAMEPAD  )                 ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
    /* Report ID if any */\
    __VA_ARGS__ \
    /* 16 bit X, Y, Rx, Ry (min -32767, max 32767 ) */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
    HID_LOGICAL_MIN_N  ( -32767, 2                              ) ,\
    HID_LOGICAL_MAX_N  ( 32767, 2                               ) ,\
    HID_REPORT_COUNT   ( 4                                      ) ,\
    HID_REPORT_SIZE    ( 16                                     ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 8 bit DPad/Hat Button Map  */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
    HID_LOGICAL_MIN    ( 1                                      ) ,\
    HID_LOGICAL_MAX    ( 8                                      ) ,\
    HID_PHYSICAL_MIN   ( 0                                      ) ,\
    HID_PHYSICAL_MAX_N ( 315, 2                                 ) ,\
    HID_REPORT_COUNT   ( 1                                      ) ,\
    HID_REPORT_SIZE    ( 8                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 16 bit Button Map */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
    HID_USAGE_MIN      ( 1                                      ) ,\
    HID_USAGE_MAX      ( 15                                     ) ,\
    HID_LOGICAL_MIN    ( 0                                      ) ,\
    HID_LOGICAL_MAX    ( 1                                      ) ,\
    HID_REPORT_COUNT   ( 16                                     ) ,\
    HID_REPORT_SIZE    ( 1                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END
#endif //COMMENTO


//uint32_t x = TU_BIT(0);


//  OPENFIRE                   TINYUSB
#define PAD_A      0    // GAMEPAD_BUTTON_A
#define PAD_B      1    // GAMEPAD_BUTTON_B
#define PAD_C      2    // GAMEPAD_BUTTON_C
#define PAD_X      3    // GAMEPAD_BUTTON_X
#define PAD_Y      4    // GAMEPAD_BUTTON_Y
#define PAD_Z      5    // GAMEPAD_BUTTON_Z
#define PAD_LB     6    // GAMEPAD_BUTTON_TL
#define PAD_RB     7    // GAMEPAD_BUTTON_TR
#define PAD_LT     8    // GAMEPAD_BUTTON_TL2
#define PAD_RT     9    // GAMEPAD_BUTTON_TR2
#define PAD_SELECT 10   // GAMEPAD_BUTTON_SELECT
#define PAD_START  11   // GAMEPAD_BUTTON_START
#define PAD_HOME   12   // GAMEPAD_BUTTON_MODE
#define PAD_LS     13   // GAMEPAD_BUTTON_THUMBL
#define PAD_RS     14   // GAMEPAD_BUTTON_THUMBR
#define PAD_15     15   // GAMEPAD_BUTTON_15      // TUTTI QUELLI DA ORA IN SEGUITO SONO CUSTOM
#define PAD_16     16   // GAMEPAD_BUTTON_16
#define PAD_17     17   // GAMEPAD_BUTTON_17
#define PAD_18     18   // GAMEPAD_BUTTON_18
#define PAD_19     19   // GAMEPAD_BUTTON_19
#define PAD_20     20   // GAMEPAD_BUTTON_20
#define PAD_21     21   // GAMEPAD_BUTTON_21
#define PAD_22     22   // GAMEPAD_BUTTON_22
#define PAD_23     23   // GAMEPAD_BUTTON_23
#define PAD_24     24   // GAMEPAD_BUTTON_24
#define PAD_25     25   // GAMEPAD_BUTTON_25
#define PAD_26     26   // GAMEPAD_BUTTON_26
#define PAD_27     27   // GAMEPAD_BUTTON_27
#define PAD_28     28   // GAMEPAD_BUTTON_28
#define PAD_29     29   // GAMEPAD_BUTTON_29
#define PAD_30     30   // GAMEPAD_BUTTON_30
#define PAD_31     31   // GAMEPAD_BUTTON_31

#ifdef COMMENTO

#define HID_PAD_TO_GAMEPAD_BUTTON    \
    { PAD_A      , GAMEPAD_BUTTON_A      }, /*  0 */ \
    { PAD_B      , GAMEPAD_BUTTON_B      }, /*  1 */ \
    { PAD_C      , GAMEPAD_BUTTON_C      }, /*  2 */ \
    { PAD_X      , GAMEPAD_BUTTON_X      }, /*  3 */ \
    { PAD_Y      , GAMEPAD_BUTTON_Y      }, /*  4 */ \
    { PAD_Z      , GAMEPAD_BUTTON_Z      }, /*  5 */ \
    { PAD_LB     , GAMEPAD_BUTTON_TL     }, /*  6 */ \
    { PAD_RB     , GAMEPAD_BUTTON_TR     }, /*  7 */ \
    { PAD_LT     , GAMEPAD_BUTTON_TL2    }, /*  8 */ \
    { PAD_RT     , GAMEPAD_BUTTON_TR2    }, /*  9 */ \
    { PAD_SELECT , GAMEPAD_BUTTON_SELECT }, /* 10 */ \
    { PAD_START  , GAMEPAD_BUTTON_START  }, /* 11 */ \
    { PAD_HOME   , GAMEPAD_BUTTON_MODE   }, /* 12 */ \
    { PAD_LS     , GAMEPAD_BUTTON_THUMBL }, /* 13 */ \
    { PAD_RS     , GAMEPAD_BUTTON_THUMBR }, /* 14 */ \
    { PAD_15     , GAMEPAD_BUTTON_15     }, /* 15 */ \
    { PAD_16     , GAMEPAD_BUTTON_16     }, /* 16 */ \
    { PAD_17     , GAMEPAD_BUTTON_17     }, /* 17 */ \
    { PAD_18     , GAMEPAD_BUTTON_18     }, /* 18 */ \
    { PAD_19     , GAMEPAD_BUTTON_19     }, /* 19 */ \
    { PAD_20     , GAMEPAD_BUTTON_20     }, /* 20 */ \
    { PAD_21     , GAMEPAD_BUTTON_21     }, /* 21 */ \
    { PAD_22     , GAMEPAD_BUTTON_22     }, /* 22 */ \
    { PAD_23     , GAMEPAD_BUTTON_23     }, /* 23 */ \
    { PAD_24     , GAMEPAD_BUTTON_24     }, /* 24 */ \
    { PAD_25     , GAMEPAD_BUTTON_25     }, /* 25 */ \
    { PAD_26     , GAMEPAD_BUTTON_26     }, /* 26 */ \
    { PAD_27     , GAMEPAD_BUTTON_27     }, /* 27 */ \
    { PAD_28     , GAMEPAD_BUTTON_28     }, /* 28 */ \
    { PAD_29     , GAMEPAD_BUTTON_29     }, /* 29 */ \
    { PAD_30     , GAMEPAD_BUTTON_30     }, /* 30 */ \
    { PAD_31     , GAMEPAD_BUTTON_31     }, /* 31 */ \
    /* TERMINATA LA LISTA DEI 32 PULSANTI */ \
    {   , GAMEPAD_BUTTON_15    }, /*  */ \
    {   , GAMEPAD_BUTTON_15    }, /*  */ \
    {   , GAMEPAD_BUTTON_15    }, /*  */ \
    {   , GAMEPAD_BUTTON_15    }, /*  */ \


#endif // COMMENTO 

// I CODICE DEI TASTI PER IL DPAD (HAT) INZIAZIONO DA 32 OVVERO DOPO I 32 (0-31) CODICI PER I BOTTONI

#define PAD_UP     32
#define PAD_DOWN   33
#define PAD_LEFT   34
#define PAD_RIGHT  35



/*
#define GAMEPAD_HAT_CENTERED 0
#define GAMEPAD_HAT_UP 1
#define GAMEPAD_HAT_UP_RIGHT 2
#define GAMEPAD_HAT_RIGHT 3
#define GAMEPAD_HAT_DOWN_RIGHT 4
#define GAMEPAD_HAT_DOWN 5
#define GAMEPAD_HAT_DOWN_LEFT 6
#define GAMEPAD_HAT_LEFT 7
#define GAMEPAD_HAT_UP_LEFT 8
*/
/*
typedef struct {
        int16_t X = 0;
        int16_t Y = 0;
        int16_t Rx = 0;
        int16_t Ry = 0;
        uint8_t hat;
        uint16_t buttons;     // button bitmask
} __attribute__ ((packed)) gamepad16Report_s;
*/
/*
/// HID Gamepad Protocol Report.
typedef struct TU_ATTR_PACKED
{
  int8_t  x;         ///< Delta x  movement of left analog-stick
  int8_t  y;         ///< Delta y  movement of left analog-stick
  int8_t  z;         ///< Delta z  movement of right analog-joystick
  int8_t  rz;        ///< Delta Rz movement of right analog-joystick
  int8_t  rx;        ///< Delta Rx movement of analog left trigger
  int8_t  ry;        ///< Delta Ry movement of analog right trigger
  uint8_t hat;       ///< Buttons mask for currently pressed buttons in the DPad/hat
  uint32_t buttons;  ///< Buttons mask for currently pressed buttons
}hid_gamepad_report_t;
*/

// incluso in "tusb_gamapad16.h" della dir include
#ifdef COMMENTO
// HID Gamepad Protocol Report. //NON E' ANCORA DEFINITA IN TINYUSB
typedef struct TU_ATTR_PACKED {
    int16_t x;         ///< Delta x  movement of left analog-stick
    int16_t y;         ///< Delta y  movement of left analog-stick
    int16_t z;         ///< Delta z  movement of right analog-joystick
    int16_t rz;        ///< Delta Rz movement of right analog-joystick
    int16_t rx;        ///< Delta Rx movement of analog left trigger
    int16_t ry;        ///< Delta Ry movement of analog right trigger
    uint8_t hat;       ///< Buttons mask for currently pressed buttons in the DPad/hat // typedef enum hid_gamepad_hat_t;
    uint32_t buttons;  ///< Buttons mask for currently pressed buttons                 // typedef enum hid_gamepad_button_bm_t;
} hid_gamepad16_report_t;
#endif //COMMENTO



class Gamepad16_ {
private:
  hid_gamepad16_report_t gamepad16Report = {0,0,0,0,0,0,0,0};
  uint16_t _x = 2048; // A META' CONSIDERANDO IL RANGE DA 0 A 4095
  uint16_t _y = 2048; // A META' CONSIDERANDO IL RANGE DA 0 A 4095
  //bool _autoReport = true; // NON UTILIZZATO, SI PUO' TOGLIERE
public:
  Gamepad16_(void);
  void moveCam(uint16_t origX, uint16_t origY);
  void moveStick(uint16_t origX, uint16_t origY);
  void press(uint8_t buttonNum);
  void release(uint8_t buttonNum);
  void padUpdate(uint8_t padMask);
  void report_gamepad16(void);
  void releaseAll(void);
  //void setAutoreport(bool state) { _autoReport = state; } // MAI USATA SI PUO' TOGLIERE
  bool stickRight;
};
extern Gamepad16_ Gamepad16;


#ifdef COMMENTO

/*****************************
 *   GAMEPAD SECTION (default 8 bit)
 *****************************/
class Gamepad_ {
  private:
    hid_gamepad_report_t gamepadReport = {0,0,0,0,0,0,0,0};
    //uint16_t _x = 2048; // A META' CONSIDERANDO IL RANGE DA 0 A 4095
    //uint16_t _y = 2048; // A META' CONSIDERANDO IL RANGE DA 0 A 4095
    //bool _autoReport = true; // NON UTILIZZATO, SI PUO' TOGLIERE
  public:
    Gamepad_(void);
    //void moveCam(uint16_t origX, uint16_t origY);
    //void moveStick(uint16_t origX, uint16_t origY);
    //void press(uint8_t buttonNum);
    //void release(uint8_t buttonNum);
    //void padUpdate(uint8_t padMask);
    //void report_gamepad16(void);
    //void releaseAll(void);
    //void setAutoreport(bool state) { _autoReport = state; } // MAI USATA SI PUO' TOGLIERE
    bool stickRight;
  };
  extern Gamepad_ Gamepad;

  #endif // COMMENTO

  #endif // _GAMEPAD_H_
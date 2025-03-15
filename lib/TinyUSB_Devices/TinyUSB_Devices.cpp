/*
 * This module (hastily) combines Mike Lynch's modified "AbsMouse5" library, and
 * Chris Young's TinyUSB Mouse and Keyboard library (the Keyboard half, anyways),
 * which in itself uses pieces of Arduino's basic Keyboard library.
 */
/*
#ifdef USE_TINYUSB
  //#define CFG_TUD_ENABLED
     //#define CFG_TUD_CDC
     //#define CFG_TUD_HID
  //#define CFG_TUH_ENABLED 1
     //#define CFG_TUH_CDC
  
  //#include <Adafruit_TinyUSB.h>
#elif defined(CFG_TUSB_MCU)
  #error Incompatible USB stack. Use Adafruit TinyUSB.
#else
  #include <HID.h>
#endif
*/
////////#include <sdkoverride/tusb_gamepad16.h>  ////////////////////////////////
#include "TinyUSB_Devices.h"

#if defined(ENABLE_BLUETOOTH_OPENFIRE)
/////#include <HID_Bluetooth.h>
////#include <PicoBluetoothHID.h>
    ////////////#include "BT_HID_SPP_OpenFIRE.h"
    ////////////PicoBluetooth_OpenFIRE_ PicoBluetooth_OpenFIRE;
    
    /// #include "PicoBluetoothHID_OpenFIRE.h"
    //// #include "SerialBT_OpenFIRE.h"
    ////////////////////////--//////////////////////////#include "BluetoothLE_OpenFIRE.h"
    #include "Bluetooth_OpenFIRE.h"
    //// PicoBluetoothHID_OpenFIRE_ PicoBluetoothHID_OpenFIRE;
    ///// SerialBT_OpenFIRE_ SerialBT_OpenFIRE;

    ////////////////////--////////////////////////////////Bluetooth_OpenFIRE_ Bluetooth_OpenFIRE;
    BT_OpenFIRE_ BT_OpenFIRE;
    
    /*
    #ifdef ENABLE_BLE
    OpenFIRE_PicoBluetoothBLEHID_* Bluetooth_OpenFIRE_HID;
    #else
    OpenFIRE_PicoBluetoothHID_* Bluetooth_OpenFIRE_HID;
    #endif
    */

    
    //extern PicoBluetooth_OpenFIRE_ PicoBluetooth_OpenFIRE;
////#include <SerialBT.h>
#endif // ARDUINO_RASPBERRY_PI_PICO_W

/*****************************
 *   GLOBAL SECTION
 *****************************/
//Adafruit_USBD_HID usbHid;

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

/*
enum HID_RID_e{
    HID_RID_KEYBOARD = 1,
    //HID_RID_CONSUMER, //2
    HID_RID_MOUSE, //2  3
    HID_RID_GAMEPAD // 3  4
};
*/

/*
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_RID_KEYBOARD)),
    //TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(HID_RID_CONSUMER)),
    TUD_HID_REPORT_DESC_ABSMOUSE(HID_REPORT_ID(HID_RID_MOUSE)),
    TUD_HID_REPORT_DESC_GAMEPAD16(HID_REPORT_ID(HID_RID_GAMEPAD))
};
*/


#if defined(ENABLE_BLUETOOTH_OPENFIRE)
enum HID_BT_e {
    HID_BT_KEYBOARD = 1,
    // HID_BT_CONSUMER, //2
    HID_BT_MOUSE,  // 2 3
    HID_BT_GAMEPAD // 34 
};

uint8_t const desc_bt_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_BT_KEYBOARD)),
    //TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(HID_BT_CONSUMER)),
     // METTERE I NUMERI 1
    //TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(HID_BT_MOUSE)),//, //2
    ////TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_BT_MOUSE))//,//, //2
    TUD_HID_REPORT_DESC_ABSMOUSE(HID_REPORT_ID(HID_BT_MOUSE)),//, //2   ***********************
    //TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(HID_BT_GAMEPAD))
    TUD_HID_REPORT_DESC_GAMEPAD16(HID_REPORT_ID(HID_BT_GAMEPAD))//,//, //3  **********************
    //TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(HID_BT_CONSUMER)) // 4
    //,
    //TUD_HID_REPORT_DESC_GAMEPAD16(HID_REPORT_ID(2))
};
/*
    #include <sdkoverride/tusb_gamepad16.h>
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(1)) 
    TUD_HID_REPORT_DESC_ABSMOUSE(HID_REPORT_ID(1))
    TUD_HID_REPORT_DESC_GAMEPAD16(HID_REPORT_ID(1)) 
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(1))
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(2))
*/


#endif // ARDUINO_RASPBERRY_PI_PICO_W


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
//const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE8, 0x0F, 0xB8}; // espe32s3 con piedini
const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE7, 0x65, 0xD4}; // espe32s3 senz apiedini
//const uint8_t peerAddress[6] = {0xF0, 0x9E, 0x9E, 0x28, 0x9E, 0xB8}; // DONGLE LILYGO
*/

/*
#define ESPNOW_WIFI_CHANNEL 4
// la potenza di trasmissione può andare da 8 a 84, dove 84 è il valore massimo che corrisponde a 20 db
#define ESPNOW_WIFI_POWER 84 
*/

// espo32s3 n16r8 workkit
////////////////////////////const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE8, 0x0F, 0xB8}; // altra periferica con cui si vuole comunicare indirizzo del ESP32


//const MacAddress peerAddress({0xA0, 0x85, 0xE3, 0xE8, 0x0F, 0xB8});
//esp_now_peer_info_t peerInfo;

//ESP_NOW_Serial_Class_OpenFIRE ESP32_NOW_Serial_OpenFIRE(peerAddress,ESPNOW_WIFI_CHANNEL,WIFI_IF_STA,NULL,false);
//ESP_NOW_Serial_Class ESP32_NOW_Serial_OpenFIRE(peerAddress,ESPNOW_WIFI_CHANNEL,WIFI_IF_STA);
  
//uint32_t msg_count = 0;

//#define MAX_BUFF_READ 128

//uint8_t data_read[MAX_BUFF_READ];


//extern USBCDC USBserial0;

//#define SerialWireless ESP32_NOW_Serial_OpenFIRE

//Stream* OpenFIRE_SerialWireless;

/*
enum PACKET_TX {
  SERIAL_TX = 0,
  KEYBOARD_TX,
  MOUSE_TX,
  GAMEPADE_TX
};
*/


////////////////////////SerialTransfer OpenFIRE_serialTransfer;
///////////////////////////////////////////////////////////////////////////SerialWireless_ SerialWireless;
//SemaphoreHandle_t tx_sem = NULL;

#ifdef COMMENTO

/////////////////////////////////////////////////////////////////// Callbacks
static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len); // callback esp_now
static void _esp_now_tx_cb(const uint8_t *mac_addr, esp_now_send_status_t status); // callback esp_now

void packet_callback_read_dongle(); // callback packet 
void packet_callback_read_gun(); // callback packet




//void serialTransfer_callback_read();
/*
void serialTransfer_callback_read()
{
  Serial.println("hi");
}
*/

// supplied as a reference - persistent allocation required
//const functionPtr callbackArr[] = { serialTransfer_callback_read };
#if defined(DONGLE)
  const uint8_t peerAddress[6] = {0x24, 0x58, 0x7C, 0xDA, 0x38, 0xA0}; // quello montato con piedini su breackboard

  const functionPtr callbackArr[] = { packet_callback_read_dongle };
#elif defined(GUN)
  //const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE8, 0x0F, 0xB8}; // espe32s3 con piedini (riceve ma non trasmette)
  const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE7, 0x65, 0xD4}; // espe32s3 senz apiedini
  //const uint8_t peerAddress[6] = {0xF0, 0x9E, 0x9E, 0x28, 0x9E, 0xB8}; // DONGLE LILYGO

  const functionPtr callbackArr[] = { packet_callback_read_gun };
#endif
///////////////////////////////////////////////////////////////////

#endif // COMMENTO

///////////////////////////////////////////////////////////////// Config Parameters
//configST myConfig;
//myConfig.debug        = true;
//myConfig.callbacks    = callbackArr;
//myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
/////////////////////////////////////////////////////////////////

/*****************************
 *   GLOBAL SECTION
 *****************************/
Adafruit_USBD_HID usbHid;


//Stream* OpenFIRE_SerialWireless;

TinyUSBDevices_::TinyUSBDevices_(void) {
}

void TinyUSBDevices_::begin(byte polRate) {
    usbHid.setPollInterval(polRate);
    usbHid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usbHid.setStringDescriptor("OpenFIRE_HID"); // ??? per nome // funziona anche su rp2040 ?????
    usbHid.begin();
    // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
    if (TinyUSBDevice.mounted()) {
      TinyUSBDevice.detach();
      delay(10);
      TinyUSBDevice.attach();
    }
    onBattery = false;
    
    #ifdef COMMENTO
    #ifdef OPENFIRE_WIRELESS_ENABLE
    // ====================================================
    // ============== VA SPOSTATO NEL MAIN ================
    // ====================================================
    //onBattery = true;
    Serial.begin(9600);   // 9600 = 1ms data transfer rates, default for MAMEHOOKER COM devices.
    #define SERIAL_TIMEOUT 1000
    Serial.setTimeout(SERIAL_TIMEOUT);

    // ====================PER GESTIONE CONNESSIONE WIFI ESP NOW =======================

    //SerialTransfer OpenFIRE_transfer_ab; // _________________________________________
    ////////////////////////onBattery = true;
    wireless_mode = WIRELESS_MODE::ENABLE_ESP_NOW_TO_DONGLE;
    SerialWireless.begin();
    onBattery = true;
    // ====================================================
    // ====================================================
    // ====================================================
    #endif //OPENFIRE_WIRELESS_ENABLE
    #endif //COMMENTO

#ifdef COMMENTO // ===============================================================================

  
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
    ///WiFi.begin();

    while (!WiFi.STA.started()) {
      delay(100);
    }
  
  
  //while(!TinyUSBDevice.mounted()) { yield(); }
  //#define SERIAL_TIMEOUT 0 //1000 DI DEFAULT .. SECONDO ME PUÒ STARE ANCHE A 0
  //Serial.begin(9600);   // 9600 = 1ms data transfer rates, default for MAMEHOOKER COM devices.  115200
  //Serial.setTimeout(SERIAL_TIMEOUT);
  
  //Serial.println("WIFI Attivo");
  //Serial.println(WiFi.macAddress());


  delay (1000);
  /*
    tft.fillScreen(BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextColor(RED);
    tft.println("WIFI Attivo");
    delay (1000);
    */
if (!ESP32_NOW_Serial_OpenFIRE.begin(0)) {
    //Serial.println("FAILED a inizializzare ESP NOW SERIAL");
    delay (1000);
    /*
    tft.setTextSize(2);
    tft.setCursor(0, 35);
    tft.setTextColor(RED);
    tft.println("Failed Esp Now");
    delay (1000);
    */
  }
  else {
    //Serial.println("ESP NOW Serial inizializzato regolarmente");
    delay (1000);
    /*
    tft.setTextSize(2);
    tft.setCursor(0, 35);
    tft.setTextColor(RED);
    tft.println("ESP NOW OK");
    delay (1000);
    */
  }

  //Serial.println("ESP-NOW Wireless");
  //Serial.println("Wi-Fi parameters:");
  //Serial.println("Mode: STA");
  //Serial.println("MAC Address: " + WiFi.macAddress());
  //Serial.printf("Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  OpenFIRE_SerialWireless = &ESP32_NOW_Serial_OpenFIRE;
  //OpenFIRE_serialTransfer.begin(OpenFIRE_SerialWireless);
  ////////////////////////OpenFIRE_serialTransfer.begin(ESP32_NOW_Serial_OpenFIRE);

  ///////////////////////////////////////////////////////////////// Config Parameters
  myConfig.debug        = true;
  myConfig.debugPort    = &Serial;
  myConfig.timeout      = DEFAULT_TIMEOUT;
  //myConfig.callbacks    = callbackArr;
  myConfig.callbacks    = NULL;
  //myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
  myConfig.callbacksLen = 0;
  /////////////////////////////////////////////////////////////////


  OpenFIRE_serialTransfer.begin(ESP32_NOW_Serial_OpenFIRE, myConfig);
  delay(500);
  // ========== VA DENTRO AL LOOP ==============
  ////////////////////////////////////////OpenFIRE_serialTransfer.tick(); //DENTRO AL LOOP
  // ========== VA DENTRO AL LOOP ==============
  /*
  struct __attribute__((packed)) STRUCT {
    char z;
    float y;
  } testStruct;
  
  char arr[] = "hello";

  testStruct.z = '$';
  testStruct.y = 4.5;
  */
  
  // use this variable to keep track of how many
  // bytes we're stuffing in the transmit buffer
  //////////uint16_t sendSize = 0;

  ///////////////////////////////////////// Stuff buffer with struct
  //sendSize = OpenFIRE_serialTransfer.txObj(testStruct, sendSize);

  ///////////////////////////////////////// Stuff buffer with array
  /////////////sendSize = OpenFIRE_serialTransfer.txObj(arr, sendSize);

  ///////////////////////////////////////// Send buffer
  ///////////////////OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
  delay(500);

#endif // COMMENTO ============================================



  /*
// ===========================================================================================

// TX SERIAL
uint8_t data[64];
data[0]= SERIAL_TX;
snprintf((char *)&data[1], sizeof(data)-1, "Hello, World! da snprintf #%lu \r\n", msg_count++);

Serial.printf("Pacchetto SERIALE message: %s\n", &data[1]);

//delay(10);
int avaliable_num = ESP32_NOW_Serial_OpenFIRE.availableForWrite(); // di default 1000 byte di buffer (si può cambiare)
if (avaliable_num) {
  Serial.printf("Avaliable byte: %d\n", avaliable_num);  
  ESP32_NOW_Serial_OpenFIRE.write((uint8_t *) data, 1 + strlen((char *)&data[1]) + 1,1000);
  //ESP32_NOW_OpenFIRE.printf("Hello, World! #%lu \r\n\0", msg_count++);

  // TX TASTIERA


  // TX MOUSE


  // TX GAMEPAD
}

// ====================================================================================================
*/

}





#if defined(ENABLE_BLUETOOTH_OPENFIRE)

///////#include <SerialBT.h> //////////////////////////////////////////////////////////

void TinyUSBDevices_::beginBT(const char *localName, const char *hidName) {
    // third arg is the type of device that this is exposed as, i.e. the icon displayed on the PC.
    // for BLE: 0x03C2 is mouse, 0x03C1 is keyboard, 0x03C4 is gamepad, 0x03C0 is "generic" bluetooth icon
    // for BT: 0x2580 is mouse, 0x2540 is keyboard, 0x2508 is gamepad, 0x25C0 is "combo".
    // also bluetooth classic for some reason has a "subclass"?
    
    //SerialBT.begin(); ///////////////////
    //SerialBT.begin(9600);

    /////////PicoBluetooth_OpenFIRE.start(localName, hidName, 0x2580, 33, desc_bt_report, sizeof(desc_bt_report),(unsigned long) 9600 , (uint16_t) SERIAL_8N1);
    //PicoBluetoothHID.startHID("DOPPIO", "TRIPRO", 0x2508, 33, desc_bt_report2, sizeof(desc_bt_report2));
    //PicoBluetoothHID.startHID(localName3, hidName3, 0x2540, 33, desc_bt_report3, sizeof(desc_bt_report3));
    //PicoBluetooth_OpenFIRE::SerialBT_OpenFIRE_.setTimeout(0);
    
        
        /*
        gap_ssp_set_enable(false);
        l2cap_init();
        #ifdef ENABLE_BLE
            // Initialize LE Security Manager. Needed for cross-transport key derivation
            sm_init();
        #endif        
        //rfcomm_init();
        sdp_init();
    
        uint16_t hidClass = 0x2580;
        uint8_t hidSubclass = 33;
        
        //PicoBluetoothHID_OpenFIRE.startHID(localName, hidName, hidClass, hidSubclass, desc_bt_report, sizeof(desc_bt_report)); // prima hid e poi seriale altrimenti non funziona, non so perchè
        
        SerialBT_OpenFIRE.begin((unsigned long) 9600 , (uint16_t) SERIAL_8N1);
        PicoBluetoothHID_OpenFIRE.startHID(localName, hidName, hidClass, hidSubclass, desc_bt_report, sizeof(desc_bt_report));

        gap_ssp_set_enable(false);
        //hci_local_ssp_activated();
        //gap_ssp_set_auto_accept(true);
        //gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);

        gap_discoverable_control(1);
        //gap_connectable_control(1);
        // Use Limited Discoverable Mode; Peripheral; Keyboard as CoD
        gap_set_class_of_device(hidClass);
        // Set local name to be identified - zeroes will be replaced by actual BD ADDR
        gap_set_local_name(localName);
        // Allow for role switch in general and sniff mode
        gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
        // Allow for role switch on outgoing connections - this allow HID Host to become master when we re-connect to it
        gap_set_allow_role_switch(true);
        
        // fine HID
        //gap_ssp_set_enable(false);
        hci_power_control(HCI_POWER_ON);
        //SerialBT_OpenFIRE.begin((unsigned long) 9600 , (uint16_t) SERIAL_8N1);
        //gap_ssp_set_enable(false);
      */    

    uint16_t hidClass = 0x25C0; //0x2580; // x BT Classic
    // HID Class
    //mouse only: 0x2580;
    //keyboard only: 0x2540;
    //joystick only: 0x2508;
    //combo device: 0x25C0;
    
    uint8_t hidSubclass = 33; //0x03C0; //33; country_code;   33=US  14=IT 32=UK 13=International (ISO)
    
    uint16_t appearance = 0x03C0; //0x03C2; // x BLE
    // HID appearance per BLE
    //mouse only: 0x03C2;
    //keyboard only: 0x03C1;
    //joystick only: 0x03C4;
    //generic HID: 0x03C0;

    int battery = 50; //100

    //Bluetooth_OpenFIRE.startHID(localName, hidName, hidClass, hidSubclass, desc_bt_report, sizeof(desc_bt_report), (unsigned long) 9600 , (uint16_t) SERIAL_8N1);
    
    //Bluetooth_OpenFIRE.startHID(localName, hidName, hidClass, hidSubclass, desc_bt_report, sizeof(desc_bt_report));
    BT_OpenFIRE.startHID(localName, hidName, appearance, hidClass, hidSubclass, desc_bt_report, sizeof(desc_bt_report), battery, (unsigned long) 9600 , (uint16_t) SERIAL_8N1);
    
    /*
    #ifdef ENABLE_BLE
    Bluetooth_OpenFIRE_HID = & BT_OpenFIRE.OpenFIRE_PicoBluetoothBLEHID;
    #else
    Bluetooth_OpenFIRE_HID = & BT_OpenFIRE.OpenFIRE_PicoBluetoothHID;
    #endif
    */
    
    /*
    #ifdef ENABLE_BLE
    Bluetooth_OpenFIRE_HID = BT_OpenFIRE.getBLEHID();
    #else
    Bluetooth_OpenFIRE_HID = BT_OpenFIRE.getHID();
    #endif
    */
    
    
    //Bluetooth_OpenFIRE_HID=
    //Bluetooth_OpenFIRE_SPP=

    onBattery = true;
    
    //#define SSP_IO_CAPABILITY_DISPLAY_YES_NO 0
    /////SerialBT.begin(9600); /////////////////////////////////////////////////////////
    //SerialBT.setName("OpenFire Serial");
    /////SerialBT.begin();
}
#endif // ARDUINO_RASPBERRY_PI_PICO_W

TinyUSBDevices_ TinyUSBDevices;
  
/*****************************
 *   MOUSE SECTION
 *****************************/
/*
#if defined(_USING_HID)
static const uint8_t HID_REPORT_DESCRIPTOR5[] PROGMEM = {
	0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
	0x09, 0x02,        // Usage (Mouse)
	0xA1, 0x01,        // Collection (Application)
	0x09, 0x01,        //   Usage (Pointer)
	0xA1, 0x00,        //   Collection (Physical)
	0x85, 0x01,        //     Report ID (1)
	0x05, 0x09,        //     Usage Page (Button)
	0x19, 0x01,        //     Usage Minimum (0x01)
	0x29, 0x05,        //     Usage Maximum (0x05)
	0x15, 0x00,        //     Logical Minimum (0)
	0x25, 0x01,        //     Logical Maximum (1)
	0x95, 0x05,        //     Report Count (5)
	0x75, 0x01,        //     Report Size (1)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x95, 0x01,        //     Report Count (1)
	0x75, 0x03,        //     Report Size (3)
	0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
	0x09, 0x30,        //     Usage (X)
	0x09, 0x31,        //     Usage (Y)
	0x16, 0x00, 0x00,  //     Logical Minimum (0)
	0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
	0x36, 0x00, 0x00,  //     Physical Minimum (0)
	0x46, 0xFF, 0x7F,  //     Physical Maximum (32767)
	0x75, 0x10,        //     Report Size (16)
	0x95, 0x02,        //     Report Count (2)
	0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0xC0,              //   End Collection
	0xC0               // End Collection
};
#endif // _USING_HID
*/
//AbsMouse5_::AbsMouse5_(uint8_t reportId) : _reportId(reportId), _buttons(0), _x(0), _y(0), _autoReport(true)
//AbsMouse5_::AbsMouse5_() : _buttons(0), _x(0), _y(0)
AbsMouse5_::AbsMouse5_()
{
/*
#if defined(_USING_HID)
	static HIDSubDescriptor descriptorNode(HID_REPORT_DESCRIPTOR5, sizeof(HID_REPORT_DESCRIPTOR5));
	HID().AppendDescriptor(&descriptorNode);
#endif // _USING_HID
*/
}

/*
void AbsMouse5_::init(bool autoReport)
{
	_autoReport = autoReport;
}
*/
#ifdef COMMENTO
void AbsMouse5_::report_absmouse5(void)
{
	#ifdef COMMENDO
  uint8_t buffer[5];
	buffer[0] = _buttons;
	buffer[1] = _x & 0xFF;
	buffer[2] = (_x >> 8) & 0xFF;
	buffer[3] = _y & 0xFF;
	buffer[4] = (_y >> 8) & 0xFF;

/// Standard HID Boot Protocol Mouse Report.
typedef struct TU_ATTR_PACKED
{
  uint8_t buttons; /**< buttons mask for currently pressed buttons in the mouse. */
  int8_t  x;       /**< Current delta x movement of the mouse. */
  int8_t  y;       /**< Current delta y movement on the mouse. */
  int8_t  wheel;   /**< Current delta wheel movement on the mouse. */
  int8_t  pan;     // using AC Pan
} hid_mouse_report_t;

typedef struct TU_ATTR_PACKED
{
    uint8_t buttons; /**< buttons mask for currently pressed buttons in the mouse. */
    int16_t x;       /**< Current x position of the mouse. */
    int16_t y;       /**< Current y position of the mouse. */
    int8_t wheel;    /**< Current delta wheel movement on the mouse. */
    int8_t pan;      // using AC Pan
} hid_abs_mouse_report_t;
#endif // COMMENTO

hid_abs_mouse_report_t report_aux;

memcpy(&report_aux, &absmouse5Report, sizeof(absmouse5Report)); //VALUTARE SE TENERLO


//buffer.buttons = _buttons;
//buffer.x = _x;
//buffer.y = _y;
//buffer.wheel = _wheel;
//buffer.pan = _pan;

/*
#if defined(_USING_HID)
	HID().SendReport(_reportId, buffer, 5);
#endif // _USING_HID
*/
#if defined(USE_TINYUSB)
    #if defined(ENABLE_BLUETOOTH_OPENFIRE)
    if(TinyUSBDevices.onBattery) {
      
      
      BT_OpenFIRE.send(HID_BT_MOUSE, &absmouse5Report, sizeof(absmouse5Report));
    } else {
      while(!usbHid.ready()) yield();
      usbHid.sendReport(HID_RID_MOUSE, &absmouse5Report, sizeof(absmouse5Report));
    }
    #else
    while(!usbHid.ready()) yield();
    
    //usbHid.sendReport(HID_RID_MOUSE, &report_aux, sizeof(report_aux));
    usbHid.sendReport(HID_RID_MOUSE, &absmouse5Report, sizeof(absmouse5Report));
 
    #endif // ARDUINO_RASPBERRY_PI_PICO_W
#endif // USE_TINYUSB
}
#endif //COMMENTO

void AbsMouse5_::report_absmouse5(void)
{

  uint8_t sendSize = 0;
  hid_abs_mouse_report_t report_aux;
  memcpy(&report_aux, &absmouse5Report, sizeof(absmouse5Report)); //VALUTARE SE TENERLO

  if(!TinyUSBDevices.onBattery) {
    while(!usbHid.ready()) yield();  
    usbHid.sendReport(HID_RID_e::HID_RID_MOUSE, &report_aux, sizeof(report_aux));
    //usbHid.sendReport(HID_RID_MOUSE, &absmouse5Report, sizeof(absmouse5Report));
  }
  #ifdef OPENFIRE_WIRELESS_ENABLE
    else {
    switch (TinyUSBDevices.wireless_mode)
    {
      #if defined(ENABLE_BLUETOOTH_OPENFIRE)
      case ENABLE_BLUETOOTH_TO_PC:
        /* code */
        //BT_OpenFIRE.send(HID_BT_MOUSE, &absmouse5Report, sizeof(absmouse5Report));
        BT_OpenFIRE.send(HID_BT_MOUSE, &report_aux, sizeof(report_aux));
        break;
      case ENABLE_BLUETOOTH_TO_DONGLE:
        /* code */
        break;
      #endif // ENABLE_BLUETOOTH_OPENFIRE
      #ifdef OPENFIRE_WIRELESS_DEVICE_ESPNOW
      case ENABLE_ESP_NOW_TO_DONGLE:  
        /* code */
        // use this variable to keep track of how many
        // bytes we're stuffing in the transmit buffer
        //uint16_t sendSize = 0;
        ///////////////////////////////////////// Stuff buffer with struct
        //sendSize = OpenFIRE_serialTransfer.txObj(testStruct, sendSize);
        ///////////////////////////////////////// Stuff buffer with array
        
        //int avaliable_num = ESP32_NOW_Serial_OpenFIRE.availableForWrite(); // di default 1000 byte di buffer (si può cambiare)

        sendSize = 0;
        if (SerialWireless.availableForWriteBin() > (sizeof(absmouse5Report) + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
          //sendSize = SerialWireless.Packet::txObj(&report_aux, sendSize, sizeof(report_aux));
          //sendSize = SerialWireless.packet.txObj(&report_aux, sendSize, sizeof(report_aux));
          //SerialWireless.Packet::sendData(sendSize,PACKET_TX::MOUSE_TX);
          //SerialWireless.packet.sendData(sendSize,PACKET_TX::MOUSE_TX);

          
          sendSize = SerialWireless.packet.txObj(&report_aux, sendSize,sizeof(report_aux));
          SerialWireless.packet.constructPacket(sizeof(report_aux), PACKET_TX::MOUSE_TX);
          //SerialWireless.packet.sendData(sendSize,PACKET_TX::GAMEPADE_TX);
          SerialWireless.writeBin(SerialWireless.packet.txBuff, sizeof(report_aux) + PREAMBLE_SIZE+POSTAMBLE_SIZE);
          SerialWireless.SendData(); // try_Send



        }
        
        /*
        // ===============================================================================================================
        if (ESP32_NOW_Serial_OpenFIRE.availableForWrite() > (sizeof(absmouse5Report) + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
          sendSize = OpenFIRE_serialTransfer.txObj(report_aux, sendSize);
          ///////////////////////////////////////// Send buffer
          OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::MOUSE_TX);
        }
        // ===============================================================================================================
        */

        
        break;
      #endif //OPENFIRE_WIRELESS_DEVICE_ESPNOW
      case ENABLE_WIFI_TO_DONGLE:
        /* code */
        break;
      case ENABLE_NRF_24L01PLUS_TO_DONGLE:
        /* code */
        break;
      default:
        break;
    }  
  }
  #endif //OPENFIRE_WIRELESS_ENABLE

}

void AbsMouse5_::move_wheel_pan(int8_t wheel, int8_t pan) {
  if(pan != absmouse5Report.pan || wheel != absmouse5Report.wheel) {
		absmouse5Report.wheel = wheel;
    absmouse5Report.pan = pan;
		//if(_autoReport) {
			report_absmouse5();
		//}
  }
}

void AbsMouse5_::move(int16_t x, int16_t y)
{
	if(x != absmouse5Report.x || y != absmouse5Report.y) {
		absmouse5Report.x = x;
		absmouse5Report.y = y;
		//if(_autoReport) {
			report_absmouse5();
		//}
	}
}

void AbsMouse5_::press(uint8_t button)
{
	absmouse5Report.buttons |= button;

	//if(_autoReport) {
		report_absmouse5();
	//}
}

void AbsMouse5_::release(uint8_t button)
{
	absmouse5Report.buttons &= ~button;

	//if(_autoReport) {
		report_absmouse5();
	//}
}

void AbsMouse5_::click(uint8_t b) {
  absmouse5Report.buttons = b;
  report_absmouse5();
  //move(_lastx, _lasty);
  absmouse5Report.buttons = 0;
  report_absmouse5();
  //move(_lastx, _lasty);
}

void AbsMouse5_::buttons(uint8_t b) {
  if (b != absmouse5Report.buttons) {
    absmouse5Report.buttons = b;
    report_absmouse5();
    //move(_lastx, _lasty);
  }
}

bool AbsMouse5_::isPressed(uint8_t b) {
  if ((b & absmouse5Report.buttons) > 0) {
    return true;
  }
  return false;
}

#ifdef USE_TINYUSB
// AbsMouse5 instance (this is hardcoded as the second definition, so weh)
//////////////////AbsMouse5_ AbsMouse5(2);
// AbsMouse5_ AbsMouse5(HID_RID_MOUSE);  // HID_RID_MOUSE =3 da tinyUSB_device.cpp
//AbsMouse5_ AbsMouse5(3);

AbsMouse5_ AbsMouse5;

#else
// AbsMouse5 instance
AbsMouse5_ AbsMouse5(1);
#endif

//AbsMouse5.init(true);


 /*****************************
 *   KEYBOARD SECTION
 *****************************/ 

  Keyboard_::Keyboard_(void) {
  }
  

  #ifdef COMMENTO
  void Keyboard_::report_keyboard(hid_keyboard_report_t* keys)
  {
    #if defined(ENABLE_BLUETOOTH_OPENFIRE)
    if(TinyUSBDevices.onBattery) {

/*     
bool tud_hid_n_keyboard_report(uint8_t instance, uint8_t report_id, uint8_t modifier, const uint8_t keycode[6]) {
  hid_keyboard_report_t report;
  report.modifier = modifier;
  report.reserved = 0;

  if (keycode) {
    memcpy(report.keycode, keycode, sizeof(report.keycode));
  } else {
    tu_memclr(report.keycode, 6);
  }

  return tud_hid_n_report(instance, report_id, &report, sizeof(report));
}
*/
      
      /*
      hid_keyboard_report_t report;    // KeyReport report;
      report.modifier = keys->modifiers;
      report.reserved = 0;
      if (keys->keys) {
        memcpy(report.keycode, keys->keys, sizeof(report.keycode));
      } else {
        memset((report.keycode), 0, (6));
        //tu_memclr(report.keycode, 6);
      }
      
      BT_OpenFIRE.send(HID_BT_KEYBOARD, &report, sizeof(report)); //keys, sizeof(keys));
      */
      BT_OpenFIRE.send(HID_BT_KEYBOARD, keys, sizeof(keys));
    } else {
      if ( TinyUSBDevice.suspended() )  {
        TinyUSBDevice.remoteWakeup();
      }
      while(!usbHid.ready()) yield();
      usbHid.keyboardReport(HID_RID_KEYBOARD, keys->modifier, keys->keycode);
    }
    #else
    if ( TinyUSBDevice.suspended() )  {
      TinyUSBDevice.remoteWakeup();
    }
    while(!usbHid.ready()) yield();
    
    
    //hid_keyboard_report_t report_aux;
  
    //memcpy(&report_aux, &_keyReport, sizeof(_keyReport));
  


    //usbHid.sendReport(HID_RID_KEYBOARD,&_keyReport, sizeof(_keyReport)); // sono la stessa cosa
    usbHid.keyboardReport(HID_RID_KEYBOARD, keys->modifier, keys->keycode); // sono la stessa cosa
    #endif // ARDUINO_RASPBERRY_PI_PICO_W
  }
#endif // COMMENTO

void Keyboard_::report_keyboard(void) 
{
  uint8_t sendSize = 0;
  hid_keyboard_report_t report_aux; //VALUTARE SE USARE UNA COPIA PER EVENTUALE PROBLEMI CON MULTITASKING  
  memcpy(&report_aux, &_keyReport, sizeof(_keyReport)); //VALUTARE SE USARE UNA COPIA PER EVENTUALE PROBLEMI CON MULTITASKING
   
  if(!TinyUSBDevices.onBattery) {
    if (TinyUSBDevice.suspended())  { TinyUSBDevice.remoteWakeup(); }
    while(!usbHid.ready()) yield();   
    //usbHid.sendReport(HID_RID_KEYBOARD,&_keyReport, sizeof(_keyReport));
    usbHid.sendReport(HID_RID_e::HID_RID_KEYBOARD,&report_aux, sizeof(report_aux));
  }
  #ifdef OPENFIRE_WIRELESS_ENABLE
  else {
    switch (TinyUSBDevices.wireless_mode)
    {
      #if defined(ENABLE_BLUETOOTH_OPENFIRE)
      case ENABLE_BLUETOOTH_TO_PC:
        /* code */
        //BT_OpenFIRE.send(HID_BT_KEYBOARD, &_keyReport, sizeof(_keyReport));
        BT_OpenFIRE.send(HID_BT_KEYBOARD, &report_aux, sizeof(report_aux));
        break;
      case ENABLE_BLUETOOTH_TO_DONGLE:
        /* code */
        break;
      #endif // ENABLE_BLUETOOTH_OPENFIRE
      #ifdef OPENFIRE_WIRELESS_DEVICE_ESPNOW
      case ENABLE_ESP_NOW_TO_DONGLE:
        /* code */
        //uint16_t sendSize = 0;
        ///////////////////////////////////////// Stuff buffer with struct
        //sendSize = OpenFIRE_serialTransfer.txObj(testStruct, sendSize);
        ///////////////////////////////////////// Stuff buffer with array
        
        sendSize = 0;
        if (SerialWireless.availableForWriteBin() > (sizeof(_keyReport) + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
          //sendSize = SerialWireless.packet.txObj(&report_aux, sendSize, sizeof(report_aux));
          
          //SerialWireless.packet.sendData(sendSize,PACKET_TX::KEYBOARD_TX);
        
          sendSize = SerialWireless.packet.txObj(&report_aux, sendSize,sizeof(report_aux));
          SerialWireless.packet.constructPacket(sizeof(report_aux), PACKET_TX::KEYBOARD_TX);
          //SerialWireless.packet.sendData(sendSize,PACKET_TX::GAMEPADE_TX);
          SerialWireless.writeBin(SerialWireless.packet.txBuff, sizeof(report_aux) + PREAMBLE_SIZE+POSTAMBLE_SIZE);
          SerialWireless.SendData(); // try_Send

        
        
        
        
        }

        /*
        // ===============================================================================================================
        if (ESP32_NOW_Serial_OpenFIRE.availableForWrite() > (sizeof(_keyReport) + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
          sendSize = OpenFIRE_serialTransfer.txObj(report_aux, sendSize);
          ///////////////////////////////////////// Send buffer
          OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::KEYBOARD_TX);
        }
        // ===============================================================================================================
        */
        
        break;
        #endif //OPENFIRE_WIRELESS_DEVICE_ESPNOW
      case ENABLE_WIFI_TO_DONGLE:
        /* code */
        break;
      case ENABLE_NRF_24L01PLUS_TO_DONGLE:
        /* code */
        break;
      default:
        break;
    }  
  }
  #endif //OPENFIRE_WIRELESS_ENABLE

}

  /*
  #define SHIFT 0x80
  const uint8_t _asciimap[128] =
  {
    0x00,             // NUL
    0x00,             // SOH
    0x00,             // STX
    0x00,             // ETX
    0x00,             // EOT
    0x00,             // ENQ
    0x00,             // ACK  
    0x00,             // BEL
    0x2a,     // BS Backspace
    0x2b,     // TAB  Tab
    0x28,     // LF Enter
    0x00,             // VT 
    0x00,             // FF 
    0x00,             // CR 
    0x00,             // SO 
    0x00,             // SI 
    0x00,             // DEL
    0x00,             // DC1
    0x00,             // DC2
    0x00,             // DC3
    0x00,             // DC4
    0x00,             // NAK
    0x00,             // SYN
    0x00,             // ETB
    0x00,             // CAN
    0x00,             // EM 
    0x00,             // SUB
    0x00,             // ESC
    0x00,             // FS 
    0x00,             // GS 
    0x00,             // RS 
    0x00,             // US 
  
    0x2c,      //  ' '
    0x1e|SHIFT,    // !
    0x34|SHIFT,    // "
    0x20|SHIFT,    // #
    0x21|SHIFT,    // $
    0x22|SHIFT,    // %
    0x24|SHIFT,    // &
    0x34,          // '
    0x26|SHIFT,    // (
    0x27|SHIFT,    // )
    0x25|SHIFT,    // *
    0x2e|SHIFT,    // +
    0x36,          // ,
    0x2d,          // -
    0x37,          // .
    0x38,          // /
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
    0x33|SHIFT,      // :
    0x33,          // ;
    0x36|SHIFT,      // <
    0x2e,          // =
    0x37|SHIFT,      // >
    0x38|SHIFT,      // ?
    0x1f|SHIFT,      // @
    0x04|SHIFT,      // A
    0x05|SHIFT,      // B
    0x06|SHIFT,      // C
    0x07|SHIFT,      // D
    0x08|SHIFT,      // E
    0x09|SHIFT,      // F
    0x0a|SHIFT,      // G
    0x0b|SHIFT,      // H
    0x0c|SHIFT,      // I
    0x0d|SHIFT,      // J
    0x0e|SHIFT,      // K
    0x0f|SHIFT,      // L
    0x10|SHIFT,      // M
    0x11|SHIFT,      // N
    0x12|SHIFT,      // O
    0x13|SHIFT,      // P
    0x14|SHIFT,      // Q
    0x15|SHIFT,      // R
    0x16|SHIFT,      // S
    0x17|SHIFT,      // T
    0x18|SHIFT,      // U
    0x19|SHIFT,      // V
    0x1a|SHIFT,      // W
    0x1b|SHIFT,      // X
    0x1c|SHIFT,      // Y
    0x1d|SHIFT,      // Z
    0x2f,          // [
    0x31,          // bslash
    0x30,          // ]
    0x23|SHIFT,    // ^
    0x2d|SHIFT,    // _
    0x35,          // `
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
    0x2f|SHIFT,    // {
    0x31|SHIFT,    // |
    0x30|SHIFT,    // }
    0x35|SHIFT,    // ~
    0       // DEL
  };
  */
  
  ////////////////////////////////uint8_t const conv_table[128][2] =  { HID_ASCII_TO_KEYCODE };
  //#include "Adafruit_USBD_HID.h"
  //extern uint8_t const _ascii2keycode[128][2];
  
  // press() adds the specified key (printing, non-printing, or modifier)
  // to the persistent key report and sends the report.  Because of the way 
  // USB HID works, the host acts like the key remains pressed until we 
  // call release(), releaseAll(), or otherwise clear the report and resend.
  size_t Keyboard_::press(uint8_t k)
  {  
    ////return usbHid.keyboardPress(HID_RID_KEYBOARD,(char)k);

	uint8_t i;
	if (k >= 136) {			// it's a non-printing key (not a modifier)
		k = k - 136;
	} else if (k >= 128) {	// it's a modifier key
		_keyReport.modifier |= (1<<(k-128));
		k = 0;
	} else {				// it's a printing key
		//k = pgm_read_byte(_asciimap + k);
		k = pgm_read_byte(KeyboardLayout_it_IT + k);
    //k = conv_table[k][1];
    if (!k) {
			setWriteError();
			return 0;
		}
		if ((k & ALT_GR) == ALT_GR) {
			_keyReport.modifier |= 0x40;   // AltGr = right Alt
			k &= 0x3F; // k &= ~ALT_GR;
		} else if ((k & SHIFT) == SHIFT) {
			_keyReport.modifier |= 0x02;	// the left shift modifier
			k &= 0x7F; // k &= ~SHIFT;
		}
		if (k == ISO_REPLACEMENT) {
			k = ISO_KEY;
		}
	}

	
  if (k >= 0xE0 && k < 0xE8) {
    // it's a modifier key
    _keyReport.modifier |= (1 << (k - 0xE0));
  } else if (k && k < 0xA5) {
  // Add k to the key report only if it's not already present
	// and if there is an empty slot.
	  if (_keyReport.keycode[0] != k && _keyReport.keycode[1] != k &&
		  _keyReport.keycode[2] != k && _keyReport.keycode[3] != k &&
		  _keyReport.keycode[4] != k && _keyReport.keycode[5] != k) {

		  for (i=0; i<6; i++) {
			  if (_keyReport.keycode[i] == 0x00) {
				  _keyReport.keycode[i] = k;
				  break;
			  }
		  }
		  if (i == 6) {
			  setWriteError();
			  return 0;
		  }
	  }
  }  else if (_keyReport.modifier == 0) {
    //not a modifier and not a key
    return 0;
  }


	report_keyboard();
  //sendReport(&_keyReport);
	return 1;



    #ifdef COMMENTO
    uint8_t i;
    if (k >= 136) {     // it's a non-printing key (not a modifier)
      k = k - 136;
    } else if (k >= 128) {  // it's a modifier key
      _keyReport.modifier |= (1<<(k-128));
      k = 0;
    } else {        // it's a printing key
         
    //k = pgm_read_byte(_asciimap + k);
     
    //uint8_t const conv_table[128][2] =  { HID_ASCII_TO_KEYCODE };
    k = conv_table[k][1];
    //uint8_t keycode[6] = { 0 };
    //uint8_t modifier   = 0;
    //if ( conv_table[k][0] ) modifier = KEYBOARD_MODIFIER_LEFTSHIFT;
    //keycode[0] = conv_table[k][1];
    //tud_hid_keyboard_report(report_id, modifier, keycode);


     /*
     uint8_t const conv_table[128][2] =  { HID_KEYCODE_TO_ASCII };
     bool shift = k & 0x80;
     char ch = shift ? conv_table[k][1] : conv_table[k][0];
    */


      if (!k) {
        setWriteError();
        return 0;
      }
      if (k & 0x80) {           // it's a capital letter or other character reached with shift
        _keyReport.modifier |= 0x02; // the left shift modifier
        k &= 0x7F;
      }
    }
    
    // Add k to the key report only if it's not already present
    // and if there is an empty slot.
    if (_keyReport.keycode[0] != k && _keyReport.keycode[1] != k && 
      _keyReport.keycode[2] != k && _keyReport.keycode[3] != k &&
      _keyReport.keycode[4] != k && _keyReport.keycode[5] != k) {
      
      for (i=0; i<6; i++) {
        if (_keyReport.keycode[i] == 0x00) {
          _keyReport.keycode[i] = k;
          break;
        }
      }
      if (i == 6) {
        setWriteError();
        return 0;
      } 
    }
    sendReport(&_keyReport);
    
    
    return 1;
    #endif //COMMENTO
  }
  
  // release() takes the specified key out of the persistent key report and
  // sends the report.  This tells the OS the key is no longer pressed and that
  // it shouldn't be repeated any more.
  size_t Keyboard_::release(uint8_t k)
  {

    /*
    memset(_keyReport.keycode, 0, 6);
    _keyReport.reserved = 0;
    _keyReport.modifier = 0;
    */
    //return usbHid.keyboardReport(HID_RID_KEYBOARD, 0, NULL);
    
    //////////////////////////////////////return usbHid.keyboardRelease(HID_RID_KEYBOARD);
    
    	uint8_t i;
	if (k >= 136) {			// it's a non-printing key (not a modifier)
		k = k - 136;
	} else if (k >= 128) {	// it's a modifier key
		_keyReport.modifier &= ~(1<<(k-128));
		k = 0;
	} else {				// it's a printing key
		//k = pgm_read_byte(_asciimap + k);
    k = pgm_read_byte(KeyboardLayout_it_IT + k);
    //k = conv_table[k][1];
		if (!k) {
			return 0;
		}
		if ((k & ALT_GR) == ALT_GR) {
			_keyReport.modifier &= ~(0x40);   // AltGr = right Alt
			k &= 0x3F; // k &= ~ALT_GR;
		} else if ((k & SHIFT) == SHIFT) {
			_keyReport.modifier &= ~(0x02);	// the left shift modifier
			k &= 0x7F; // k &= ~SHIFT;
		}
		if (k == ISO_REPLACEMENT) {
			k = ISO_KEY;
		}
	}

	if (k >= 0xE0 && k < 0xE8) {
    // it's a modifier key
    _keyReport.modifier &= ~(1 << (k - 0xE0));
  } else if (k && k < 0xA5) {
    // Test the key report to see if k is present.  Clear it if it exists.
    // Check all positions in case the key is present more than once (which it shouldn't be)  
    for (i=0; i<6; i++) {
		  if (0 != k && _keyReport.keycode[i] == k) {
			  _keyReport.keycode[i] = 0x00;
		  }
	  }
  }
  

	//sendReport(&_keyReport);
  report_keyboard();
	return 1;




    #ifdef COMMENTO
    uint8_t i;
    if (k >= 136) {     // it's a non-printing key (not a modifier)
      k = k - 136;
    } else if (k >= 128) {  // it's a modifier key
      _keyReport.modifier &= ~(1<<(k-128));
      k = 0;
    } else {        // it's a printing key
      
      //k = pgm_read_byte(_asciimap + k);
      k = conv_table[k][1];

      if (!k) {
        return 0;
      }
      if (k & 0x80) {             // it's a capital letter or other character reached with shift
        _keyReport.modifier &= ~(0x02);  // the left shift modifier
        k &= 0x7F;
      }
    }
    
    // Test the key report to see if k is present.  Clear it if it exists.
    // Check all positions in case the key is present more than once (which it shouldn't be)
    for (i=0; i<6; i++) {
      if (0 != k && _keyReport.keycode[i] == k) {
        _keyReport.keycode[i] = 0x00;
      }
    }
    sendReport(&_keyReport);
    return 1;
    #endif //COMMENTO
  }
  
  void Keyboard_::releaseAll(void)
  {
  //return usbHid.keyboardRelease(HID_RID_KEYBOARD);
  //usbHid.keyboardRelease(HID_RID_KEYBOARD);
  tu_memclr(&_keyReport.keycode, 6);
  _keyReport.modifier = 0;
  //_keyReport.reserved = 0;
  report_keyboard();
  #ifdef COMMENTO    
    _keyReport.keycode[0] = 0;
    _keyReport.keycode[1] = 0; 
    _keyReport.keycode[2] = 0;
    _keyReport.keycode[3] = 0; 
    _keyReport.keycode[4] = 0;
    _keyReport.keycode[5] = 0; 
    _keyReport.modifier = 0;
    sendReport(&_keyReport);
  #endif // COMMENTO
  }
  
  size_t Keyboard_::write(uint8_t c)
  {
    uint8_t p = press(c);  // Keydown
    //delay(10); // da arduino pico
    release(c);            // Keyup
    //delay(10); // da arduino pico
    return p;              // just return the result of press() since release() almost always returns 1
  }
  
  size_t Keyboard_::write(const uint8_t *buffer, size_t size) {
    size_t n = 0;
    while (size--) {
      if (*buffer != '\r') {
        if (write(*buffer)) {
          n++;
        } else {
          break;
        }
      }
      buffer++;
    }
    return n;
  }

  Keyboard_ Keyboard;//create an instance of the Keyboard object

/*****************************
 *   GAMEPAD SECTION 16
 *****************************/
  Gamepad16_::Gamepad16_(void) {
  }

  void Gamepad16_::moveCam(uint16_t origX, uint16_t origY) {
    if(stickRight) {
        gamepad16Report.x = map(origX, 0, 32767, -32767, 32767);
        gamepad16Report.y = map(origY, 0, 32767, -32767, 32767);
    } else {
        gamepad16Report.rx = map(origX, 0, 32767, -32767, 32767);
        gamepad16Report.ry = map(origY, 0, 32767, -32767, 32767);
    }
    //if(_autoReport) {
        report_gamepad16();
    //}
  }

  void Gamepad16_::moveStick(uint16_t origX, uint16_t origY) {
    // TODO: inverted output for Cabela's Top Shot Elite sticks, but it might be backwards for others.
    if(origX != _x || origY != _y) {
        _x = origX, _y = origY;
        if(stickRight) {
            gamepad16Report.rx = map(_x, 0, 4095, 32767, -32767);
            gamepad16Report.ry = map(_y, 0, 4095, 32767, -32767);
        } else {
            gamepad16Report.x = map(_x, 0, 4095, 32767, -32767);
            gamepad16Report.x = map(_y, 0, 4095, 32767, -32767);
        }
        //if(_autoReport) {
            report_gamepad16();
        //}
    }
  }

  void Gamepad16_::press(uint8_t buttonNum) {
    bitSet(gamepad16Report.buttons, buttonNum);
    //if(_autoReport) {
        report_gamepad16();
    //}
  }

  void Gamepad16_::release(uint8_t buttonNum) {
    bitClear(gamepad16Report.buttons, buttonNum);
    //if(_autoReport) {
        report_gamepad16();
    //}
  }

  void Gamepad16_::padUpdate(uint8_t padMask) {
    gamepad16Report.hat = padMask;
    //if(_autoReport) {
        report_gamepad16();
    //}
  }

  void Gamepad16_::releaseAll() {
    gamepad16Report.buttons = 0;
    gamepad16Report.hat = 0;
    gamepad16Report.x = 0;
    gamepad16Report.y = 0;
    gamepad16Report.rx = 0;
    gamepad16Report.ry = 0;
    report_gamepad16();
  }



#ifdef COMMENTO
  void Gamepad16_::report_gamepad16() {
    #if defined(ENABLE_BLUETOOTH_OPENFIRE)
    if(TinyUSBDevices.onBattery) {
      // this doesn't work for some reason :(
      //PicoBluetoothHID.send(2, &gamepad16Report, sizeof(gamepad16Report));
      BT_OpenFIRE.send(HID_BT_GAMEPAD, &gamepad16Report, sizeof(gamepad16Report));
    } else {
      if ( TinyUSBDevice.suspended() )  {
        TinyUSBDevice.remoteWakeup();
      }
      while(!usbHid.ready()) yield();
      usbHid.sendReport(HID_RID_GAMEPAD, &gamepad16Report, sizeof(gamepad16Report));
    }
    #else
    if ( TinyUSBDevice.suspended() )  {
      TinyUSBDevice.remoteWakeup();
    }
    while(!usbHid.ready()) yield();
    usbHid.sendReport(HID_RID_GAMEPAD, &gamepad16Report, sizeof(gamepad16Report));
    #endif // ARDUINO_RASPBERRY_PI_PICO_W
  }
#endif // COMMENTO


void Gamepad16_::report_gamepad16()
{
  uint8_t sendSize = 0;
  hid_gamepad16_report_t report_aux; //VALUTARE SE USARE UNA COPIA PER EVENTUALE PROBLEMI CON MULTITASKING  
  memcpy(&report_aux, &gamepad16Report, sizeof(gamepad16Report)); //VALUTARE SE USARE UNA COPIA PER EVENTUALE PROBLEMI CON MULTITASKING
   
  if(!TinyUSBDevices.onBattery) {
    if (TinyUSBDevice.suspended())  { TinyUSBDevice.remoteWakeup(); }
    while(!usbHid.ready()) yield();   
    //usbHid.sendReport(HID_RID_GAMEPAD,&gamepad16Report,sizeof(gamepad16Report));
    usbHid.sendReport(HID_RID_e::HID_RID_GAMEPAD,&report_aux, sizeof(report_aux));
  }
  #ifdef OPENFIRE_WIRELESS_ENABLE
  else {
    switch (TinyUSBDevices.wireless_mode)
    {
      #if defined(ENABLE_BLUETOOTH_OPENFIRE)
      case ENABLE_BLUETOOTH_TO_PC:
        /* code */
        //BT_OpenFIRE.send(HID_BT_GAMEPAD, &gamepad16Report, sizeof(gamepad16Report));
        BT_OpenFIRE.send(HID_BT_GAMEPAD, &report_aux, sizeof(report_aux));
        break;
      case ENABLE_BLUETOOTH_TO_DONGLE:
        /* code */
        break;
      #endif // ENABLE_BLUETOOTH_OPENFIRE
      #ifdef OPENFIRE_WIRELESS_DEVICE_ESPNOW
      case ENABLE_ESP_NOW_TO_DONGLE:
        /* code */
        //uint16_t sendSize = 0;
        ///////////////////////////////////////// Stuff buffer with struct
        //sendSize = OpenFIRE_serialTransfer.txObj(testStruct, sendSize);
        ///////////////////////////////////////// Stuff buffer with array
        
        sendSize = 0;
        if (SerialWireless.availableForWriteBin() > (sizeof(gamepad16Report) + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
          sendSize = SerialWireless.packet.txObj(&report_aux, sendSize,sizeof(report_aux));
          SerialWireless.packet.constructPacket(sizeof(report_aux), PACKET_TX::GAMEPADE_TX);
          //SerialWireless.packet.sendData(sendSize,PACKET_TX::GAMEPADE_TX);
          SerialWireless.writeBin(SerialWireless.packet.txBuff, sizeof(report_aux) + PREAMBLE_SIZE+POSTAMBLE_SIZE);
          SerialWireless.SendData(); // try_Send
        }
        
        /*
        // ===============================================================================================================
        if (ESP32_NOW_Serial_OpenFIRE.availableForWrite() > (sizeof(gamepad16Report) + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
          sendSize = OpenFIRE_serialTransfer.txObj(report_aux, sendSize);
          ///////////////////////////////////////// Send buffer
          OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::GAMEPADE_TX);
        }
        // ===============================================================================================================
        */
        
        break;
        #endif //OPENFIRE_WIRELESS_DEVICE_ESPNOW
      case ENABLE_WIFI_TO_DONGLE:
        /* code */
        break;
      case ENABLE_NRF_24L01PLUS_TO_DONGLE:
        /* code */
        break;
      default:
        break;
    }  
  }
  #endif // OPENFIRE_WIRELESS_ENABLE

}

  Gamepad16_ Gamepad16;

// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
#ifdef COMMENTO
/////////////////////////////////////////////////////////////////// Callbacks
static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len); // callback esp_now
static void _esp_now_tx_cb(const uint8_t *mac_addr, esp_now_send_status_t status); // callback esp_now

void packet_callback_read_dongle(); // callback packet 
void packet_callback_read_gun(); // callback packet

#if defined(DONGLE)
  const uint8_t peerAddress[6] = {0x24, 0x58, 0x7C, 0xDA, 0x38, 0xA0}; // quello montato con piedini su breackboard

  const functionPtr callbackArr[] = { packet_callback_read_dongle };
#elif defined(GUN)
  //const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE8, 0x0F, 0xB8}; // espe32s3 con piedini (riceve ma non trasmette)
  const uint8_t peerAddress[6] = {0xA0, 0x85, 0xE3, 0xE7, 0x65, 0xD4}; // espe32s3 senz apiedini
  //const uint8_t peerAddress[6] = {0xF0, 0x9E, 0x9E, 0x28, 0x9E, 0xB8}; // DONGLE LILYGO

  const functionPtr callbackArr[] = { packet_callback_read_gun };
#endif
///////////////////////////////////////////////////////////////////

/*****************************
 *   SERIAL WIRELESS SECTION
 *****************************/

int SerialWireless_::peek() {
  /*
  if (OpenFIRE_serialTransfer.bytesRead) {
    return OpenFIRE_serialTransfer.packet.rxBuff[0];
    //return OpenFIRE_serialTransfer.packet.rxBuff[OpenFIRE_serialTransfer.bytesRead - 1];
  }
  return -1;
  */
 if (lenBufferSerialRead) {
  return bufferSerialRead[_readerSerialRead];
}
return -1;

}

int SerialWireless_::peekBin() {
  /*
  if (OpenFIRE_serialTransfer.bytesRead) {
    return OpenFIRE_serialTransfer.packet.rxBuff[0];
    //return OpenFIRE_serialTransfer.packet.rxBuff[OpenFIRE_serialTransfer.bytesRead - 1];
  }
  return -1;
  */
 if (_readLen) {
  return _queue[_reader];
}
return -1;

}

int SerialWireless_::read() {
  //if (_readLen_atomic.load()) {
    if (lenBufferSerialRead) {
      uint8_t ret = bufferSerialRead[_readerSerialRead];
      //uint16_t aux_reader = _reader;
      //uint8_t ret = _queue[aux_reader];
      lenBufferSerialRead --;

      //asm volatile("" ::: "memory"); // Ensure the value is read before advancing
      // Aggiorna _reader senza l'uso dell'operatore %
      //uint16_t next_reader = (_reader + 1);
      _readerSerialRead ++;
      if (_readerSerialRead >= FIFO_SIZE_READ_SERIAL) {
        _readerSerialRead -= FIFO_SIZE_READ_SERIAL;
      }
      //asm volatile("" ::: "memory"); // Ensure the reader value is only written once, correctly
      //_reader = next_reader;
      
      //_reader = aux_reader;
      //_readLen --;
      //_readLen_atomic.fetch_sub(1);
      return ret;
  }
return -1;  

}


// ===============================================================================================================
#ifdef COMMENDO
  #define PACKET_CONTINUE             3
  #define PACKET_NEW_DATA             2
  #define PACKET_NO_DATA              1 
  #define PACKET_CRC_ERROR            0
  #define PACKET_PAYLOAD_ERROR       -1
  #define PACKET_STOP_BYTE_ERROR     -2
  #define PACKET_STALE_PACKET_ERROR  -3
#endif // COMMENTO

bool SerialWireless_::checkForRxPacket() {

// AGGIUNGERE UN TIMEOUT e USCIRE QUANDO LETTO TUTTO UN PACCHETTO  - decidere altrimenti se usare la callback
//bool packetComplete = false;
uint16_t numAvailableBin = availableBin();
uint8_t dato;
for (uint16_t i = 0; i<numAvailableBin; i++)
//for (uint16_t i = 0; i<numAvailableBin && !packetComplete; i++)
//while (availableBin() && !packetComplete) 
{
  //uint8_t dato;
  /*
  dato = ESP32_NOW_Serial_OpenFIRE.read();
  ESP32_NOW_Serial_OpenFIRE.read(&dato, 1);
  */
  dato = (uint8_t) readBin();
  packet.parse(dato, true);
  //if (dato==START_BYTE) Serial.println(" === START_BYTE ===");

#ifdef COMMENTO

switch (packet.status)
{
case PACKET_CONTINUE:
  /* code */
  //Serial.println("PACKET_CONTINUE");
  break;
case PACKET_NEW_DATA :
  // =======================================================================
  switch (packet.currentPacketID())
  {
  case PACKET_TX::SERIAL_TX:
    /* code */
    write_on_rx_serialBuffer(&packet.rxBuff[PREAMBLE_SIZE], packet.bytesRead);
    packetComplete = true;
    break;
  case PACKET_TX::MOUSE_TX :
    /* code */
    usbHid.sendReport(HID_RID_MOUSE, &packet.rxBuff[PREAMBLE_SIZE], packet.bytesRead);
    packetComplete = true;
    break;
  case PACKET_TX::GAMEPADE_TX:
    /* code */
    usbHid.sendReport(HID_RID_GAMEPAD, &packet.rxBuff[PREAMBLE_SIZE], packet.bytesRead);
    packetComplete = true;
    break;
  case PACKET_TX::KEYBOARD_TX:
    /* code */
    usbHid.sendReport(HID_RID_KEYBOARD, &packet.rxBuff[PREAMBLE_SIZE], packet.bytesRead);
    packetComplete = true;
    break; 
  default:
    break;
  }
  // ============================================================
  
  /* code */
  //Serial.println("PACKET_NEW_DATA");
  ///////////////////////////////////////////Serial.write(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE],SerialWireless.packet.bytesRead);
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
#endif // COMMENTO

}
  return true;
}

// =====================================================================================================

int SerialWireless_::readBin() {
  //if (_readLen_atomic.load()) {
    if (_readLen) {
        uint8_t ret = _queue[_reader];
        //uint16_t aux_reader = _reader;
        //uint8_t ret = _queue[aux_reader];
        _readLen --;

        //asm volatile("" ::: "memory"); // Ensure the value is read before advancing
        // Aggiorna _reader senza l'uso dell'operatore %
        //uint16_t next_reader = (_reader + 1);
        _reader ++;
        if (_reader >= FIFO_SIZE_READ) {
            _reader -= FIFO_SIZE_READ;
        }
        //asm volatile("" ::: "memory"); // Ensure the reader value is only written once, correctly
        //_reader = next_reader;
        
        //_reader = aux_reader;
        //_readLen --;
        //_readLen_atomic.fetch_sub(1);
        return ret;
    }
  return -1;  

  /*
  uint8_t _queue[FIFO_SIZE_READ]; //poi cambiare


  if (OpenFIRE_serialTransfer.currentPacketID() == PACKET_TX::SERIAL_TX) {
    uint16_t recSize = 0;

    recSize = OpenFIRE_serialTransfer.rxObj(_queue, recSize, OpenFIRE_serialTransfer.bytesRead);
  
    //OpenFIRE_serialTransfer.bytesRead;
    //OpenFIRE_serialTransfer.packet.rxBuff[i];
  }
  // dai un carattere
  return -1;
  */
}

int SerialWireless_::available() {
  return lenBufferSerialRead;
}

int SerialWireless_::availableBin() {
  return _readLen;
  /*
  return OpenFIRE_serialTransfer.available();
  //return _readLen;
*/
#ifdef COMMENTO  
if (OpenFIRE_serialTransfer.available())
  {
    if (OpenFIRE_serialTransfer.currentPacketID() == PACKET_TX::SERIAL_TX) {
      uint16_t recSize = 0;
      uint16_t size;
      size = OpenFIRE_serialTransfer.bytesRead;
      //recSize = OpenFIRE_serialTransfer.rxObj(_queue, recSize, OpenFIRE_serialTransfer.bytesRead);

      // =====================================================================================================
      // =====================================================================================================
      //if ((FIFO_SIZE_READ - _readLen_atomic.load()) >= size) {
        if ((FIFO_SIZE_READ - _readLen) >= size) {
            size_t firstChunk = FIFO_SIZE_READ - _writer;
            //size_t firstChunk = FIFO_SIZE_READ - _writer_atomic.load();
            if (firstChunk > size) {
                firstChunk = size;
            }
            // Copia il primo blocco di dati nel buffer circolare
            memcpy(&_queue[_writer], OpenFIRE_serialTransfer.packet.rxBuff, firstChunk);
            //memcpy(&_queue[_writer_atomic.load()], packet, firstChunk);
            //asm volatile("" ::: "memory");
            //_writer += firstChunk;
            _writer += firstChunk;
            //asm volatile("" ::: "memory");
            if (_writer == FIFO_SIZE_READ) {
                //asm volatile("" ::: "memory");
                _writer = 0;
                //asm volatile("" ::: "memory");
            }
            uint16_t remaining = size - firstChunk;
            if (remaining > 0) {
                memcpy(&_queue[_writer], OpenFIRE_serialTransfer.packet.rxBuff + firstChunk, remaining);
                //asm volatile("" ::: "memory");
                _writer += remaining;
                //asm volatile("" ::: "memory");
                /*
                if (_writer == _fifoSize) {
                    _writer = 0;
                }
                */
            }
            //asm volatile("" ::: "memory");
            _readLen += size;
            //_readLen_atomic.fetch_add(size);
            //asm volatile("" ::: "memory"); // Assicura che la coda venga scritta prima di avanzare il conteggio scritt
        } else {
            _overflow = true;       
        }
        //mutex_exit(&_mutex_read);       
    //}


      // =====================================================================================================
      // =====================================================================================================





    }
  }
  return _readLen;
  #endif //COMMENTO
}

int SerialWireless_::availableForWrite() {
  return FIFO_SIZE_WRITE_SERIAL - lenBufferSerialWrite;
  
  //return (ESP32_NOW_Serial_OpenFIRE.availableForWrite() - PREAMBLE_SIZE - POSTAMBLE_SIZE);
}

int SerialWireless_::availableForWriteBin() {
  return BUFFER_SIZE - _writeLen;
  
  //return (ESP32_NOW_Serial_OpenFIRE.availableForWrite() - PREAMBLE_SIZE - POSTAMBLE_SIZE);
}

void SerialWireless_::flush() {
  uint8_t sendSize = 0;
  if (availableForWriteBin() > (lenBufferSerialWrite + PREAMBLE_SIZE + POSTAMBLE_SIZE)) {
    sendSize = packet.txObj(bufferSerialWrite, sendSize, lenBufferSerialWrite);
    packet.constructPacket(lenBufferSerialWrite, PACKET_TX::SERIAL_TX);
    //SerialWireless.packet.sendData(sendSize,PACKET_TX::GAMEPADE_TX);
    writeBin(SerialWireless.packet.txBuff, lenBufferSerialWrite + PREAMBLE_SIZE+POSTAMBLE_SIZE);
    lenBufferSerialWrite = 0;
  }
  
  
  SendData(); // try a send
  /*
  OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
  sendSize = 0;
  */
}

void SerialWireless_::flushBin() {
  SendData();
  /*
  OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
  sendSize = 0;
  */
}


void SerialWireless_::SendPacket() {

}

void SerialWireless_::SendData() {
  uint8_t buffer_espnow[ESP_NOW_MAX_DATA_LEN];
  
  uint16_t dataAvailable = _writeLen;
  if (dataAvailable > 0)
  {
    uint16_t len_tx = dataAvailable > ESP_NOW_MAX_DATA_LEN ? ESP_NOW_MAX_DATA_LEN : dataAvailable;
  
    // Copia i dati nel buffer di uscita
    //uint16_t bytesToSendEnd = min(len_tx, BUFFER_SIZE - readIndex);
    uint16_t bytesToSendEnd = len_tx > (BUFFER_SIZE - readIndex) ? BUFFER_SIZE - readIndex : len_tx;
    memcpy(buffer_espnow, &buffer[readIndex], bytesToSendEnd);
    if (len_tx > bytesToSendEnd) {
      memcpy(&buffer_espnow[bytesToSendEnd], &buffer[0], len_tx - bytesToSendEnd);
    }

    if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    esp_err_t result = esp_now_send(peerAddress, buffer_espnow, len_tx);
    if (result == ESP_OK) {
    
      readIndex += len_tx; 
      if (readIndex >= BUFFER_SIZE) { readIndex -= BUFFER_SIZE; }
      _writeLen -= len_tx;  
    
      /////////////Serial.println("INVIATO CORRETTAMENTE da esp_now_send");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      Serial.println("ESPNOW not init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("Our of memory");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else if (result == ESP_ERR_ESPNOW_IF) {
      Serial.println("Interface does not match.");
    } else Serial.println("Errore sconosciuto");
    }
  }

/*
  OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
  sendSize = 0;
*/

}

size_t SerialWireless_::write(uint8_t c) {
  //sendSize = OpenFIRE_serialTransfer.txObj(c, sendSize);
  return write(&c, 1);
}


size_t SerialWireless_::writeBin(uint8_t c) {
  //sendSize = OpenFIRE_serialTransfer.txObj(c, sendSize);
  return writeBin(&c, 1);
}


size_t SerialWireless_::write(const uint8_t *data, size_t len) { 
  /*
  #define FIFO_SIZE_WRITE_SERIAL 200
  #define TIME_OUT_SERIAL_WRITE 3
  uint8_t bufferSerialWrite[FIFO_SIZE_WRITE_SERIAL];
  unsigned long startTimeSerialWrite = 0; // = millis();
  volatile uint16_t lenBufferSerialWrite = 0;
  */
  
  /////////////////////////uint8_t *_queue;

    //bool inizializzato = false;
    //uint16_t sendSize = 0;
    //int rx_avalaible = 0;
    //unsigned long startTimeSerialWrite = 0; // = millis();
    //volatile uint16_t lenBufferSerialWrite = 0;
  
  //#define TIME_OUT_SERIAL_WRITE 3

  #ifdef COMMENTO
  // =====================
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
    sendSize = 0;

    //sendSize = OpenFIRE_serialTransfer.txObj(_queue, sendSize, rx_avalaible);
    //OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
    rx_avalaible = 0;
  } 



  // =====================
#endif // COMMENTO

memcpy(&bufferSerialWrite[lenBufferSerialWrite], data, len);
lenBufferSerialWrite += len;
flush();

#ifdef COMMENTO
// =======================
  if (lenBufferSerialWrite == 0)
  {
     startTimeSerialWrite = millis();
  } 
  
  if ((lenBufferSerialWrite + len) > FIFO_SIZE_WRITE_SERIAL) {
    // invia tutto
    flush();
    return 0;
  }
  else
  {
    memcpy(&bufferSerialWrite[lenBufferSerialWrite], data, len);
    lenBufferSerialWrite += len;
    if ((data[len-1] == '\n') || (millis() > (startTimeSerialWrite + TIME_OUT_SERIAL_WRITE))) {
      // invia tutto
      flush();
      //return len;
    }
    //startTimeSerialWrite = millis();  ========== VALUTARE ==========
  }
// ==========================
#endif //COMMENTO
  
return len;

  /*  
  if (millis() > (startTimeSerialWrite + TIME_OUT_SERIAL_WRITE))
  {
    //invia tutto
    flush();
    return len;

  }
  */

 /* 
  if (millis() > (startTimeSerialWrite + TIME_OUT_SERIAL_WRITE)) {
    rgbLedWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);  // Blue
    //rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);  // Red
    //rgbLedWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);  // Green
    
    Serial.println("Pacchetto SERIALE in arrivo da PC");
    
    if (rx_avalaible > FIFO_SIZE_READ_SERIAL) rx_avalaible= FIFO_SIZE_READ_SERIAL;

    Serial.readBytes(bufferSerial, rx_avalaible);
    bufferSerial[rx_avalaible]='\0';
    Serial.println((char *)bufferSerial);
    sendSize = 0;

    //sendSize = OpenFIRE_serialTransfer.txObj(_queue, sendSize, rx_avalaible);
    //OpenFIRE_serialTransfer.sendData(sendSize,PACKET_TX::SERIAL_TX);
    rx_avalaible = 0;
  } 
 */

  //return len;
}

size_t SerialWireless_::writeBin(const uint8_t *data, size_t len) {
  // ==============================================================================
  // COPIA I DATI IN UN BUFFER CIRCOLARE DA '*data' per 'len' lunghezza in byte
  if ((BUFFER_SIZE - _writeLen) >= len) {
    size_t firstChunk = BUFFER_SIZE - writeIndex;
    if (firstChunk < len) {
        memcpy(&buffer[writeIndex], data, firstChunk);
        writeIndex = len - firstChunk;  
        memcpy(&buffer[0], data + firstChunk, writeIndex);
    }
    else {
      memcpy(&buffer[writeIndex], data, len);
      writeIndex += len;
      if (writeIndex == BUFFER_SIZE) writeIndex = 0;
    }
    _writeLen += len;
    return len;
    }
  else {
    _overflow_write = true;
    Serial.println("Overflow nello scrivere nel BUFFER scrittura");
    return 0;
  }
  
  // ==========================================================================================
  /*  
  sendSize = OpenFIRE_serialTransfer.txObj(p, sendSize, len);
  if (p[len-1] == '\n') flush();
  return len; 
  */
}

// ===== INIZIO SCRITTURA ================

void SerialWireless_::SendPacketKeyboard() {

  // Now trigger the ISR to read data from the ring buffer.
  if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    checkForTxData();
  }
  //Prova a trasmettere
}
void SerialWireless_::SendPacketMouse() {

  // Now trigger the ISR to read data from the ring buffer.
  if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    checkForTxData();
  }
  //Prova a trasmettere
}
void SerialWireless_::SendPacketGamepad() {

  // Now trigger the ISR to read data from the ring buffer.
  if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    checkForTxData();
  }
  //Prova a trasmettere
}
void SerialWireless_::SendPacketSerial() {
  
  // Now trigger the ISR to read data from the ring buffer.
  if (xSemaphoreTake(tx_sem, 0) == pdTRUE) {
    checkForTxData();
  }
  //Prova a trasmettere
}
// ====== FINE SCRITTURA ==============

// ====== LETTURA ==============

void SerialWireless_::ReadPacketKeyboard() {

}
void SerialWireless_::ReadPacketMouse() {

}
void SerialWireless_::ReadPacketGamepad() {

}
void SerialWireless_::ReadPacketSerial() {

}
// ===== FINE LETTURA =============

// ===== INIZIO UTILITA' ===========

bool SerialWireless_::checkForTxData() {
/*
  //do we have something that failed the last time?
  resend_count = 0;
  if (queued_buff == NULL) {
    queued_buff = (uint8_t *)xRingbufferReceiveUpTo(tx_ring_buf, &queued_size, 0, ESP_NOW_MAX_DATA_LEN);
  } else {
    log_d(MACSTR " : PREVIOUS", MAC2STR(addr()));
  }
  if (queued_buff != NULL) {
    return tryToSend() > 0;
  }
  //log_d(MACSTR ": EMPTY", MAC2STR(addr()));
  xSemaphoreGive(tx_sem);
  return false;
*/
return tryToSend() > 0;
}

size_t SerialWireless_::tryToSend() {
  /*
  //log_d(MACSTR ": %u", MAC2STR(addr()), queued_size);
  size_t sent = send(queued_buff, queued_size);
  if (!sent) {
    //_onSent will not be called anymore
    //the data is lost in this case
    vRingbufferReturnItem(tx_ring_buf, queued_buff);
    queued_buff = NULL;
    xSemaphoreGive(tx_sem);
    end();
  }
  return sent;
*/
return 0;
}

// ===== FINE UTILITA' =============

void SerialWireless_::write_on_rx_serialBuffer(const uint8_t *data, int len) {
    
  // ==============================================================================
  // COPIA I DATI IN UN BUFFER CIRCOLARE DA '*data' per 'len' lunghezza in byte
  if ((FIFO_SIZE_READ_SERIAL - lenBufferSerialRead) >= len) {
    size_t firstChunk = FIFO_SIZE_READ_SERIAL - _writerSerialRead;
    if (firstChunk < len) {
        memcpy(&bufferSerialRead[_writerSerialRead], data, firstChunk);
        _writerSerialRead = len - firstChunk;  
        memcpy(&bufferSerialRead[0], data + firstChunk, _writerSerialRead);
    }
    else {
      memcpy(&bufferSerialRead[_writerSerialRead], data, len);
      _writerSerialRead += len;
      if (_writerSerialRead == FIFO_SIZE_READ_SERIAL) _writerSerialRead = 0;
    }
    lenBufferSerialRead += len;
    }
  else {
    _overflow_bufferSerialRead = true;
    Serial.println("Overflow nello scrivere nel BUFFER lettura del SERIAL BUFFER");
  }
  // ==========================================================================================
  
  // ORA BISOGNA ESAMINARE I DATI PRESENTI NEL BUFFER 
  
  }

int SerialWireless_::availablePacket() {

  return numAvailablePacket;

}

void SerialWireless_::begin() {

  #define ESPNOW_WIFI_CHANNEL 4
  // la potenza di trasmissione può andare da 8 a 84, dove 84 è il valore massimo che corrisponde a 20 db
  #define ESPNOW_WIFI_POWER 84 
  
  esp_now_peer_info_t peerInfo; // variabile di utilità per configurazione
  configST myConfig; // variabile di utilità per configurazione
  

  tx_sem = xSemaphoreCreateBinary();
  //xSemaphoreTake(tx_sem, 0);
  xSemaphoreGive(tx_sem);
  
  WiFi.mode(WIFI_STA);
  
  //WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  
  esp_err_t err = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_channel failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }
  
  err = esp_wifi_set_max_tx_power(ESPNOW_WIFI_POWER); // tra 8 e 84 corrispondenti a 2dbm a 20 dbm);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_max_tx_power failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }
  WiFi.disconnect();
  
  ///WiFi.begin();

  /* // quello buono
  while (!WiFi.STA.started()) {
    delay(100);
  }
  */
  
  //Serial.println("Wi-Fi parameters:");
  //Serial.println("  Mode: STA");
  //Serial.println("  MAC Address: " + WiFi.macAddress());
  //Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);



  #ifndef COMMENTO

  delay (1000);
  /*
  esp_err_t err = esp_wifi_start();
  if (err != ESP_OK) {
    Serial.printf("WiFi not started! 0x%x)", err);
  }
  */

  
  err = esp_now_init();
  if (err != ESP_OK) {
    Serial.printf("esp_now_init failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }

    //esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = ESPNOW_WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Errore nell'aggiunta del peer");
    }
  /* // IMPOSTARE SOLO PER CIFRATURA DATI
  if (pmk) {
    err = esp_now_set_pmk(pmk);
    if (err != ESP_OK) {
      log_e("esp_now_set_pmk failed! 0x%x", err);
      _esp_now_has_begun = false;
      return false;
    }
  }
  */

  err = esp_now_register_recv_cb(_esp_now_rx_cb);
  if (err != ESP_OK) {
    Serial.printf("esp_now_register_recv_cb failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }

  err = esp_now_register_send_cb(_esp_now_tx_cb);
  if (err != ESP_OK) {
    Serial.printf("esp_now_register_send_cb failed! 0x%x", err);
    //_esp_now_has_begun = false;
  }

#endif // COMMENTO

/*
if (!ESP32_NOW_Serial_OpenFIRE.begin(0)) {
    //Serial.println("FAILED a inizializzare ESP NOW SERIAL");
    delay (1000);
  }
  else {
    //Serial.println("ESP NOW Serial inizializzato regolarmente");
    delay (1000);
  }
*/
  ////////////////////////////////////////////////////OpenFIRE_SerialWireless = &ESP32_NOW_Serial_OpenFIRE;
  //OpenFIRE_serialTransfer.begin(OpenFIRE_SerialWireless);
  ////////////////////////OpenFIRE_serialTransfer.begin(ESP32_NOW_Serial_OpenFIRE);

  ///////////////////////////////////////////////////////////////// Config Parameters
  myConfig.port         = &Serial; // questo andrà tolta - rimasta solo per contabilità =========================================
  myConfig.debug        = true;
  myConfig.debugPort    = &Serial;
  myConfig.timeout      = DEFAULT_TIMEOUT; // 50ms
  myConfig.callbacks    = callbackArr;
  /////myConfig.callbacks    = NULL;
  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
  /////myConfig.callbacksLen = 0;
  /////////////////////////////////////////////////////////////////
  /*
  OpenFIRE_serialTransfer.begin(ESP32_NOW_Serial_OpenFIRE, myConfig);
  delay(500);
  */

  packet.begin(myConfig);

}

// ============================================================
// ============================================================


#ifdef COMMENTO

void serialTransfer_callback_read() {

  if (OpenFIRE_serialTransfer.currentPacketID() == PACKET_TX::SERIAL_TX) {
    uint16_t recSize = 0;
    uint16_t size;
    size = OpenFIRE_serialTransfer.bytesRead;
    //recSize = OpenFIRE_serialTransfer.rxObj(_queue, recSize, OpenFIRE_serialTransfer.bytesRead);

    // =====================================================================================================
    // =====================================================================================================
    //if ((FIFO_SIZE_READ - _readLen_atomic.load()) >= size) {
      if ((FIFO_SIZE_READ - SerialWireless._readLen) >= size) {
          size_t firstChunk = FIFO_SIZE_READ - SerialWireless._writer;
          //size_t firstChunk = FIFO_SIZE_READ - _writer_atomic.load();
          if (firstChunk > size) {
              firstChunk = size;
          }
          // Copia il primo blocco di dati nel buffer circolare
          memcpy(&SerialWireless._queue[SerialWireless._writer], OpenFIRE_serialTransfer.packet.rxBuff, firstChunk);
          //memcpy(&_queue[_writer_atomic.load()], packet, firstChunk);
          //asm volatile("" ::: "memory");
          //_writer += firstChunk;
          SerialWireless._writer += firstChunk;
          //asm volatile("" ::: "memory");
          if (SerialWireless._writer == FIFO_SIZE_READ) {
              //asm volatile("" ::: "memory");
              SerialWireless._writer = 0;
              //asm volatile("" ::: "memory");
          }
          uint16_t remaining = size - firstChunk;
          if (remaining > 0) {
              memcpy(&SerialWireless._queue[SerialWireless._writer], OpenFIRE_serialTransfer.packet.rxBuff + firstChunk, remaining);
              //asm volatile("" ::: "memory");
              SerialWireless._writer += remaining;
              //asm volatile("" ::: "memory");
              /*
              if (_writer == _fifoSize) {
                  _writer = 0;
              }
              */
          }
          //asm volatile("" ::: "memory");
          SerialWireless._readLen += size;
          //_readLen_atomic.fetch_add(size);
          //asm volatile("" ::: "memory"); // Assicura che la coda venga scritta prima di avanzare il conteggio scritt
      } else {
        SerialWireless._overflow_read = true;       
      }
      //mutex_exit(&_mutex_read);       
  //}


    // =====================================================================================================
    // =====================================================================================================





  }

  /*
  uint8_t _queue[FIFO_SIZE_READ]; //poi cambiare
  if (OpenFIRE_serialTransfer.currentPacketID() == PACKET_TX::SERIAL_TX) {
    uint16_t recSize = 0;

    recSize = OpenFIRE_serialTransfer.rxObj(_queue, recSize, OpenFIRE_serialTransfer.bytesRead);
    //OpenFIRE_serialTransfer.bytesRead;
    //OpenFIRE_serialTransfer.packet.rxBuff[i];
  }
  
  //Serial.println("hi");

  */
}


#endif // COMMENTO


void packet_callback_read_dongle() {
  ///Serial.println("DONGLE packet callback chiamata");
  switch (SerialWireless.packet.currentPacketID())
  {
  case PACKET_TX::SERIAL_TX:
    /* code */
    
    //VALUTARE SE USARE IL BUFFER O MENO E CONTROLLARE SERIAL.AVALIABLEFORWRITE
    //SerialWireless.write_on_rx_serialBuffer(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    Serial.write(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    // inviare dati seriale al PC .. il buffer forse neppure serve
    //controllare quandi dati puoi inviare al PC 'aviablele write' ed inviare solo quielli che si possono inviare, magari fare una ruotine
    // chiama routine per inviare pacchetti al PC

    break;
  case PACKET_TX::MOUSE_TX :
    /* code */
    usbHid.sendReport(HID_RID_MOUSE, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::GAMEPADE_TX:
    /* code */
    usbHid.sendReport(HID_RID_GAMEPAD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::KEYBOARD_TX:
    /* code */
    usbHid.sendReport(HID_RID_KEYBOARD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break; 
  default:
    break;
  }
}

void packet_callback_read_gun() {
  ///Serial.println("GUN packet callback chiamata");
  switch (SerialWireless.packet.currentPacketID())
  {
  case PACKET_TX::SERIAL_TX:
    /* code */
    ///Serial.println("packet callback Serial_TX");
    SerialWireless.write_on_rx_serialBuffer(&SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::MOUSE_TX :
    /* code */
    //usbHid.sendReport(HID_RID_MOUSE, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::GAMEPADE_TX:
    /* code */
    //usbHid.sendReport(HID_RID_GAMEPAD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break;
  case PACKET_TX::KEYBOARD_TX:
    /* code */
    //usbHid.sendReport(HID_RID_KEYBOARD, &SerialWireless.packet.rxBuff[PREAMBLE_SIZE], SerialWireless.packet.bytesRead);
    
    break; 
  default:
    break;
  }
}



static void _esp_now_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  ///////////////////Serial.printf("%s from " MACSTR ", data length : %u", "RICEVUTO", MAC2STR(info->src_addr), len);
  ///Serial.println("esp now rx callback chiamata");
  
  // ==============================================================================
  // COPIA I DATI IN UN BUFFER CIRCOLARE DA '*data' per 'len' lunghezza in byte
  if ((FIFO_SIZE_READ - SerialWireless._readLen) >= len) {
    size_t firstChunk = FIFO_SIZE_READ - SerialWireless._writer;
    if (firstChunk < len) {
        memcpy(&SerialWireless._queue[SerialWireless._writer], data, firstChunk);
        SerialWireless._writer = len - firstChunk;  
        memcpy(&SerialWireless._queue[0], data + firstChunk, SerialWireless._writer);
    }
    else {
      memcpy(&SerialWireless._queue[SerialWireless._writer], data, len);
      SerialWireless._writer += len;
      if (SerialWireless._writer == FIFO_SIZE_READ) SerialWireless._writer = 0;
    }
    SerialWireless._readLen += len;
    // SerialWireless.checkForRxPacket(); // ??????????????????????????????????????????????????

  }
  else {
    SerialWireless._overflow_read = true;
    Serial.println("Overflow nello scrivere nel BUFFER lettura");
  }
  // ==========================================================================================
  SerialWireless.checkForRxPacket();
  // ORA BISOGNA ESAMINARE I DATI PRESENTI NEL BUFFER 


  /*
  bool broadcast = memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, ESP_NOW_ETH_ALEN) == 0;
  log_v("%s from " MACSTR ", data length : %u", broadcast ? "Broadcast" : "Unicast", MAC2STR(info->src_addr), len);
  log_buf_v(data, len);
  if (!esp_now_is_peer_exist(info->src_addr) && new_cb != NULL) {
    log_v("Calling new_cb, peer not found.");
    new_cb(info, data, len, new_arg);
    return;
  }
  //find the peer and call it's callback
  for (uint8_t i = 0; i < ESP_NOW_MAX_TOTAL_PEER_NUM; i++) {
    if (_esp_now_peers[i] != NULL) {
      log_v("Checking peer " MACSTR, MAC2STR(_esp_now_peers[i]->addr()));
    }
    if (_esp_now_peers[i] != NULL && memcmp(info->src_addr, _esp_now_peers[i]->addr(), ESP_NOW_ETH_ALEN) == 0) {
      log_v("Calling onReceive");
      _esp_now_peers[i]->onReceive(data, len, broadcast);
      return;
    }
  }
*/
  }

static void _esp_now_tx_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //////////////Serial.printf(MACSTR " : STATUS : %s", MAC2STR(mac_addr), (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS\n" : "FAILED\n");
  // PRONTO PER TRASMETTERE ANCORA - GESTIRE TRAMITE UN SEMAFORO BINARIO (LIBERARE IL FILE BINARIO)
  xSemaphoreGive(SerialWireless.tx_sem);
  SerialWireless.SendData();
  
  /*
  log_v(MACSTR " : %s", MAC2STR(mac_addr), (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS" : "FAILED");
  //find the peer and call it's callback
  for (uint8_t i = 0; i < ESP_NOW_MAX_TOTAL_PEER_NUM; i++) {
    if (_esp_now_peers[i] != NULL && memcmp(mac_addr, _esp_now_peers[i]->addr(), ESP_NOW_ETH_ALEN) == 0) {
      _esp_now_peers[i]->onSent(status == ESP_NOW_SEND_SUCCESS);
      return;
    }
  }
  */
  }
#endif //COMMENTO
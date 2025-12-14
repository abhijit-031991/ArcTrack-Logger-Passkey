const char* tagID = "ArcTrack-Passkey-001";

#define TX     PA2          // LPUART1 TX
#define RX      PA3         // LPUART1 RX
#define DTR    PA4          // DTR OUT
#define BATSNS    PA7       // BATTERY SENSE
#define BLELED     PA15     // LED PIN
#define PWRLED    PA14      // POWER LED
#define SWITCH     PA1      // RESET LOGGER SWITCH


#define PING_SIZE 14
#define DATA_SIZE 16
#define SETTING_SIZE 27
#define REQ_SIZE 3

struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    float hdop;
    float x;
    float y;
    float z;
    unsigned int count;
    uint16_t id;
}__attribute__((__packed__));

struct settings{
    uint16_t tag;
    int gpsFrq;
    int gpsTout;
    int hdop;
}__attribute__((__packed__));

struct reqPing{
    uint16_t tag;
    byte request;
  }__attribute__((__packed__));
    
  struct calibrationData{
      float lat;
      float lng;
      float hdop;
      float bat;
      float signal;
      uint16_t tag;
      bool mqtt;
      bool gprs;
      bool network;
  }__attribute__((__packed__)); // Calibration Struct - Store GPS data during calibration.
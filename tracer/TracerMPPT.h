#ifndef __TRACER_MPPT_H__
#define __TRACER_MPPT_H__

#include <stdint.h>

//commands
const uint8_t CMD_READREALTIME = 0xA0;
const uint8_t CMD_MANUALCONTROL = 0xAA;

//communication
const uint8_t CONTROLLER_ID = 0x16;
const uint8_t PWL_START[] = {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55}; 
const uint8_t COMM_SYNC[] = {0xEB, 0x90, 0xEB, 0x90, 0xEB, 0x90};
const uint8_t DATA_END[] = { 0x7F };

#define ERR_LENGTH_UNEXPECTED 251;

struct ManualControlResult {
  bool isDataValid;
  int errorCode;
  bool isLoadOn;
};

struct TracerData {
  bool isDataValid;
  int errorCode;
  float batteryVoltage;
  float pvVoltage;
  float loadCurrent;
  float overDischarge;
  float batteryMax;
  bool isLoadOn;
  bool isOverload;
  bool isLoadShort;
  bool isBatteryOverload;
  bool isBatteryOverDischarge;
  bool isBatteryFull;
  bool isBatteryCharging;
  int batteryTemperature;
  float chargeCurrent;
};

class TracerMPPT {
  public: 
    TracerMPPT(int receivePin, int transmitPin);
    ~TracerMPPT();
    void begin();
    TracerData getRealtimeData();
    void setLoad(bool on);
    void setLoad(bool on, ManualControlResult* result);
    void getRealtimeData(TracerData* data);

  private:
    bool _isError(uint8_t n);
    void _send(const uint8_t cmd, uint8_t* args, uint8_t argLen, uint8_t* debugOutput);
    uint16_t _crc(uint8_t* s, uint8_t sLen);
    void _pack(uint16_t d, uint8_t result[2]);
    uint8_t _read(uint8_t expectedCmd, uint8_t* args, uint8_t* buf);
    void _unpack(char data[2], uint16_t* result);
    void _processRealTimeData(uint8_t* args, uint8_t cnt, TracerData* data);
    void _processManualControl(uint8_t* args, uint8_t cnt, ManualControlResult* result); 
};

#endif


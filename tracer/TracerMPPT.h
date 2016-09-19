#ifndef __TRACER_MPPT_H__
#define __TRACER_MPPT_H__

#include <stdint.h>

//commands
const uint8_t CMD_READREALTIME = 0xA0;
const uint8_t CMD_MANUALCONTROL = 0xAA;

//errors. This error strategy is effective iff all possible results are below the final value of errorMinIndex, which is the case with the current Tracer protocol AFAIK.
enum {
 ERROR_LENGTH_UNEXPECTED = 255,
 ERROR_INCORRECT_TERMINATOR = 254,// = errorMinIndex--;
 ERROR_MISSING_TERMINATOR = 253,// = errorMinIndex--;
 ERROR_CRC_MISMATCH = 252,// = errorMinIndex--;
 ERROR_MISSING_CRC2 = 251,// = errorMinIndex--;
 ERROR_MISSING_CRC1 = 250,// = errorMinIndex--;
 ERROR_INCOMPLETE_ARGS = 249,// = errorMinIndex--;
 ERROR_MISSING_ARGSLENGHT = 248,// = errorMinIndex--;
 ERROR_MISSING_COMMAND = 247,// = errorMinIndex--;
 ERROR_COMMAND_MISMATCH = 246,// = errorMinIndex--;
 ERROR_MISSING_ID = 245,// = errorMinIndex--;
 ERROR_MISSING_SYNC = 244,// = errorMinIndex--;
 ERROR_READ_TIMEOUT= 243,// = errorMinIndex--;
 ERROR_INDISTINGUISHABLE_FROM_ERROR= 242,// = errorMinIndex--; //If a result enters the error range, this will be returned instead of the result in order to avoid a false error code
 ERROR_SYNC_FAILIURE = 241,
 ERROR_MPPT_NOT_FOUND = 240,
 MIN_ERROR_VALUE = 240
};

//communication
const uint8_t CONTROLLER_ID = 0x16;
const uint8_t PWL_START[] = {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55}; 
const uint8_t COMM_SYNC[] = {0xEB, 0x90, 0xEB, 0x90, 0xEB, 0x90};
const uint8_t DATA_END[] = { 0x7F };

struct ResultValidity {
  bool isDataValid;
  int errorCode;
  char* errorString;
};

struct ManualControlResult {
  ResultValidity validity;
  bool isLoadOn;
};

struct TracerData {
  ResultValidity validity;
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
    char* _errorToString(uint8_t err);
};

#endif


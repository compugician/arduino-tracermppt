#include "TracerMPPT.h"
#include "SoftwareSerial.h"

#include "Arduino.h"

SoftwareSerial *mpptSerial = NULL; 

static int _bufPos = 0;
  
void resetBufPos() {
  _bufPos = 0;
}

int getBufPos() {
  return _bufPos;
}

void bufAdd(uint8_t* buf, const uint8_t* data, uint8_t dataLen) {
  for (int i=0; i<dataLen; i++) {
    buf[_bufPos++] = data[i];
  }
}

int bufToInt(uint8_t* b, int offset) {
  return b[offset];
}

bool bufToBool(uint8_t* b, int offset){
  return (bool)b[offset];
}

float bufToFloat(uint8_t* b, int offset){
  unsigned short full = b[offset+1] << 8 | b[offset];
  return full / 100.0;
}

TracerMPPT::TracerMPPT(int receivePin, int transmitPin) {
  mpptSerial = new SoftwareSerial(receivePin, transmitPin);
}

TracerMPPT::~TracerMPPT()
{
  delete mpptSerial;
}

void TracerMPPT::begin() {
  mpptSerial->begin(9600);
}

bool TracerMPPT::_isError(uint8_t n) {
  return (n>=240);
}

void TracerMPPT::_processRealTimeData(uint8_t* args, uint8_t cnt, TracerData* data) {
  
  if (_isError(cnt)) {
    data->isDataValid = false;
    data->errorCode = cnt; //TODO: give errors names, move to defines in h file, document limitations this error handling form imposes.
    return;
  }

  if (24!=cnt) {
    data->isDataValid = false;
    data->errorCode = ERR_LENGTH_UNEXPECTED;
  }

  data->isDataValid = true;
  data->errorCode = 0;

  data->batteryVoltage = bufToFloat(args,0);
  data->pvVoltage = bufToFloat(args,2);
  data->loadCurrent = bufToFloat(args,6);
  data->overDischarge = bufToFloat(args,8);
  data->batteryMax = bufToFloat(args,10);
  data->isLoadOn = bufToBool(args,12);
  data->isOverload = bufToBool(args,13);
  data->isLoadShort = bufToBool(args,14);
  data->isBatteryOverload = bufToBool(args,16);
  data->isBatteryOverDischarge = bufToBool(args,17);
  data->isBatteryFull = bufToBool(args,18);
  data->isBatteryCharging = bufToBool(args,19);
  data->batteryTemperature  = bufToInt(args,20); //TODO offset??!!
  data->chargeCurrent = bufToFloat(args,21);
 
}


void TracerMPPT::_processManualControl(uint8_t* args, uint8_t cnt, ManualControlResult* result) {
  
  if (_isError(cnt)) {
    result->isDataValid = false;
    result->errorCode = cnt; //TODO: give errors names, move to defines in h file, document limitations this error handling form imposes.
    return;
  }

  if (13!=cnt) {
    result->isDataValid = false;
    result->errorCode = ERR_LENGTH_UNEXPECTED;
  }

  result->isDataValid = true;
  result->errorCode = 0;

  result->isLoadOn = bufToBool(args,0); 
}

void TracerMPPT::getRealtimeData(TracerData* data) {
  uint8_t resArgs[50];
  _send(CMD_READREALTIME, NULL, 0, NULL);
  delay(50);
  uint8_t readCnt = _read(CMD_READREALTIME, resArgs, NULL);

  _processRealTimeData(resArgs,readCnt, data);
}


void TracerMPPT::setLoad(bool on) {
  setLoad(on,NULL);
}


void TracerMPPT::setLoad(bool on, ManualControlResult* result) {
  uint8_t args[1] = {0x01}; 
  if (!on) { 
    args[0] = {0x00};
  }
  _send(CMD_MANUALCONTROL, args, 1, NULL);
  delay(50);
  
  uint8_t resArgs[50];
  uint8_t readCnt = _read(CMD_MANUALCONTROL, resArgs, NULL);

  if (NULL!=result) {
    _processManualControl(resArgs, readCnt, result); 
  }
}


void TracerMPPT::_send(const uint8_t cmd, uint8_t* args, uint8_t argLen, uint8_t* debugOutput) {
  uint8_t bufSize = sizeof(PWL_START) + sizeof(COMM_SYNC) + (3 + argLen) + 2 + sizeof(DATA_END);
  uint8_t out[bufSize];

  resetBufPos();
  
  bufAdd(out,PWL_START,sizeof(PWL_START));
  bufAdd(out,COMM_SYNC,sizeof(COMM_SYNC));

  uint8_t dataLen = 3+argLen;
  uint8_t data[dataLen];
  
  data[0] = CONTROLLER_ID;
  data[1] = cmd;
  data[2] = argLen;
  for (int i=0; i<argLen; i++) {
    data[3+i] = args[i];
  }

  bufAdd(out, data, dataLen);

  //get CRC
  uint8_t checksum[2];
  _pack(_crc(data,dataLen),checksum);
  
  bufAdd(out, checksum, 2);
  bufAdd(out, DATA_END, sizeof(DATA_END));

  int finalLen = getBufPos();
  for (int i=0; i<finalLen; i++) {
    mpptSerial->write(out[i]);
    if (NULL!=debugOutput) {
      debugOutput[i] = out[i];  
    }
  }
}

uint8_t TracerMPPT::_read(uint8_t expectedCmd, uint8_t* args, uint8_t* buf) {

  const int MAX_READ_TIMEOUT = 255;
  int readEffective = 0;
  int readTotalCnt = 0;
 
  int syncpos = 0;

  while (mpptSerial->available() && syncpos < sizeof(COMM_SYNC) && readTotalCnt < MAX_READ_TIMEOUT) {
    char c = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=c;
    }
    readTotalCnt++;
    
    
    if (-1 == c) {
      return 240; //failiure to synchronize
    }

    if (c == (char)COMM_SYNC[syncpos]) {
      syncpos++;
    } else {
      readEffective=0;
      syncpos=0;
    }
  }

  uint8_t id;
  if (mpptSerial->available()) {
    //id
    id = mpptSerial->read();  
    if (NULL!=buf) {
      buf[readEffective++]=id;  
    }
  } else {
    return 241;
  }

  uint8_t cmd;
  if (mpptSerial->available()) {
    //cmd
      cmd = mpptSerial->read(); 
      if (NULL!=buf) {
        buf[readEffective++]=cmd;    
      }
     if (cmd != expectedCmd) {
      return 248;
     }
  } else {
    return 242;
  }
  
  char argsLen =0;
  if (mpptSerial->available()) {
    //payload length
    argsLen = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=argsLen;  
    }
  } else {
    return 243;
  }

  for (int i=0; i<argsLen && mpptSerial->available(); i++) {
    args[i] = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=args[i];
    }
    if (-1==args[i]) {
      return 244; //timeout waiting for payload data
    }
  }

  uint8_t payload[argsLen+3];

  payload[0] = id; //id
  payload[1] = cmd; //cmd
  payload[2] = argsLen; 
  for (int i=0; i<argsLen; i++) {
    payload[i+3] = args[i];
  }

  uint16_t expectedCRC = _crc(payload,argsLen+3);

  char crc[2];
    
  if (mpptSerial->available()) {
    crc[0] = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=crc[0];
    }
  } else {
    return 245;
  }

  if (mpptSerial->available()) {
    crc[1] = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=crc[1];
    }
  } else {
    return 246;
  }

  if (mpptSerial->available()) {
    char terminator = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=terminator;
    }
    if (terminator != DATA_END[0]) {
      return 249;
    }
  } else {
    return 247;
  }

  uint16_t receivedCRC;
  _unpack(crc,&receivedCRC);

  if (expectedCRC!=receivedCRC) {
    return 250;
  }

  return argsLen;
}


void TracerMPPT::_pack(uint16_t d, uint8_t result[2]) {
  result[0] = ((d & 0xFF00) >> 8);
  result[1] = (d & 0x00FF);
}

void  TracerMPPT::_unpack(char data[2], uint16_t* result) {
  uint16_t calc = 0;
  
  calc += reinterpret_cast<unsigned char&>(data[0]) << 8;
  calc += reinterpret_cast<unsigned char&>(data[1]);
  
  *result = calc;
}

uint16_t TracerMPPT::_crc(uint8_t* s, uint8_t sLen) {      
      int r1, r2, r3, r4, p=0;

      r1 = (uint8_t)s[p++];
      r2 = (uint8_t)s[p++];
      for (int i=0; i<sLen; i++) {
        r3 = (p < sLen)  ? ((uint8_t)s[p++]) : 0x00;
        for (int j=0; j<8; j++) {
          r4 = r1;
          r1 = (r1 << 1) & 0xFF;
          if ((r2 & 0x80) != 0) {
              r1 = (r1 + 1) & 0xFF;
          }
          r2 = (r2 << 1) & 0xFF;
          if ((r3 & 0x80) != 0) {
              r2 = (r2 + 1) & 0xFF;
          }
          r3 = (r3 << 1) & 0xFF;
          if ((r4 & 0x80) != 0) {
              r1 = r1 ^ 0x10;
              r2 = r2 ^ 0x41;
          }
        }
      }

      return ((r1) << 8) | (r2 & 0xFF);
}


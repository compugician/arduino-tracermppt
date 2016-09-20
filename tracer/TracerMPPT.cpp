#include "TracerMPPT.h"
#include "SoftwareSerial.h"

#include "Arduino.h"

//set to 0 to disable
#define DEBUG 1

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
  return (n>=MIN_ERROR_VALUE);
}

void TracerMPPT::_processRealTimeData(uint8_t* args, uint8_t cnt, TracerData* data) {
  
  if (_isError(cnt)) {
    data->validity.isDataValid = false;
    data->validity.errorCode = cnt;
    data->validity.errorString = _errorToString(data->validity.errorCode);
    return;
  }

  if (24!=cnt) {
    data->validity.isDataValid = false;
    data->validity.errorCode = ERROR_LENGTH_UNEXPECTED;
    data->validity.errorString = _errorToString(data->validity.errorCode);
    return;
  }

  data->validity.isDataValid = true;
  data->validity.errorCode = 0;
  data->validity.errorString = "";


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

char* TracerMPPT::_errorToString(uint8_t err) {
  
  if (ERROR_LENGTH_UNEXPECTED==err) return "ERR_LENGTH_UNEXPECTED";
  if (ERROR_INCORRECT_TERMINATOR==err) return "ERROR_INCORRECT_TERMINATOR";
  if (ERROR_MISSING_TERMINATOR==err) return "ERROR_MISSING_TERMINATOR";
  if (ERROR_CRC_MISMATCH==err) return "ERROR_CRC_MISMATCH";
  if (ERROR_MISSING_CRC2==err) return "ERROR_MISSING_CRC2";
  if (ERROR_MISSING_CRC1==err) return "ERROR_MISSING_CRC1";
  if (ERROR_INCOMPLETE_ARGS==err) return "ERROR_INCOMPLETE_ARGS";
  if (ERROR_MISSING_ARGSLENGTH==err) return "ERROR_MISSING_ARGSLENGTH";
  if (ERROR_MISSING_COMMAND==err) return "ERROR_MISSING_COMMAND";
  if (ERROR_COMMAND_MISMATCH==err) return "ERROR_COMMAND_MISMATCH";
  if (ERROR_MISSING_ID==err) return "ERROR_MISSING_ID";
  if (ERROR_MISSING_SYNC==err) return "ERROR_MISSING_SYNC";
  if (ERROR_READ_TIMEOUT==err) return "ERROR_READ_TIMEOUT";
  if (ERROR_SYNC_FAILIURE==err) return "ERROR_SYNC_FAILIURE";
  if (ERROR_INDISTINGUISHABLE_FROM_ERROR==err) return "ERROR_INDISTINGUISHABLE_FROM_ERROR";
  if (ERROR_MPPT_NOT_FOUND==err) return "ERROR_MPPT_NOT_FOUND";
  char* result = "RESULT";
  result = itoa((long)err,result,10);
  return strcat("Error (No Description) #",result);
}


void TracerMPPT::_processManualControl(uint8_t* args, uint8_t cnt, ManualControlResult* result) {
  
  if (_isError(cnt)) {
    result->validity.isDataValid = false;
    result->validity.errorCode = cnt; 
    result->validity.errorString = _errorToString(result->validity.errorCode);
    return;
  }

  if (13!=cnt) {
    result->validity.isDataValid = false;
    result->validity.errorCode = ERROR_LENGTH_UNEXPECTED;
    result->validity.errorString = _errorToString(result->validity.errorCode);
  }

  result->validity.isDataValid = true;
  result->validity.errorCode = 0;
  result->validity.errorString = "";

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
  uint8_t args[] = {0x01}; 
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

/** 
 *  housekeeping for error logging
 */

uint16_t errorCounts[NUM_ERRORS]; //the number of occurances of various errors

uint8_t TracerMPPT::_logAndThrowError(uint8_t err) {
  if (errorCounts[err - MIN_ERROR_VALUE]<MAX_ERROR_COUNT) {
    errorCounts[err - MIN_ERROR_VALUE]+=1;
  }

  return err;
}

uint16_t* TracerMPPT::getErrorCounts() {
  return errorCounts;
}

uint8_t TracerMPPT::getTotalNumberOfErrorTypes() {
  return NUM_ERRORS;
}

uint8_t TracerMPPT::getMinErrorValue() {
  return MIN_ERROR_VALUE;
}

static uint32_t numReads = 0;

uint32_t TracerMPPT::getTotalNumberOfReads() {
  return numReads;
}

uint8_t TracerMPPT::_read(uint8_t expectedCmd, uint8_t* args, uint8_t* buf) {
  numReads++;

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
      return _logAndThrowError(ERROR_MISSING_SYNC); //failiure to synchronize; should never occur here since serial->available() is a precondition
    }

    if (c == (char)COMM_SYNC[syncpos]) {
      syncpos++;
    } else {
      readEffective=0;
      syncpos=0;
    }
  }

  //check if we sync'ed or not, part 1
  if (!(mpptSerial->available())) {
    if (0==readTotalCnt) {
      return _logAndThrowError(ERROR_MPPT_NOT_FOUND);
    }
    return _logAndThrowError(ERROR_SYNC_FAILIURE);
  }
  
  //check if we sync'ed or not, part 2
  if (readTotalCnt >= MAX_READ_TIMEOUT) {
    return _logAndThrowError(ERROR_READ_TIMEOUT);
  }

  uint8_t id;
  if (mpptSerial->available()) {
    //id
    id = mpptSerial->read();  
    if (NULL!=buf) {
      buf[readEffective++]=id;  
    }
  } else {
    return _logAndThrowError(ERROR_MISSING_ID);
  }

  uint8_t cmd;
  if (mpptSerial->available()) {
    //cmd
      cmd = mpptSerial->read(); 
      if (NULL!=buf) {
        buf[readEffective++]=cmd;    
      }
     if (cmd != expectedCmd) {
      return _logAndThrowError(ERROR_COMMAND_MISMATCH);
     }
  } else {
    return _logAndThrowError(ERROR_MISSING_COMMAND);
  }
  
  char argsLen =0;
  if (mpptSerial->available()) {
    //payload length
    argsLen = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=argsLen;  
    }
  } else {
    return _logAndThrowError(ERROR_MISSING_ARGSLENGTH);
  }

  for (int i=0; i<argsLen && mpptSerial->available(); i++) {
    args[i] = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=args[i];
    }
    if (-1==args[i]) {
      return _logAndThrowError(ERROR_INCOMPLETE_ARGS); //timeout waiting for payload data
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
    return _logAndThrowError(ERROR_MISSING_CRC1);
  }

  if (mpptSerial->available()) {
    crc[1] = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=crc[1];
    }
  } else {
    return _logAndThrowError(ERROR_MISSING_CRC2);
  }

  if (mpptSerial->available()) {
    char terminator = mpptSerial->read();
    if (NULL!=buf) {
      buf[readEffective++]=terminator;
    }
    if (terminator != DATA_END[0]) {
      return _logAndThrowError(ERROR_INCORRECT_TERMINATOR);
    }
  } else {
    return _logAndThrowError(ERROR_MISSING_TERMINATOR);
  }

  uint16_t receivedCRC;
  _unpack(crc,&receivedCRC);

  if (expectedCRC!=receivedCRC) {
    return _logAndThrowError(ERROR_CRC_MISMATCH);
  }

  if (_isError(argsLen)) {
    //if our result is in the range reserved for error codes
    return _logAndThrowError(ERROR_INDISTINGUISHABLE_FROM_ERROR);
  } else {
    return argsLen;
  }
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


#ifndef ODRI_MICRODRIVER_H
#define ODRI_MICRODRIVER_H

#include "quad_crc.h"
#include <SPI.h>
#include <Arduino.h>
#define K_RPM_TO_RAD_PER_SEC (2*PI/60.0)*1000.0
#define TURN_TO_RAD 2*PI

uint16_t swapBytes16(uint16_t value) {
  return (value >> 8) | (value << 8);
}

uint32_t swapBytes32(uint32_t value) {
  return ((value >> 24) & 0xFF) |       // Move byte 3 to byte 0
         ((value >> 8) & 0xFF00) |     // Move byte 2 to byte 1
         ((value << 8) & 0xFF0000) |   // Move byte 1 to byte 2
         ((value << 24) & 0xFF000000); // Move byte 0 to byte 3
}

struct MicroDriverState
  {
    uint16_t status; 
    uint16_t stamp; 
    int32_t pos[2];
    int16_t vel[2];
    int16_t Iq[2];
    uint16_t coil_resistance[2];
    int16_t adc[2];
    uint16_t index;
    uint32_t crc;
};


#pragma pack(push, 1) // Enforce 1-byte alignment
union StatePacket
{
  struct MicroDriverState data;
  uint8_t buffer[34];
};


struct ActuatorState
{
  float q;
  float dq;
  float Iq;
};

struct ActuatorCommand
{
  float q;
  float dq;
  float Iq;
};

typedef struct ActuatorState ActuatorState_t;
typedef struct ActuatorCommand ActuatorCommand_t;

typedef union StatePacket StatePacket_t;


class MicroDriver
{ 
  public:
    MicroDriver(uint8_t slaveSelectPin)
    {
      this->slaveSelectPin = slaveSelectPin;
      pinMode (slaveSelectPin, OUTPUT);
      digitalWrite(slaveSelectPin,HIGH);
      Iq_cmd[0] = 0.0;
      Iq_cmd[0] = 0.0;
    }

    uint32_t extractPacketCRC(uint8_t *buffer)
    {
      return buffer[32] << 24 | buffer[33] << 16 | buffer[30] << 8 | buffer[31];
    }
    void loadPacketCRC(uint8_t *buffer, uint32_t crc)
    {
      buffer[30] = (crc >> 24) & 0xFF;
      buffer[31] = (crc >> 16) & 0xFF;
      buffer[32] = (crc >> 8) & 0xFF;
      buffer[33] = crc & 0xFF;
    }
    void update(void)
    {
      cmdBuffer[0] = (1<<7)|(1<<6)|(1<<5); // Enable both motors
      cmdBuffer[1] = 0; // Set timeout value
      // Apply the feedforward current commands
      int16_t Iq1_cmd = Iq_cmd[0]*1024;
      int16_t Iq2_cmd = Iq_cmd[1]*1024;
      cmdBuffer[14] = (Iq1_cmd>>8) & 0xFF;
      cmdBuffer[15] = Iq1_cmd & 0xFF;
      cmdBuffer[16] = (Iq2_cmd>>8) & 0xFF;
      cmdBuffer[17] =  Iq2_cmd & 0xFF;
      // compute the CRC
      loadPacketCRC(cmdBuffer, CRC_compute(cmdBuffer, 30));
      // perform the SPI transaction
      SPISettings settings(4000000, MSBFIRST, SPI_MODE0);
      SPI.beginTransaction(settings);
      digitalWrite(slaveSelectPin,LOW);
      SPI.transfer(cmdBuffer, driver_state.buffer, 34);
      digitalWrite(slaveSelectPin,HIGH);
      SPI.endTransaction();
      
      float q1 =  TURN_TO_RAD*((float)(int32_t)swapBytes32(driver_state.data.pos[0])/16777216.);
      float q2 =  TURN_TO_RAD*((float)(int32_t)swapBytes32(driver_state.data.pos[1])/16777216.);
      float dq1 = K_RPM_TO_RAD_PER_SEC * ((int16_t) swapBytes16(driver_state.data.vel[0]))/2048.;
      float dq2 = K_RPM_TO_RAD_PER_SEC * ((int16_t) swapBytes16(driver_state.data.vel[1]))/2048.;
      float Iq1 = ((int16_t) swapBytes16(driver_state.data.Iq[0]))/1024.; 
      float Iq2 = ((int16_t) swapBytes16(driver_state.data.Iq[1]))/1024.; 
      uint32_t crc = CRC_compute(driver_state.buffer, 30);
      uint32_t crc2 = extractPacketCRC(driver_state.buffer);
      // Update the state measurements only if the CRC matches
      if(crc2==crc)
      {
        status = swapBytes16(driver_state.data.status);
        q_state[0] = q1;
        q_state[1] = q2;
        dq_state[0] = dq1;
        dq_state[1] = dq2;
        Iq_state[0] = Iq1;
        Iq_state[1] = Iq2;
      }
    }
    void printTXBuffer(void)
    {
      for(int i=0;i<34;i++)
      {
        Serial.print(cmdBuffer[i],HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    void printRXBuffer(void)
    {
      for(int i=0;i<34;i++)
      {
        Serial.print(driver_state.buffer[i],HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    void printStates()
    {
      // Serial.print("Status: ");
      // Serial.print(status, BIN);
      Serial.print("q1: ");
      Serial.print(q_state[0]);
      Serial.print(" dq1: ");
      Serial.print(dq_state[0]);
      Serial.print(" Iq1: ");
      Serial.print(Iq_state[0]);
      Serial.print(" q2: ");
      Serial.print(q_state[1]);
      Serial.print(" dq2: ");
      Serial.print(dq_state[1]);
      Serial.print(" Iq2: ");
      Serial.print(Iq_state[1]);
      Serial.println();
    }
    void setCommand(float Iq1, float Iq2)
    {
      Iq_cmd[0] = Iq1;
      Iq_cmd[1] = Iq2;
    }
    void getState(float *q, float *dq, float *Iq)
    {
        q[0] = q_state[0];
        q[1] = q_state[1];
        dq[0] = dq_state[0];
        dq[1] = dq_state[1];
        Iq[0] = Iq_state[0];
        Iq[1] = Iq_state[1];
    }
    
  private:
    StatePacket_t driver_state;
    uint8_t cmdBuffer[34];
    ActuatorState_t state[2];
    float Iq_cmd[2];
    float Iq_state[2];
    float q_state[2];
    float dq_state[2];
    uint16_t status;
    uint8_t slaveSelectPin;
};

#endif
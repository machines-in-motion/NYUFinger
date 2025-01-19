#include <Arduino.h>

#include <SPI.h>  // include the SPI library:
#include "odri_microdriver.hpp"
#include <math.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

const int DRIVER1_CS = 10;
const int DRIVER2_CS = 9;
const float MAX_CURRENT = 1.0;
const float SAFETY_KV = 0.1;
const float SPEED_SATURATION_SAFETY_KV = 0.16;

#define UDP_CMD_TIMEOUT 100
#define MAX_JOINT_OMEGA 20.0

//Ethernet Params
// The MAC address set for the bridge. Should be changed by the user. 
byte mac_address[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(192, 168, 123, 10);    // The IP address of the DAQ:
IPAddress host_ip(192, 168, 123, 1);    // The IP address of the host computer:
unsigned int localPort = 5000;      // DAQ UDP port
EthernetUDP Udp;
char packetBuffer[1024];  // buffer to hold incoming packet,
uint8_t udp_cmd_buffer[128];

union UDPStatePacket{
  uint8_t buffer[3*12];
  struct{
    float q[3];
    float dq[3];
    float current[3];
  }data;
};

typedef union UDPStatePacket UDPStatePacket_t;

union UDPCommandPacket{
  uint8_t buffer[5*12];
  struct{
    float current[3];
    float q[3];
    float dq[3];
    float kp[3];
    float kv[3];
  }data;
};

typedef union UDPCommandPacket UDPCommandPacket_t;

UDPStatePacket_t state_packet;
UDPCommandPacket_t command_packet;


/*
Assumption:
Driver 1: idx 0 -> hip, idx 1 -> knee
Driver 2: idx 0 -> thigh
*/

MicroDriver driver1(DRIVER1_CS);
MicroDriver driver2(DRIVER2_CS);
IntervalTimer loopTimer;

float sturate(float value, float max)
{
  if (value > max)
  {
    return max;
  }
  else if (value < -max)
  {
    return -max;
  }
  return value;
}

float computePD(float q_des, float dq_des, float q, float dq, float kp=3.0, float kv=0.001, float I_max=1.0)
{
  float u = kp*(q_des - q) + kv*(dq_des - dq);
  return sturate(u, I_max);
}

float computeSafetyPD(float safety_kv, float dq,float I_max)
{
  return sturate(-safety_kv*dq, I_max);
}


// Todo: Generalize the following operations to a system with N motors
float thigh_cmd = 0.0;
float hip_cmd = 0.0;
float knee_cmd = 0.0;

float knee_q = 0.0;
float knee_dq = 0.0;
float knee_curr = 0.0;
float thigh_q = 0.0;
float thigh_dq = 0.0;
float thigh_curr = 0.0;
float hip_q = 0.0;
float hip_dq = 0.0;
float hip_curr = 0.0;

float tau_ff_hip = 0.0;
float tau_ff_thigh = 0.0;
float tau_ff_knee = 0.0;

uint32_t latest_udp_cmd_stamp = 0;
uint32_t transmit_counter = 0;

float speedSaturatorController(float dq, float kv, float max_dq)
{
  if (dq > max_dq)
  {
    return -kv*(dq - max_dq);
  }
  else if (dq < -max_dq)
  {
    return -kv*(dq + max_dq);
  }
  return 0.0;
}
void controlLoop(void)
{
  driver1.update();
  driver2.update();

  float q[2];
  float dq[2];
  float Iq[2];
  driver1.getState(q, dq, Iq);
  hip_q = q[0];
  hip_dq = dq[0];
  hip_curr = Iq[0];
  knee_q = q[1];
  knee_dq = dq[1];
  knee_curr = Iq[1];
  driver2.getState(q, dq, Iq);
  thigh_q = q[0];
  thigh_dq = dq[0];
  thigh_curr = Iq[0];
  if(millis() - latest_udp_cmd_stamp > UDP_CMD_TIMEOUT)
  {
    hip_cmd = computeSafetyPD(SAFETY_KV, hip_dq, MAX_CURRENT);
    thigh_cmd = computeSafetyPD(SAFETY_KV, thigh_dq, MAX_CURRENT);
    knee_cmd = computeSafetyPD(SAFETY_KV, knee_dq, MAX_CURRENT);
  }
  else
  {
    hip_cmd = tau_ff_hip + speedSaturatorController(hip_dq, SAFETY_KV, MAX_JOINT_OMEGA);
    thigh_cmd = tau_ff_thigh + speedSaturatorController(thigh_dq, SAFETY_KV, MAX_JOINT_OMEGA);
    knee_cmd = tau_ff_knee + speedSaturatorController(knee_dq, SPEED_SATURATION_SAFETY_KV, MAX_JOINT_OMEGA);
  }
  driver1.setCommand(hip_cmd, knee_cmd);
  driver2.setCommand(thigh_cmd, 0.0);
  state_packet.data.q[0] = hip_q;
  state_packet.data.q[1] = thigh_q;
  state_packet.data.q[2] = knee_q;
  state_packet.data.dq[0] = hip_dq;
  state_packet.data.dq[1] = thigh_dq;
  state_packet.data.dq[2] = knee_dq;
  state_packet.data.current[0] = hip_curr;
  state_packet.data.current[1] = thigh_curr;
  state_packet.data.current[2] = knee_curr;
  // Transmit the latest state packet every 1ms in case the ethernet cable is connected
  if(Ethernet.linkStatus() != LinkOFF && transmit_counter%2==0)
  {
    Udp.beginPacket(host_ip, localPort);
    Udp.write(state_packet.buffer, sizeof(state_packet.buffer));
    Udp.endPacket();
  }
  transmit_counter++;
}

void setup() {
  SPI.begin();
  Serial.begin(9600);
  Serial.println("Starting the microdriver bridge ...");
  delay(100);
  Ethernet.begin(mac_address, ip);
  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected. Retrying ...");
    delay(500);
  }
  Udp.begin(localPort);
  latest_udp_cmd_stamp = millis();
  loopTimer.begin(controlLoop, 500); // Run the system control loop at 2 kHz
}

void loop() {  
  int packetSize = Udp.parsePacket();
  if (packetSize) 
  {
    Udp.read(udp_cmd_buffer, packetSize);
    Serial.println(packetSize);
    memcpy(command_packet.buffer, udp_cmd_buffer, sizeof(command_packet.buffer));
    tau_ff_hip = command_packet.data.current[0];
    tau_ff_thigh = command_packet.data.current[1];
    tau_ff_knee = command_packet.data.current[2];
    // Serial.println(tau_ff_knee);
    latest_udp_cmd_stamp = millis();
  }
}


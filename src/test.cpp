#include "test.h"

//Start,position1 to position2
//Motor1 run to position1
void Motor1_P1()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x141;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0xa4;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x64; //speed
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0x0b; //position
  tx_frame.data.u8[5] = 0xf4;
  tx_frame.data.u8[6] = 0xff;
  tx_frame.data.u8[7] = 0xff;
  ESP32Can.CANWriteFrame(&tx_frame);
}

//Motor2 run to position1
void Motor2_P1()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x142;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0xa4;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x64;
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0xd0;
  tx_frame.data.u8[5] = 0xf1;
  tx_frame.data.u8[6] = 0xff;
  tx_frame.data.u8[7] = 0xff;
  ESP32Can.CANWriteFrame(&tx_frame);
}

//Motor3 run to position1
void Motor3_P1()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x143;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0xa4;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x64;
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0x00;
  tx_frame.data.u8[5] = 0x00;
  tx_frame.data.u8[6] = 0x00;
  tx_frame.data.u8[7] = 0x00;
  ESP32Can.CANWriteFrame(&tx_frame);
}

//Motor1 run to position2
void Motor1_P2()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x141;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0xa4;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x64;
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0x43;
  tx_frame.data.u8[5] = 0x0c;
  tx_frame.data.u8[6] = 0x00;
  tx_frame.data.u8[7] = 0x00;
  ESP32Can.CANWriteFrame(&tx_frame);
}

//Motor2 run to position2
void Motor2_P2()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x142;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0xa4;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x64;
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0x34;
  tx_frame.data.u8[5] = 0x0e;
  tx_frame.data.u8[6] = 0x00;
  tx_frame.data.u8[7] = 0x00;
  ESP32Can.CANWriteFrame(&tx_frame);
}

//Motor3 run to position2
void Motor3_P2()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x143;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0xa4;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x64;
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0x3e;
  tx_frame.data.u8[5] = 0xde;
  tx_frame.data.u8[6] = 0xff;
  tx_frame.data.u8[7] = 0xff;
  ESP32Can.CANWriteFrame(&tx_frame);
}

//Motors stop
void Motor1_Stop()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x141;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0x80;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x00;
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0x00;
  tx_frame.data.u8[5] = 0x00;
  tx_frame.data.u8[6] = 0x00;
  tx_frame.data.u8[7] = 0x00;
  ESP32Can.CANWriteFrame(&tx_frame);
}

void Motor2_Stop()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x142;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0x80;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x00;
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0x00;
  tx_frame.data.u8[5] = 0x00;
  tx_frame.data.u8[6] = 0x00;
  tx_frame.data.u8[7] = 0x00;
  ESP32Can.CANWriteFrame(&tx_frame);
}

void Motor3_Stop()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x143;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0x80;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x00;
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0x00;
  tx_frame.data.u8[5] = 0x00;
  tx_frame.data.u8[6] = 0x00;
  tx_frame.data.u8[7] = 0x00;
  ESP32Can.CANWriteFrame(&tx_frame);
}

void Motors_Stop()
{
  CAN_frame_t rx_frame;
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x280;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = 0x80;
  tx_frame.data.u8[1] = 0x00;
  tx_frame.data.u8[2] = 0x00;
  tx_frame.data.u8[3] = 0x00;
  tx_frame.data.u8[4] = 0x00;
  tx_frame.data.u8[5] = 0x00;
  tx_frame.data.u8[6] = 0x00;
  tx_frame.data.u8[7] = 0x00;
  ESP32Can.CANWriteFrame(&tx_frame);
}

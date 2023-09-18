/*!
 * \author Cody Roberson
 * \details Arduino based control firmware for the Nano 33 IOT and Nano Every
 * \version 2.0a
 * \date 20230918
 * \copyright 2023 © Arizona State University, All Rights Reserved.
 *
 * Uses xmodem crc16 algorithm sourced here:
 *  http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html

  SoftwareControlSkeleton
  Copyright (C) 2023 Arizona State University

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <Arduino.h>

struct Packet
{
  uint32_t command;
  union
  {
    uint32_t u;
    float f;
  } arg1;

  union
  {
    uint32_t u;
    float f;
  } arg2;

  union
  {
    uint32_t u;
    float f;
  } arg3;

  uint32_t checksum;
};

uint16_t crc_xmodem_update(uint16_t crc, uint8_t data)
{
  int i;

  crc = crc ^ ((uint16_t)data << 8);
  for (i = 0; i < 8; i++)
  {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021; //(polynomial = 0x1021)
    else
      crc <<= 1;
  }
  return crc;
}

uint16_t calc_crc(char *msg, int n)
{

  uint16_t x = 0;

  while (n--)
  {
    x = crc_xmodem_update(x, (uint16_t)*msg++);
  }
  return (x);
}

void setup()
{
  Serial.begin(115200);
}

/*!
  Process incomming command
  \param pkt Incomming packet
 */
Packet do_command(Packet pkt)
{
  Packet retpacket;
  switch (pkt.command)
  {
  case 1: // Check Connection
    retpacket.command = pkt.command;
    retpacket.arg1.u = 1;
    break;

  default:
    retpacket.command = 0xffffffff;
    break;
  }
  retpacket.checksum = calc_crc((char *)&retpacket, sizeof(retpacket) - sizeof(retpacket.checksum));
  return retpacket;
}

void loop()
{
  // Read a packet from Serial
  Packet receivedPacket;
  Packet ret;
  unsigned int savail = (unsigned int)Serial.available();
  if (savail >= sizeof(receivedPacket))
  {
    Serial.readBytes((char *)&receivedPacket, sizeof(receivedPacket));

    // Calculate CRC-16 checksum of the received packet (excluding the checksum field)
    uint16_t calculatedChecksum = calc_crc((char *)&receivedPacket, sizeof(receivedPacket) - sizeof(receivedPacket.checksum));

    while (Serial.available() > 0)
      Serial.read();

    // Check if the received checksum matches the calculated checksum
    if (calculatedChecksum == receivedPacket.checksum)
    {
      // Check the command byte and perform actions accordingly
      ret = do_command(receivedPacket);
      Serial.write((uint8_t *)&ret, sizeof(ret));
    }
    else
    {
      // Checksum mismatch, handle accordingly
      // ...
      ret.command = 0xfffffffe;
      Serial.write((uint8_t *)&ret, sizeof(ret));
    }
  }
  delay(50);
}

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
#include <SPI.h>

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

//********************************************************************************
#define SPI_CHIPSELECT_ATTENUATOR_PIN 10
#define DEFAULT_ATENUATION 127 // 31.75dB -> 127
uint32_t atten[4] = {0, 0, 0, 0};
SPISettings spi_attenuators(4000000, LSBFIRST, SPI_MODE0);

/**
 * @brief Sets a specified attenuator to a given value.
 *
 * @param address attenuator address [1, 4]
 * @param atten_val desired attenuation in db
 * @return int status: -1 fail and 1 success
 */
uint32_t setAttenuation(uint32_t address, uint32_t atten_val)
{
  int status = 0;
  // Round atten_val to nearest quarter
  atten[address - 1] = atten_val;
  digitalWrite(SPI_CHIPSELECT_ATTENUATOR_PIN, LOW);
  SPI.beginTransaction(spi_attenuators);
  SPI.transfer((uint8_t)atten_val);
  SPI.transfer(address);
  SPI.endTransaction();
  digitalWrite(SPI_CHIPSELECT_ATTENUATOR_PIN, HIGH);
  status = 1;
  return status;
}

/**
 * @brief Set all of the available attenuators to a given value
 *
 * @param val attenuation in db
 * @return int status: -1 fail and 1 success
 */
int setAllAttenuation(uint32_t val)
{
  int status = 0;
  for (size_t i = 1; i < 5; i++)
  {
    status = setAttenuation(i, val);
    if (!status) // error occured, abort and report
      return status;
  }
  return status;
}

//********************************************************************************

void setup()
{
  Serial.begin(115200);
  SPI.begin();

  pinMode(SPI_CHIPSELECT_ATTENUATOR_PIN, OUTPUT);
  digitalWrite(SPI_CHIPSELECT_ATTENUATOR_PIN, HIGH);
  setAllAttenuation(DEFAULT_ATENUATION); // initialize the attenuators to a predefined value: set in 'project.h'
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

  case 2: // Set attenuation
    retpacket.command = pkt.command;
    retpacket.arg1.u = setAttenuation(pkt.arg1.u, pkt.arg2.u);
    break;

  case 3: // Get attenuation
    retpacket.command = pkt.command;
    retpacket.arg1.u = atten[pkt.arg1.u - 1];
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

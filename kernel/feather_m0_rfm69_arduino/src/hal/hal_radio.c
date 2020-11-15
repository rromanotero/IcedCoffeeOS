/**
*   This file is part of IcedCoffeeOS
*   (https://github.com/rromanotero/IcedCoffeeOS).
*
*   and adapted from MiniOS:
*   (https://github.com/rromanotero/minios).
*
*   Copyright (c) 2020 Rafael Roman Otero.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
**/

/**
*	HAL Radio Init
*
*/
void hal_radio_init(void){
	//For compatibility
}

uint32_t hal_radio_write(tRadioTransceiver* transceiver, tRadioMessage* message ){
  if (!transceiver->internal_manager->sendtoWait(message->payload, sizeof(message->payload), message->address))
        return HAL_RADIO_SEND_FAILED;

  return HAL_SUCCESS;
}

uint32_t hal_radio_read(tRadioTransceiver* transceiver, tRadioMessage* message ){
  uint8_t len = sizeof(message->payload);
  uint8_t from;

  if( transceiver->internal_manager->recvfromAck( message->payload, &len, &from ) ){
    message->len = strlen( (const char*)message->payload );
    message->address = from;
    message->rssi = abs( transceiver->internal_driver->lastRssi() );
    return HAL_SUCCESS;
  }else{
    return HAL_RADIO_READ_FAILED;
  }
}

uint32_t hal_radio_available(tRadioTransceiver* transceiver){
  return transceiver->internal_manager->available();
}

uint32_t hal_radio_create_transceiver(tRadioTransceiver* transceiver, tRadioId id, uint32_t address, uint32_t tx_power ){
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4

  // Make sure these two are STATIC!
  static RH_RF69 rf69(RFM69_CS, RFM69_INT);
  static RHReliableDatagram rf69_manager(rf69, address);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init())
    return HAL_RADIO_INIT_FAILED;

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(915.0))
    return HAL_RADIO_INIT_FAILED;

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(HAL_RADIO_TX_POWER_MIN, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  transceiver->id = id;
  transceiver->io_type = IoPoll;
  transceiver->internal_driver = &rf69;
  transceiver->internal_manager = &rf69_manager;

  return HAL_SUCCESS;
}

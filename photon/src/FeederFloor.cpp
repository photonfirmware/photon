#include "FeederFloor.h"

FeederFloor::FeederFloor(OneWire* oneWire) : _oneWire(oneWire) {

}

uint8_t FeederFloor::read_floor_address() {
  // reset the 1-wire line, and return false if no chip detected
  if(!_oneWire->reset()) {
    return 0xFF;
  }

  // Send 0x3C to indicate skipping the ROM selection step; there'll only ever be one ROM on the bus
  _oneWire->skip();

  // array with the commands to initiate a read, DS28E07 device expect 3 bytes to start a read: command,LSB&MSB adresses
  uint8_t leemem[3] = {
    0xF0,
    0x00,
    0x00
  };

  // sending those three bytes
  _oneWire->write_bytes(leemem, sizeof(leemem), 1);

  uint8_t addr = _oneWire->read();  // Start by reading our address byte

  // Read the next 31 bytes, discarding their value. Each page is 32 bytes so we need 32 read commands
  for (uint8_t i = 0; i < 31; i++) {
    _oneWire->read();
  }

  // return the first byte from returned data
  return addr;
}

bool FeederFloor::write_floor_address(uint8_t address) {
/*
    wriıte_floor_address()
      success returns programmed address byte
      failure returns 0xFF

This function takes a byte as in input, and flashes it to address 0x0000 in the eeprom (where the floor ID is stored).
The DS28E07 requires a million and one steps to make this happen. Reference the datasheet for details:
https://datasheets.maximintegrated.com/en/ds/DS28E07.pdf
*/


  byte i;                         // This is for the for loops
  //-----
  // Write To Scratchpad
  //-----

  byte data[8] = {address, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // reset the 1-wire line, and return false if no chip detected
  if(!_oneWire->reset()){
    return false;
  }

  // Send 0x3C to indicate skipping the ROM selection step; there'll only ever be one ROM on the bus
  _oneWire->skip();

  // array with the commands to initiate a write to the scratchpad, DS28E07 device expect 3 bytes to start a read: command,LSB&MSB adresses
  byte leemem[3] = {
    0x0F,
    0x00,
    0x00
  };

  // sending those three bytes
  _oneWire->write_bytes(leemem, sizeof(leemem), 1);

  // Now it's time to actually write the data to the scratchpad
  for ( i = 0; i < 8; i++) {
    _oneWire->write(data[i], 1);
  }

  // read back the CRC
  //byte ccrc = _oneWire->read();

  //-----
  // Read Scratchpad
  //-----

  // reset the 1-wire line, and return failure if no chip detected
  if(!_oneWire->reset()){
    return false;
  }

  // Send 0x3C to indicate skipping the ROM selection step; there'll only ever be one ROM on the bus
  _oneWire->skip();

  // send read scratchpad command
  _oneWire->write(0xAA, 1);

  // array for the data we'll read back
  byte read_data[11];

  //read in TA1, TA2, and E/S bytes, then the 8 bytes of data
  for ( i = 0; i < sizeof(read_data); i++) {
    read_data[i] = _oneWire->read();
  }

#if 0  // TODO we should probably be checking the CRC
  // byte for the ccrc the eeprom will send us
  byte scratchpad_ccrc = _oneWire->read();

  byte ccrc_calc = OneWire::crc8(read_data, sizeof(read_data));

  // TODO need to be checking CCRC. never returns true, even when data is identical.
  // if(scratchpad_ccrc != ccrc_calc){
  //   // do nothing
  // }
#endif

  //-----
  // Copy Scratchpad to Memory
  //-----

  // reset the 1-wire line, and return false if no chip detected
  if(!_oneWire->reset()){
    return false;
  }

  // Send 0x3C to indicate skipping the ROM selection step; there'll only ever be one ROM on the bus
  _oneWire->skip();

  // copy scratchpad command
  _oneWire->write(0x55, 1);

  // sending auth bytes from scratchpad read, which is the first 3 bytes
  _oneWire->write_bytes(read_data, 3, 1);

  // wait for programming, we'll get alternating 1s and 0s when done
  float timer = millis();
  while(true){
    if(_oneWire->read() == 0xAA){
      break;
    }
    if( (millis() - timer) > 20 ){ // datasheet says it should only ever take 12ms at most to program
      break;
    }
  }

  // send reset
  if(!_oneWire->reset()){
    return false;
  }

  // check the floor address by reading
  byte written_address = this->read_floor_address();

  if(written_address == address) {
    //return new address
    return true;
  }

  return false;
}
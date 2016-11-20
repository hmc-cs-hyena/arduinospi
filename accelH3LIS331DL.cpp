#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <accelH3LIS331DL.h>




accelH3LIS331DL::accelH3LIS331DL(int8_t cspin)
  : _cs(cspin)
{ }


bool accelH3LIS331DL::begin(void) {

    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);
    // hardware SPI
    SPI.begin();

  // enable all axes, normal mode
  writeRegister8(LIS3DH_REG_CTRL1, 0x07);
  // 400Hz rate
  setDataRate(LIS3DH_DATARATE_400_HZ);

  // High res & BDU enabled
  writeRegister8(LIS3DH_REG_CTRL4, 0x88);

  // DRDY on INT1
  writeRegister8(LIS3DH_REG_CTRL3, 0x10);

  // Turn on orientation config
  //writeRegister8(LIS3DH_REG_PL_CFG, 0x40);

  // enable adcs
  //writeRegister8(LIS3DH_REG_TEMPCFG, 0x80);


  return true;
}


void accelH3LIS331DL::read(void) {
  // read x y z at once

    SPI.beginTransaction(SPISettings(50000000000000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    SPI.transfer(LIS3DH_REG_OUT_X_L | 0x80 | 0x40); // read multiple, bit 7&6 high
    //SPI.transfer((0x24); 
    
    // 0xFF - send nothing
    x_raw = SPI.transfer(0xFF); x_raw |= ((uint16_t)SPI.transfer(0xFF)) << 8;
    y_raw = SPI.transfer(0xFF); y_raw |= ((uint16_t)SPI.transfer(0xFF)) << 8;
    z_raw = SPI.transfer(0xFF); z_raw |= ((uint16_t)SPI.transfer(0xFF)) << 8;

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();              // release the SPI bus
    uint8_t range = getRange();
    //uint8_t range = LIS3DH_RANGE_16_G;
    uint16_t divider = 1;
    if (range == LIS3DH_RANGE_16_G) divider = 1365; // different sensitivity at 16g
    if (range == LIS3DH_RANGE_8_G) divider = 4096;
    if (range == LIS3DH_RANGE_4_G) divider = 8190;
    if (range == LIS3DH_RANGE_2_G) divider = 16380;

    x_g = (float)x_raw / divider;
    y_g = (float)y_raw / divider;
    z_g = (float)z_raw / divider;
                                          
    x = x_g * SENSORS_GRAVITY_STANDARD;
    y = y_g * SENSORS_GRAVITY_STANDARD;
    z = z_g * SENSORS_GRAVITY_STANDARD;

}





/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void accelH3LIS331DL::setRange(lis3dh_range_t range)
{
  uint8_t r = readRegister8(LIS3DH_REG_CTRL4);
  r &= ~(0x30);
  r |= range << 4;
  writeRegister8(LIS3DH_REG_CTRL4, r);
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
lis3dh_range_t accelH3LIS331DL::getRange(void)
{
  /* Read the data format register to preserve bits */
  return (lis3dh_range_t)((readRegister8(LIS3DH_REG_CTRL4) >> 4) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
void accelH3LIS331DL::setDataRate(lis3dh_dataRate_t dataRate)
{
  uint8_t ctl1 = readRegister8(LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  writeRegister8(LIS3DH_REG_CTRL1, ctl1);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
lis3dh_dataRate_t accelH3LIS331DL::getDataRate(void)
{
  return (lis3dh_dataRate_t)((readRegister8(LIS3DH_REG_CTRL1) >> 4)& 0x0F);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool accelH3LIS331DL::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;

  read();

  event->acceleration.x = x_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = y_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = z_g * SENSORS_GRAVITY_STANDARD;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void accelH3LIS331DL::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LIS3DH", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 0;
  sensor->min_value   = 0;
  sensor->resolution  = 0;
}


/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void accelH3LIS331DL::writeRegister8(uint8_t reg, uint8_t value) {
    
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    SPI.transfer(reg & ~0x80); // write, bit 7 low
    SPI.transfer(value);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();              // release the SPI bus

}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t accelH3LIS331DL::readRegister8(uint8_t reg) {
    uint8_t value;

    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    SPI.transfer(reg | 0x80); // read, bit 7 high
    value = SPI.transfer(0);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();              // release the SPI bus

  return value;
}

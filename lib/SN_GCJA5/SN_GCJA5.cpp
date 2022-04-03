/*
  This is a library written for the Panasonic SN-GCJA5 particle sensor
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support open source hardware. Buy a board!
  https://www.sparkfun.com/products/17123

  Written by Nathan Seidle @ SparkFun Electronics, September 8th, 2020
  This library handles reads the Particle Matter and Particle Count.
  https://github.com/sparkfun/SparkFun_Particle_Sensor_SN-GCJA5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.

  * November 2020 / paulvha
  * Changed the way I2C read is performed. Now it will read all values
  * into a buffer in a single I2C. The data will be read from that buffer.
  * If a data point has already been read from the buffer, the buffer will
  * be refreshed. This is done because the SN-GCJA5 sensor can not handle
  * many I2C calls after each other and will lock with keeping SCL line Low.
  * This way we can also get a snap shot of the data at the same time.
  *
*/

#include "SN_GCJA5.h"

//Test to see if the device is connected
bool SFE_PARTICLE_SENSOR::begin(TwoWire &wirePort)
{
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  return (isConnected()); //Check to see if device acks to its address.
}

//Given a mass density PM register, do conversion and return mass density
float SFE_PARTICLE_SENSOR::getPM(uint8_t pmRegister)
{
  uint32_t count = readRegister32(pmRegister);
  return (count / 1000.0);
}
float SFE_PARTICLE_SENSOR::getPM1_0()
{
  Serial.println("PM1_0: ");
  /* trigger new read if needed */
  if (Pm1HasBeenReported == true)
        readMeasurement(); //Pull in new data from sensor

  Pm1HasBeenReported = true;
  return (getPM(SNGCJA5_PM1_0));
}
float SFE_PARTICLE_SENSOR::getPM2_5()
{
  Serial.println("PM2_5: ");
  /* trigger new read if needed */
  if (Pm25HasBeenReported == true)
        readMeasurement(); //Pull in new data from sensor

  Pm25HasBeenReported = true;
  return (getPM(SNGCJA5_PM2_5));
}
float SFE_PARTICLE_SENSOR::getPM10()
{
  Serial.println("PM10: ");
  /* trigger new read if needed */
  if (Pm10HasBeenReported == true)
        readMeasurement(); //Pull in new data from sensor

  Pm10HasBeenReported = true;
  return (getPM(SNGCJA5_PM10));
}

//Particle count functions
uint16_t SFE_PARTICLE_SENSOR::getPC0_5()
{
  Serial.println("PC0_5: ");
  /* trigger new read if needed */
  if (Pc05HasBeenReported == true)
    readMeasurement(); //Pull in new data from sensor

  Pc05HasBeenReported = true;
  return (readRegister16(SNGCJA5_PCOUNT_0_5));
}
uint16_t SFE_PARTICLE_SENSOR::getPC1_0()
{
  Serial.println("PC1_0: ");
  /* trigger new read if needed */
  if (Pc1HasBeenReported == true)
    readMeasurement(); //Pull in new data from sensor

  Pc1HasBeenReported = true;
  return (readRegister16(SNGCJA5_PCOUNT_1_0));
}
uint16_t SFE_PARTICLE_SENSOR::getPC2_5()
{
  Serial.println("PC2_5: ");
  /* trigger new read if needed */
  if (Pc25HasBeenReported == true)
    readMeasurement(); //Pull in new data from sensor

  Pc25HasBeenReported = true;
  return (readRegister16(SNGCJA5_PCOUNT_2_5));
}
uint16_t SFE_PARTICLE_SENSOR::getPC5_0()
{
  Serial.println("PC5_0: ");
  /* trigger new read if needed */
  if (Pc5HasBeenReported == true)
    readMeasurement(); //Pull in new data from sensor

  Pc5HasBeenReported = true;
  return (readRegister16(SNGCJA5_PCOUNT_5_0));
}
uint16_t SFE_PARTICLE_SENSOR::getPC7_5()
{
  Serial.println("PC7_5: ");
  /* trigger new read if needed */
  if (Pc75HasBeenReported == true)
    readMeasurement(); //Pull in new data from sensor

  Pc75HasBeenReported = true;
  return (readRegister16(SNGCJA5_PCOUNT_7_5));
}
uint16_t SFE_PARTICLE_SENSOR::getPC10()
{
  Serial.println("PC10: ");
  /* trigger new read if needed */
  if (Pc10HasBeenReported == true)
    readMeasurement(); //Pull in new data from sensor

  Pc10HasBeenReported = true;
  return (readRegister16(SNGCJA5_PCOUNT_10));
}

// paulvha added to test other registers
bool SFE_PARTICLE_SENSOR::TestReg(uint8_t addr, uint8_t *r)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(addr);
  if (_i2cPort->endTransmission(false) != 0)
    return (false); //Sensor did not ACK

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1);
  if (_i2cPort->available()){
    *r = _i2cPort->read();
    return(true);
  }

  return (false); //Sensor did not respond
}

//State functions
uint8_t SFE_PARTICLE_SENSOR::getState()
{
  /* trigger new read if needed */
  if (StateHasBeenReported == true)
        readMeasurement(); //Pull in new data from sensor

  StateHasBeenReported = true;

  return(readRegister8(SNGCJA5_STATE));
}
uint8_t SFE_PARTICLE_SENSOR::getStatusSensors()
{
  /* trigger new read if needed */
  if (StSenHasBeenReported == true)
        readMeasurement(); //Pull in new data from sensor

  StSenHasBeenReported = true;

  return((readRegister8(SNGCJA5_STATE) >> 6) & 0b11);
}
uint8_t SFE_PARTICLE_SENSOR::getStatusPD()
{
  /* trigger new read if needed */
  if (StPdHasBeenReported == true)
        readMeasurement(); //Pull in new data from sensor

  StPdHasBeenReported = true;

  return ((readRegister8(SNGCJA5_STATE) >> 4) & 0b11);
}
uint8_t SFE_PARTICLE_SENSOR::getStatusLD()
{
  /* trigger new read if needed */
  if (StLdHasBeenReported == true)
        readMeasurement(); //Pull in new data from sensor

  StLdHasBeenReported = true;

  return((readRegister8(SNGCJA5_STATE) >> 2) & 0b11);
}
uint8_t SFE_PARTICLE_SENSOR::getStatusFan()
{
  /* trigger new read if needed */
  if (StFanHasBeenReported == true)
        readMeasurement(); //Pull in new data from sensor

  StFanHasBeenReported = true;

  return((readRegister8(SNGCJA5_STATE) >> 0) & 0b11);
}

//Low level I2C functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Returns true if device acknowledges its address
bool SFE_PARTICLE_SENSOR::isConnected()
{
  _i2cPort->beginTransmission(_deviceAddress);
  if (_i2cPort->endTransmission() != 0){
    return (false); //Sensor did not ACK
  }
  return (true);    //All good

}

/**
 * @brief : Read all the registers to buffer
 */
bool SFE_PARTICLE_SENSOR::readMeasurement()
{
  // reset buffer
  memset (read_buf,0x0, sizeof(read_buf));
  offset = 0;

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(SNGCJA5_PM1_0);

  if (_i2cPort->endTransmission(false) != 0)
    return (false); //Sensor did not ACK

  _i2cPort->requestFrom(_deviceAddress, (uint8_t)40);

  while (_i2cPort->available())
    read_buf[offset++] = _i2cPort->read();

  Pm1HasBeenReported = false;
  Pm25HasBeenReported = false;
  Pm10HasBeenReported = false;

  Pc05HasBeenReported = false;
  Pc1HasBeenReported = false;
  Pc25HasBeenReported = false;
  Pc5HasBeenReported = false;
  Pc75HasBeenReported = false;
  Pc10HasBeenReported = false;

  StateHasBeenReported = false;

  StSenHasBeenReported = false;
  StPdHasBeenReported = false;
  StLdHasBeenReported = false;
  StFanHasBeenReported = false;

  return(true);
}

//Reads from a given location
uint8_t SFE_PARTICLE_SENSOR::readRegister8(uint8_t addr)
{
    return(read_buf[addr]);
}

//Reads two consecutive bytes from a given location
uint16_t SFE_PARTICLE_SENSOR::readRegister16(uint8_t addr)
{
    return ((uint16_t)read_buf[addr+1] << 8 | read_buf[addr]);
}

//Reads four consecutive bytes from a given location
uint32_t SFE_PARTICLE_SENSOR::readRegister32(uint8_t addr)
{
    return (((uint32_t)read_buf[addr+3] << 24) | ((uint32_t)read_buf[addr+2] << 16) | \
    ((uint32_t)read_buf[addr+1] << 8) | ((uint32_t)read_buf[addr] << 0));
}


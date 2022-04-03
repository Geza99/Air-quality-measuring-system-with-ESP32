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
  *
  * November 2020 / paulvha
  * Changed the way I2C read is performed. Now it will read all values
  * into a buffer in a single I2C. The data will be read from that buffer.
  * If a data point has already been read from the buffer, the buffer will
  * be refreshed. This is done because the SN-GCJA5 sensor can not handle
  * many I2C calls after each other and will lock with keeping SCL line LOW.
  * This way we can also get a snap shot of the data at the same time.
*/

#ifndef SPARKFUN_PARTICLE_SENSOR_SNGCJA5_ARDUINO_LIBRARY_H
#define SPARKFUN_PARTICLE_SENSOR_SNGCJA5_ARDUINO_LIBRARY_H

#include "Arduino.h"
#include <Wire.h>

class SFE_PARTICLE_SENSOR
{
  public:

    enum SNGCJA5_REGISTERS {
      SNGCJA5_PM1_0 = 0x00,
      SNGCJA5_PM2_5 = 0x04,
      SNGCJA5_PM10 = 0x08,
      SNGCJA5_PCOUNT_0_5 = 0x0C,
      SNGCJA5_PCOUNT_1_0 = 0x0E,
      SNGCJA5_PCOUNT_2_5 = 0x10,
      SNGCJA5_PCOUNT_5_0 = 0x14,
      SNGCJA5_PCOUNT_7_5 = 0x16,
      SNGCJA5_PCOUNT_10 = 0x18,
      SNGCJA5_STATE = 0x26,
    };

    //By default use the default I2C address, and use Wire port
    bool begin(TwoWire &wirePort = Wire1);
    bool isConnected();

    float getPM1_0();
    float getPM2_5();
    float getPM10();

    uint16_t getPC0_5();
    uint16_t getPC1_0();
    uint16_t getPC2_5();
    uint16_t getPC5_0();
    uint16_t getPC7_5();
    uint16_t getPC10();

    uint8_t getState();
    uint8_t getStatusSensors();
    uint8_t getStatusPD();
    uint8_t getStatusLD();
    uint8_t getStatusFan();
    bool TestReg(uint8_t addr, uint8_t *r); // paulvha added to test other registers

  private:

    float getPM(uint8_t pmRegister);
    bool readMeasurement();
    uint8_t readRegister8(uint8_t addr);
    uint16_t readRegister16(uint8_t addr);
    uint32_t readRegister32(uint8_t addr);

    uint8_t read_buf[100];         // buffer to store data read
    uint8_t offset;                // offset in read_buf
    TwoWire *_i2cPort;             // The generic connection to user's chosen I2C hardware
    uint8_t _deviceAddress = 0x33; // Default, unchangable address

    /*! keeps track whether to read from Sensor */
    bool Pm1HasBeenReported = true;
    bool Pm25HasBeenReported = true;
    bool Pm10HasBeenReported = true;

    bool Pc05HasBeenReported = true;
    bool Pc1HasBeenReported = true;
    bool Pc25HasBeenReported = true;
    bool Pc5HasBeenReported = true;
    bool Pc75HasBeenReported = true;
    bool Pc10HasBeenReported = true;

    bool StateHasBeenReported = true;
    bool StSenHasBeenReported = true;
    bool StPdHasBeenReported = true;
    bool StLdHasBeenReported = true;
    bool StFanHasBeenReported = true;
};

#endif

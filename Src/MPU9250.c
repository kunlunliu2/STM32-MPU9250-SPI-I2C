#include "MPU9250.h"


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
   uint8_t data_write[3];
   data_write[0] = subAddress;
   data_write[1] = data;

#if I2C_dev
	HAL_I2C_Master_Transmit(&Trans, address<<1, (uint8_t *) data_write, 2, 100);
#else

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);

	data_write[0] = subAddress;
    data_write[1] = data;
 	while(HAL_SPI_TransmitReceive(&Trans,(uint8_t*) data_write,(uint8_t*) data_write,2,0x1000)!=HAL_OK);

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);

#endif

}

char readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data[1];
    uint8_t data_write[2];
#if I2C_dev
	HAL_I2C_Mem_Read(&Trans,address<<1,subAddress,1,data,1,100);
    return data[0];
#else
	data_write[0] = address;
    data_write[1] = subAddress;

	if(address == MPU9250_ADDRESS){
		HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
	   	data_write[0] = subAddress | READWRITE_CMD;
	   //	while(HAL_SPI_Transmit(&Trans, data_write, 1, HAL_MAX_DELAY)!=HAL_OK);
	   	HAL_SPI_Transmit(&Trans, data_write, 1, HAL_MAX_DELAY);
	    HAL_SPI_Receive(&Trans,data,1,0xFFFF);
		HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
	    return data[0];
	}
	else
	{
		// set slave 0 to the AK8963 and set for read
		writeByte(AK8963_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | READWRITE_CMD);

		// set the register to the desired AK8963 sub address
		writeByte(AK8963_ADDRESS, I2C_SLV0_REG, subAddress);

		// enable I2C and request the bytes
		writeByte(AK8963_ADDRESS, I2C_SLV0_CTRL, I2C_SLV0_EN | 1);

		// takes some time for these registers to fill
		HAL_Delay(1);

		HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
	   	data_write[0] = EXT_SENS_DATA_00 | READWRITE_CMD;
		HAL_SPI_Transmit(&Trans,(uint8_t*) data_write,1,HAL_MAX_DELAY);
		HAL_SPI_Receive(&Trans, data, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
	    return data[0];
	}
#endif

}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{

    uint8_t data_write[14];

#if I2C_dev
    HAL_I2C_Mem_Read(&Trans,address<<1,subAddress,1,dest,count,100);
#else

	if(address == MPU9250_ADDRESS){
		HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
	   	data_write[0] = subAddress | READWRITE_CMD;
	   	//while(HAL_SPI_Transmit(&Trans, data_write, 1, HAL_MAX_DELAY)!=HAL_OK);
	   	HAL_SPI_Transmit(&Trans, data_write, 1, HAL_MAX_DELAY);
	    HAL_SPI_Receive(&Trans,dest,count,0xFFFF);
		HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
	}
	else
	{
		// set slave 0 to the AK8963 and set for read
		writeByte(AK8963_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | READWRITE_CMD);

		// set the register to the desired AK8963 sub address
		writeByte(AK8963_ADDRESS, I2C_SLV0_REG, subAddress);

		// enable I2C and request the bytes
		writeByte(AK8963_ADDRESS, I2C_SLV0_CTRL, I2C_SLV0_EN | count);

		// takes some time for these registers to fill
		HAL_Delay(1);

		HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
	   	data_write[0] = EXT_SENS_DATA_00 | READWRITE_CMD;
		HAL_SPI_Transmit(&Trans,(uint8_t*) data_write,1,HAL_MAX_DELAY);
		HAL_SPI_Receive(&Trans, dest, count, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
	}

#endif

}


void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
          break;
  }
}


void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0f/32768.0f;
          break;
    case GFS_500DPS:
          gRes = 500.0f/32768.0f;
          break;
    case GFS_1000DPS:
          gRes = 1000.0f/32768.0f;
          break;
    case GFS_2000DPS:
          gRes = 2000.0f/32768.0f;
          break;
  }
}


void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0f/32768.0f;
          break;
    case AFS_4G:
          aRes = 4.0f/32768.0f;
          break;
    case AFS_8G:
          aRes = 8.0f/32768.0f;
          break;
    case AFS_16G:
          aRes = 16.0f/32768.0f;
          break;
  }
}

void readMPU9250Data(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
#if I2C_dev
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
#else
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
 #endif

  uint8_t c = rawData[6]; // End data read by reading ST2 register
  if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
}


void resetMPU9250() {
  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  }


void initAK8963(float * destination)
{
#if I2C_dev
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3] = {0, 0, 0};  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00); // Power down magnetometer
  HAL_Delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x0F); // Enter Fuse ROM access mode
  HAL_Delay(10);
 // readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  rawData[0] = readByte(AK8963_ADDRESS, AK8963_ASAX);
  rawData[1] = readByte(AK8963_ADDRESS, AK8963_ASAY);
  rawData[2] = readByte(AK8963_ADDRESS, AK8963_ASAZ);
  destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
  writeByte(AK8963_ADDRESS, AK8963_CNTL1, 0x00); // Power down magnetometer
  HAL_Delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL1, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  HAL_Delay(10);
#else
  /* AK8963_Reset ---------------------------------------------------------*/
  uint8_t rawData[3] = {0, 0, 0};  // x/y/z gyro calibration data stored here
// enable I2C master mode
  writeByte(MPU9250_ADDRESS, USER_CTRL,I2C_MST_EN); // Enable I2c master
// set the I2C bus speed to 400 kHz
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL,I2C_MST_CLK); // Enable I2c master

/* get the magnetometer calibration */

// set AK8963 to Power Down
  writeByte(AK8963_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS); // Enable I2c slaver
  writeByte(AK8963_ADDRESS, I2C_SLV0_REG, AK8963_CNTL1); // Assign slaver register
  writeByte(AK8963_ADDRESS, I2C_SLV0_DO, PWR_DOWN); //
  writeByte(AK8963_ADDRESS, I2C_SLV0_CTRL, 0x81); //
// long wait between AK8963 mode changes
  HAL_Delay(100);

// set AK8963 to FUSE ROM access
  writeByte(AK8963_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS); // Enable I2c slaver
  writeByte(AK8963_ADDRESS, I2C_SLV0_REG, AK8963_CNTL1); // Assign slaver register
  writeByte(AK8963_ADDRESS, I2C_SLV0_DO, 0x0F); //
  writeByte(AK8963_ADDRESS, I2C_SLV0_CTRL, 0x81); //

// set AK8963 to 16 bit resolution, 100 Hz update rate
  writeByte(AK8963_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS); // Enable I2c slaver
  writeByte(AK8963_ADDRESS, I2C_SLV0_REG, AK8963_CNTL1); // Assign slaver register
  writeByte(AK8963_ADDRESS, I2C_SLV0_DO, Mscale << 4 | Mmode); //
  writeByte(AK8963_ADDRESS, I2C_SLV0_CTRL, 0x81); //

  HAL_Delay(500);

// read the AK8963 ASA registers and compute magnetometer scale factors
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, rawData);
  destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
  HAL_Delay(500);

#endif
}

void initMPU9250()
{
 // Initialize MPU9250 device
 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  HAL_Delay(100);          // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
 // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(MPU9250_ADDRESS, GYRO_CONFIG);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

 // Set accelerometer configuration
  c =  readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  HAL_Delay(100);

// Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  HAL_Delay(100);

// Configure MPU9250 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  HAL_Delay(100); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
*/
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void magcalMPU9250(float * dest1, float * dest2, float * magCalibration)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    // shoot for ~fifteen seconds of mag data
    if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms

   for(ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data

    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }

    if(Mmode == 0x02) HAL_Delay(125);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) HAL_Delay(10);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

   // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0, 0, 0}, aAvg[3] = {0, 0, 0}, aSTAvg[3] = {0, 0, 0}, gSTAvg[3] = {0, 0, 0};
   float factoryTrim[6];
   uint8_t FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   HAL_Delay(250); // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }

 // Configure the gyro and accelerometer for normal operation
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
   HAL_Delay(250); // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO); // X-axis gyro self-test results
   selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
   selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i] = 100.0f*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0f; // Report percent differences
     destination[i+3] = 100.0f*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0f; // Report percent differences
   }

}



// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
//void MadgwickQuaternionUpdate(MPU9250_t * MPU)
       void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float * q, float deltat)
        {
           float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
           float norm;
           float hx, hy, _2bx, _2bz;
           float s1, s2, s3, s4;
           float qDot1, qDot2, qDot3, qDot4;

           // Auxiliary variables to avoid repeated arithmetic
           float _2q1mx;
           float _2q1my;
           float _2q1mz;
           float _2q2mx;
           float _4bx;
           float _4bz;
           float _2q1 = 2.0f * q1;
           float _2q2 = 2.0f * q2;
           float _2q3 = 2.0f * q3;
           float _2q4 = 2.0f * q4;
           float _2q1q3 = 2.0f * q1 * q3;
           float _2q3q4 = 2.0f * q3 * q4;
           float q1q1 = q1 * q1;
           float q1q2 = q1 * q2;
           float q1q3 = q1 * q3;
           float q1q4 = q1 * q4;
           float q2q2 = q2 * q2;
           float q2q3 = q2 * q3;
           float q2q4 = q2 * q4;
           float q3q3 = q3 * q3;
           float q3q4 = q3 * q4;
           float q4q4 = q4 * q4;

           // Normalise accelerometer measurement
           norm = sqrtf(ax * ax + ay * ay + az * az);
           if (norm == 0.0f) return; // handle NaN
           norm = 1.0f/norm;
           ax *= norm;
           ay *= norm;
           az *= norm;

           // Normalise magnetometer measurement
           norm = sqrtf(mx * mx + my * my + mz * mz);
           if (norm == 0.0f) return; // handle NaN
           norm = 1.0f/norm;
           mx *= norm;
           my *= norm;
           mz *= norm;

           // Reference direction of Earth's magnetic field
           _2q1mx = 2.0f * q1 * mx;
           _2q1my = 2.0f * q1 * my;
           _2q1mz = 2.0f * q1 * mz;
           _2q2mx = 2.0f * q2 * mx;
           hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
           hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
           _2bx = sqrtf(hx * hx + hy * hy);
           _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
           _4bx = 2.0f * _2bx;
           _4bz = 2.0f * _2bz;

           // Gradient decent algorithm corrective step
           s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
           s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
           s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
           s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
           norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
           norm = 1.0f/norm;
           s1 *= norm;
           s2 *= norm;
           s3 *= norm;
           s4 *= norm;

           // Compute rate of change of quaternion
           qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
           qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
           qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
           qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

           // Integrate to yield quaternion
           q1 += qDot1 * deltat;
           q2 += qDot2 * deltat;
           q3 += qDot3 * deltat;
           q4 += qDot4 * deltat;
           norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
           norm = 1.0f/norm;
           q[0] = q1 * norm;
           q[1] = q2 * norm;
           q[2] = q3 * norm;
           q[3] = q4 * norm;

        }



 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones.
 //void MahonyQuaternionUpdate(MPU9250_t * dest)
            void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float * q, float deltat)
         {
                float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
                float eInt[3] = {0.0,0.0,0.0};
                 float norm;
                 float hx, hy, bx, bz;
                 float vx, vy, vz, wx, wy, wz;
                 float ex, ey, ez;
                 float pa, pb, pc;

                 // Auxiliary variables to avoid repeated arithmetic
                 float q1q1 = q1 * q1;
                 float q1q2 = q1 * q2;
                 float q1q3 = q1 * q3;
                 float q1q4 = q1 * q4;
                 float q2q2 = q2 * q2;
                 float q2q3 = q2 * q3;
                 float q2q4 = q2 * q4;
                 float q3q3 = q3 * q3;
                 float q3q4 = q3 * q4;
                 float q4q4 = q4 * q4;

                 // Normalise accelerometer measurement
                 norm = sqrtf(ax * ax + ay * ay + az * az);
                 if (norm == 0.0f) return; // handle NaN
                 norm = 1.0f / norm;        // use reciprocal for division
                 ax *= norm;
                 ay *= norm;
                 az *= norm;

                 // Normalise magnetometer measurement
                 norm = sqrtf(mx * mx + my * my + mz * mz);
                 if (norm == 0.0f) return; // handle NaN
                 norm = 1.0f / norm;        // use reciprocal for division
                 mx *= norm;
                 my *= norm;
                 mz *= norm;

                 // Reference direction of Earth's magnetic field
                 hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
                 hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
                 bx = sqrtf((hx * hx) + (hy * hy));
                 bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

                 // Estimated direction of gravity and magnetic field
                 vx = 2.0f * (q2q4 - q1q3);
                 vy = 2.0f * (q1q2 + q3q4);
                 vz = q1q1 - q2q2 - q3q3 + q4q4;
                 wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
                 wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
                 wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

                 // Error is cross product between estimated direction and measured direction of gravity
                 ex = (ay * vz - az * vy) + (my * wz - mz * wy);
                 ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
                 ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
                 if (Ki > 0.0f)
                 {
                     eInt[0] += ex;      // accumulate integral error
                     eInt[1] += ey;
                     eInt[2] += ez;
                 }
                 else
                 {
                     eInt[0] = 0.0f;     // prevent integral wind up
                     eInt[1] = 0.0f;
                     eInt[2] = 0.0f;
                 }

                 // Apply feedback terms
                 gx = gx + Kp * ex + Ki * eInt[0];
                 gy = gy + Kp * ey + Ki * eInt[1];
                 gz = gz + Kp * ez + Ki * eInt[2];

                 // Integrate rate of change of quaternion
                 pa = q2;
                 pb = q3;
                 pc = q4;
                 q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
                 q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
                 q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
                 q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

                 // Normalise quaternion
                 norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
                 norm = 1.0f / norm;
                 q[0] = q1 * norm;
                 q[1] = q2 * norm;
                 q[2] = q3 * norm;
                 q[3] = q4 * norm;

        }



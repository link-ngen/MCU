/*
 * MPU6050.h
 *
 *  Created on: 15.06.2020
 *      Author: nguyen
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU6050_ADDR 	(0x68 << 1)	// Device address when ADO = 0

/* Who I am register value */
#define MPU6050_I_AM	0x68
//
//	MPU6050 Main Registers addresses
//
#define REG_WHO_AM_I 		0x75	// This register is used to verify the identity of the device
#define REG_PWR_MGMT_1		0x6B	// This register allows the user to configure the power mode and clock source. It also provides a bit for
// resetting the entire device, and a bit for disabling the temperature sensor.
#define REG_PWR_MGMT_2		0x6C	// This register allows the user to configure the frequency of wake-ups in Accelerometer Only Low
// Power Mode. This register also allows the user to put individual axes of the accelerometer and
// gyroscope into standby mode.
#define REG_CONFIG			0x1A	// This register configures the external Frame Synchronization (FSYNC) pin sampling and the Digital
// Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers.
#define REG_SMPRT_DIV		0x19	// This register specifies the divider from the gyroscope output rate used to generate the Sample Rate for the MPU-60X0.
#define REG_GYRO_CONFIG		0x1B	// This register is used to trigger gyroscope self-test and configure the gyroscopes’ full scale range.
#define REG_ACCEL_CONFIG	0x1C	// This register is used to trigger accelerometer self test and configure the accelerometer full scale
// range. This register also configures the Digital High Pass Filter
#define REG_INT_PIN_CFG		0x37	// This register configures the behavior of the interrupt signals at the INT pins.
#define REG_INT_ENABLE		0x38	// This register enables interrupt generation by interrupt sources.

#define REG_SIGNAL_PATH_RESET 0x68	// This register is used to reset the analog and digital signal paths of the
// gyroscope, accelerometer and temperature sensors.

// Not in documentation end
#define MPU6050_RA_INT_STATUS       0x3A

// These registers store the most recent accelerometer measurements.
#define	REG_ACCEL_XOUT_H	0x3B
#define	REG_ACCEL_XOUT_L	0x3C

#define	REG_ACCEL_YOUT_H	0x3D
#define	REG_ACCEL_YOUT_L	0x3E

#define	REG_ACCEL_ZOUT_H	0x3F
#define	REG_ACCEL_ZOUT_L	0x40

// These registers store the most recent temperature sensor measurement.
#define	REG_TEMP_OUT_H		0x41
#define	REG_TEMP_OUT_L		0x42

// These registers store the most recent gyroscope measurements.
#define REG_GYRO_XOUT_H		0x43
#define REG_GYRO_XOUT_L		0x44

#define REG_GYRO_YOUT_H		0x45
#define REG_GYRO_YOUT_L		0x46

#define REG_GYRO_ZOUT_H		0x47
#define REG_GYRO_ZOUT_L		0x48

// FIFO Registers addresses
#define REG_USER_CTRL		0x6A	// This register allows the user to enable and disable the FIFO buffer.
#define REG_FIFO_EN			0x23	// This register determines which sensor measurements are loaded into the FIFO buffer.

// These registers keep track of the number of samples currently in the FIFO buffer.
#define REG_FIFO_COUNT_H	0x72
#define REG_FIFO_COUNT_L	0x73

#define REG_FIFO_R_W		0x74	// This register is used to read and write data from the FIFO buffer.
// Fifo start address

#define ZEROS				0x00
#define I2C_TIMEOUT			10

#define PWR1_DEVICE_RESET_BIT		(1<<7)

#define XG_ST				0x00	// Setting this bit causes the X axis gyroscope to perform self test
#define YG_ST				0x00	// Setting this bit causes the Y axis gyroscope to perform self test.
#define ZG_ST				0x00	// Setting this bit causes the Z axis gyroscope to perform self test.

#define XA_ST				0x00	// When set to 1, the X- Axis accelerometer performs self test.
#define YA_ST				0x00	// When set to 1, the Y- Axis accelerometer performs self test.
#define ZA_ST				0x00	// When set to 1, the Z- Axis accelerometer performs self test.

/* Gyro sensitivities in °/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g */
#define MPU6050_ACCE_SENS_2G		((float) 16384)
#define MPU6050_ACCE_SENS_4G		((float) 8192)
#define MPU6050_ACCE_SENS_8G		((float) 4096)
#define MPU6050_ACCE_SENS_16G		((float) 2048)

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define RAD_TO_DEG	57.29577951f

#ifndef MPU6050_I2C_PORT
#define MPU6050_I2C_PORT			hi2c2
#endif

#ifndef MPU6050_RX_DMA
#define MPU6050_RX_DMA				hdma_i2c2_rx
#endif

//#define USE_DMA

extern I2C_HandleTypeDef MPU6050_I2C_PORT;
extern DMA_HandleTypeDef MPU6050_RX_DMA;

typedef enum
{
	MPU6050_OK = 0,				// everything ok
	MPU6050_DEV_NOT_CONNECTED,	// no device with valid address
	MPU6050_DEV_INVALID			// device is not mpu6050
} MPU6050_StateTypeDef;

// selects the full scale range of the accelerometer outputs:
typedef enum
{
	MPU6050_AFS_SEL_2G = 0x00,
	MPU6050_AFS_SEL_4G = 0x01,
	MPU6050_AFS_SEL_8G = 0x02,
	MPU6050_AFS_SEL_16G = 0x03
} MPU6050_InitAccScale;

// selects the full scale range of the gyroscope outputs:
typedef enum
{
	MPU6050_FS_SEL_250 = 0x00,
	MPU6050_FS_SEL_500 = 0x01,
	MPU6050_FS_SEL_1000 = 0x02,
	MPU6050_FS_SEL_2000 = 0x03
} MPU6050_InitGyrosScale;

typedef struct
{
	uint8_t DevAddress;
	float GyroScale;
	float AcceScale;

	int16_t RawAccX;
	int16_t RawAccY;
	int16_t RawAccZ;
	int16_t RawGyroX;
	int16_t RawGyroY;
	int16_t RawGyroZ;

	float AccX;		// values in g: 1g = 9.81m/s^2
	float AccY;		// values in g: 1g = 9.81m/s^2
	float AccZ;		// values in g: 1g = 9.81m/s^2
	float GyroX;		// values in (degree per second)
	float GyroY;		// values in (degree per second)
	float GyroZ;		// values in (degree per second)
	float Temperature;	// values in degree per Celsius

	float Roll;
	float Pitch;
} MPU6050_t;

typedef struct {
	float AccErrorOffsetX;
	float AccErrorOffsetY;
	//float AccErrorOffsetZ;
	float GyroErrorOffsetX;
	float GyroErrorOffsetY;
	float GyroErrorOffsetZ;
} MPU6050_ErrorOffset_t;



/* private function */
/**
 * @brief  calculate error offset from sensor
 */
static void MPU6050_calculate_error_offset(MPU6050_t *data_struct);

/* public functions */

/**
 * @brief  Initializes the MPU6050 device.
 */
MPU6050_StateTypeDef MPU6050_Init(MPU6050_t *data_struct, MPU6050_InitAccScale accSensitivity,
		MPU6050_InitGyrosScale gyroSensitivity);

/**
 * @brief  Enables interrupts
 */
void MPU6050_enableInterrupts(void);

/**
 * @brief  Get Address from device.
 */
uint8_t MPU6050_get_device_address(void);

// Reading data
/**
 * @brief  Read raw temperature from sensor.
 */
void MPU6050_read_temperature(MPU6050_t *data_struct);

/**
 * @brief  Read raw acceleration data from sensor.
 */
void MPU6050_readRAW_acceleration(MPU6050_t *data_struct);

/**
 * @brief  Read scaled acceleration data from sensor.
 */
void MPU6050_read_scaled_acceleration(MPU6050_t *data_struct);

/**
 * @brief  Read raw rotation data from sensor.
 */
void MPU6050_readRAW_rotation(MPU6050_t *data_struct);

/**
 * @brief  Read scaled rotation data from sensor.
 */
void MPU6050_read_scaled_rotation(MPU6050_t *data_struct);

/**
 * @brief  Read all raw data from sensor.
 */
void MPU6050_read_raw_data(MPU6050_t *data_struct);

/**
 * @brief  Read all scaled data from sensor.
 */
void MPU6050_read_scaled_data(MPU6050_t *data_struct);

void MPU6050_readAll_DMA(MPU6050_t *data_struct, uint8_t *buffer);

/**
 * @brief  Read roll, pitch, yaw from sensor
 */
void MPU6050_read_rpy(MPU6050_t *data_struct);

/**
 * @brief  Read INT_STATUS register
 */
uint8_t MPU6050_getIntStatusRegister(void);


#endif /* MPU6050_H_ */


//const DMP_ROM = 'icm20948Dmp.bin'

//LIBRARY Informations

let LIBNAME = "ICM20948"
let LIBVERSION = "0.99-1.0-(typescript)"

// This library drives the TDK ICM20948 9-axis sensor
// It can work:
//    - As a basic sensor to return sensor values
//    - As a semi-autonomous streaming sensor recorder, using a FIFO queue
//    - As an fully-autonomous DMP providing sensor-fusion

// It includes:
//    Quaternion support functions for the basic sensor (see QUpdateFull or QUpdateNomag)
//
//			+---------------------------------------------------+
//			| This extension has been developed for use in      |
//			| MakeCode projects that use the ICM20948 chip.     |
//			|													|
//			| It started as a direct translation into	        |
//			| typescript from Dobodo's micropython version at	|
//			|													|
//			|https://github.com/dobodu/ICM20948_DMP_Micropython |
//			|													|
//          | To integrate with the MakeCode libraries, all the |
//          | low-level I2C communication has been factored out |
//          | into an independent i2c.ts module.                |
//			| (Multi-byte transfers also use the Buffer object) |
//			+---------------------------------------------------+
//
// Many thanks to : SparkFun's helpful library
//                  (see https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary) 
//
//                  Master Thesis of John CAPPELLE
//                  (see https://jonacappelle.github.io/Master-Thesis)
//
// All readings are converted with this orientation
//
//         ^                        Magnetometer
//         | Y+    . Z+                          X Z+ 
//     ----------                     ---------
//    |°1        |                   |°1       |
//    | ICM20948 |  -> X+            | AK09916 |  -> X+
//    |          |                   |         |
//     ----------                     ---------
//    Accelerometer                      |
//    Gyrometer                          v Y+
//
// Thus  XMag_ICM20948 =  XMag_AK9916
//       YMag_ICM20948 = -YMag_AK9916
//       ZMag_ICM20948 = -ZMag_AK9916
// 
// Transfer Matrix is
//
//        [[ 1  0  0]
//         [ 0 -1  0]
//         [ 0  0 -1]]
//
// Working / Not working DMP Sensors
//
//  Sensor                        Working
//  -----------------------------------------------------
//  ACCELEROMETER                 OK
//  GYROSCOPE                     OK
//  RAW_ACCELEROMETER             OK
//  RAW_GYROSCOPE                 OK
//  MAGNETIC_FIELD_UNCALIBRATED   OK
//  GYROSCOPE_UNCALIBRATED        OK          
//  ACTIVITY_CLASSIFICATON        NO          Missing ANDROID_SENSORS_CTRL_BITS
//  STEP_DETECTOR                 NO          Missing header2 ?
//  STEP_COUNTER                  NO          Missing header2 ?
//  GAME_ROTATION_VECTOR          OK
//  ROTATION_VECTOR               OK
//  GEOM_ROTATION_VECTOR          OK
//  GEOM_FIELD                    OK
//  GRAVITY                       OK
//  LINEAR_ACCELERATION           OK
//  ORIENTATION                   OK
//
// To do:
//
// Implement DMP bias writing for accelerometer and gyro
// Check this.DMPSetGyroSf()
// Check quaternions
// Check DMP Gyro Bias values
// Write DMP interrupt enable function
// Understand and rewrite Configure Acceleration Only Gains, Alpha Var and AVAr
//
/* 
DEBUGGING AND TRACE
===================
The <trace> function (found in separate source trace.ts) is invoked throughout 
this library to track progress and report any possible errors.
Its is invoked with a power-of-two <level> indicating its significance:

1  : ICM Chip Debug (Outputs library main messages)
2  : DMP Chip debug (Outputs DMP memory access)
4  : DMP FIFO processing debug (Outputs values during FIFO processing)
8  : DMP Output debug (Outputs values after FIFO process)
16 : DMP Register / I2C debug (Outputs bus communication)
32 : Low-level debug

A global number, TRACE_LEVEL, is a mask that selects which level(s) to log.
*/
let TRACE_LEVEL = 1


/* DATA-COLLECTION MODES
   =====================
Library functions allow use of ICM20948 chip in three different data-collection modes:

ONESHOT: Fresh sensor readings are taken synchronously each time sense() is called.

CYCLED: Sampling rates are chosen, and at precisely regular intervals readings from all
    enabled, Sensors are acquired and added onto the IMU's internal FIFO buffer. 
    In this mode the sense() function de-queues and returns the next set of readings 
    from the queue. This should be called often enough to (on average), keep up with 
    the collection rate, or the growing queue would eventually over-run the FIFO buffer,
    over-writing the earliest un-read values. Note that the magnetometer uses a separate
    (lower) background sampling rate so its queued readings change more infrequently. 

AUTO: This mode runs the embedded Digital Motion Processor on regularly sensed data, 
    adding packets of computed data reports onto the IMU's internal FIFO buffer.
    In this mode the sense() function de-queues and digests the next packet of reports,
    updating the relevant ICM properties, and returning the standard sensor readings.
    Again, sense() should be called often enough to keep the queue short and prevent
    the internal FIFO buffer over-running.
    In addition to the standard sensor readings, the DMP will (if asked) use sensor fusion 
    to autonomously derive a rich range of higher-level properties whose states will be 
    updated by the call to sense() and can subsequently be queried.
*/

//================================================================================
// CHIP ADRESSES AND IDS
//================================================================================

const ICM_ADDRESS = (0x68, 0x69)
const ICM_CHIP_ID = 0xEA
const AK_I2C_ADDR = 0x0C
const AK_CHIP_ID = 0x09

//================================================================================
// ICM20948 REGISTERS
//================================================================================

//BANK COMMON
const ICM_BANK_SEL = 0x7F

//BANK0
const ICM_WHO_AM_I = 0x00
const ICM_USER_CTRL = 0x03
const ICM_USER_CTRL_DMP_EN = 0b10000000
const ICM_USER_CTRL_FIFO_EN = 0b01000000
const ICM_USER_CTRL_I2C_MST_EN = 0b00100000
const ICM_USER_CTRL_I2C_IF_DIS = 0b00010000
const ICM_USER_CTRL_DMP_RST = 0b00001000
const ICM_USER_CTRL_SRAM_RST = 0b00000100
const ICM_USER_CTRL_I2C_MST_RST = 0b00000010
const ICM_LP_CFG = 0x05
const ICM_LP_CFG_MST = 0b01000000
const ICM_LP_CFG_ACC = 0b00100000
const ICM_LP_CFG_GYRO = 0b00010000
const ICM_PWR_MGMT_1 = 0x06
const ICM_PWR_MGMT_1_RESET = 0x80
const ICM_PWR_MGMT_1_SLEEP = 0x40
const ICM_PWR_MGMT_1_LP = 0x20
const ICM_PWR_MGMT_1_NO_TEMP = 0x08
const ICM_PWR_MGMT_1_CLOCK_RESET = 0x07
const ICM_PWR_MGMT_1_CLOCK_AUTO = 0x01
const ICM_PWR_MGMT_1_CLOCK_INTERNAL = 0x00
const ICM_PWR_MGMT_2 = 0x07
const ICM_INT_PIN_CFG = 0x0F
const ICM_INT_PIN_CFG_ACTL = 0x80
const ICM_INT_PIN_CFG_OPEN = 0x40
const ICM_INT_PIN_CFG_LATCH__EN = 0x20
const ICM_INT_PIN_CFG_ANYRD_2CLEAR = 0x10
const ICM_INT_PIN_CFG_ACTL_FSYNC = 0x08
const ICM_INT_PIN_CFG_FSYNC_MODE_EN = 0x04
const ICM_INT_PIN_CFG_BYPASS_EN = 0x02
const ICM_INT_ENABLE = 0x10
const ICM_INT_ENABLE_1 = 0x11
const ICM_INT_ENABLE_2 = 0x12
const ICM_INT_ENABLE_3 = 0x13
const ICM_I2C_MST_STATUS = 0x17
const ICM_I2C_MST_STATUS_SLV4_DONE = 0x40
const ICM_I2C_MST_STATUS_LOST_ARB = 0x20
const ICM_I2C_MST_STATUS_SLV4_NACK = 0x10
const ICM_I2C_MST_STATUS_SLV3_NACK = 0x08
const ICM_I2C_MST_STATUS_SLV2_NACK = 0x04
const ICM_I2C_MST_STATUS_SLV1_NACK = 0x02
const ICM_I2C_MST_STATUS_SLV0_NACK = 0x01
const ICM_DMP_INT_STATUS = 0x18 //thanks Sparkfun
const ICM_INT_STATUS = 0x19
const ICM_INT_STATUS_1 = 0x1A
const ICM_INT_STATUS_2 = 0x1B
const ICM_INT_STATUS_3 = 0x1C
const ICM_SINGLE_FIFO_PRIORITY_SEL = 0x26 //thanks Sparkfun
const ICM_DELAY_TIME_H = 0x28
const ICM_ACCEL_XOUT_H = 0x2D
const ICM_GYRO_XOUT_H = 0x33
const ICM_TEMP_OUT_H = 0x39
const ICM_EXT_SLV_SENS_DATA_00 = 0x3B // Linked to AK_ST1 (0x10) or AK_RSV2 (0x03)
const ICM_EXT_SLV_SENS_DATA_01 = 0x3C // Linked to AK_HXL (0x11) or ??? (0x04)
const ICM_EXT_SLV_SENS_DATA_02 = 0x3D // Linked to AK_HXH (0x12) or AK_HXH (0x05)
const ICM_EXT_SLV_SENS_DATA_03 = 0x3E // Linked to AK_HYL (0x13) or AK_HXL (0x06)
const ICM_EXT_SLV_SENS_DATA_04 = 0x3F // Linked to AK_HYH (0x14) or AK_HYH (0x07)
const ICM_EXT_SLV_SENS_DATA_08 = 0x43 // Linked to AK_ST2
// Add others here if needed
const ICM_FIFO_EN_1 = 0x66
const ICM_FIFO_EN_2 = 0x67
const ICM_FIFO_RST = 0x68
const ICM_FIFO_MODE = 0x69
const ICM_FIFO_COUNTH = 0x70
const ICM_FIFO_COUNTL = 0x71
const ICM_FIFO_R_W = 0x72
const ICM_DATA_RDY_STATUS = 0x74
const ICM_HW_FIX_DISABLE = 0x75  //thanks Sparkfun
const ICM_FIFO_CFG = 0x76
// (MEM means DMP's memory)
const ICM_MEM_START_ADDR = 0x7C // target DMP register
const ICM_MEM_R_W = 0x7D // R/W DMP registers (auto-incrementing from ICM_MEM_START_ADDRESS)
const ICM_MEM_BANK_SEL = 0x7F   // for setting DMP memory bank

//BANK1
const ICM_SELF_TEST_X_GYRO = 0x02
const ICM_SELF_TEST_Y_GYRO = 0x03
const ICM_SELF_TEST_Z_GYRO = 0x04
const ICM_SELF_TEST_X_ACCEL = 0x0E
const ICM_SELF_TEST_Y_ACCEL = 0x0F
const ICM_SELF_TEST_Z_ACCEL = 0x10
const ICM_XA_OFFS_H = 0x14
const ICM_YA_OFFS_H = 0x17
const ICM_ZA_OFFS_H = 0x1A
const ICM_TIMEBASE_CORRECTION_PLL = 0x28

// BANK 2 (Note: masks inverted: they now identify a field's bits)
const ICM_GYRO_SMPLRT_DIV = 0x00
const ICM_GYRO_CONFIG_1 = 0x01
//const ICM_GYRO_CONFIG_1_GYRO_FS_SEL_MASK   = 0b11111001 was inverse
const ICM_GYRO_CONFIG_1_GYRO_FS_SEL_MASK = 0b00000110
//const ICM_GYRO_CONFIG_1_GYRO_DLPCFCFG_MASK = 0b10001110 was inverse
const ICM_GYRO_CONFIG_1_GYRO_DLPCFCFG_MASK = 0b01110001
const ICM_GYRO_CONFIG_2 = 0x02
const ICM_ODR_ALIGN_EN = 0x09
const ICM_ACCEL_SMPLRT_DIV_1 = 0x10
const ICM_ACCEL_SMPLRT_DIV_2 = 0x11
const ICM_ACCEL_INTEL_CTRL = 0x12
const ICM_ACCEL_WOM_THR = 0x13
const ICM_ACCEL_CONFIG_1 = 0x14
// const ICM_ACCEL_CONFIG_1_ACCEL_FS_SEL_MASK = 0b11111001 was inverse
const ICM_ACCEL_CONFIG_1_ACCEL_FS_SEL_MASK = 0b00000110
//const ICM_ACCEL_CONFIG_1_ACCEL_DLPFCFG_MASK = 0b10001110 was inverse
const ICM_ACCEL_CONFIG_1_ACCEL_DLPFCFG_MASK = 0b01110001
const ICM_ACCEL_CONFIG_2 = 0x15
const ICM_PRS_ODR_CONFIG = 0x20
const ICM_PRGM_START_ADDRH = 0x50
const ICM_PRGM_START_ADDRL = 0x51
const ICM_FSYNC_CONFIG = 0x52
const ICM_TEMP_CONFIG = 0x53
const ICM_MOD_CTRL_USR = 0x54
const ICM_MOD_CTRL_USR_REG_LP_DMP_EN = 0x01

//BANK3
const ICM_I2C_MST_ODR_CONFIG = 0x00
const ICM_I2C_MST_CTRL = 0x01
const ICM_I2C_MST_CTRL_MULTI = 0b10000000 //Multi master
const ICM_I2C_MST_CTRL_NSR = 0b00010000 //Stop between reads
const ICM_I2C_MST_DELAY_CTRL = 0x02
const ICM_I2C_SLV0_ADDR = 0x03
const ICM_I2C_SLV0_REG = 0x04
const ICM_I2C_SLV0_CTRL = 0x05
const ICM_I2C_SLV0_DO = 0x06
const ICM_I2C_SLV1_ADDR = 0x07
const ICM_I2C_SLV1_REG = 0x08
const ICM_I2C_SLV1_CTRL = 0x09
const ICM_I2C_SLV1_DO = 0x0A
const ICM_I2C_SLV2_ADDR = 0x0B
const ICM_I2C_SLV2_REG = 0x0C
const ICM_I2C_SLV2_CTRL = 0x0D
const ICM_I2C_SLV2_DO = 0x0E
const ICM_I2C_SLV3_ADDR = 0x0F
const ICM_I2C_SLV3_REG = 0x10
const ICM_I2C_SLV3_CTRL = 0x11
const ICM_I2C_SLV3_DO = 0x12
const ICM_I2C_SLV4_ADDR = 0x13
const ICM_I2C_SLV4_REG = 0x14
const ICM_I2C_SLV4_CTRL = 0x15
const ICM_I2C_SLV4_DO = 0x16
const ICM_I2C_SLV4_DI = 0x17
//BANK 3 COMMON
const ICM_I2C_SLV_ADDR_RNW = 0x80
const ICM_I2C_SLV_CTRL_SLV_ENABLE = 0x80
const ICM_I2C_SLV_CTRL_BYTE_SWAP = 0x40
const ICM_I2C_SLV_CTRL_REG_DIS = 0x20
const ICM_I2C_SLV_CTRL_REG_GROUP = 0x10

//================================================================================
// AK09916 REGISTERS
//================================================================================

const AK_WIA1 = 0x00
const AK_WIA2 = 0x01
const AK_RSV1 = 0x02
const AK_RSV2 = 0x03 //Reserved register, used for DMP reading
const AK_HXH_RSV = 0x04 // ? Hidden Magnetometer Data in Big Indian Format
const AK_ST1 = 0x10
const AK_ST1_DOR = 0b00000010   // Data overflow bit
const AK_ST1_DRDY = 0b00000001  // Data ready bit
const AK_HXL = 0x11 // followed by HXH,HYL,HYH,HZL,HZH (i.e Little-endian readings)
const AK_ST2 = 0x18
const AK_ST2_HOFL = 0b00001000   // Magnetic sensor overflow bit
const AK_CNTL2 = 0x31
const AK_CNTL2_MODE_OFF = 0
const AK_CNTL2_MODE_SINGLE = 1
const AK_CNTL2_MODE_10HZ = 2
const AK_CNTL2_MODE_20HZ = 4
const AK_CNTL2_MODE_50HZ = 6
const AK_CNTL2_MODE_100HZ = 8
const AK_CNTL2_MODE_TEST = 16
const AK_CNTL3 = 0x32
const AK_CNTL3_RESET = 0x01

//================================================================================
// CONSTANTS RELATED TO ICM20948 AND AK09916
//================================================================================

// OFFSETS AND SENSITIVITY
const ICM_TEMPERATURE_SENSITIVITY = 333.87   // CHAPTER 3.4
const ICM_ROOM_TEMP_OFFSET = 21			     // CHAPTER 3.4
const ICM_TEMPERATURE_DEGREES_OFFSET = 21	 // CHAPTER 8.31
const AK_MAGNETOMETER_SENSITIVITY = 0.15	 // CHAPTER 3.3

//RANGES AND SENSITIVITY
interface Mapper { [index: number]: number }

// CHAPTER 3.1
const GYRO_SCALE_RANGE: Mapper = { 250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11 }
const GYRO_SENSITIVITY_FACTOR = [131, 65.5, 32.8, 16.4]
// CHAPTER 3.2				            // CHAPTER 3.1
const ACC_SCALE_RANGE: Mapper = { 2: 0b00, 4: 0b01, 8: 0b10, 16: 0b11 }
const ACC_SENSITIVITY_FACTOR = [16384.0, 8192.0, 4096.0, 2048.0]

// Clock dividers and best available low-pass averaging for the six SAMPLING speeds
const RATE_DIV = [5, 10, 22, 55, 112, 224,]
const RATE_AVG = [7, 6, 5, 4, 3, 2]

//================================================================================
// DMP MEM REGISTER
//================================================================================

//DMP INFOS
const DMP_MEM_BANK_SIZE = 256
//DMP MEMORY 
const DMP_START_ADDRESS = 0x1000
const DMP_LOAD_START = 0x90
const DMP_MAX_WRITE = 16
//DATA OUTPUT CONTROL
const DMP_DATA_OUT_CTL1 = 0x40 // 4*16+0 - 16 bits
const DMP_DATA_OUT_CTL2 = 0x42 // 4*16+2 - 16 bits
const DMP_DATA_INTR_CTL = 0x4C // 4*16+12 - 16 bits
const DMP_DATA_FIFO_WATERMARK = 0x1FE // 31*16+14 - 16 bits
//MOTION EVENT CONTROL
const DMP_DATA_MOTION_EVENT_CTRL = 0x4E // 4*16+14 - 16 bits
//INDICATE TO DMP WHICH SENSOR ARE AVAILABLE
const DMP_DATA_RDY_STATUS = 0x8A // 8*16+10
//BATCH MODE
const DMP_BM_BATCH_CNTR = 0x1B0 // 27*16+0
const DMP_BM_BATCH_THLD = 0x13C // 19*16+12
const DMP_BM_BATCH_MASK = 0x15E // 21*16+14
//SENSOR OUTPUT DATA RATE - ALL 16 BITS
const DMP_ODR_GEOMAG = 0xA0 // 10*16+0
const DMP_ODR_PQUAT6 = 0xA4 // 10*16+4
const DMP_ODR_QUAT9 = 0xA8 // 10*16+8
const DMP_ODR_QUAT6 = 0xAC // 10*16+12
const DMP_ODR_ALS = 0xB2 // 11*16+2
const DMP_ODR_CPASS_CALIBR = 0xB4 // 11*16+4
const DMP_ODR_CPASS = 0xB6 // 11*16+6
const DMP_ODR_GYRO_CALIB = 0xB8 // 11*16+8
const DMP_ODR_GYRO = 0xBA // 11*16+10
const DMP_ODR_PRESSURE = 0xBC // 11*16+12
const DMP_ODR_ACCEL = 0xBE // 11*16+14
//SENSOR OUTPUT DATA RATE COUNTER - ALL 16 BITS
const DMP_ODR_CNTR_GEOMAG = 0x80 // 8*16+0
const DMP_ODR_CNTR_PQUAT6 = 0x84 // 8*16+4
const DMP_ODR_CNTR_QUAT9 = 0x88 // 8*16+8
const DMP_ODR_CNTR_QUAT6 = 0x8C // 8*16+12
const DMP_ODR_CNTR_ALS = 0x92 // 9*16+2
const DMP_ODR_CNTR_CPASS_CALIBR = 0x94 // 9*16+4
const DMP_ODR_CNTR_CPASS = 0x96 // 9*16+6
const DMP_ODR_CNTR_GYRO_CALIBR = 0x98 // 9*16+8
const DMP_ODR_CNTR_GYRO = 0x9A // 9*16+10
const DMP_ODR_CNTR_PRESSURE = 0x9C // 9*16+12
const DMP_ODR_CNTR_ACCEL = 0x9E // 9*16+14
//MOUNTING MATRIX - ALL 32 BITS
const DMP_CPASS_MTX_00 = 0x170 // 23*16+0
const DMP_CPASS_MTX_01 = 0x174 // 23*16+4
const DMP_CPASS_MTX_02 = 0x178 // 23*16+8
const DMP_CPASS_MTX_10 = 0x17C // 23*16+12
const DMP_CPASS_MTX_11 = 0x180 // 24*16+0
const DMP_CPASS_MTX_12 = 0x184 // 24*16+4
const DMP_CPASS_MTX_20 = 0x188 // 24*16+8
const DMP_CPASS_MTX_21 = 0x18C // 24*16+12
const DMP_CPASS_MTX_22 = 0x190 // 25*16+0
//BIAS CALIBRATION  - ALL 32 BITS
// Scaled by ACC 2^12 (FSR 4G) - GYRO 2^15 - COMPASS 2^16
const DMP_ACCEL_BIAS_X = 0x6E4 // 110*16+4
const DMP_ACCEL_BIAS_Y = 0x6E8 // 110*16+8
const DMP_ACCEL_BIAS_Z = 0x6EC // 110*16+12
const DMP_CPASS_BIAS_X = 0x7E4 // 126*16+4
const DMP_CPASS_BIAS_Y = 0x7E8 // 126*16+8
const DMP_CPASS_BIAS_Z = 0x7EC // 126*16+12
const DMP_GYRO_ACCURACY = 0x8A2 // 138*16+2
const DMP_GYRO_BIAS_X = 0x8B4 // 139*16+4
const DMP_GYRO_BIAS_Y = 0x8B8 // 139*16+8
const DMP_GYRO_BIAS_Z = 0x8BC // 139*16+12
const DMP_GYRO_BIAS_SET = 0x8A6 // 138*16+6
const DMP_GYRO_LAST_TEMPR = 0x860 // 134*16+0
const DMP_GYRO_SLOPE_X = 0x4E4 // 78*16+4
const DMP_GYRO_SLOPE_Y = 0x4E8 // 78*16+8
const DMP_GYRO_SLOPE_Z = 0x4EC // 78*16+12
// ACCELEROMETER CALIBRATION
const DMP_ACCEL_ACCURACY = 0x610 // 97*16+0
const DMP_ACCEL_CAL_RESET = 0x4D0 // 77*16+0
const DMP_ACCEL_VARIANCE_THRESH = 0x5D0 // 93*16+0
const DMP_ACCEL_CAL_RATE = 0x5E4 // 94*16+4   16-bit: 0 (225Hz, 112Hz, 56Hz)
const DMP_ACCEL_PRE_SENSOR_DATA = 0x614 // 97*16+4
const DMP_ACCEL_COVARIANCE = 0x658 // 101*16+8
const DMP_ACCEL_ALPHA_VAR = 0x5B0 // 91*16+0  32-bit: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
const DMP_ACCEL_A_VAR = 0x5C0 // 92*16+0 32-bit: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
const DMP_ACCEL_CAL_INIT = 0x5E2 // 94*16+2
const DMP_ACCEL_CAL_SCALE_COVQ_IN_RANGE = 0xC20 // 194*16+0
const DMP_ACCEL_CAL_SCALE_COVQ_OUT_RANGE = 0xC30 // 195*16+0
const DMP_ACCEL_CAL_TEMPERATURE_SENSITIVITY = 0xC24 // 194*16+4
const DMP_ACCEL_CAL_TEMPERATURE_OFFSET_TRIM = 0xC2C // 194*16+12
const DMP_CPASS_ACCURACY = 0x250 // 37*16+0
const DMP_CPASS_BIAS_SET = 0x22E // 34*16+14
const DMP_MAR_MODE = 0x252 // 37*16+2
const DMP_CPASS_COVARIANCE = 0x730 // 115*16+0
const DMP_CPASS_COVARIANCE_CUR = 0x768 // 118*16+8
const DMP_CPASS_REF_MAG_3D = 0x7A0 // 122*16+0
const DMP_CPASS_CAL_INIT = 0x720 // 114*16+0
const DMP_CPASS_EST_FIRST_BIAS = 0x710 // 113*16+0
const DMP_MAG_DISTURB_STATE = 0x712 // 113*16+2
const DMP_CPASS_VAR_COUNT = 0x706 // 112*16+6
const DMP_CPASS_COUNT_7 = 0x572 // 87*16+2
const DMP_CPASS_MAX_INNO = 0x7C0 // 124*16+0
const DMP_CPASS_BIAS_OFFSET = 0x714 // 113*16+4
const DMP_CPASS_CUR_BIAS_OFFSET = 0x724 // 114*16+4
const DMP_CPASS_PRE_SENSOR_DATA = 0x574 // 87*16+4
//COMPASS CALIBRATION PARAMS TO BE ADJUSTED WITH SAMPLING RATE
const DMP_CPASS_TIME_BUFFER = 0x70E // 112*16+14
const DMP_CPASS_RADIUS_3D_THRESH_ANOMALY = 0x708 // 112*16+8 : 4 bytes
const DMP_CPASS_STATUS_CHK = 0x19C // 25*16+12
//GAINS
const DMP_ACCEL_FB_GAIN = 0x220 // 34*16+0
const DMP_ACCEL_ONLY_GAIN = 0x10C // 16*16+12 - 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
const DMP_GYRO_SF = 0x130 // 19*16+0 - 32-bit: gyro scaling factor
//9-axis
const DMP_MAGN_THR_9X = 0x500 // 80*16+0
const DMP_MAGN_LPF_THR_9X = 0x508 // 80*16+8
const DMP_QFB_THR_9X = 0x50C // 80*16+12
//DMP RUNNING COUNTER
const DMP_DMPRATE_CNTR = 0x124 // 18*16+4
//PEDOMETER
const DMP_PEDSTD_BP_B = 0x31C // 49*16+12
const DMP_PEDSTD_BP_A4 = 0x340 // 52*16+0
const DMP_PEDSTD_BP_A3 = 0x344 // 52*16+4
const DMP_PEDSTD_BP_A2 = 0x348 // 52*16+8
const DMP_PEDSTD_BP_A1 = 0x34C // 52*16+12
const DMP_PEDSTD_SB = 0x328 // 50*16+8
const DMP_PEDSTD_SB_TIME = 0x32C // 50*16+12
const DMP_PEDSTD_PEAKTHRSH = 0x398 // 57*16+8
const DMP_PEDSTD_TIML = 0x32A // 50*16+10
const DMP_PEDSTD_TIMH = 0x32E // 50*16+14
const DMP_PEDSTD_PEAK = 0x394 // 57*16+4
const DMP_PEDSTD_STEPCTR = 0x360 // 54*16+0
const DMP_PEDSTD_STEPCTR2 = 0x3A8 // 58*16+8
const DMP_PEDSTD_TIMECTR = 0x3C4 // 60*16+4
const DMP_PEDSTD_DECI = 0x3A0 // 58*16+0
const DMP_PEDSTD_SB2 = 0x3CE // 60*16+14
const DMP_STPDET_TIMESTAMP = 0x128 // 18*16+8
const DMP_PEDSTEP_IND = 0x134 // 19*16+4
const DMP_PED_Y_RATIO = 0x110 // 17*16+0
// SMD
const DMP_SMD_VAR_TH = 0x8DC // 141*16+12
const DMP_SMD_VAR_TH_DRIVE = 0x8FC // 143*16+12
const DMP_SMD_DRIVE_TIMER_TH = 0x8F8 // 143*16+8
const DMP_SMD_TILT_ANGLE_TH = 0xB3C // 179*16+12
const DMP_BAC_SMD_ST_TH = 0xB38 // 179*16+8
const DMP_BAC_ST_ALPHA4 = 0xB4C // 180*16+12
const DMP_BAC_ST_ALPHA4A = 0xB0C // 176*16+12
//WAKE ON MOTION
const DMP_WOM_ENABLE = 0x40E // 64*16+14
const DMP_WOM_STATUS = 0x406 // 64*16+6
const DMP_WOM_THRESHOLD = 0x400 // 64*16+0
const DMP_WOM_CNTR_TH = 0x40C // 64*16+12
//ACTIVITY RECOGNITION
const DMP_BAC_RATE = 0x30A // 48*16+10
const DMP_BAC_STATE = 0xB30 // 179*16+0
const DMP_BAC_STATE_PREV = 0xB34 // 179*16+4
const DMP_BAC_ACT_ON = 0xB60 // 182*16+0
const DMP_BAC_ACT_OFF = 0xB70 // 183*16+0
const DMP_BAC_STILL_S_F = 0xB10 // 177*16+0
const DMP_BAC_RUN_S_F = 0xB14 // 177*16+4
const DMP_BAC_DRIVE_S_F = 0xB20 // 178*16+0
const DMP_BAC_WALK_S_F = 0xB24 // 178*16+4
const DMP_BAC_SMD_S_F = 0xB28 // 178*16+8
const DMP_BAC_BIKE_S_F = 0xB2C // 178*16+12
const DMP_BAC_E1_SHORT = 0x920 // 146*16+0
const DMP_BAC_E2_SHORT = 0x924 // 146*16+4
const DMP_BAC_E3_SHORT = 0x928 // 146*16+8
const DMP_BAC_VAR_RUN = 0x94C // 148*16+12
const DMP_BAC_TILT_INIT = 0xB50 // 181*16+0
const DMP_BAC_MAG_ON = 0xE10 // 225*16+0
const DMP_BAC_PS_ON = 0x4A0 // 74*16+0
const DMP_BAC_BIKE_PREFERENCE = 0xAD8 // 173*16+8
const DMP_BAC_MAG_I2C_ADDR = 0xE58 // 229*16+8
const DMP_BAC_PS_I2C_ADDR = 0x4B4 // 75*16+4
const DMP_BAC_DRIVE_CONFIDENCE = 0x900 // 144*16+0
const DMP_BAC_WALK_CONFIDENCE = 0x904 // 144*16+4
const DMP_BAC_SMD_CONFIDENCE = 0x908 // 144*16+8
const DMP_BAC_BIKE_CONFIDENCE = 0x90C // 144*16+12
const DMP_BAC_STILL_CONFIDENCE = 0x910 // 145*16+0
const DMP_BAC_RUN_CONFIDENCE = 0x914 // 145*16+4
const DMP_BAC_MODE_CNTR = 0x960 // 150*16+0
const DMP_BAC_STATE_T_PREV = 0xB94 // 185*16+4
const DMP_BAC_ACT_T_ON = 0xB80 // 184*16+0
const DMP_BAC_ACT_T_OFF = 0xB84 // 184*16+4
const DMP_BAC_STATE_WRDBS_PREV = 0xB98 // 185*16+8
const DMP_BAC_ACT_WRDBS_ON = 0xB88 // 184*16+8
const DMP_BAC_ACT_WRDBS_OFF = 0xB8C // 184*16+12
const DMP_BAC_ACT_ON_OFF = 0xBE2 // 190*16+2
const DMP_PREV_BAC_ACT_ON_OFF = 0xBC2 // 188*16+2
const DMP_BAC_CNTR = 0x302 // 48*16+2
//FLIP AND PICK-UP
const DMP_FP_VAR_ALPHA = 0xF58 // 245*16+8
const DMP_FP_STILL_TH = 0xF64 // 246*16+4
const DMP_FP_MID_STILL_TH = 0xF48 // 244*16+8
const DMP_FP_NOT_STILL_TH = 0xF68 // 246*16+8
const DMP_FP_VIB_REJ_TH = 0xF18 // 241*16+8
const DMP_FP_MAX_PICKUP_T_TH = 0xF4C // 244*16+12
const DMP_FP_PICKUP_TIMEOUT_TH = 0xF88 // 248*16+8
const DMP_FP_STILL_CONST_TH = 0xF6C // 246*16+12
const DMP_FP_MOTION_CONST_TH = 0xF08 // 240*16+8
const DMP_FP_VIB_COUNT_TH = 0xF28 // 242*16+8
const DMP_FP_STEADY_TILT_TH = 0xF78 // 247*16+8
const DMP_FP_STEADY_TILT_UP_TH = 0xF2C // 242*16+12
const DMP_FP_Z_FLAT_TH_MINUS = 0xF38 // 243*16+8
const DMP_FP_Z_FLAT_TH_PLUS = 0xF3C // 243*16+12
const DMP_FP_DEV_IN_POCKET_TH = 0x4CC // 76*16+12
const DMP_FP_PICKUP_CNTR = 0xF74 // 247*16+4
const DMP_FP_RATE = 0xF0C // 240*16+12
//GYRO FSR
const DMP_GYRO_SCALE = 0x48C // 72*16+12
//ACCEL FSR
const DMP_ACC_SCALE = 0x1E0 // 30*16+0
const DMP_ACC_SCALE2 = 0x4F4 // 79*16+4
//EIS AUTHENTIFICATION
const DMP_EIS_AUTH_INPUT = 0xA04 // 160*16+4
const DMP_EIS_AUTH_OUTPUT = 0xA00 // 160*16+0
//B2S
const DMP_B2S_RATE = 0x308 // 48*16+8
//BRING TO SEE MOUNTING MATRIX 
const DMP_B2S_MTX_00 = 0xD00 // 208*16+0
const DMP_B2S_MTX_01 = 0xD04 // 208*16+4
const DMP_B2S_MTX_02 = 0xD08 // 208*16+8
const DMP_B2S_MTX_10 = 0xD0C // 208*16+12
const DMP_B2S_MTX_11 = 0xD10 // 209*16+0
const DMP_B2S_MTX_12 = 0xD14 // 209*16+4
const DMP_B2S_MTX_20 = 0xD18 // 209*16+8
const DMP_B2S_MTX_21 = 0xD1C // 209*16+12
const DMP_B2S_MTX_22 = 0xD20 // 210*16+0
//DMP ORIENTATION PARAMETERS (Q30) INITIALISATION 
const DMP_Q0_QUAT6 = 0x210 // 33*16+0
const DMP_Q1_QUAT6 = 0x214 // 33*16+4
const DMP_Q2_QUAT6 = 0x218 // 33*16+8
const DMP_Q3_QUAT6 = 0x21C // 33*16+12

//================================================================================
// DMP CONSTANTS AND RELATED STUFF
//================================================================================

// DMP DATA-SIZES IN BYTES 
const DMP_Header_Bytes = 2
const DMP_Header2_Bytes = 2
const DMP_Raw_Accel_Bytes = 6
const DMP_Raw_Gyro_Bytes = 6
const DMP_Gyro_Bias_Bytes = 6
const DMP_Compass_Bytes = 6
const DMP_ALS_Bytes = 8 // Byte[0]: Dummy, Byte[2:1]: Ch0DATA, Byte[4:3]: Ch1DATA, Byte[6:5]: PDATA, Byte[7]: Dummy
const DMP_Quat6_Bytes = 12 // 3 x 4_Bytes data Q1, Q2, Q3 (Q0 is deduced through Q0^2+Q1^2+Q2^2+Q3^2=1)
const DMP_Quat9_Bytes = 14 // 3 x 4_Bytes data + 2_Bytes heading accuracy
const DMP_Ped_Quat6_Bytes = 6
const DMP_Geomag_Bytes = 14 //same as Quat9, The quaternion data is scaled by 2^30
const DMP_Pressure_Bytes = 6 // Byte [2:0]: Pressure data, Byte [5:3]: Temperature data
const DMP_Gyro_Calibr_Bytes = 12 // Hardware unit scaled by 2^15
const DMP_Compass_Calibr_Bytes = 12 // Hardware unit scaled by 2^16
const DMP_Step_TimeStamp_Bytes = 4
const DMP_Accel_Accuracy_Bytes = 2
const DMP_Gyro_Accuracy_Bytes = 2
const DMP_Compass_Accuracy_Bytes = 2
const DMP_Fsync_Detection_Bytes = 2
const DMP_Pickup_Bytes = 2
const DMP_Activity_Recognition_Bytes = 6 // Byte [0]: State-Start, Byte [1]: State-End, Byte [5:2]: timestamp
const DMP_Secondary_On_Off_Bytes = 2
const DMP_Footer_Bytes = 2
const DMP_Maximum_Bytes = 14


//DMP Data_Output_Control_1 (from highest bit to lowest bit)
//also used for Header_1 Bitmap check in FIFO decoding
const DMP_DO_Ctrl_1_Accel = 0x8000 //16 bit
const DMP_DO_Ctrl_1_Gyro = 0x4000 //16 bit
const DMP_DO_Ctrl_1_Compass = 0x2000 //16 bit
const DMP_DO_Ctrl_1_ALS = 0x1000 //16 bit
const DMP_DO_Ctrl_1_Quat6 = 0x0800 //32 bit 6 axis
const DMP_DO_Ctrl_1_Quat9 = 0x0400 //32 bit 6 axis
const DMP_DO_Ctrl_1_Pedom_Quat6 = 0x0200 //16 bit
const DMP_DO_Ctrl_1_Geomag = 0x0100 // 32 bit + heading accuracy
const DMP_DO_Ctrl_1_Pressure = 0x0080 //16 bit
const DMP_DO_Ctrl_1_Gyro_Calibr = 0x0040 //32 bit
const DMP_DO_Ctrl_1_Compass_Calibr = 0x0020 //32 bit
const DMP_DO_Ctrl_1_Step_TimeStamp = 0x0010 //Pedometer Step detector
const DMP_DO_Ctrl_1_Header2 = 0x0008 //Header 2
//const DMP_DO_Ctrl_1_Step_Ind_2 = 0x0004 //Pedometer Step Indicator Bit 2
//const DMP_DO_Ctrl_1_Step_Ind_1 = 0x0002 //Pedometer Step Indicator Bit 1
//const DMP_DO_Ctrl_1_Step_Ind_0 = 0x0001 //Pedometer Step Indicator Bit 0
const DMP_DO_Ctrl_1_Steps = 0x0007 //3-bit count of extra steps detected

//DMP Data_Output_Control_2 (from highest bit to lowest bit)
//also used for Header_2 Bitmap check in FIFO decoding
const DMP_DO_Ctrl_2_Accel_Accuracy = 0x4000
const DMP_DO_Ctrl_2_Gyro_Accuracy = 0x2000
const DMP_DO_Ctrl_2_Compass_Accuracy = 0x1000
const DMP_DO_Ctrl_2_Fsync = 0x0800
const DMP_DO_Ctrl_2_Pickup = 0x0400
const DMP_DO_Ctrl_2_Batch_Mode_Enable = 0x0100 // ?? contributes no Bytes to packet
const DMP_DO_Ctrl_2_Activity_Recog = 0x0080
const DMP_DO_Ctrl_2_Secondary_On_Off = 0x0040

//DMP Interruption Masks
//Determine wich sensor needs to be on (32bits)
let INV_NEEDS_ACCEL_MASK0 = 0b11100010100111101000111000001010
let INV_NEEDS_ACCEL_MASK1 = 0b00000000000000000000011011101000
let INV_NEEDS_GYRO_MASK0 = 0b11100110000000011000111000011000
let INV_NEEDS_GYRO_MASK1 = 0b00000000000000000000100000011000
let INV_NEEDS_COMPAS_MASK0 = 0b10000011000100000100100000001100
let INV_NEEDS_COMPAS_MASK1 = 0b00000000000000000000000010000100
let INV_NEEDS_PRES_MASK0 = 0b00010000000000000000000001000000
let INV_NEEDS_PRES_MASK1 = 0b00000000000000000000000000000000

//DMP Data ready
const DMP_Data_Ready_Gyro = 0x0001
const DMP_Data_Ready_Accel = 0x0002
const DMP_Data_Ready_Secondary_Compass = 0x0008

//DMP Event Control
const DMP_Motion_Event_Control_BAC_Wearable = 0x8000
const DMP_Motion_Event_Control_Activity_Recog_Pedom = 0x4000
const DMP_Motion_Event_Control_Pedometer_Interrupt = 0x2000
const DMP_Motion_Event_Control_Tilt_Interrupt = 0x1000
const DMP_Motion_Event_Control_Significant_Motion_Det = 0x0800
const DMP_Motion_Event_Control_Accel_Calibr = 0x0200
const DMP_Motion_Event_Control_Gyro_Calibr = 0x0100
const DMP_Motion_Event_Control_Compass_Calibr = 0x0080
const DMP_Motion_Event_Control_9axis = 0x0040
const DMP_Motion_Event_Control_BTS = 0x0020
const DMP_Motion_Event_Control_Pickup = 0x0010
const DMP_Motion_Event_Control_Geomag = 0x0008
const DMP_Motion_Event_Control_Bring_Look_To_See = 0x0004
const DMP_Motion_Event_Control_Activity_Recog_Pedom_Accel = 0x0002

const DMP_SENSORS_2_ANDROID: { [key: number]: number } = {
    0: 1, 1: 4, 2: 42, 3: 43, 4: 14, 5: 16, 6: 47, 7: 18, 8: 19,
    9: 15, 10: 11, 11: 20, 12: 2, 13: 17, 14: 46, 15: 41, 16: 9,
    17: 10, 18: 3, 19: 45, 20: 44
}

//Android sensor control bits, 45 to 46 are trials
const ANDROID_SENSORS_CTRL_BITS: { [key: number]: number } = {
    0: 0xFFFF, 1: 0x8008, 2: 0x0028, 3: 0x0408,
    4: 0x4048, 5: 0x1008, 6: 0x0088, 7: 0xFFFF, 8: 0xFFFF, 9: 0x0808,
    10: 0x8808, 11: 0x0408, 12: 0xFFFF, 13: 0xFFFF, 14: 0x2008, 15: 0x0808,
    16: 0x4008, 17: 0x0000, 18: 0x0018, 19: 0x0010, 20: 0x0108, 21: 0xFFFF,
    22: 0xFFFF, 23: 0x8008, 24: 0x0028, 25: 0x0408, 26: 0x4048, 27: 0x1008,
    28: 0x0088, 29: 0x0808, 30: 0x8808, 31: 0x0408, 32: 0xFFFF, 33: 0xFFFF,
    34: 0x2008, 35: 0x0808, 36: 0x4008, 37: 0x0018, 38: 0x0010, 39: 0x0108,
    40: 0xFFFF, 41: 0x0000, 42: 0x8008, 43: 0x4048, 44: 0xFDF8, 45: 0xFFFF,
    46: 0xFFFF, 47: 0x4000
}

const DMP_SENSORS: { [key: string]: number } = {
    "ACCELEROMETER": 0, "GYROSCOPE": 1, "RAW_ACCELEROMETER": 2, "RAW_GYROSCOPE": 3,
    "MAGNETIC_FIELD_UNCALIBRATED": 4, "GYROSCOPE_UNCALIBRATED": 5, "ACTIVITY_CLASSIFICATON": 6,
    "STEP_DETECTOR": 7, "STEP_COUNTER": 8, "GAME_ROTATION_VECTOR": 9, "ROTATION_VECTOR": 10,
    "GEOM_ROTATION_VECTOR": 11, "GEOM_FIELD": 12, "WAKEUP_SIGNIFICANT_MOTION": 13,
    "FLIP_PICKUP": 14, "WAKEUP_TILT_DETECTOR": 15, "GRAVITY": 16, "LINEAR_ACCELERATION": 17,
    "ORIENTATION": 18, "B2S": 19, "ALL": 20
}

const DMP_ACTIVITY = { "Drive": 0x01, "Walk": 0x02, "Run": 0x04, "Bike": 0x08, "Tilt": 0x10, "Still": 0x20 }
const DMP_SECONDARY = { "Gyro_Off": 0x01, "Gyro_On": 0x02, "Compass_Off": 0x04, "Compass_On": 0x08, "Prox_Off": 0x10, "Prox_On": 0x20 }

//================================================================================
// LIBRARY ITSELF
//================================================================================
//
// i2c = i2c bus :                       Mandatory
// add = ICM20948 i2c adress :           Optional
// dmp = Bool true if D2M must be used : Optional
//
// Uses debug.ts with a bit-significant debug level:
//         bit 0 : 01 : ICM Chip Level
//         bit 1 : 02 : DMP Chip Access
//         bit 2 : 04 : DMP Fifo processing
//         bit 3 : 08 : DMP Output
//         bit 4 : 16 : ICM Register Access
//         bit 5 : 32 : Low-level debug
//
//================================================================================

enum MODE {
    ONESHOT,
    CYCLED,
    AUTO
}

// A few useful sampling settings, named as MS<a>_PC<b> 
// where <a> = ms between samples
//   and <b> = % RMS noise level (faster means higher RMS noise)
enum SAMPLING {
    MS200_PC4,
    MS100_PC6,
    MS50_PC9,
    MS20_PC12,
    MS10_PC17,
    MS5_PC24
}

enum RATE {
    HZ4,
    HZ16
}

enum SMOOTH {
    AVG1,
    AVG2,
    AVG4,
    AVG8,
    AVG16,
    AVG32,
    AVG64,
    AVG128
}

class ICM20948 {
    debugLevel: number
    // object variables
    device: number
    status: number
    mode: MODE
    useAcc: boolean
    useGyr: boolean
    useMag: boolean
    useDmp: boolean

    //Properties
    ready: boolean
    dmpReady: boolean

    //Variables for ICM or DMP
    icmBank: number
    magBank: number
    dmpBank: number
    dmpChoice1: number // enabled report flags
    dmpChoice2: number	// enabled report flags (continued)

    dmpHeader1: number // actual packet headers read (should match dmpChoice)
    dmpHeader2: number

    fifoBuffer: Buffer
    androidSensorBitmask_0: number
    androidSensorBitmask_1: number
    dmpDataOutCtl1: number
    dmpDataOutCtl2: number
    dmpPacketSize: number
    dmpDataIntrCtl: number  // interrupt ccontrol mask

    gyroScaleFactor: number // Gyro Scale-factor
    gyroPllVariation: number // calibrated % clock inaccuracy of this particular chip

    //Other buffers
    twoByte: Buffer
    oneInt32: Buffer
    //threePairs: Buffer
    buffer: Buffer
    data: Buffer
    dataOrdered: Buffer

    //Sensitivity 
    accScale: number
    gyrScale: number
    magScale: number

    // XYZ Biases to re-centre readings
    accBias: number[]
    gyrBias: number[]
    magBias: number[]
    accUsingBias: boolean
    gyrUsingBias: boolean
    magUsingBias: boolean


    // Sensor readings
    accXYZ: number[]
    gyrXYZ: number[]
    magXYZ: number[]

    // Sensor reading accuracy metric [0...3] returned by DMP
    accQual: number
    gyrQual: number
    magQual: number

    // Calibrated Sensor readings (what for? and why no Accelerometer?)
    gyrXYZCal: number[]
    magXYZCal: number[]

    quat6: number[]
    pquat6: number[]
    quat9: number[]
    quat9Qual: number
    gquat6: number[]
    gquat6Qual: number
    press: number
    temp: number
    als: number
    stepStamp: number
    steps: number
    fsync: number
    pickup: number
    secOnoff: number
    gyrCount: number
    actRecog: number[]
    bacState: number
    bacTs: number

    // Settings for Cycled Sensor sampling
    // Current sampling rates (spacing in ms)
    accMsGap: number
    gyrMsGap: number
    magMsGap: number

    // Current low-pass smoothing (log2 of #averaged samples)
    accAvgLog2: number
    gyrAvgLog2: number
    magAvgLog2: number

    // Rolling average delay (lag-time in ms)
    // (partly due to rolling average; partly time spent on queue)
    accMsLag: number
    gyrMsLag: number
    magMsLag: number

    // Times when next readings should be available based on Gaps
    accWhenNext: number
    gyrWhenNext: number
    magWhenNext: number

    lastUpdate: number // timestamp of latest update call

    //Quaternion and Orientation
    quat: number[]
    northAngle: number
    beta: number
    pitch: number
    heading: number
    roll: number

    lasttime: number
    newtime: number
    odrCount: number

    // copy of transient raw data (to help debug)
    accRaw: number[]
    gyrRaw: number[]
    magRaw: number[]





    constructor(device: number = 0x68, mode: MODE = MODE.ONESHOT) {
        this.initialise()
        this.device = device
        this.mode = mode
        this.hello()
        if ((this.status & 3) != 3) return
        // Now do mode-specific setup
        switch (mode) {
            case MODE.ONESHOT: this.setupOneShotMode()
                break
            case MODE.CYCLED: this.setupCycledMode()
                break
            case MODE.AUTO: this.setupAutoMode()
                break
        }
    }






    // *********** USER INTERFACE METHODS **********
    /*
    // basic sensor readings in any mode
    readAcc()
    readGyr()
    readMag()
    readTemp() // ...because it's there!

    // streamed readings (in CYCLED or AUTO modes)
    --adopt (and return) nearest workable sampling rates to the rates wanted
    for the three sensors.
    Different sensors impose different limits on rate achievable:
    Accelerometer can support rates from 0.3 Hz to 1125 Hz, given by formula
        rate = 1125/(1+n) Hz, where n is an integer between 0 and 4095

    Gyroscope can support rates from 4.4 Hz to 1125 Hz, given by formula
        rate = 1125/(1+n) Hz, where n is an integer between 0 and 255

    Magnetometer can support rates from 0.7 Hz to 1125 Hz, given by formula
        rate = 1125/(2^n) Hz, where n is an integer between 0 and 15
    
    A rate of zero disables that sensor, and the actual cycle rate is that of the 
    first enabled sensor out of (gyroscope, accelerometer, magnetometer).

    rateUsed(ratesWanted):number{}
    
    useSensors(tags:string)
    useMetaSensors(tags:string)
    start()
    sense()
    stop()

    // additional DMP functions (in AUTO mode)
    reportOn(feature:string, enabled:boolean)
    get(feature:string):any
    */










    // *********** INTERNAL METHODS **********

    // Initialize all class variables
    initialise() {

        this.status = 0
        this.ready = false
        this.dmpReady = false

        this.icmBank = -1
        this.magBank = -1
        this.dmpBank = -1

        //????who uses this space???? Isn't the FIFO living in on-chip register space?
        //this.fifoBuffer = pins.createBuffer(4096) // Chap 1.2 buffer is 4kB     

        this.androidSensorBitmask_0 = 0
        this.androidSensorBitmask_1 = 0
        //this.DMPdataOutCtl1 = 0
        //this.DMPdataOutCtl2 = 0
        this.dmpDataIntrCtl = 0
        this.gyroScaleFactor = 0
        this.gyroPllVariation = 0

        //Other buffers
        this.twoByte = pins.createBuffer(2)
        //this.threePairs = pins.createBuffer(6)
        this.oneInt32 = pins.createBuffer(4)
        //this.buffer = pins.createBuffer(1)
        this.data = pins.createBuffer(128) // general FIFO input
        //this.dataOrdered = pins.createBuffer(14)

        //Sensitivity 
        this.accScale = 1 // acc
        this.gyrScale = 1 // gyro
        this.magScale = AK_MAGNETOMETER_SENSITIVITY // preset mag

        //Bias variables for calibration
        this.accUsingBias = false
        this.gyrUsingBias = false
        this.magUsingBias = false
        this.accBias = [0, 0, 0] // acc bias
        this.gyrBias = [0, 0, 0] // gyr bias
        this.magBias = [0, 0, 0] // mag bias

        //Sensor values (most recent to be read from FIFO)
        this.accXYZ = [0, 0, 0]
        this.accQual = 0
        this.gyrXYZ = [0, 0, 0]
        this.gyrQual = 0
        this.magXYZ = [0, 0, 0]
        this.magQual = 0
        this.gyrXYZCal = [0, 0, 0]
        this.magXYZCal = [0, 0, 0]
        this.quat6 = [0, 0, 0]
        this.pquat6 = [0, 0, 0]
        this.quat9 = [0, 0, 0]
        this.quat9Qual = 0
        this.gquat6 = [0, 0, 0]
        this.gquat6Qual = 0
        this.press = 0
        this.temp = 0
        this.als = 0
        this.steps = 0
        this.fsync = 0
        this.pickup = 0
        this.secOnoff = 0
        this.gyrCount = 0
        this.actRecog = [0, 0, 0]
        this.bacState = 0
        this.bacTs = 0

        //Quaternion and Orientation
        this.quat = [1.0, 0.0, 0.0, 0.0] // vector to hold quaternion
        this.northAngle = 0 // angle from magnetic north to real north
        this.beta = Math.sqrt(3.0 / 4.0) * this.radians(40) // compute beta (see README)
        this.pitch = 0
        this.heading = 0
        this.roll = 0

        this.lasttime = input.runningTimeMicros()
        this.newtime = 0
        this.odrCount = 0
    }

    hello() {
        //Check Chip ID
        if ((this.icmRead(0, ICM_WHO_AM_I) == ICM_CHIP_ID)) {
            this.status |= 1
            trace(1, "Found ICM20948: ID", "N", "value", ICM_CHIP_ID)
        } else {
            trace(1, "Runtime Error: can't find ICM20948")
        }

        //Reset the Chip 
        //this.regConfig(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_RESET, true)
        this.icmAdjustFlags(0, ICM_PWR_MGMT_1, 0, ICM_PWR_MGMT_1_RESET)
        pause(10)

        //Set Clock Auto 
        this.icmWrite(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_CLOCK_AUTO)

        //Put all sensors On
        this.icmWrite(0, ICM_PWR_MGMT_2, 0x00)

        //Configure I2C Master Clock
        this.icmWrite(3, ICM_I2C_MST_CTRL, ICM_I2C_MST_CTRL_NSR | 0x07)  //I2C MSTR CLOCK = 07 = 345, 6kHz

        //Activate I2C Master
        this.icmAdjustFlags(0, ICM_USER_CTRL, 0, ICM_USER_CTRL_I2C_MST_EN)

        //Configure Output Data Rate
        //write(2,  ICM_ODR_ALIGN_EN,  0x01)

        // Check if we can access AK09916 Magnetometer
        /*
        // enabling Slave 0 Read AK_WIA through AK_I2C_ADD
        // reading result through ICM_EXT_SLV_SENS_DATA_00
        this.slaveConfig(0, AK_I2C_ADDR, AK_WIA2, 1, true, true, false, false, false)
        if ((this.icmRead(0, ICM_EXT_SLV_SENS_DATA_00) != AK_CHIP_ID)) {
        */
        trace(32, "find mag...")
        if ((this.magRead(AK_WIA2) == AK_CHIP_ID)) {
            this.status |= 2
            trace(1, "AK09916 Magnetometer found OK")
        } else {
            trace(1, "Runtime Error: Can't find AK09916 Magnetometer")
        }

        // Reset the magnetometer
        // Write Slave 0 WK_CNTL3: the Reset bit OK_CNTL3_RESET
        // Then enable Slave 0 to read AK_CNTL3 through AK_I2C_ADD
        // reading result through ICM_EXT_SLV_SENS_DATA_00
        this.slaveConfig(0, AK_I2C_ADDR, AK_CNTL3, 1, false, true, false, false, false, AK_CNTL3_RESET)
        this.slaveConfig(0, AK_I2C_ADDR, AK_CNTL3, 1, true, true, false, false, false)
        while ((this.icmRead(0, ICM_EXT_SLV_SENS_DATA_00) == 0x01)) { //Loop util reset bit remains on
            pause(10)
        }
        trace(1, "AK09916 Magnetometer Chip Reset OK... ")

        //If we reached here, everything is fine
        this.ready = true

    }

    /** prepare for use in ONESHOT mode */
    setupOneShotMode() {
        this.icmConfig()

    }

    /** prepare for slow use in CYCLED mode */
    setupCycledMode() {
        //this.prepareQueueing(true,true,true,RATE.HZ4,SMOOTH.AVG16)

    }

    /** prepare for use in AUTO mode */
    setupAutoMode() {
        this.useDmp = true
        this.dmpConfig()
        this.dmpReady = true
    }





    /** Request Sensor reading-spacing for FIFO queueing.
     * A value of zero us used to disable that sensor.
     * 
    */
    askForSpacing(useAccGap: number, useGyrGap: number, useMagGap: number): [number, number, number] {
        // get closest divisor
        let accDiv = Math.round(1.125 * useAccGap) - 1
        let gyrDiv = Math.round(1.125 * useGyrGap) - 1
        let magDiv = Math.round(1.125 * useMagGap) - 1
        // constrain to permissable Ranges
        // ???? 
        // work out the equivalent actual rate (Hz)
        let accHz = 1125 / (1 + accDiv)
        let gyrHz = 1125 / (1 + gyrDiv)
        let magHz = 1125 / (1 + magDiv)
        // back-convert to return reading spacing in ms
        return [1000 / accHz, 1000 / gyrHz, 1000 / magHz]
    }

    askForSmoothing(useAccAvg: number, useGyrAvg: number, useMagAvg: number): [number, number, number] {
        // return smoothing requested (as constrained by current reading spacing)
        return [0, 0, 0]
    }

    /** Update the ICM with a new set of readings (source depends on MODE) */
    sense(timeout: number): number {
        let outcome = 0 // bit-significant state vector
        switch (this.mode) {
            case MODE.ONESHOT:
                if (this.useAcc) this.accXYZ = this.readAcc()
                if (this.useGyr) this.gyrXYZ = this.readGyr()
                if (this.useMag) this.magXYZ = this.readMag()
                break
            case MODE.CYCLED:
                this.popQueue(timeout)
                break
            case MODE.AUTO:
                this.popPacket(timeout)
                break

        }
        // compute effective timestamp for latest readings
        /* Four components:
            1. aquisition delay (small)
            2. smoothing delay (half the averaging window)
            3. (if QUEUEING or AUTO) dwell-time spent on the queue
            4. (if AUTO) DMP processing time (small-ish)
        */

        // derive current lag-time (how far in the past the timestamp was)

        return outcome
    }

    /** de-queue next set of sensor-readings from the FIFO buffer,
     * (waiting if necessary, but only until timeout expires)
     */
    popQueue(timeout: number) {

    }

    /** de-queue next packet of reports from the FIFO buffer,
     * (waiting if necessary, but only until timeout expires)
     */
    popPacket(timeout: number) {

    }


    //=========== Below are all the ICM setup property functions ===========================

    //Get accelerator sensitivity
    //@property
    getAccSensitivity() {
        // Read accelerometer full scale range
        let scale = (this.icmRead(2, ICM_ACCEL_CONFIG_1) & 0x06) >> 1
        return (1 / ACC_SENSITIVITY_FACTOR[scale])
    }

    //@property
    getGyroSensitivity() {
        // Read back the degrees per second rate
        let scale = (this.icmRead(2, ICM_GYRO_CONFIG_1) & 0x06) >> 1
        return (1 / GYRO_SENSITIVITY_FACTOR[scale])
    }

    //Set the accelerometer sample rate in Hz (125Hz - 1.125 kHz)
    setAccSampleRate(rate = 125) {
        // SampleRate = 1125 Hz / (1 + AccSampleRateDivider)
        // So AccSampleRateDivider = (1125 / sampleRate) - 1 
        rate = Math.floor((1125.0 / rate) - 1)
        this.icmWrite(2, ICM_ACCEL_SMPLRT_DIV_1, rate >> 8)
        this.icmWrite(2, ICM_ACCEL_SMPLRT_DIV_2, rate & 0xff)

    }

    //Set the acceleration full scale range to +- supplied g value in
    //uses ACC_SCALE_RANGE = {2: 0b00, 4: 0b01, 8: 0b10, 16: 0b11}
    setAccFullScale(scale = 16) {
        //Set Acc Full scale in ICM Registers
        let accRange = ACC_SCALE_RANGE[scale]
        /*let value = this.icmRead(2, ICM_ACCEL_CONFIG_1) & ICM_ACCEL_CONFIG_1_ACCEL_FS_SEL_MASK
        value |= accRange << 1
        this.icmWrite(2, ICM_ACCEL_CONFIG_1, value)
        */
        this.icmAdjustFlags(2, ICM_ACCEL_CONFIG_1,
            ICM_ACCEL_CONFIG_1_ACCEL_FS_SEL_MASK, accRange << 1)
        this.accScale = this.getAccSensitivity()

        //Set DMP Acc Full scale in DMP Memory
        if (this.useDmp) {
            //Configure Acceleration scaling within DMP
            //Internally DMP scales Acc Raw data to 2^25 = 1g when FSR = 4g
            //Inv library tells to write DMP_ACC_SCALE = 0x0400 0000 = 2^26 for 4g 
            //We extrapolated to 2G : 2^25 // 4G : 2^26 // 8G : 2^27 // 16G : 2^28
            let value = 0x01 << (25 + accRange)
            this.oneInt32.setNumber(NumberFormat.UInt32LE, 0, value)
            this.dmpWriteBuffer(DMP_ACC_SCALE, this.oneInt32)

            //In order to output hardware unit data as configured FSR write
            //Inv library tells to write DMP_ACC_SCALE2 = 0x4 0000 = 2^18 for 4g
            //We extrapolated to 1G : 2^17 // 4G : 2^18 // 8G : 2^19 // 16G : 2^20
            value = value >> 8 // 256 times smaller
            this.oneInt32.setNumber(NumberFormat.UInt32LE, 0, value)
            this.dmpWriteBuffer(DMP_ACC_SCALE2, this.oneInt32)
        }
        //Finally Adjust sensitivity    
        this.accScale = 1 / ACC_SENSITIVITY_FACTOR[accRange]
    }

    //Configure the accelerometer low pass filter
    setAccLowPass(enabled = true, mode = 5) {
        let value = this.icmRead(2, ICM_ACCEL_CONFIG_1) & ICM_ACCEL_CONFIG_1_ACCEL_DLPFCFG_MASK
        if (enabled) {
            value |= 0b1  // ACCEL_FCHOICE bit set to enable low-pass filtering 
        }
        value |= (mode & 0x07) << 4
        this.icmWrite(2, ICM_ACCEL_CONFIG_1, value)
    }

    //Set the gyro sample rate in Hz
    setGyroSampleRate(rate = 125) {
        // SampleRate = 1125 Hz / (1 + GyroSampleRateDivider)
        // So GyroSampleRateDivider = (1125 / sampleRate) - 1
        rate = ((1125.0 / rate) - 1) | 0 //..as an integer
        this.icmWrite(2, ICM_GYRO_SMPLRT_DIV, rate & 0xff)
    }

    //Set the gyro full scale range to +- supplied value
    //uses GYRO_SCALE_RANGE = {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}
    setGyroFullScale(scale = 250) {

        //Set Gyro Full scale in ICM Registers
        let value = this.icmRead(2, ICM_GYRO_CONFIG_1) & ICM_GYRO_CONFIG_1_GYRO_FS_SEL_MASK
        let gyroRange = GYRO_SCALE_RANGE[scale]
        value |= gyroRange << 1
        this.icmWrite(2, ICM_GYRO_CONFIG_1, value)
        this.gyrScale = this.getGyroSensitivity()
        //Set DMP Gyro Full scale in DMP Memory
        if (this.useDmp) {
            //Inv library tells to write DMP_GYRO_SCALE = 0x10000000 = 2^28 for 2000 dps 
            //We extrapolated to 250 : 2^25 // 500 : 2^26 // 1000 : 2^27 // 2000 : 2^28   
            value = 0x01 << (25 + gyroRange)
            this.oneInt32.setNumber(NumberFormat.UInt32LE, 0, value)
            this.dmpWriteBuffer(DMP_GYRO_SCALE, this.oneInt32)
        }
        //Finally Adjust sensitivity    
        this.gyrScale = 1 / GYRO_SENSITIVITY_FACTOR[gyroRange]
    }

    //Configure the gyro low pass filter
    setGyroLowPass(enabled = true, mode = 5) {
        let value = this.icmRead(2, ICM_GYRO_CONFIG_1) & ICM_GYRO_CONFIG_1_GYRO_DLPCFCFG_MASK
        if (enabled) {
            value |= 0b1 // GYRO_FCHOICE bit set to enable low-pass filtering 
        }
        value |= (mode & 0x07) << 4
        this.icmWrite(2, ICM_GYRO_CONFIG_1, value)
    }

    //Configure the temperature low pass filter
    setTempLowPass(enabled = true, mode = 1) {
        let value = 0x00
        if (enabled) {
            value = mode & 0x07
        }
        this.icmWrite(2, ICM_TEMP_CONFIG, value)
    }

    //=========== Below are all the sensor calibration functions ===================================

    /** Calibrate All sensors (explore ranges by tilting in all directions) */
    calibrateAll(enable = true, timeout = 2000) {
        if (enable) {
            timeout *= 1000 // ms --> us 
            // Initialise max and min arrays with current set of values
            let accmax = this.readAcc()
            let gyrmax = this.readGyr()
            let magmax = this.readMag()
            let accmin = accmax
            let gyrmin = gyrmax
            let magmin = magmax

            this.lasttime = input.runningTimeMicros()
            while ((input.runningTimeMicros() - this.lasttime) < timeout) {
                let accxyz = this.readAcc()
                let gyrxyz = this.readGyr()
                let magxyz = this.readMag()
                for (let j = 0; j < 3; j++) {
                    // maxima
                    accmax[j] = Math.max(accmax[j], accxyz[j])
                    gyrmax[j] = Math.max(gyrmax[j], gyrxyz[j])
                    magmax[j] = Math.max(magmax[j], magxyz[j])
                    // minima
                    accmin[j] = Math.min(accmin[j], accxyz[j])
                    gyrmin[j] = Math.min(gyrmin[j], gyrxyz[j])
                    magmax[j] = Math.min(magmax[j], magxyz[j])
                    // bias values are the mid-points
                    this.accBias[j] = (accmax[j] + accmin[j]) / 2
                    this.gyrBias[j] = (gyrmax[j] + gyrmin[j]) / 2
                    this.magBias[j] = (magmax[j] + magmax[j]) / 2
                }
            }
            this.accUsingBias = true
            this.gyrUsingBias = true
            this.magUsingBias = true
            trace(1, "Calibration done: bias:", "PPP", "X", this.accBias[0], "Y", this.accBias[1], "Z", this.accBias[2])
        } else {
            this.accBias = [0, 0, 0]
            this.gyrBias = [0, 0, 0]
            this.magBias = [0, 0, 0]
            this.accUsingBias = false
            this.gyrUsingBias = false
            this.magUsingBias = false
            trace(1, "Calibration deactivated")
        }
    }

    /** Calibrate Accelerometer (explore range by shaking in all directions) */
    calibrateAcc(enable = true, timeout = 2000) {
        if (enable) {
            timeout *= 1000 // ms --> us
            let accmax = this.readAcc() // Initialise max and min arrays with current set of values
            let accmin = accmax
            this.lasttime = input.runningTimeMicros()
            while ((input.runningTimeMicros() - this.lasttime) < timeout) {
                let accxyz = this.readAcc()
                for (let j = 0; j < 3; j++) {
                    accmax[j] = Math.max(accmax[j], accxyz[j])
                    accmin[j] = Math.min(accmin[j], accxyz[j])
                    this.accBias[j] = (accmax[j] + accmin[j]) / 2 // mid-points
                }
            }
            this.accUsingBias = true
            trace(1, "Accelerometer calibration done: bias:", "PPP", "X", this.accBias[0], "Y", this.accBias[1], "Z", this.accBias[2])
        } else {
            this.accBias = [0, 0, 0]
            this.accUsingBias = false
            trace(1, "Accelerometer calibration deactivated")
        }
    }

    /** Calibrate Gyroscope (explore range by twisting in all directions) */
    calibrateGyr(enable = true, timeout = 2000) {
        if (enable) {
            timeout *= 1000
            let gyrmax = this.readGyr() // Initialise max and min arrays with current values
            let gyrmin = gyrmax
            this.lasttime = input.runningTimeMicros()
            while ((input.runningTimeMicros() - this.lasttime) < timeout) {
                let gyrxyz = this.readGyr()
                for (let j = 0; j < 3; j++) {
                    gyrmax[j] = Math.max(gyrmax[j], gyrxyz[j])
                    gyrmin[j] = Math.min(gyrmin[j], gyrxyz[j])
                    this.gyrBias[j] = (gyrmax[j] + gyrmin[j]) / 2 // mid-points
                }
            }
            this.gyrUsingBias = true

            // Convert biases to signed int16 and send over to DMP (yet to be debugged)
            if (this.dmpReady) {
                let biasX = Math.round(2 ** 15 * this.gyrBias[0])
                let biasY = Math.round(2 ** 15 * this.gyrBias[1])
                let biasZ = Math.round(2 ** 15 * this.gyrBias[2])
                this.twoByte.setNumber(NumberFormat.UInt16BE, 0, biasX)
                this.dmpWriteBuffer(DMP_GYRO_BIAS_X, this.twoByte)
                this.twoByte.setNumber(NumberFormat.UInt16BE, 0, biasY)
                this.dmpWriteBuffer(DMP_GYRO_BIAS_Y, this.twoByte)
                this.twoByte.setNumber(NumberFormat.UInt16BE, 0, biasZ)
                this.dmpWriteBuffer(DMP_GYRO_BIAS_Z, this.twoByte)

                // Reset the DMP
                this.icmAdjustFlags(0, ICM_USER_CTRL, 0, ICM_USER_CTRL_DMP_RST)
            }
            trace(1, "Gyroscope calibration done: bias", "PPP", "X", this.gyrBias[0], "Y", this.gyrBias[1], "Z", this.gyrBias[2])
        } else {
            this.gyrBias = [0, 0, 0]
            this.gyrUsingBias = false
            trace(1, "Gyroscope calibration deactivated")
        }
    }

    /** Calibrate Magnetometer (explore range by twisting in all directions) */
    calibrateMag(enable = true, timeout = 2000) {
        if (enable) {
            timeout *= 1000
            let magmax = this.readMag() // Initialise max and min arrays with current values
            let magmin = magmax
            this.lasttime = input.runningTimeMicros()
            while ((input.runningTimeMicros() - this.lasttime) < timeout) {
                let magxyz = this.readMag()
                for (let j = 0; j < 3; j++) {
                    magmax[j] = Math.max(magmax[j], magxyz[j])
                    magmax[j] = Math.min(magmax[j], magxyz[j])
                    this.magBias[j] = (magmax[j] + magmax[j]) / 2 // mid-points
                }
            }
            this.magUsingBias = true
            trace(1, "Magnetometer calibration done: bias", "PPP", "X", this.magBias[0], "Y", this.magBias[1], "Z", this.magBias[2])
        } else {
            this.magBias = [0, 0, 0]
            this.magUsingBias = false
            trace(1, "Magnetometer calibration desactivated")
        }
    }

    //=========== Below are all the sensor reading functions ===================================

    //Read the current IMU temperature
    readTemp() {
        // PWR_MGMT_1 defaults to leave temperature enabled
        let tempRaw = i2cReadWordsBE(0, ICM_TEMP_OUT_H, 1)
        let degC = ((tempRaw[0] - ICM_ROOM_TEMP_OFFSET) / ICM_TEMPERATURE_SENSITIVITY) + ICM_TEMPERATURE_DEGREES_OFFSET
        return degC
    }

    // Read acceleration data as a triple
    //@property
    readAcc(): number[] {
        if (this.useDmp) {
            this.dmpUpdate() // do an update cycle from the FIFO
        } else {
            this.data = this.icmReadBuffer(0, ICM_ACCEL_XOUT_H, 6)
            this.unpackAcc(this.data)
        }
        return this.accXYZ
    }

    //Read gyroscope data as a triple
    //@property
    readGyr(): number[] {
        if (this.dmpReady) {
            this.dmpUpdate() // do an update cycle from the FIFO
        } else {
            this.data = this.icmReadBuffer(0, ICM_GYRO_XOUT_H, 6)
            let [gx, gy, gz] = this.data.toArray(NumberFormat.Int16BE)
            gx *= this.gyrScale
            gy *= this.gyrScale
            gz *= this.gyrScale
            if (this.gyrUsingBias) {
                gx -= this.gyrBias[0]
                gy -= this.gyrBias[1]
                gz -= this.gyrBias[2]
            }
            this.gyrXYZ = [gx, gy, gz]
        }
        return this.gyrXYZ
    }

    //Read magnetometer data straight from slave DATA_01 (linked to AK_HXL)
    // @property
    readMag(): number[] {
        if (this.dmpReady) {
            this.dmpUpdate() // do an update cycle from the FIFO
        } else {
            this.data = this.icmReadBuffer(0, ICM_EXT_SLV_SENS_DATA_01, 6)
            let [mx, my, mz] = this.data.toArray(NumberFormat.Int16BE)
            mx *= this.magScale
            my *= this.magScale
            mz *= this.magScale
            if (this.magUsingBias) {
                mx -= this.magBias[0]
                my -= this.magBias[1]
                mz -= this.magBias[2]
            }
            this.magXYZ = [mx, my, mz]
        }
        return this.magXYZ
    }


    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    /**I2C Master Slave configuration*/
    slaveConfig(slave: number, addr: number, reg: number, length: number,
        RnW: boolean, En: boolean, Swp: boolean, Dis: boolean, Grp: boolean, DO: number = null) {
        if ((slave > 4)) {
            return
        }
        //Recalculate I2C_SLx Adresses
        let i2cSlvAddr = ICM_I2C_SLV0_ADDR + 4 * slave
        let i2cSlvReg = ICM_I2C_SLV0_REG + 4 * slave
        let i2cSlvCtrl = ICM_I2C_SLV0_CTRL + 4 * slave
        let i2cSlvDo = ICM_I2C_SLV0_DO + 4 * slave

        if (RnW) {
            addr |= ICM_I2C_SLV_ADDR_RNW
        }
        this.icmWrite(3, i2cSlvAddr, addr)

        if (DO != null) {
            this.icmWrite(3, i2cSlvDo, DO)
        }
        this.icmWrite(3, i2cSlvReg, reg)

        let slvCtrl = length
        if (En) {
            slvCtrl |= ICM_I2C_SLV_CTRL_SLV_ENABLE
        }
        if (Swp) {
            slvCtrl |= ICM_I2C_SLV_CTRL_BYTE_SWAP
        }
        if (Dis) {
            slvCtrl |= ICM_I2C_SLV_CTRL_REG_DIS
        }
        if (Grp) {
            slvCtrl |= ICM_I2C_SLV_CTRL_REG_GROUP
        }
        this.icmWrite(3, i2cSlvCtrl, slvCtrl)

        // ???
        //Activate I2C Master so I2CSlave setup can be propagated to slave itself
        //this.regConfig(0, ICM_USER_CTRL, ICM_USER_CTRL_I2C_MST_EN, true)
    }

    reset_FIFO() {
        this.icmWrite(0, ICM_FIFO_RST, 0x1F)
        pause(5)
        this.icmWrite(0, ICM_FIFO_RST, 0x1E)
    }

    // ==========================================================
    //  TODO: ICM Mag config (needed when not using DMP)
    // ==========================================================

    icmConfig() {
        //Enable the slave 0 to write (false) to AK_I2C magnetometer, by writing 1 byte MODE_100Hz to AK_CNTL2 REG
        this.slaveConfig(0, AK_I2C_ADDR, AK_CNTL2, 1, false, true, false, false, false, AK_CNTL2_MODE_100HZ)

        //Enable the slave to read (true) from AK_ST1 to AK_ST2 (9 bytes) including magnetometer values AK_HXL
        this.slaveConfig(0, AK_I2C_ADDR, AK_ST1, 9, true, true, false, false, false)

        //Configure scales, SR, LP and get sensitivity values
        this.setGyroSampleRate() //default of 125 Hz
        this.setGyroLowPass(true, 5)
        this.setGyroFullScale(2000)
        this.setAccSampleRate()
        this.setAccLowPass(true, 5)
        this.setAccFullScale(4)
        this.setTempLowPass(true, 1)

        //Set Low Power On
        //this.regConfig(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, true)
        this.icmAdjustFlags(0, ICM_PWR_MGMT_1, 0, ICM_PWR_MGMT_1_LP)
    }


    radians(degrees: number) {
        return degrees * Math.PI / 180.0
    }

    /*Only for debugging purposes
    Dbg(level,  * args, ** kwargs) {
        if (level & debug) {
            print("DBG:\t", LIBNAME, ":\t", " ===>" if (level & 16) else "", * args, ** kwargs)
        }
    }
    */

    //=================== Below are all the Sensor Correction functions ==============================
    // (These functions expect bytes[] Buffer to contain three 16-bit raw readings)
    unpackAcc(bytes: Buffer) {
        let [ax, ay, az] = bytes.toArray(NumberFormat.Int16BE)
        this.accRaw = [ax, ay, az] // just for debug...
        ax *= this.accScale //adjust result with sensitivity
        ay *= this.accScale
        az *= this.accScale
        if (this.accUsingBias) {
            ax -= this.accBias[0]
            ay -= this.accBias[1]
            az -= this.accBias[2]
        }
        this.accXYZ = [ax, ay, az]
        trace(1, "Accelerometer:", "NNN", "X", this.accXYZ[0], "Y", this.accXYZ[1], "Z", this.accXYZ[2])

    }

    // bytes buffer contains a triple of 16-bit readings
    unpackGyr(bytes: Buffer) {
        let [gx, gy, gz] = bytes.toArray(NumberFormat.Int16BE)
        this.gyrRaw = [gx, gy, gz] // just for debug...
        gx *= this.gyrScale //adjust result with sensitivity
        gy *= this.gyrScale
        gz *= this.gyrScale
        if (this.gyrUsingBias) {
            gx -= this.gyrBias[0]
            gy -= this.gyrBias[1]
            gz -= this.gyrBias[2]
        }
        this.gyrXYZ = [gx, gy, gz]
        trace(1, "Gyro:", "NNN", "X", this.accBias[0], "Y", this.accBias[1], "Z", this.accBias[2])


    }

    // reported by DMP alongside every gyro reading (?)
    unpackGyrBias(bytes: Buffer) {
        let [bx, by, bz] = bytes.toArray(NumberFormat.Int16BE)
        this.gyrRaw = [bx, by, bz] // just for debug...
        bx *= this.gyrScale //adjust result with sensitivity
        by *= this.gyrScale
        bz *= this.gyrScale
        this.gyrBias = [bx, by, bz]
        trace(1, "Gyro Bias:", "NNN", "X", this.gyrBias[0], "Y", this.gyrBias[1], "Z", this.gyrBias[2])

    }

    unpackMag(bytes: Buffer) {
        // TODO: check if these are little-endian?
        let [mx, my, mz] = bytes.toArray(NumberFormat.Int16BE)
        this.magRaw = [mx, my, mz] // just for debug...
        mx *= this.magScale
        my *= this.magScale
        mz *= this.magScale
        if (this.magUsingBias) {
            mx -= this.magBias[0]
            my -= this.magBias[1]
            mz -= this.magBias[2]
        }
        // 
        // To make AK09916 match ICM20948, we must always negate the Y & Z axes
        this.magXYZ = [mx, -my, -mz]
        trace(8, "FIFO Compass", "NNN", "X", mx, "Y", -my, "Z", -mz)

    }

    //=================== Below are all DMP related functions ==============================

    //Load the DMP's firmware to the Chipset (from literal data Buffer: dmpImg[])
    DMPLoadFirmware(burstmode = true) {
        let bank = 0 // DMP memory bank
        let startAddress = DMP_LOAD_START
        let dataPos = 0
        let allImg = dmpImg.length
        let remaining = allImg

        // Write DMP firmware to memory
        while (dataPos < remaining) {
            let writeLen = Math.min((DMP_MEM_BANK_SIZE - startAddress), remaining)
            let chunk = dmpImg.slice(dataPos, dataPos + writeLen) // create a chunk[]
            let address = startAddress
            this.dmpBankSelect(bank)

            //Write firmware byte-by-byte (Original but slow)
            if (!burstmode) {
                for (let i = 0; i < allImg; i++) {
                    let d = chunk[i]
                    this.icmWrite(0, ICM_MEM_START_ADDR, address)
                    this.icmWrite(0, ICM_MEM_R_W, d)
                    address += 1
                }
            } else {
                //Write firmware in burst mode (up to 256 byte at a time : Damn fast)
                this.icmWrite(0, ICM_MEM_START_ADDR, address)
                this.icmWriteBuffer(0, ICM_MEM_R_W, chunk)
            }

            dataPos += writeLen
            bank += 1
            startAddress = 0
            let percent = 100 * dataPos / allImg
            trace(1, "Installing DMP Firmware (" + percent + "%)")
        }

        trace(1, "DMP Firmware Upload Successful !")
    }

    /* (New version reads firmware from a file)
    DMPLoadFirmware(burstmode = true) {
    
        let memBank = 0
        let startAddress = DMP_LOAD_START
        let dataPos = 0
    
        with open(DMP_ROM, 'rb') as f {
            DMPimg = f.read()	
        }
        
    
        // Write DMP firmware to memory
        while (dataPos < len(DMPimg)) {
            let writeLen = min((DMP_MEM_BANK_SIZE - startAddress, len(DMPimg[dataPos:])))
            let data = DMPimg[dataPos: dataPos + writeLen]
            let address = startAddress
            this.DMPBankSelect(memBank)
    
            //Write firmware Byte per byte (Original but slow)
            if (not burstmode) {
                for (d in data) {
                    write(0,  ICM_MEM_START_ADDR,  address)
                    write(0,  ICM_MEM_R_W,  d)
                    address += 1
                }
            } else {
                //Write firmware in burst mode (up to 256 byte at a time : Damn fast)
                write(0,  ICM_MEM_START_ADDR,  address)
                this.writeBuffer(0,  ICM_MEM_R_W,  bytes(data))
            }
    
            dataPos += writeLen
            memBank += 1
            startAddress = 0
            text = "\rDBG:\t ICM20948 : \t Uploading DMP Microcode {:.0f}%".format(100 * dataPos / len(DMPimg))
            print(text, end = "\r")
        }
        this.dbg(1, "DMP Firmware Upload Successfull !")
    }
    */


    // ==========================================================
    //Configure Digital Motion Processor
    // ==========================================================

    /* First off, 
    */
    dmpConfig() {

        //Enable (En = true) SLV0 to read (RnW = true and nothing in DO) 10 byte AK09916 RSV2 register
        //Reserved data is in Big Indian so let's swap byte (Swp true)
        //Data are in group of 2 bytes (Grp = true)
        //    slaveConfig(sl, addr,       reg,     len, RnW, En,   Swp,  Dis,   Grp, DO)
        this.slaveConfig(0, AK_I2C_ADDR, AK_RSV2, 10, true, true, true, false, true)

        //Enable (En = true) SLV1 to write (RnW = false) 1 byte to AK09916 CTLN2 register
        //telling to run Single Mode (DO = AK_CNTL2_MODE_SINGLE)
        //No need of grouped, big indian or whater,
        //    slaveConfig(sl, addr,       reg,      len, RnW,   En,   Swp,   Dis,   Grp,   DO)
        this.slaveConfig(1, AK_I2C_ADDR, AK_CNTL2, 1, false, true, false, false, false, AK_CNTL2_MODE_SINGLE)

        //Configure ODR to 68,75 Hz = 1100/2**4
        this.icmWrite(3, ICM_I2C_MST_ODR_CONFIG, 0x04)

        //Place in LP Mode Only I2CMaster and be sure ACC and GYRO are not in LP Mode
        //this.regConfig(0, ICM_LP_CFG, ICM_LP_CFG_MST, true)
        this.icmAdjustFlags(0, ICM_LP_CFG, 0, ICM_LP_CFG_MST)
        //this.regConfig(0, ICM_LP_CFG, ICM_LP_CFG_ACC | ICM_LP_CFG_GYRO, false)
        this.icmAdjustFlags(0, ICM_LP_CFG, ICM_LP_CFG_ACC | ICM_LP_CFG_GYRO, 0)

        //Disable DMP and FIFO
        //this.regConfig(0, ICM_USER_CTRL, ICM_USER_CTRL_DMP_EN | ICM_USER_CTRL_FIFO_EN, false)
        this.icmAdjustFlags(0, ICM_USER_CTRL, ICM_USER_CTRL_DMP_EN | ICM_USER_CTRL_FIFO_EN, 0)

        //Set Gyro full scale range 2000 dps
        //this.setGyroFullScale(2000) // Global setup below
        //--now done after DMP setup

        //Set Acc full scale range 4g
        //this.setAccFullScale(4)  // Global setup below
        //--now done after DMP setup

        //Enable Gyro DLPF
        this.setGyroLowPass(true, 0)

        //Turn Off whatever would be configured as FIFO
        this.icmWrite(0, ICM_FIFO_EN_1, 0x00)
        this.icmWrite(0, ICM_FIFO_EN_2, 0x00)

        //Reset FIFO
        this.reset_FIFO()

        //Turn Off data ready interrupt
        //this.regConfig(0, ICM_INT_ENABLE_1, 0x1, false)
        this.icmAdjustFlags(0, ICM_INT_ENABLE_1, 0x1, 0)

        //Upload DMP firmware
        this.DMPLoadFirmware()

        //Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
        this.buffer = pins.createBuffer(2)
        this.buffer[0] = DMP_START_ADDRESS >> 8
        this.buffer[1] = DMP_START_ADDRESS & 0xff
        this.icmWriteBuffer(2, ICM_PRGM_START_ADDRH, this.buffer)

        //Set the Hardware Fix Disable register to 0x48
        this.icmWrite(0, ICM_HW_FIX_DISABLE, 0x48)

        //Set the Single FIFO Priority Select register to 0xE4
        this.icmWrite(0, ICM_SINGLE_FIFO_PRIORITY_SEL, 0xE4)

        //Configure Acceleration and Gyroscope : Ranges, Samples Rates and DMP scaling factors 
        this.setAccFullScale(4)
        this.setGyroFullScale(2000)
        this.setAccSampleRate(56.25)
        this.setGyroSampleRate(56.25)
        this.DMPGyroScaling(56.25, 2000)
        this.DMPAccScaling(56.25)

        //Configure Compass Mount Matrix
        //As explained at the top, Matrix will be
        //  [[ 1  0  0]  =  [[ 00 01 02]
        //   [ 0 -1  0]      [ 10 11 12]
        //   [ 0  0 -1]]     [ 20 21 22]]
        // 1 value needs to be scaled from hardware unit to uT
        //Inside DMP, AK9916 output 16bit signed (+/- 32752 corresponding to +/-4912uT
        //1 unit = 0.15 uT
        //Max 2^30 * 0.15 = 161061273 = 0x9999999
        //-0x9999999 = 0xF6666667
        //DMP Compass Output will be in uT
        //let DMP_COMPAS_MOUNT_MATRIX_SCALED_ZERO = [0x00, 0x00, 0x00, 0x00]
        //let DMP_COMPAS_MOUNT_MATRIX_SCALED_PLUS1 = [0x09, 0x99, 0x99, 0x99]
        //let DMP_COMPAS_MOUNT_MATRIX_SCALED_MINUS1 = [0xF6, 0x66, 0x66, 0x67]

        let scaledZero = pins.createBuffer(4)
        let scaledPlus1 = pins.createBuffer(4)
        let scaledMinus1 = pins.createBuffer(4)
        scaledZero.fill(0, 0, 4)
        scaledPlus1.setNumber(NumberFormat.UInt32BE, 0, 0x09999999)
        scaledMinus1.setNumber(NumberFormat.UInt32BE, 0, 0xF6666667)
        this.dmpWriteBuffer(DMP_CPASS_MTX_00, scaledPlus1)
        this.dmpWriteBuffer(DMP_CPASS_MTX_01, scaledZero)
        this.dmpWriteBuffer(DMP_CPASS_MTX_02, scaledZero)
        this.dmpWriteBuffer(DMP_CPASS_MTX_10, scaledZero)
        this.dmpWriteBuffer(DMP_CPASS_MTX_11, scaledMinus1)
        this.dmpWriteBuffer(DMP_CPASS_MTX_12, scaledZero)
        this.dmpWriteBuffer(DMP_CPASS_MTX_20, scaledZero)
        this.dmpWriteBuffer(DMP_CPASS_MTX_21, scaledZero)
        this.dmpWriteBuffer(DMP_CPASS_MTX_22, scaledMinus1)

        //Configure B2S Mounting Matrix
        // Values taken from InvenSense Nucleo Example (thanks Sparkfun)
        //let DMP_B2S_MOUNT_MATRIX_SCALED_PLUS1 = [0x40, 0x00, 0x00, 0x00]
        scaledPlus1.setNumber(NumberFormat.UInt32BE, 0, 0x40000000)

        this.dmpWriteBuffer(DMP_B2S_MTX_00, scaledPlus1)
        this.dmpWriteBuffer(DMP_B2S_MTX_01, scaledZero)
        this.dmpWriteBuffer(DMP_B2S_MTX_02, scaledZero)
        this.dmpWriteBuffer(DMP_B2S_MTX_10, scaledZero)
        this.dmpWriteBuffer(DMP_B2S_MTX_11, scaledPlus1)
        this.dmpWriteBuffer(DMP_B2S_MTX_12, scaledZero)
        this.dmpWriteBuffer(DMP_B2S_MTX_20, scaledZero)
        this.dmpWriteBuffer(DMP_B2S_MTX_21, scaledZero)
        this.dmpWriteBuffer(DMP_B2S_MTX_22, scaledPlus1)


        //Configure the Compass Time Buffer
        //The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
        //Let's set the Compass Time Buffer to 69 (Hz) = 0x45.
        //let DMP_CPASS_TIME_BUFFER_FACTOR = [0x00, 0x45]
        this.twoByte[0] = 0x00
        this.twoByte[1] = 0x45
        this.dmpWriteBuffer(DMP_CPASS_TIME_BUFFER, this.twoByte)

        //If needed, can set DMP Output Data Rate ODR = (DMP running rate / ODR) - 1
        // Here 0 means Compass is running same speed than DMP (1 would be half speed)
        this.dmpOutputDataRate(DMP_ODR_CPASS, 0)
        //this.DMPwrite(DMP_ODR_CNTR_CPASS_CALIBR, 0) //?not needed?
    }

    //=========================== FIFO Management ===============================

    // The FIFO is used in both MODE.CYCLED and MODE.AUTO.


    /** Read count of bytes available in the FIFO */
    fifoCount(): number {
        let twoByte = this.icmReadBuffer(0, ICM_FIFO_COUNTH, 2)
        let count = ((twoByte[0] & 0x1F) << 8) | twoByte[1] // (top 3 bits reserved)
        trace(4, "FIFO holds", "N", "value", count)
        return count
    }

    /** return TRUE if <needed> bytes sucessfully read from FIFO into this.data Buffer, else return FALSE */
    // for speed, we always overwrite from the beginning of this.data Buffer 
    gotFifoData(needed: number): boolean {
        let ok = false
        this.data = Buffer.create(0)
        if (this.fifoCount() >= needed) {
            // We can read out at most 32 bytes at a time:
            let off = 0
            while (off < needed) {
                let more = Math.min((needed - off), 32)
                this.data.write(off, this.icmReadBuffer(0, ICM_FIFO_R_W, more))
                off += more
            }
            /*
                while (need > 0)
                {
                   
                    size_t need2 = std::min(need, size_t(32));
                    size_t got = getDMPsome(blocking, need2);
                    if (got < need2) return d;
                    need -= got;
                }

            */
            this.data = this.icmReadBuffer(0, ICM_FIFO_R_W, needed)
            trace(4, "FIFO fetched", "N", "value", needed)
            ok = true
        }
        return ok
    }



    /** IN DMP mode, check next FIFO packet header matches expected content flags */
    DMPAsExpected(): boolean {
        return ((this.dmpHeader1 == this.dmpChoice1) && (this.dmpHeader2 == this.dmpChoice2))
    }

    /** For speed, use Content flags to compute the expected packet-size for future DMP FIFO packets */
    computeDmpPacketSize() {
        /* A packet comprises an assembly of reports of diverse lengths enclosed by
        a Header and a Footer.
        Contributions for enabled reports always appear in the packet in a set order.
        Their expected presence/absence is governed by which flags are set in dmpDataOutCtl1 & dmpDataOutCtl2
        (which have already been sent to the DMP as DMP_DATA_OUT_CTL1, DMP_DATA_OUT_CTL2)
        As packets are read from the FIFO, their headers are checked against expectations.
        Any mismatch is BAD NEWS!
    	
        ***TODO: Check if setting DMP_DO_Ctrl_2_Batch_Mode_Enable affects the packet-size***
    	
        */
        let count = DMP_Header_Bytes;

        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Accel) count += DMP_Raw_Accel_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Gyro) count += DMP_Raw_Gyro_Bytes + DMP_Gyro_Bias_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Compass) count += DMP_Compass_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_ALS) count += DMP_ALS_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Quat6) count += DMP_Quat6_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Quat9) count += DMP_Quat9_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Pedom_Quat6) count += DMP_Ped_Quat6_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Geomag) count += DMP_Geomag_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Pressure) count += DMP_Pressure_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Gyro_Calibr) count += DMP_Gyro_Calibr_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Compass_Calibr) count += DMP_Compass_Calibr_Bytes;
        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Step_TimeStamp) count += DMP_Step_TimeStamp_Bytes;

        if (this.dmpChoice1 & DMP_DO_Ctrl_1_Header2) count += DMP_Header2_Bytes;

        if (this.dmpChoice2 & DMP_DO_Ctrl_2_Accel_Accuracy) count += DMP_Accel_Accuracy_Bytes;
        if (this.dmpChoice2 & DMP_DO_Ctrl_2_Gyro_Accuracy) count += DMP_Gyro_Accuracy_Bytes;
        if (this.dmpChoice2 & DMP_DO_Ctrl_2_Compass_Accuracy) count += DMP_Compass_Accuracy_Bytes;
        if (this.dmpChoice2 & DMP_DO_Ctrl_2_Fsync) count += DMP_Fsync_Detection_Bytes;
        if (this.dmpChoice2 & DMP_DO_Ctrl_2_Pickup) count += DMP_Pickup_Bytes;
        //if (this.DMPcontent2 & DMP_DO_Ctrl_2_Batch_Mode_Enable) count += DMP_DO_Ctrl_2_ODR_CNT_GYRO_Bytes; // ??? is this correct?
        if (this.dmpChoice2 & DMP_DO_Ctrl_2_Activity_Recog) count += DMP_Activity_Recognition_Bytes;
        if (this.dmpChoice2 & DMP_DO_Ctrl_2_Secondary_On_Off) count += DMP_Secondary_On_Off_Bytes;

        count += DMP_Footer_Bytes;

        this.dmpPacketSize = count
    }


    // ####################################################################################################
    //================= DMP UPDATE ===========================
    // 1. Expected packet structure is given by Content flags enabled
    // 2. Expected Packet-size is sum of enabled report contributions
    // 3. Only proceed once this many bytes are available in FIFO
    // 4. Sanity check that next packet Header bits match Content flags
    // 5. Unpack all reports in order of set Header bits

    // Shares data-unpackers for raw sensor readings with the other MODEs


    dmpUpdate() {
        let off = 0
        if (this.gotFifoData(this.dmpPacketSize)) {
            this.dmpHeader1 = this.data.getNumber(NumberFormat.Int16BE, off)
            trace(4, "Header 1:", "P", "New packet", this.dmpHeader1)
            off += 2
            this.dmpHeader2 = 0	//Read possible Header2
            if ((this.dmpHeader1 & DMP_DO_Ctrl_1_Header2) != 0) {
                this.dmpHeader2 = this.data.getNumber(NumberFormat.Int16BE, off)
                trace(4, "Header 2:", "P", "New packet", this.dmpHeader2)
            }
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Accel) {
            this.unpackAcc(this.data.slice(off, DMP_Raw_Accel_Bytes))
            off += DMP_Raw_Accel_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Gyro) {
            this.unpackGyr(this.data.slice(off, DMP_Raw_Gyro_Bytes))
            off += DMP_Raw_Accel_Bytes;
            // this will always be followed by the bias data
            this.unpackGyrBias(this.data.slice(off, DMP_Gyro_Bias_Bytes))
            off += DMP_Raw_Accel_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Compass) {
            this.unpackMag(this.data.slice(off, DMP_Compass_Bytes))
            off += DMP_Compass_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_ALS) {
            // Not supported by InvenSense?
            off += DMP_ALS_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Quat6) {
            //quat6[0] = (packet[off + 0] << 24) | (packet[off + 1] << 16) | (packet[off + 2] << 8) | packet[off + 3];
            //quat6[1] = (packet[off + 4] << 24) | (packet[off + 5] << 16) | (packet[off + 6] << 8) | packet[off + 7];
            //quat6[2] = (packet[off + 8] << 24) | (packet[off + 9] << 16) | (packet[off + 10] << 8) | packet[off + 11];
            this.unpackQuat6(this.data.slice(off, DMP_Quat6_Bytes))
            off += DMP_Quat6_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Quat9) {
            //quat9[0] = (packet[off + 0] << 24) | (packet[off + 1] << 16) | (packet[off + 2] << 8) | packet[off + 3];
            //quat9[1] = (packet[off + 4] << 24) | (packet[off + 5] << 16) | (packet[off + 6] << 8) | packet[off + 7];
            //quat9[2] = (packet[off + 8] << 24) | (packet[off + 9] << 16) | (packet[off + 10] << 8) | packet[off + 11];
            //quat9acc = (packet[off + 12] << 24) | (packet[off + 13] << 16);
            this.unpackQuat9(this.data.slice(off, DMP_Quat9_Bytes))
            off += DMP_Quat9_Bytes;
        }


        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Pedom_Quat6) {
            //pquat6[0] = (packet[off + 0] << 24) | (packet[off + 1] << 16) | (packet[off + 2] << 8) | packet[off + 3];
            //pquat6[1] = (packet[off + 4] << 24) | (packet[off + 5] << 16) | (packet[off + 6] << 8) | packet[off + 7];
            //pquat6[2] = (packet[off + 8] << 24) | (packet[off + 9] << 16) | (packet[off + 10] << 8) | packet[off + 11];
            this.unpackPedQuat6(this.data.slice(off, DMP_Ped_Quat6_Bytes))
            off += DMP_Ped_Quat6_Bytes;
        }


        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Geomag) {
            //geomag[0] = (packet[off + 0] << 24) | (packet[off + 1] << 16) | (packet[off + 2] << 8) | packet[off + 3];
            //geomag[1] = (packet[off + 4] << 24) | (packet[off + 5] << 16) | (packet[off + 6] << 8) | packet[off + 7];
            //geomag[2] = (packet[off + 8] << 24) | (packet[off + 9] << 16) | (packet[off + 10] << 8) | packet[off + 11];
            //geomagacc = (packet[off + 12] << 24) | (packet[off + 13] << 16);
            this.unpackGeomag(this.data.slice(off, DMP_Geomag_Bytes))
            off += DMP_Geomag_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Pressure) {
            // Not supported by InvenSense?
            off += DMP_Pressure_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Gyro_Calibr) {
            //gyrobias[0] = (packet[off + 0] << 8) | packet[off + 1];
            //gyrobias[1] = (packet[off + 2] << 8) | packet[off + 3];
            //gyrobias[2] = (packet[off + 4] << 8) | packet[off + 5];
            this.unpackGyrCalib(this.data.slice(off, DMP_Gyro_Calibr_Bytes))
            off += DMP_Gyro_Calibr_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Compass_Calibr) {
            //cpasscal[0] = (packet[off + 0] << 24) | (packet[off + 1] << 16) | (packet[off + 2] << 8) | packet[off + 3];
            //cpasscal[1] = (packet[off + 4] << 24) | (packet[off + 5] << 16) | (packet[off + 6] << 8) | packet[off + 7];
            //cpasscal[2] = (packet[off + 8] << 24) | (packet[off + 9] << 16) | (packet[off + 10] << 8) | packet[off + 11];
            this.unpackMagCalib(this.data.slice(off, DMP_Compass_Calibr_Bytes))
            off += DMP_Compass_Calibr_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_1_Step_TimeStamp) {
            //stepts = (packet[off + 0] << 24) | (packet[off + 1] << 16) | (packet[off + 2] << 8) | packet[off + 3];
            this.stepStamp = this.data.getNumber(NumberFormat.Int32BE, off)
            this.steps = this.dmpHeader1 & DMP_DO_Ctrl_1_Steps; // a 3-bit count of extra steps detected

            // FIXME: docs say we should get given the number of steps, but it is always 0...
            if (this.steps == 0) this.steps = 1;

            off += DMP_Step_TimeStamp_Bytes;
        }

        if (this.dmpHeader2 & DMP_DO_Ctrl_2_Accel_Accuracy) {
            //accelacc = (packet[off + 0] << 8) | packet[off + 1];
            this.accQual = this.data.getNumber(NumberFormat.Int16BE, off)
            off += DMP_Accel_Accuracy_Bytes;
        }

        if (this.dmpHeader2 & DMP_DO_Ctrl_2_Gyro_Accuracy) {
            //gyroacc = (packet[off + 0] << 8) | packet[off + 1];
            this.gyrQual = this.data.getNumber(NumberFormat.Int16BE, off)
            off += DMP_Gyro_Accuracy_Bytes;
        }

        if (this.dmpHeader2 & DMP_DO_Ctrl_2_Compass_Accuracy) {
            //cpassacc = (packet[off + 0] << 8) | packet[off + 1];
            this.magQual = this.data.getNumber(NumberFormat.Int16BE, off)
            off += DMP_Compass_Accuracy_Bytes;
        }

        if (this.dmpHeader2 & DMP_DO_Ctrl_2_Fsync) {
            //fsync = (packet[off + 0] << 8) | packet[off + 1];
            this.fsync = this.data.getNumber(NumberFormat.Int16BE, off)
            off += DMP_Fsync_Detection_Bytes;
        }

        if (this.dmpHeader2 & DMP_DO_Ctrl_2_Pickup) {
            //pickup = (packet[off + 0] << 8) | packet[off + 1];
            this.pickup = this.data.getNumber(NumberFormat.Int16BE, off)
            off += DMP_Pickup_Bytes;
        }

        if (this.dmpHeader2 & DMP_DO_Ctrl_2_Activity_Recog) {
            //bacstate = (packet[off + 0] << 8) | packet[off + 1];
            this.bacState = this.data.getNumber(NumberFormat.Int16BE, off)
            //bacts = (packet[off + 2] << 24) | (packet[off + 3] << 16) | (packet[off + 4] << 8) | packet[off + 5];
            this.bacTs = this.data.getNumber(NumberFormat.Int32BE, off)
            off += DMP_Activity_Recognition_Bytes;
        }

        if (this.dmpHeader1 & DMP_DO_Ctrl_2_Secondary_On_Off) {
            // *** Unclear how these 2-bytes are mapped
            off += DMP_Secondary_On_Off_Bytes;
        }
        // Output data rate counter (always here but unclear what it is)
        // odrcnt = (packet[off + 0] << 8) | packet[off + 1];
        this.odrCount = this.data.getNumber(NumberFormat.Int32BE, off)
        off += DMP_Footer_Bytes;

        trace(4, "Decoded " + off + " bytes from " + this.dmpPacketSize + " bytes of IMU packet")

    } // else try again later...




    /* MAIN ENTRY_POINT TO READ LATEST SENSOR DATA FROM FIFO AND UPDATE PROPERTIES
    
    Checks what (if any) data is available from the FIFO and processes it.
    
    In DMP Mode, data is delivered in "packets". A control bit-mask defines the 
    shopping-basket of required "reports". This gets reflected in each packet-header,
    telling us what groups of data will follow in the body of the packet.
    
    
    A call to update() can start reading and processing a packet as soon as its 
    header is detected, but may run out of data from the FIFO if the DMP has yet 
    to complete all reports in the packet. Rather than blocking to wait for more data,
    update() will abandon further processing and return.
    
    Analysis of an abandoned, incomplete packet will then be resumed on a subsequent call.
    
    DMPFifoQueue1 and DMPFifoQueue2 copy the Header1 and Header2 of a new packet.
    As each contributing report is read, its bit is removed from the queue bit-masks.
    When both are zero, the current packet is complete, and the next is expected.
    
   
*/

    // unpackers for other datasets queued in the FIFO only by the DMP...
    unpackQuat6(bytes: Buffer) {
        //let q1, q2, q3 = unpackFrom(">3l", this.data)
        let [q1, q2, q3] = bytes.toArray(NumberFormat.Int32BE)
        q1 /= (2 ** 30)  // The quaternion data is scaled by 2^30.
        q2 /= (2 ** 30)
        q3 /= (2 ** 30)
        this.quat6 = [q1, q2, q3]
        //To do process
        trace(1, "Quaternion-6:", "NNN", "X", this.quat6[0], "Y", this.quat6[1], "Z", this.quat6[2])
        //trace(8, q2, "FIFO Quaternion_6\tq1 {:.4f}\tq2 {:.4f}\tq3 {:.4f}".format(q1, q3))
    }

    unpackQuat9(bytes: Buffer) {
        //let [q1, q2, q3, acc] = unpackFrom(">3lh", this.data)
        let [q1, q2, q3] = bytes.slice(0, 12).toArray(NumberFormat.Int32BE)
        q1 /= (2 ** 30)  // The quaternion data is scaled by 2^30.
        q2 /= (2 ** 30)
        q3 /= (2 ** 30)
        this.quat9 = [q1, q2, q3]

        let acc = bytes.getNumber(NumberFormat.UInt16BE, 12)
        acc /= (2 ** 16)
        this.quat9Qual = acc
        trace(1, "Quaternion-9:", "NNNN", "X", this.quat9[0], "Y", this.quat9[1], "Z", this.quat9[2], "Accuracy", this.quat9Qual)
        //trace(8, q2, "FIFO Quaternion_9\tq1 {:.4f}\tq2 {:.4f}\tq3 {:.4f}\taccuracy {:.4f}".format(q1, q3, acc))
    }
    unpackPedQuat6(bytes: Buffer) {
        //let q1, q2, q3 = unpackFrom(">3h", this.data)
        let [q1, q2, q3] = bytes.toArray(NumberFormat.Int32BE)
        q1 /= (2 ** 30)  // The quaternion data is scaled by 2^30.
        q2 /= (2 ** 30)
        q3 /= (2 ** 30)
        this.pquat6 = [q1, q2, q3]
        //To do: process
        trace(1, "Pedometer Quaternion-6", "NNN", "X", this.pquat6[0], "Y", this.pquat6[1], "Z", this.pquat6[2])
        //trace(8, q2, "FIFO PQuaternion_6\tq1 {:.4f}\tq2 {:.4f}\tq3 {:.4f}".format(q1, q3))
    }

    unpackGeomag(bytes: Buffer) {
        let [q1, q2, q3] = bytes.slice(0, 12).toArray(NumberFormat.Int32BE)
        q1 /= (2 ** 30)
        q2 /= (2 ** 30)
        q3 /= (2 ** 30)
        this.gquat6 = [q1, q2, q3]

        let acc = bytes.getNumber(NumberFormat.UInt16BE, 12)
        acc /= 2 ** 16
        this.gquat6Qual = acc
        trace(1, "Magnetometer Quaternion-6:", "NNNN", "X", this.gquat6[0], "Y", this.gquat6[1], "Z", this.gquat6[2], "Accuracy", this.gquat6Qual)

        //trace(8, q2, "FIFO Geomag\tq1 {:.4f}\tq2 {:.4f}\tq3 {:.4f}\taccuracy {:.4f}".format(q1, q3, acc))
    }

    unpackGyrCalib(bytes: Buffer) {
        //let gxCal, gyCal, gzCal = unpackFrom(">3l", this.data)
        let [gxCal, gyCal, gzCal] = bytes.toArray(NumberFormat.Int16BE)
        gxCal /= (2 ** 30)
        gyCal /= (2 ** 30)
        gzCal /= (2 ** 30)
        this.gyrXYZCal = [gxCal, gyCal, gzCal]
        //trace(8, gyCal, "FIFO Gyro Calibration\tgxc {:.4f}\tgyc {:.4f}\tgzc {:.4f}".format(gxCal, gzCal))

        trace(1, "Gyro Calibration:", "NNN", "X", this.gyrXYZCal[0], "Y", this.gyrXYZCal[1], "Z", this.gyrXYZCal[2])

    }

    unpackMagCalib(bytes: Buffer) {
        //let mxCal, myCal, mzCal = unpackFrom(">3l", this.data)
        let [mxCal, myCal, mzCal] = bytes.toArray(NumberFormat.Int16BE)
        mxCal /= (2 ** 30)
        myCal /= (2 ** 30)
        mzCal /= (2 ** 30)
        this.magXYZCal = [mxCal, myCal, mzCal]
        //trace(8, myCal, "FICO Compass Calibration\tmxc {:.4f}\tmyc {:.4f}\tmzc {:.4f}".format(mxCal, mzCal))

        trace(1, "Magnetometer Calibration:", "NNN", "X", this.magXYZCal[0], "Y", this.magXYZCal[1], "Z", this.magXYZCal[2])
    }


    //Set DMP Output Data Rate for a particular sensor
    dmpOutputDataRate(DMPordSensor: number, interval: number) {
        // Wake ICM from low-power mode
        //this.regConfig(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_SLEEP, enable = false)
        //this.regConfig(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, enable = false)
        this.icmAdjustFlags(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_SLEEP, 0)
        this.icmAdjustFlags(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, 0)

        this.twoByte.setNumber(NumberFormat.UInt16BE, 0, interval)
        this.dmpWriteBuffer(DMPordSensor, this.twoByte)
        //Write 0 to related counter (they always live 32 bytes earlier in DMP mem)
        this.twoByte.fill(0)
        this.dmpWriteBuffer(DMPordSensor - 0x20, this.twoByte)
        //Set LP again
        //this.regConfig(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, enable = true)
        this.icmAdjustFlags(0, ICM_PWR_MGMT_1, 0, ICM_PWR_MGMT_1_LP)
    }

    /**********
     * Choosing Content information.
     * In simple FIFO mode, only raw sensor content can be placed in the FIFO.
     * When DMP is in use, a much richer set of processed reports are on offer.
     * 
     */

    // ================================================================
    // (de)Activate specific DMP sensor/report 
    // ================================================================
    /* icmSensor is the (string) name of the sensor/report to be switched on/off.

        Any change requires reconstruction of the DMP packet controls
        {DATA_OUT_CTL1, DATA_OUT_CTL2, DATA_RDY_STATUS, INV_EVENT_CTRL}
        which are then sent down to the chip.
        
        The FIFO then needs to be reset, and the new packet-size computed for future use.
        
        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    
    */


    DMPEnableSensor(icmSensor: string, enable = true) {
        // Convert the named ICMSensor to equivalent "Android" Sensor number
        let androidSensor = DMP_SENSORS_2_ANDROID[DMP_SENSORS[icmSensor]]
        // Get its Control Bits
        let androidCtlBits = ANDROID_SENSORS_CTRL_BITS[androidSensor]
        trace(4, "Activation of ICM sensor", "NP", icmSensor, androidSensor, "Android CTRL Bits", androidCtlBits)
        // set/unset this sensor's bit, living in either AndroidSensorBitmask_0 or AndroidSensorBitmask_1
        if (androidSensor < 32) {
            let Bitmask = 0x1 << androidSensor
            if (enable) {
                this.androidSensorBitmask_0 |= Bitmask
            } else {
                this.androidSensorBitmask_0 &= ~Bitmask
            }
        } else {
            let Bitmask = 0x1 << (androidSensor - 32)
            if (enable) {
                this.androidSensorBitmask_1 |= Bitmask
            } else {
                this.androidSensorBitmask_1 &= ~Bitmask
            }
        }

        trace(4, "New Android Sensor BitMask:", "QQ", "Mask_0", this.androidSensorBitmask_0, "Mask_1", this.androidSensorBitmask_1)

        // Reconstruct DATA_OUT_CTL1 from AndroidSensorBitmask_0 & 1
        let flags1 = 0
        let flags2 = 0
        let DataRdyStatus = 0
        let InvEventCtl = 0
        for (let x = 0; x < 32; x++) {
            let Bitmask = 0x1 << x
            if ((this.androidSensorBitmask_0 & Bitmask) > 0) {
                flags1 |= ANDROID_SENSORS_CTRL_BITS[x]
            }
            if ((this.androidSensorBitmask_1 & Bitmask) > 0) {
                flags1 |= ANDROID_SENSORS_CTRL_BITS[x + 32]
            }

            //Reconstruct DATA_RDY_STATUS and INV_EVENT_CTRL
            //Case Acceleration
            if (((this.androidSensorBitmask_0 & Bitmask & INV_NEEDS_ACCEL_MASK0)
                | (this.androidSensorBitmask_1 & Bitmask & INV_NEEDS_ACCEL_MASK1)) > 0) {
                DataRdyStatus |= DMP_Data_Ready_Accel
                InvEventCtl |= DMP_Motion_Event_Control_Accel_Calibr
            }
            //Case Gyro
            if (((this.androidSensorBitmask_0 & Bitmask & INV_NEEDS_GYRO_MASK0)
                | (this.androidSensorBitmask_1 & Bitmask & INV_NEEDS_GYRO_MASK1)) > 0) {
                DataRdyStatus |= DMP_Data_Ready_Gyro
                InvEventCtl |= DMP_Motion_Event_Control_Gyro_Calibr
            }
            //Case Compass
            if (((this.androidSensorBitmask_0 & Bitmask & INV_NEEDS_COMPAS_MASK0)
                | (this.androidSensorBitmask_1 & Bitmask & INV_NEEDS_COMPAS_MASK1)) > 0) {
                DataRdyStatus |= DMP_Data_Ready_Secondary_Compass
                InvEventCtl |= DMP_Motion_Event_Control_Compass_Calibr
            }

            //Reconstruct DATA_OUT_CTL2
            if ((flags1 & DMP_DO_Ctrl_1_Accel) > 0) {
                flags2 |= DMP_DO_Ctrl_2_Accel_Accuracy
            }
            if (((flags1 & DMP_DO_Ctrl_1_Gyro)
                | (flags1 & DMP_DO_Ctrl_1_Gyro_Calibr)) > 0) {
                flags2 |= DMP_DO_Ctrl_2_Gyro_Accuracy
            }
            if (((flags1 & DMP_DO_Ctrl_1_Compass)
                | (flags1 & DMP_DO_Ctrl_1_Compass_Calibr)
                | (flags1 & DMP_DO_Ctrl_1_Quat9)
                | (flags1 & DMP_DO_Ctrl_1_Geomag)) > 0) {
                flags2 |= DMP_DO_Ctrl_2_Compass_Accuracy
            }

            //Reconstruc INV_EVENT_CTRL
            if ((flags1 & DMP_DO_Ctrl_1_Quat9) > 0) {
                InvEventCtl |= DMP_Motion_Event_Control_9axis
            }
            if ((flags1 & DMP_DO_Ctrl_1_Geomag) > 0) {
                InvEventCtl |= DMP_Motion_Event_Control_Geomag
            }
            if (((flags1 & DMP_DO_Ctrl_1_Step_TimeStamp)
                | (flags1 & DMP_DO_Ctrl_1_Steps)) > 0) {
                InvEventCtl |= DMP_Motion_Event_Control_Pedometer_Interrupt

            }

            // **** store the newly constructed control flags
            this.dmpDataOutCtl1 = flags1
            this.dmpDataOutCtl2 = flags2

            //Make sure the chip is not in Low Power mode nor in Sleep mode
            //this.regConfig(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_SLEEP, enable = false)
            //this.regConfig(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, enable = false)
            this.icmAdjustFlags(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_SLEEP, 0)
            this.icmAdjustFlags(0, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_LP, 0)

            //Write datas
            this.buffer = pins.createBuffer(2)

            //Write DATA_OUT_CTL1
            this.buffer[0] = flags1 >> 8
            this.buffer[1] = flags1 & 0xFF
            trace(2, "set DMP_DATA_OUT_CTRL1", "P", "value:", flags1)
            this.dmpWriteBuffer(DMP_DATA_OUT_CTL1, this.buffer)

            //Write DATA_OUT_CTL2
            this.buffer[0] = flags2 >> 8
            this.buffer[1] = flags2 & 0xFF
            trace(2, "set DMP_DATA_OUT_CTRL2", "P", "value:", hexPair(flags2))
            this.dmpWriteBuffer(DMP_DATA_OUT_CTL2, this.buffer)

            //Write DATA_RDY_STATUS
            this.buffer[0] = DataRdyStatus >> 8
            this.buffer[1] = DataRdyStatus & 0xFF
            trace(2, "set DMP_DATA_RDY_STATUS", "P", "value:", hexPair(DataRdyStatus))
            this.dmpWriteBuffer(DMP_DATA_RDY_STATUS, this.buffer)

            //Write MOTION_EVENT_CTL
            this.buffer[0] = InvEventCtl >> 8
            this.buffer[1] = InvEventCtl & 0xFF
            trace(2, "DMP_DATA_MOTION_EVENT_CTRL", "P", "value:", hexPair(InvEventCtl))
            this.dmpWriteBuffer(DMP_DATA_MOTION_EVENT_CTRL, this.buffer)

            //Enable FIFO and DMP
            //this.regConfig(0, ICM_USER_CTRL, ICM_USER_CTRL_DMP_EN | ICM_USER_CTRL_FIFO_EN, true)
            this.icmAdjustFlags(0, ICM_USER_CTRL, 0, ICM_USER_CTRL_DMP_EN | ICM_USER_CTRL_FIFO_EN)

            //Reset DMP
            //this.regConfig(0, ICM_USER_CTRL, ICM_USER_CTRL_DMP_RST, true)
            this.icmAdjustFlags(0, ICM_USER_CTRL, 0, ICM_USER_CTRL_DMP_RST)

            //Reset FIFO
            this.reset_FIFO()

            // ?needed? Compute the size of new packets to be fetched from the FIFO 
            this.computeDmpPacketSize()
        }
    }

    /** Calculate and set gyro scale factor for rate and range, taking into account
     *  the calibrated % clock inaccuracy of this particular chip.
    */
    DMPGyroScaling(gyroRate = 56.25, gyroRange = 2000) {
        let result = 0
        let div = Math.round((1125.0 / gyroRate) - 1)
        let gyroScale = GYRO_SCALE_RANGE[gyroRange]
        const MagicConstant = 264446880937391
        const MagicConstantScale = 100000

        // Read TimebaseCorrection_PLL register from bank 1
        // (This is a signed byte, so subtract 128)
        let pll = this.icmRead(1, ICM_TIMEBASE_CORRECTION_PLL) - 128
        this.gyroPllVariation = pll
        /* cope with sign of INT8
        if (pll & 0x80) {
            result = MagicConstant * (0x01 << gyroScale) * (1 + div) / (1270 - (pll & 0x7F)) / MagicConstantScale
        } else {
            result = MagicConstant * (0x01 << gyroScale) * (1 + div) / (1270 + pll) / MagicConstantScale
        }
        */
        result = MagicConstant * (0x01 << gyroScale) * (1 + div) / (1270 + pll) / MagicConstantScale
        // new scale-factor is a 32-bit int
        if (result > 0x7FFFFFFF) {
            this.gyroScaleFactor = 0x7FFFFFFF
        } else {
            this.gyroScaleFactor = Math.round(result)
        }

        /*let buffer = pins.createBuffer(4)
        buffer[0] = this.gyroSf >> 24
        buffer[1] = this.gyroSf >> 16
        buffer[2] = this.gyroSf >> 8
        buffer[3] = this.gyroSf & 0xFF
        */
        this.oneInt32.setNumber(NumberFormat.UInt32BE, 0, this.gyroScaleFactor)
        trace(2, "DMP_SET_GYRO_SF PLL:", "BQ", "value: ", pll, "DMP_GYRO_SF", this.gyroScaleFactor)
        this.dmpWriteBuffer(DMP_GYRO_SF, this.oneInt32)

    }

    /** Three internal scaling factors depend on the accRate setting */
    DMPAccScaling(accRate = 56.25) {
        //general case will assume accRate = 56.25
        //DMP_ACC_ONLY_GAIN_FACTOR = [0x03, 0xA4, 0x92, 0x49]
        //DMP_ACCEL_ALPHA_VAR_FACTOR = [0x34, 0x92, 0x49, 0x25]
        //DMP_ACCEL_A_VAR_FACTOR = [0x0B, 0x6D, 0xB6, 0xDB]
        let accOnlyGain = 0x03A49249
        let accelAlphaVar = 0x34924925
        let accelAVar = 0x0B6DB6DB

        if (accRate == 225) {
            //DMP_ACC_ONLY_GAIN_FACTOR = [0x00, 0xE8, 0xBA, 0x2E]
            //DMP_ACCEL_ALPHA_VAR_FACTOR = [0x3D, 0x27, 0xD2, 0x7D]
            //DMP_ACCEL_A_VAR_FACTOR = [0x02, 0xD8, 0x2D, 0x83]
            let accOnlyGain = 0x00E8BA2E
            let accelAlphaVar = 0x3D27D27D
            let accelAVar = 0x02D82D83
        }
        if (accRate == 112.5) {
            //DMP_ACC_ONLY_GAIN_FACTOR = [0x01, 0xD1, 0x74, 0x5D]
            //DMP_ACCEL_ALPHA_VAR_FACTOR = [0x3A, 0x49, 0x24, 0x92]
            //DMP_ACCEL_A_VAR_FACTOR = [0x02, 0xD8, 0x2D, 0x83]
            let accOnlyGain = 0x01D1745D
            let accelAlphaVar = 0x3A492492
            let accelAVar = 0x02D82D83
        }
        // Poke these factors into DMP registers
        this.oneInt32.setNumber(NumberFormat.UInt32BE, 0, accOnlyGain)
        this.dmpWriteBuffer(DMP_ACCEL_ONLY_GAIN, this.oneInt32)
        this.oneInt32.setNumber(NumberFormat.UInt32BE, 0, accelAlphaVar)
        this.dmpWriteBuffer(DMP_ACCEL_ALPHA_VAR, this.oneInt32)
        this.oneInt32.setNumber(NumberFormat.UInt32BE, 0, accelAVar)
        this.dmpWriteBuffer(DMP_ACCEL_A_VAR, this.oneInt32)

        // Configure the Accel Cal Rate
        //let DMP_ACCEL_CAL_RATE_FACTOR = [0x00, 0x00]
        this.twoByte.fill(0)
        this.dmpWriteBuffer(DMP_ACCEL_CAL_RATE, this.twoByte)
    }
    //=======================================================================================

















    /*
    ===================== Register Access functions =============================
    We need to access three separate sets of registers: ICM, MAG & DMP.
    Additional methods support reading of data from the FIFO.
    These methods make use of the MakeCode micro-bit pins.i2c...() functionality
    =============================================================================
    */

    // ICM REGISTERS 

    /** Read a single byte from an ICM register */
    icmRead(bank: number, offset: number) {
        this.icmBankSelect(bank)
        pins.i2cWriteNumber(this.device, offset, NumberFormat.UInt8LE) // select register
        let value = pins.i2cReadNumber(this.device, NumberFormat.UInt8LE, false) // read
        trace(16, "ICM Read", "BBB", "Bank", bank, "Offset", offset, "Value", value)
        return value
    }

    /** Read a sequence of ICM registers into a Buffer of <length> bytes */
    icmReadBuffer(bank: number, offset: number, length: number) {
        this.icmBankSelect(bank)
        let bytes = pins.createBuffer(length)
        pins.i2cWriteNumber(this.device, offset, NumberFormat.UInt8LE) // (the starting register)
        bytes = pins.i2cReadBuffer(this.device, length, false)
        trace(16, "ICM Read Buffer", "BBD", "Bank", bank, "Offset", offset, "Value", bytes)
        return bytes
    }

    /** Write a single byte to an ICM register */
    icmWrite(bank: number, offset: number, value: number) {
        this.icmBankSelect(bank)
        let twoBytes = pins.createBuffer(2)
        twoBytes[0] = offset
        twoBytes[1] = value
        pins.i2cWriteBuffer(this.device, twoBytes, false)
        trace(16, "ICM Write", "BBB", "Bank", bank, "Offset", offset, "Value", value)
    }

    /** Write a multi-byte Buffer to a sequence of ICM registers */
    icmWriteBuffer(bank: number, offset: number, bytes: Buffer) {
        pins.i2cWriteNumber(this.device, offset, NumberFormat.UInt8LE) // (the starting register)
        pins.i2cWriteBuffer(this.device, bytes, false)
        trace(16, "ICM Write Buffer", "BBD", "Bank", bank, "Offset", offset, "Value", bytes)
    }


    /** Modify one or more flags in a register on this I2C device 
     * (to update a multibit field : unsetMask maps all bits; setMask gives new value, aligned)
    */
    icmAdjustFlags(bank: number, offset: number, unsetMask: number, setMask: number) {
        let setting = this.icmRead(bank, offset)
        setting &= (0xff ^ unsetMask) // clear these bits
        setting |= setMask // set these bits
        this.icmWrite(bank, offset, setting)
    }

    // (internal): Switch register bank only when it changes
    icmBankSelect(bank: number) {
        if (!(this.icmBank == bank)) {
            let twoBytes = pins.createBuffer(2)
            twoBytes[0] = ICM_MEM_BANK_SEL
            twoBytes[1] = bank << 4 // align with USER_BANK field
            pins.i2cWriteBuffer(this.device, twoBytes, false)
            this.icmBank = bank
        }
    }


    // MAG REGISTERS (AK09916, accessed as Slave Device)
    /*
    For reading or writing individual single-byte AK09916 registers we use Slave-4.
    For multi-byte reads, we use Slave-0 which can deliver at most 15 bytes of data
    into the Master buffer area in ICM bank 0, starting at ICM_EXT_SLV_SENS_DATA_00.
    */

    /** Read a single byte from an AK09916 register */
    magRead(offset: number): number {

        trace(16, "MAG Read...")
        // We use Slave-4, which delivers just a single byte into ICM_I2C_SLV4_DI
        this.icmWrite(3, ICM_I2C_SLV4_ADDR, ICM_I2C_SLV_ADDR_RNW | AK_I2C_ADDR);
        this.icmWrite(3, ICM_I2C_SLV4_REG, offset)
        this.icmWrite(3, ICM_I2C_SLV4_CTRL, ICM_I2C_SLV_CTRL_SLV_ENABLE);
        this.waitForSlave4()
        let value = this.icmRead(3, ICM_I2C_SLV4_DI)
        trace(16, "...MAG Read completed")
        return value
    }

    /** Write a single byte to an AK09916 register */
    magWrite(register: number, value: number) {
        trace(16, "MAG Write...")
        // We use Slave-4, which sends just a single byte from ICM_I2C_SLV4_DO
        this.icmWrite(3, ICM_I2C_SLV4_ADDR, AK_I2C_ADDR); // (with RNW bit cleared)
        this.icmWrite(3, ICM_I2C_SLV4_REG, register)
        this.icmWrite(3, ICM_I2C_SLV4_DO, value)
        this.icmWrite(3, ICM_I2C_SLV4_CTRL, ICM_I2C_SLV_CTRL_SLV_ENABLE);
        this.waitForSlave4(); // ensure transfer completed
        trace(16, "...MAG Write completed")
    }

    // Wait until the Slave 4 data is ready:
    waitForSlave4() {
        let tooLate = control.millis() + 300
        do {
            pause(10)
            // Poll the I2C Master Status (resets all NACK flags, the SLV4_DONE, and SLV4_NACK)
            let status = this.icmRead(0, ICM_I2C_MST_STATUS)
            if (status & ICM_I2C_MST_STATUS_SLV4_NACK) {
                trace(1, "Failed to communicate with AK09916: NACK")
                return
            }
            if (status & ICM_I2C_MST_STATUS_SLV4_DONE) {
                return // Transaction on slave has completed ok
            }
        }
        while (control.millis() < tooLate)
        trace(1, "Failed to communicate with AK09916: timeout")
    }

    /** Read a sequence of up to 15 AK09916 registers into a Buffer */
    magReadBuffer(register: number, length: number): Buffer {
        trace(16, "MAG Read Buffer...")
        let count = length & 0x0f // max size is limited by 4-bit I2C_SLV0_LENG field
        // We use Slave-0, which delivers data from ICM_EXT_SLV_SENS_DATA_00 onwards
        this.icmWrite(3, ICM_I2C_SLV0_ADDR, ICM_I2C_SLV_ADDR_RNW | AK_I2C_ADDR)
        this.icmWrite(3, ICM_I2C_SLV0_REG, register)
        this.icmWrite(3, ICM_I2C_SLV0_CTRL, ICM_I2C_SLV_CTRL_SLV_ENABLE | count)
        // wat a bit for command to execute...
        pause(10)
        let bytes = this.icmReadBuffer(0, ICM_EXT_SLV_SENS_DATA_00, count)
        trace(16, "...MAG Read Buffer completed")
        return bytes
    }


    // DMP REGISTERS (read/write through ICM_MEM... portal)

    // 	NOTE: DMP register consts have been defined as two-byte <bank:offset>

    /** Read a single byte from a DMP register */
    dmpRead(register: number) {
        let bank = register >> 8
        let offset = register & 0xff
        this.dmpBankSelect(bank)
        this.icmWrite(0, ICM_MEM_START_ADDR, offset)
        let value = this.icmRead(0, ICM_MEM_R_W)
        trace(16, "DMP Read", "BBB", "Bank", bank, "Offset", offset, "Value", value)
        return value
    }

    /** Read a sequence of DMP registers of <length> bytes into a Buffer */
    dmpReadBuffer(register: number, length: number) {
        let bank = register >> 8
        let offset = register & 0xff
        this.dmpBankSelect(bank)
        this.icmWrite(0, ICM_MEM_START_ADDR, offset)
        let bytes = this.icmReadBuffer(0, ICM_MEM_R_W, length)
        trace(2, "DMP Read Buffer", "BBD", "Bank", bank, "Offset", offset, "Data", bytes)
        return bytes
    }

    /** Write a single byte to a DMP register */
    dmpWrite(register: number, value: number) {
        let bank = register >> 8
        let offset = register & 0xff
        this.dmpBankSelect(bank)
        this.icmWrite(0, ICM_MEM_START_ADDR, offset)
        this.icmWrite(0, ICM_MEM_R_W, value)
        trace(2, "DMP Write", "BBB", "Bank", bank, "Offset", offset, "Value", value)
    }

    /** Write a multi-byte Buffer to a sequence of DMP registers */
    dmpWriteBuffer(register: number, bytes: Buffer) {
        let bank = register >> 8
        let offset = register & 0xff
        this.dmpBankSelect(bank)
        this.icmWrite(0, ICM_MEM_START_ADDR, offset)
        this.icmWriteBuffer(0, ICM_MEM_R_W, bytes)
        trace(2, "DMP Write Buffer", "BBD", "Bank", bank, "Offset", offset, "Data", bytes)
    }
    // (for internal use)
    dmpBankSelect(bank: number) {
        if (this.dmpBank != bank) {
            this.icmWrite(0, ICM_MEM_BANK_SEL, bank)
            this.dmpBank = bank
        }
    }
}
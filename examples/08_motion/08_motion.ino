#define MIN_LIBRARY_VER_BUILD_NO (0x0008)
#include <VivicoreSerial.h>
#include <Wire.h>
#include "lsm6ds3_reg.h"
#include "lsm6ds3tr_c_reg.h"

#define MAX_VALUE (100)
#define MIN_VALUE (-1 * MAX_VALUE)
#define MAX_ACCEL_RAW (2000)                 // Unit: mG force
#define MAX_ACCEL     (MAX_ACCEL_RAW / 1000) // Unit: G force
#define MAX_GYRO_RAW  (500000)               // Unit: mdeg/s
#define MAX_GYRO      (MAX_GYRO_RAW / 1000)  // Unit: deg/s
#define SLAVE_ADR (0x6B)
#define BOOT_TIME (20) // Unit: ms

const uint16_t USER_FW_VER = 0x000B;
const uint32_t BRANCH_TYPE = 0x00000008;

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DC_GROUP_1, DC_NATURE_OUT, DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 1: Accel X
  {DC_GROUP_1, DC_NATURE_OUT, DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 2: Accel Y
  {DC_GROUP_1, DC_NATURE_OUT, DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 3: Accel Z
  {DC_GROUP_2, DC_NATURE_OUT, DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 4: Gyro X
  {DC_GROUP_2, DC_NATURE_OUT, DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 5: Gyro Y
  {DC_GROUP_2, DC_NATURE_OUT, DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 6: Gyro Z
};

/* Initialize mems driver interface */
typedef float_t (*CONVERT_UNIT)(int16_t raw);
typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;
static uint8_t whoamI = 0;
static int32_t writeReg(void*, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t readReg(void*, uint8_t reg, uint8_t *bufp, uint16_t len);
static stmdev_ctx_t dev_ctx = {
  writeReg,
  readReg,
  NULL,  // Customizable optional pointer
};

/* Write generic device register (platform dependent) */
static int32_t writeReg(void*, uint8_t reg, uint8_t *bufp, uint16_t len) {
	int32_t ret = 0;

  Wire.beginTransmission(SLAVE_ADR);
  Wire.write(reg);
  Wire.write(bufp, len);
  if (Wire.endTransmission() != 0) {
    ret = -1;
  }

  return ret;
}

/* Read generic device register (platform dependent) */
static int32_t readReg(void*, uint8_t reg, uint8_t *bufp, uint16_t len) {
	uint8_t i = 0;
	uint8_t c = 0;
	int32_t ret = 0;

  Wire.beginTransmission(SLAVE_ADR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    ret = -1;
  } else {
    Wire.requestFrom(SLAVE_ADR, len);
    while ((Wire.available()) && (i < len)) {
      c = Wire.read();
      *bufp = c;
      bufp++;
      i++;
    }
  }

  return ret;
}

static uint8_t getAccelCode(const uint8_t maxG) {
  uint8_t accel = 0;

  if (maxG <= 2) {
    accel = (whoamI == LSM6DS3_ID) ? (uint8_t)LSM6DS3_2g : (uint8_t)LSM6DS3TR_C_2g;
  } else if (maxG <= 4) {
    accel = (whoamI == LSM6DS3_ID) ? (uint8_t)LSM6DS3_4g : (uint8_t)LSM6DS3TR_C_4g;
  } else if (maxG <= 8) {
    accel = (whoamI == LSM6DS3_ID) ? (uint8_t)LSM6DS3_8g : (uint8_t)LSM6DS3TR_C_8g;
  } else {
    accel = (whoamI == LSM6DS3_ID) ? (uint8_t)LSM6DS3_16g : (uint8_t)LSM6DS3TR_C_16g;
  }

  DebugPlainPrint0("Full scale accel:");
  DebugHexPrint0(accel);
  DebugPlainPrintln0();
  return accel;
}

static uint8_t getGyroCode(const uint16_t maxDps) {
  uint8_t gyro = 0;

  if (maxDps <= 125) {
    gyro = (whoamI == LSM6DS3_ID) ? (uint8_t)LSM6DS3_125dps : (uint8_t)LSM6DS3TR_C_125dps;
  } else if (maxDps <= 250) {
    gyro = (whoamI == LSM6DS3_ID) ? (uint8_t)LSM6DS3_250dps : (uint8_t)LSM6DS3TR_C_250dps;
  } else if (maxDps <= 500) {
    gyro = (whoamI == LSM6DS3_ID) ? (uint8_t)LSM6DS3_500dps : (uint8_t)LSM6DS3TR_C_500dps;
  } else if (maxDps <= 1000) {
    gyro = (whoamI == LSM6DS3_ID) ? (uint8_t)LSM6DS3_1000dps : (uint8_t)LSM6DS3TR_C_1000dps;
  } else {
    gyro = (whoamI == LSM6DS3_ID) ? (uint8_t)LSM6DS3_2000dps : (uint8_t)LSM6DS3TR_C_2000dps;
  }

  DebugPlainPrint0("Full scale gyro:");
  DebugHexPrint0(gyro);
  DebugPlainPrintln0();
  return gyro;
}

static void initSensor(const uint8_t maxG, const uint16_t maxDps) {
  uint8_t rst = 0;

  /* Wait sensor boot time */
  delay(BOOT_TIME);
  /* Check device ID */
  lsm6ds3_device_id_get(&dev_ctx, &whoamI);

  if (whoamI == LSM6DS3_ID) {
    const lsm6ds3_xl_fs_t accel = (lsm6ds3_xl_fs_t)getAccelCode(maxG);
    const lsm6ds3_fs_g_t gyro = (lsm6ds3_fs_g_t)getGyroCode(maxDps);

    DebugPlainPrintln0("LSM6DS3");

    /* Restore default configuration */
    lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
      lsm6ds3_reset_get(&dev_ctx, &rst);
    } while (rst);
    DebugPlainPrintln0("Reset done");

    /* Enable Block Data Update */
    lsm6ds3_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set full scale */
    lsm6ds3_xl_full_scale_set(&dev_ctx, accel);
    lsm6ds3_gy_full_scale_set(&dev_ctx, gyro);
    /* Set Output Data Rate for Acc and Gyro */
    lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_52Hz);
    lsm6ds3_gy_data_rate_set(&dev_ctx, LSM6DS3_GY_ODR_52Hz);
  } else {
    const lsm6ds3tr_c_fs_xl_t accel = (lsm6ds3tr_c_fs_xl_t)getAccelCode(maxG);
    const lsm6ds3tr_c_fs_g_t gyro = (lsm6ds3tr_c_fs_g_t)getGyroCode(maxDps);

    DebugPlainPrintln0("LSM6DS3TR_C");

    /* Restore default configuration */
    lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
      lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
    } while (rst);
    DebugPlainPrintln0("Reset done");

    /* Enable Block Data Update */
    lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_52Hz);
    lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_52Hz);
    /* Set full scale */
    lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, accel);
    lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, gyro);
    /* Configure filtering chain(No aux interface) */
    /* Accelerometer - analog filter */
    lsm6ds3tr_c_xl_filter_analog_set(&dev_ctx,
                                    LSM6DS3TR_C_XL_ANA_BW_400Hz);
    /* Accelerometer - LPF1 path ( LPF2 not used )*/
    lsm6ds3tr_c_xl_lp1_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LP1_ODR_DIV_2);
    /* Accelerometer - LPF1 + LPF2 path */
    //lsm6ds3tr_c_xl_lp2_bandwidth_set(&dev_ctx,
    //                                LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100);
    /* Accelerometer - High Pass / Slope path */
    //lsm6ds3tr_c_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
    //lsm6ds3tr_c_xl_hp_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_HP_ODR_DIV_100);
    /* Gyroscope - filtering chain */
    lsm6ds3tr_c_gy_band_pass_set(&dev_ctx, LSM6DS3TR_C_LP2_ONLY);
  }
}

static bool isSensorDataReady(void) {
  uint8_t acel_ready = 0;
  uint8_t gyro_ready = 0;

  /* Read output only if new value is available */
  if (whoamI == LSM6DS3_ID) {
    lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &acel_ready);
    lsm6ds3_gy_flag_data_ready_get(&dev_ctx, &gyro_ready);
  } else {
    lsm6ds3tr_c_xl_flag_data_ready_get(&dev_ctx, &acel_ready);
    lsm6ds3tr_c_gy_flag_data_ready_get(&dev_ctx, &gyro_ready);
  }

  return ((acel_ready != 0) && (gyro_ready != 0));
}

static void getAccelData(const uint8_t maxG, float *accelValues) {
  static const CONVERT_UNIT converters[] = {
    lsm6ds3_from_fs2g_to_mg,      // LSM6DS3_2g  = 0
    lsm6ds3_from_fs16g_to_mg,     // LSM6DS3_16g = 1
    lsm6ds3_from_fs4g_to_mg,      // LSM6DS3_4g  = 2
    lsm6ds3_from_fs8g_to_mg,      // LSM6DS3_8g  = 3
  };
  static const CONVERT_UNIT convertersTrc[] = {
    lsm6ds3tr_c_from_fs2g_to_mg,  // LSM6DS3TR_C_2g  = 0
    lsm6ds3tr_c_from_fs16g_to_mg, // LSM6DS3TR_C_16g = 1
    lsm6ds3tr_c_from_fs4g_to_mg,  // LSM6DS3TR_C_4g  = 2
    lsm6ds3tr_c_from_fs8g_to_mg,  // LSM6DS3TR_C_8g  = 3
  };
  const uint8_t accel = getAccelCode(maxG);
  const CONVERT_UNIT accelConverter =
    (whoamI == LSM6DS3_ID) ? converters[accel] : convertersTrc[accel];
  axis3bit16_t rawAccel = {};

  /* Read acceleration field data */
  if (whoamI == LSM6DS3_ID) {
    lsm6ds3_acceleration_raw_get(&dev_ctx, rawAccel.u8bit);
  } else {
    lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, rawAccel.u8bit);
  }

  /* Convert unit and change the direction of axis for motion branch */
  accelValues[0] = accelConverter(rawAccel.i16bit[1]);
  accelValues[1] = accelConverter(rawAccel.i16bit[0]) * (-1.0);
  accelValues[2] = accelConverter(rawAccel.i16bit[2]) * (-1.0);
}

static void getGyroData(const uint16_t maxDps, float *gyroValues) {
  static const CONVERT_UNIT converters[] = {
    lsm6ds3_from_fs250dps_to_mdps,       // LSM6DS3_250dps  = 0
    lsm6ds3_from_fs125dps_to_mdps,       // LSM6DS3_125dps  = 1
    lsm6ds3_from_fs500dps_to_mdps,       // LSM6DS3_500dps  = 2
    NULL,                                // No dps
    lsm6ds3_from_fs1000dps_to_mdps,      // LSM6DS3_1000dps = 4
    NULL,                                // No dps
    lsm6ds3_from_fs2000dps_to_mdps,      // LSM6DS3_2000dps = 6
  };
  static const CONVERT_UNIT convertersTrc[] = {
    lsm6ds3tr_c_from_fs250dps_to_mdps,   // LSM6DS3TR_C_250dps  = 0
    lsm6ds3tr_c_from_fs125dps_to_mdps,   // LSM6DS3TR_C_125dps  = 1
    lsm6ds3tr_c_from_fs500dps_to_mdps,   // LSM6DS3TR_C_500dps  = 2
    NULL,                                // No dps
    lsm6ds3tr_c_from_fs1000dps_to_mdps,  // LSM6DS3TR_C_1000dps = 4
    NULL,                                // No dps
    lsm6ds3tr_c_from_fs2000dps_to_mdps,  // LSM6DS3TR_C_2000dps = 6
  };
  const uint8_t gyro = getGyroCode(maxDps);
  const CONVERT_UNIT gyroConverter =
    (whoamI == LSM6DS3_ID) ? converters[gyro] : convertersTrc[gyro];
  axis3bit16_t rawAngular = {};

  /* Read angular rate field data */
  if (whoamI == LSM6DS3_ID) {
    lsm6ds3_angular_rate_raw_get(&dev_ctx, rawAngular.u8bit);
  } else {
    lsm6ds3tr_c_angular_rate_raw_get(&dev_ctx, rawAngular.u8bit);
  }

  /* Convert unit and change the direction of axis for motion branch */
  gyroValues[0] = gyroConverter(rawAngular.i16bit[1]) * (-1.0);
  gyroValues[1] = gyroConverter(rawAngular.i16bit[0]);
  gyroValues[2] = gyroConverter(rawAngular.i16bit[2]);
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);
  Wire.begin();
  initSensor(MAX_ACCEL, MAX_GYRO);
}

void loop() {
  static const uint8_t DC_NUM = sizeof(dcInfo) / sizeof(dcInfo[0]);
  static const float maxRawValues[DC_NUM] = {
    MAX_ACCEL_RAW, MAX_ACCEL_RAW, MAX_ACCEL_RAW,
    MAX_GYRO_RAW, MAX_GYRO_RAW, MAX_GYRO_RAW,
  };
  static int8_t prevValues[DC_NUM] = {
    INT8_MIN, INT8_MIN, INT8_MIN,
    INT8_MIN, INT8_MIN, INT8_MIN,
  };
  float readValues[DC_NUM] = {};
  uint8_t statbuffer[DC_NUM * 2] = {};
  uint8_t bufIndex = 0;

  delay(30);

  /* Read acceleration/angular rate field data */
  if (!isSensorDataReady()) {
    return;
  }
  getAccelData(MAX_ACCEL, &readValues[0]);
  getGyroData(MAX_GYRO, &readValues[3]);

  for (uint8_t i = 0; i < DC_NUM; i++) {
    const float minRawValue = -1.0 * maxRawValues[i];
    const float maxRawValue = maxRawValues[i];
    const long constrainedValue =
      (long)(constrain(readValues[i], minRawValue, maxRawValue));
    const int8_t mappedValue = (int8_t)map(
      constrainedValue,
      (long)minRawValue, (long)maxRawValue,
      MIN_VALUE, MAX_VALUE);

    if (prevValues[i] != mappedValue) {
      prevValues[i] = mappedValue;
      statbuffer[bufIndex++] = i + 1;
      statbuffer[bufIndex++] = mappedValue;
    }

    DebugPlainPrint0(readValues[i]);
    DebugPlainPrint0(",");
    DebugPlainPrint0(constrainedValue);
    DebugPlainPrint0(",");
    DebugPlainPrint0(mappedValue);
    DebugPlainPrint0(",");
    DebugHexPrint0((uint8_t)mappedValue);
    DebugPlainPrint0(" ");
  }
  DebugPlainPrintln0();

  if (bufIndex) {
    Vivicore.write(statbuffer, bufIndex);
    Vivicore.flush();
  }
}

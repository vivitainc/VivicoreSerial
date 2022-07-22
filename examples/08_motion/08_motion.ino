#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>
#include <Wire.h>
#include "lsm6ds3_reg.h"
#include "lsm6ds3tr_c_reg.h"

#define MAX_VALUE (100)
#define MIN_VALUE (-1 * MAX_VALUE)
#define MAX_ACCEL (2)   // Unit: G force
#define MAX_GYRO  (500) // Unit: deg/s
#define MAX_RAW   (INT16_MAX)
#define MIN_RAW   (INT16_MIN)
#define SLAVE_ADR (0x6B)
#define BOOT_TIME (20) // Unit: ms

const uint8_t  USER_FW_MAJOR_VER = 0x00;
const uint8_t  USER_FW_MINOR_VER = 0x0E;
const uint16_t USER_FW_VER       = (((uint16_t)(USER_FW_MAJOR_VER) << 8) + ((uint16_t)(USER_FW_MINOR_VER)));
const uint32_t BRANCH_TYPE       = 0x00000008;

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_1BYTE, MIN_VALUE,
   MAX_VALUE}, // 1: Accel X
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_1BYTE, MIN_VALUE,
   MAX_VALUE}, // 2: Accel Y
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_1BYTE, MIN_VALUE,
   MAX_VALUE}, // 3: Accel Z
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 4: Gyro X
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 5: Gyro Y
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 6: Gyro Z
};

/* Initialize mems driver interface */
typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;
static uint8_t      whoamI = 0;
static int32_t      writeReg(void *, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t      readReg(void *, uint8_t reg, uint8_t *bufp, uint16_t len);
static stmdev_ctx_t dev_ctx = {
  writeReg, readReg,
  NULL, // Customizable optional pointer
};

static inline float mapf(const float x, const float in_min, const float in_max, const float out_min,
                         const float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Write generic device register (platform dependent) */
static int32_t writeReg(void *, uint8_t reg, uint8_t *bufp, uint16_t len) {
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
static int32_t readReg(void *, uint8_t reg, uint8_t *bufp, uint16_t len) {
  uint8_t i   = 0;
  uint8_t c   = 0;
  int32_t ret = 0;

  Wire.beginTransmission(SLAVE_ADR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    ret = -1;
  } else {
    Wire.requestFrom(SLAVE_ADR, len);
    while ((Wire.available()) && (i < len)) {
      c     = Wire.read();
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
    const lsm6ds3_fs_g_t  gyro  = (lsm6ds3_fs_g_t)getGyroCode(maxDps);

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
    const lsm6ds3tr_c_fs_g_t  gyro  = (lsm6ds3tr_c_fs_g_t)getGyroCode(maxDps);

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
    lsm6ds3tr_c_xl_filter_analog_set(&dev_ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz);
    /* Accelerometer - LPF1 path ( LPF2 not used )*/
    lsm6ds3tr_c_xl_lp1_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LP1_ODR_DIV_2);
    /* Accelerometer - LPF1 + LPF2 path */
    // lsm6ds3tr_c_xl_lp2_bandwidth_set(&dev_ctx,
    //                                LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100);
    /* Accelerometer - High Pass / Slope path */
    // lsm6ds3tr_c_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
    // lsm6ds3tr_c_xl_hp_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_HP_ODR_DIV_100);
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

static void getAccelData(int16_t *accelValues) {
  axis3bit16_t rawAccel = {};

  /* Read acceleration field data */
  if (whoamI == LSM6DS3_ID) {
    lsm6ds3_acceleration_raw_get(&dev_ctx, rawAccel.u8bit);
  } else {
    lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, rawAccel.u8bit);
  }

  /* Change the direction of axis for motion branch */
  accelValues[0] = rawAccel.i16bit[1];
  accelValues[1] = rawAccel.i16bit[0] * (-1);
  accelValues[2] = rawAccel.i16bit[2] * (-1);
}

static void getGyroData(int16_t *gyroValues) {
  axis3bit16_t rawAngular = {};

  /* Read angular rate field data */
  if (whoamI == LSM6DS3_ID) {
    lsm6ds3_angular_rate_raw_get(&dev_ctx, rawAngular.u8bit);
  } else {
    lsm6ds3tr_c_angular_rate_raw_get(&dev_ctx, rawAngular.u8bit);
  }

  /* Change the direction of axis for motion branch */
  gyroValues[0] = rawAngular.i16bit[1] * (-1);
  gyroValues[1] = rawAngular.i16bit[0];
  gyroValues[2] = rawAngular.i16bit[2];
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);
  Wire.begin();
  initSensor(MAX_ACCEL, MAX_GYRO);
}

void loop() {
  static const uint8_t DC_NUM               = sizeof(dcInfo) / sizeof(dcInfo[0]);
  static const int16_t maxRawValues[DC_NUM] = {
    MAX_RAW, MAX_RAW, MAX_RAW, MAX_RAW, MAX_RAW, MAX_RAW,
  };
  static const int16_t minRawValues[DC_NUM] = {
    MIN_RAW, MIN_RAW, MIN_RAW, MIN_RAW, MIN_RAW, MIN_RAW,
  };
  static int8_t prevValues[DC_NUM] = {
    INT8_MIN, INT8_MIN, INT8_MIN, INT8_MIN, INT8_MIN, INT8_MIN,
  };
  int16_t readValues[DC_NUM] = {};

  delay(30);

  /* Read acceleration/angular rate field data */
  if (!isSensorDataReady()) {
    return;
  }
  getAccelData(&readValues[0]);
  getGyroData(&readValues[3]);

  for (uint8_t i = 0; i < DC_NUM; i++) {
    const float minRawValue      = minRawValues[i];
    const float maxRawValue      = maxRawValues[i];
    const long  constrainedValue = (long)(constrain(readValues[i], minRawValue, maxRawValue));
    const float mappedFloatValue =
      mapf((float)constrainedValue, (float)minRawValue, (float)maxRawValue, (float)MIN_VALUE, (float)MAX_VALUE);
    const int8_t mappedValue = (int8_t)round(mappedFloatValue);

    if (prevValues[i] != mappedValue) {
      prevValues[i] = mappedValue;
      Vivicore.write(i + 1, mappedValue);
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

  Vivicore.flush();
}

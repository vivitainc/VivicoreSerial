#define MIN_LIBRARY_VER_BUILD_NO (0x0004)
#include <VivicoreSerial.h>
#include <Wire.h>
#include <VL53L0X.h>

// Use alghrythm to determine infinite distance
#define INFINITE_ALGO_REFPWR (1)
#define INFINITE_ALGO_HYST   (2)
#define USE_INFINITE_ALGO    (INFINITE_ALGO_HYST)

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.
#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed
//#define HIGH_SPEED
//#define HIGH_ACCURACY
#if defined HIGH_SPEED
#  define BUDGET_US (20000)
#elif defined HIGH_ACCURACY
#  define BUDGET_US (200000)
#endif

#if defined LONG_RANGE
#  define MAX_VALUE (1000)
#else
#  define MAX_VALUE (300)
#endif
#define MIN_VALUE (0)

#define VALUE_ERROR_CORRECTION  (-47) // Average error measured by VIVITA HW team
#define VALUE_WITH_NO_DISTANCE  (80 + VALUE_ERROR_CORRECTION)
#define VALUE_INFINITE          (8000 + VALUE_ERROR_CORRECTION)
#define HYST_SIZE               (7)
#define REFLECTION_POWER_THRESH (50)

#define REFLECTION_POWER_L (0x19)
#define REFLECTION_POWER_H (0x18)

#define XSHUT_PIN      (10)
#define T_BOOT_MS      (5)  // tBOOT is 1.2ms max as written on VL53L0X DS
#define ACK_TIMEOUT_MS (1000)

#define INTERVAL_MS (30)

enum rangeStat {
  RANGE_STAT_VALID = 0,
  RANGE_STAT_FAIL_SIGMA = 1,
  RANGE_STAT_FAIL_SIGNAL = 2,
  RANGE_STAT_FAIL_MIN = 3,
  RANGE_STAT_FAIL_PHASE = 4,
  RANGE_STAT_FAIL_HW = 5,
  RANGE_STAT_NONE = 255,
};

const uint16_t USER_FW_VER = 0x000A;
const uint32_t BRANCH_TYPE = 0x00000007;

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DC_GROUP_1, DC_NATURE_OUT, DC_TYPE_ANALOG_2BYTES, MIN_VALUE, MAX_VALUE}  // 1: Distance
};
uint8_t statbuffer[] = {
  1, 0x00, 0x00,  // 1: Distance
};

VL53L0X sensor;

rangeStat getRangeStatus(void) {
  // Refer to the ST official implementation of
  // VL53L0X_get_pal_range_status() on vl53l0x_api_core.c
  const uint8_t range_stat_reg = sensor.readReg(VL53L0X::RESULT_RANGE_STATUS);
  const uint8_t range_stat_internal = (range_stat_reg & 0x78) >> 3;
  const bool none_flag = (
    (range_stat_internal == 0) ||
    (range_stat_internal == 5) ||
    (range_stat_internal == 7) ||
    (range_stat_internal == 12) ||
    (range_stat_internal == 13) ||
    (range_stat_internal == 14) ||
    (range_stat_internal == 15));
  rangeStat range_status = RANGE_STAT_VALID;

  if (none_flag) {
    range_status = RANGE_STAT_NONE;
  } else if (range_stat_internal == 1 ||
        range_stat_internal == 2 ||
        range_stat_internal == 3) {
    range_status = RANGE_STAT_FAIL_HW;
  } else if (range_stat_internal == 6 ||
        range_stat_internal == 9) {
    range_status = RANGE_STAT_FAIL_PHASE;
// TODO: Need more porting ST official driver
//  } else if (range_stat_internal == 8 ||
//        range_stat_internal == 10 ||
//        SignalRefClipflag == 1) {
//    range_status = RANGE_STAT_FAIL_MIN;
//  } else if (range_stat_internal == 4 ||
//        RangeIgnoreThresholdflag == 1) {
//    range_status = RANGE_STAT_FAIL_SIGNAL;
//  } else if (SigmaLimitflag == 1) {
//    range_status = RANGE_STAT_FAIL_SIGMA;
  } else {
    range_status = RANGE_STAT_VALID;
  }

  return range_status;
}

uint16_t getUnofficialReflectionPower(void) {
  uint16_t high_byte = sensor.readReg(REFLECTION_POWER_H);
  uint16_t low_byte = sensor.readReg(REFLECTION_POWER_L);
  return ((high_byte << 8) | low_byte);
}

bool isNoTarget(const uint16_t raw_val) {
  bool no_target = false;

#if defined(USE_INFINITE_ALGO)
#  if (USE_INFINITE_ALGO == INFINITE_ALGO_REFPWR)
  (void)raw_val;  // Avoid static analysis warning
  no_target = (getUnofficialReflectionPower() < REFLECTION_POWER_THRESH);
#  elif (USE_INFINITE_ALGO == INFINITE_ALGO_HYST)
  static uint16_t raw_vals[HYST_SIZE] = {};
  static uint8_t raw_idx = 0;

  raw_vals[raw_idx++] = raw_val;
  raw_idx = (raw_idx >= HYST_SIZE) ? 0 : raw_idx;
  for (uint8_t i = 0; i < HYST_SIZE; i++) {
    no_target |= (raw_vals[i] > VALUE_INFINITE);
  }
#  endif
#endif

  return no_target;
}

void initSensor(void) {
  Wire.end();

  digitalWrite(XSHUT_PIN, LOW);
  pinMode(XSHUT_PIN, OUTPUT);
  delay(100);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(T_BOOT_MS);

  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED || defined HIGH_ACCURACY
  // increase/reduce timing budget (default is about 33 ms)
  sensor.setMeasurementTimingBudget(BUDGET_US);
#endif
  sensor.startContinuous();
}

static void sendToCore(uint16_t val) {
  statbuffer[1] = highByte(val);
  statbuffer[2] = lowByte(val);

  Vivicore.write(statbuffer, 3);
  Vivicore.flush();
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);
  initSensor();
}

void loop() {
  static unsigned long prevMills = 0;
  static unsigned long firstNoAckMills = 0;
  unsigned long curMills = millis();

  if (curMills - prevMills > INTERVAL_MS) {
    static uint16_t presend_val = 0;
    const uint16_t raw_read_val = sensor.readRangeContinuousMillimeters();
    const uint16_t raw_val = (int32_t)raw_read_val + VALUE_ERROR_CORRECTION < 0 ? 0 : raw_read_val + VALUE_ERROR_CORRECTION;
    const bool has_ack = !(sensor.last_status || sensor.timeoutOccurred());
    uint16_t send_val = 0;
    bool ackTimeout = false;

    prevMills = curMills;

    if (!firstNoAckMills && !has_ack) {
      firstNoAckMills = curMills;
    }

    if (firstNoAckMills) {
      if (has_ack) {
        firstNoAckMills = 0;
      } else if (curMills - firstNoAckMills > ACK_TIMEOUT_MS) {
        ackTimeout = true;
        firstNoAckMills = 0;
        initSensor();
      }
    }

    if (has_ack) {
      send_val = (isNoTarget(raw_val)) ? VALUE_INFINITE : raw_val;
      if (send_val > MAX_VALUE) {
        send_val = MAX_VALUE;
      } else if (send_val < VALUE_WITH_NO_DISTANCE) {
        send_val = MIN_VALUE;
      }

      if (presend_val != send_val) {
        presend_val = send_val;
        sendToCore(send_val);
      }
    } else if (ackTimeout) {
      // Resend last Measure distance to Core/App
      sendToCore(presend_val);
    }

    DebugPlainPrint0(curMills);
    DebugPlainPrint0(", ");
    DebugPlainPrint0(raw_read_val);
    DebugPlainPrint0(", ");
    DebugPlainPrint0(raw_val);
    DebugPlainPrint0(", ");
    DebugPlainPrint0(send_val);
    DebugPlainPrint0(", ");
    DebugPlainPrint0(getRangeStatus());
    DebugPlainPrint0(", ");
    DebugPlainPrint0(getUnofficialReflectionPower());
    DebugPlainPrint0(", ");
    DebugPlainPrint0(has_ack);
    DebugPlainPrint0(", ");
    DebugPlainPrintln0();
  }
}

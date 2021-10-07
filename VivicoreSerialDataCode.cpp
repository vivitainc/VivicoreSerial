/*
  Copyright (c) 2021 VIVIWARE JAPAN, Inc.  All right reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

/**
 * @cond
 * @file VivicoreSerialDataCode.cpp
 * @brief Data code translater library to encode/decode DCDT for VIVIWARE Cell Branch and Custom
 * @endcond
 */

#include <Arduino.h>
#include "VivicoreSerialDataCode.h"
#include "VivicoreSerialDebug.h"

DataCodeTranslator::DataCodeTranslator(const dcInfo_t *dc_info, const uint8_t dc_num, bool *initialized) {
  *initialized = init(dc_info, dc_num);
}

bool DataCodeTranslator::init(const dcInfo_t *dc_info, const uint8_t dc_num) {
  _is_dc_num_in_max = (dc_num <= NUM_MAX_DC);
  _dc_num           = _is_dc_num_in_max ? dc_num : NUM_MAX_DC;

  if (!_is_dc_num_in_max) {
    DebugStringPrint0("F: DC number ");
    DebugPlainPrint0(dc_num);
    DebugStringPrintln0(" is over max limit");
  }

  memcpy(_dc_info, dc_info, sizeof(dcInfo_t) * _dc_num);

  // Currently both of binary and scaler cannot be included in a DC info array
  _has_binary         = false;
  _has_truncated_data = false;
  _is_known_dc_type   = true;
  _is_dc_minmax_valid = true;

  for (uint8_t i = 0; i < _dc_num; i++) {
    const DcType_t   data_type          = _dc_info[i].data_type;
    const DcNature_t data_nature        = _dc_info[i].data_nature;
    const int32_t    data_min           = static_cast<int32_t>(_dc_info[i].data_min);
    const int32_t    data_max           = static_cast<int32_t>(_dc_info[i].data_max);
    const int32_t    data_ini           = static_cast<int32_t>(_dc_info[i].data_ini);
    const bool       is_dc_minmax_valid = (data_max >= data_min);

    if (getDataSize(data_type) == 0) {
      DebugStringPrint0("F: Unknown type ");
      DebugPlainPrint0(static_cast<uint8_t>(data_type));
      DebugStringPrint0(" of DC ");
      DebugPlainPrintln0(i + 1);
      _is_known_dc_type = false;
      continue;
    }
    if (!is_dc_minmax_valid) {
      DebugStringPrint0("F: min ");
      DebugPlainPrint0(data_min);
      DebugStringPrint0(" is over max ");
      DebugPlainPrint0(data_max);
      DebugStringPrint0(" on DC ");
      DebugPlainPrintln0(i + 1);
      _is_dc_minmax_valid = false;
    }

    const Range_t range              = getDataRange(data_type);
    const int32_t truncated_min      = is_dc_minmax_valid ? constrain(data_min, range.min, range.max) : 0;
    const int32_t truncated_max      = is_dc_minmax_valid ? constrain(data_max, range.min, range.max) : 0;
    const int32_t truncated_ini      = is_dc_minmax_valid ? constrain(data_ini, truncated_min, truncated_max) : 0;
    const bool    has_truncated_data = (data_min != truncated_min) || (data_max != truncated_max) ||
                                    ((data_nature == DcNature_t::DC_NATURE_IN) && (data_ini != truncated_ini));

    _dc_info[i].data_min = truncated_min;
    _dc_info[i].data_max = truncated_max;
    _dc_info[i].data_ini = truncated_ini;
    _has_truncated_data |= has_truncated_data;
    _has_binary |= (data_type == DcType_t::DC_TYPE_BINARY);
  }

  _max_data_body_length        = getDataBodyLength();
  _is_dc_info_in_packet_length = (_max_data_body_length <= NUM_MAX_PKT_BODY_DATA_DC);
  if (!_is_dc_info_in_packet_length) {
    DebugStringPrint0("F: Data body length ");
    DebugPlainPrint0(_max_data_body_length);
    DebugStringPrint0(" is over max ");
    DebugPlainPrintln0(NUM_MAX_PKT_BODY_DATA_DC);
  }

  return !hasError() && !hasFatalError();
}

bool DataCodeTranslator::encode(const int16_t *encoding_values, data_pkt *out) {
  bool is_set[NUM_MAX_DC] = {};
  for (uint8_t i = 0; i < _dc_num; i++) {
    is_set[i] = true;
  }
  return encode(encoding_values, is_set, out, true);
}

bool DataCodeTranslator::encode(const int16_t *encoding_values, const bool *is_set, data_pkt *out,
                                const bool is_initial_data) {
  uint8_t       bit_cnt            = 0; // 0 to 7
  uint16_t      dc_flag            = 0;
  const uint8_t data_body_length   = min(_max_data_body_length + sizeof(dc_flag), NUM_MAX_UART_PKT_BODY_DATA);
  bool          has_skipped_data   = false;
  bool          has_truncated_data = false;
  bool          is_over_limit      = false;

  memset(out, 0, sizeof(data_pkt));

  if (hasFatalError()) {
    DebugStringPrintln0("F: Fatal error in DC info");
    return false;
  }

  out->datalen = sizeof(dc_flag);

  for (uint8_t i = 0; i < _dc_num; i++) {
    if (!is_set[i]) {
      continue;
    }

    const uint8_t  dc_n           = i + 1;
    const DcType_t dc_type        = _dc_info[i].data_type;
    const size_t   dc_size        = getDataSize(dc_type);
    int16_t        encoding_value = encoding_values[i];

    if ((encoding_value < _dc_info[i].data_min) || (encoding_value > _dc_info[i].data_max)) {
      DebugStringPrint0("F: Encoding data is out of range for DC ");
      DebugPlainPrint0(dc_n);
      if (is_initial_data) {
        DebugStringPrintln0(" and truncated to min/max");
        has_truncated_data = true;
        encoding_value     = constrain(encoding_value, _dc_info[i].data_min, _dc_info[i].data_max);
      } else {
        DebugStringPrintln0(" and skipeed");
        has_skipped_data = true;
        continue;
      }
    } else if ((dc_n != 1) && (dc_type == DcType_t::DC_TYPE_BINARY)) {
      DebugStringPrint0("F: DC_TYPE_BINARY cannot be in ");
      DebugPlainPrintln0(dc_n);
      has_skipped_data = true;
      continue;
    } else if (!is_initial_data && (_dc_info[i].data_nature != DcNature_t::DC_NATURE_OUT)) {
      DebugStringPrint0("F: Encoding data to DC_NATURE_IN ");
      DebugPlainPrintln0(dc_n);
      has_skipped_data = true;
      continue;
    } else if (!is_initial_data && (dc_type == DcType_t::DC_TYPE_BINARY)) {
      DebugStringPrint0("F: Scaler cannot be encoded on DC_TYPE_BINARY in ");
      DebugPlainPrintln0(dc_n);
      has_skipped_data = true;
      continue;
    }

    if (((dc_type == DcType_t::DC_TYPE_BOOLEAN) && (7 < bit_cnt)) ||
        ((dc_type != DcType_t::DC_TYPE_BOOLEAN) && (0 < bit_cnt))) {
      // 1 byte is just filled out
      bit_cnt = 0;
      if (++out->datalen >= data_body_length) {
        is_over_limit = true;
        break;
      }
    }

    // Stop storing data and making dc_flag if room for data_body is unavailable
    if (out->datalen + dc_size > data_body_length) {
      is_over_limit = true;
      break;
    }

    if (dc_type == DcType_t::DC_TYPE_BOOLEAN) {
      bitWrite(out->data[out->datalen], 7 - bit_cnt, encoding_value);
      bit_cnt++;
    } else {
      if (dc_type == DcType_t::DC_TYPE_ANALOG_2BYTES) {
        out->data[out->datalen++] = static_cast<uint8_t>(encoding_value >> 8);
        out->data[out->datalen++] = static_cast<uint8_t>(encoding_value);
      } else { // including DcType_t::DC_TYPE_ANALOG_1BYTE, DcType_t::DC_TYPE_BINARY
        out->data[out->datalen++] = static_cast<uint8_t>(encoding_value);
      }
    }

    bitSet(dc_flag, NUM_MAX_DC - dc_n);
  }

  if (!is_over_limit && (0 < bit_cnt)) {
    if (out->datalen < data_body_length) {
      out->datalen++;
    } else {
      is_over_limit = true;
    }
  }

  if (dc_flag != 0) {
    out->data[0] = static_cast<uint8_t>(dc_flag >> 8);
    out->data[1] = static_cast<uint8_t>(dc_flag);
  } else {
    memset(out, 0, sizeof(data_pkt));
  }

  return !has_skipped_data && !has_truncated_data && !is_over_limit && (dc_flag != 0);
}

bool DataCodeTranslator::decode(const data_pkt *in, int16_t *values, uint8_t *dc_nums, uint8_t *dc_nums_count) {
  uint8_t       bit_cnt                      = 0; // 0 to 7
  uint16_t      dc_flag                      = 0;
  const uint8_t data_body_length             = min(_max_data_body_length + sizeof(dc_flag), NUM_MAX_UART_PKT_BODY_DATA);
  uint8_t       bcursor                      = sizeof(dc_flag); // buffer cursor
  uint8_t       dc_nums_cnt                  = 0;
  size_t        decoded_size                 = 0;
  bool          has_skipped_data             = false;
  bool          is_over_max_data_body_length = false;

  *dc_nums_count = 0;

  if (hasFatalError()) {
    DebugStringPrintln0("F: Fatal error in DC info");
    return false;
  }

  dc_flag = static_cast<uint16_t>(in->data[0]) << 8;
  dc_flag |= static_cast<uint16_t>(in->data[1]);

  if (dc_flag == 0) {
    return false;
  }

  for (uint8_t i = 0; i < _dc_num; i++) {
    const uint8_t  dc_n    = i + 1;
    const DcType_t dc_type = _dc_info[i].data_type;
    const size_t   dc_size = getDataSize(dc_type);

    if (bitRead(dc_flag, 15 - i) == 0) {
      continue;
    }

    if (((dc_type == DcType_t::DC_TYPE_BOOLEAN) && (7 < bit_cnt)) || // current data is bool
        ((dc_type != DcType_t::DC_TYPE_BOOLEAN) && (0 < bit_cnt))) { // current data is not bool
      bit_cnt = 0;
      bcursor++;
    }

    if (dc_type == DcType_t::DC_TYPE_BOOLEAN) {
      if (bcursor >= in->datalen) {
        DebugStringPrintln0("F: Read buffer pointer overflow");
        has_skipped_data = true;
        break;
      }
      if (bit_cnt == 0) {
        decoded_size += dc_size;
      }
      const uint8_t val = bitRead(in->data[bcursor], 7 - bit_cnt);
      values[i]         = static_cast<int16_t>(val ? 1 : 0);
      bit_cnt++;
    } else {
      if (bcursor + dc_size > in->datalen) {
        DebugStringPrintln0("F: Read buffer pointer overflow");
        has_skipped_data = true;
        break;
      }

      if (dc_type == DcType_t::DC_TYPE_ANALOG_1BYTE) {
        values[i] = static_cast<int16_t>(static_cast<int8_t>(in->data[bcursor++]));
      } else if (dc_type == DcType_t::DC_TYPE_ANALOG_2BYTES) {
        const uint16_t high = static_cast<uint16_t>(in->data[bcursor++]);
        const uint16_t low  = static_cast<uint16_t>(in->data[bcursor++]);
        values[i]           = static_cast<int16_t>((high << 8) | low);
      } else { // dc_type is DcType_t::DC_TYPE_BINARY
        DebugStringPrint0("F: Invalid type of DC number ");
        DebugPlainPrintln0(dc_n);
        return false;
      }
      decoded_size += dc_size;
    }

    dc_nums[dc_nums_cnt++] = dc_n;
  }

  if (decoded_size + sizeof(dc_flag) > data_body_length) {
    DebugStringPrint0("F: Decoding data size is over max length ");
    DebugPlainPrintln0(data_body_length);
    is_over_max_data_body_length = true;
  }

  *dc_nums_count = dc_nums_cnt;
  return !has_skipped_data && !is_over_max_data_body_length;
}

uint8_t DataCodeTranslator::getDataBodyLength(void) {
  uint8_t data_len = 0;
  uint8_t bit_cnt  = 0; // 0 to 7

  for (int i = 0; i < _dc_num; i++) {
    const DcType_t dc_type = _dc_info[i].data_type;
    const size_t   dc_size = getDataSize(dc_type);

    if (dc_size == 0) {
      // Unknown type and size
      continue;
    }

    if (((dc_type == DcType_t::DC_TYPE_BOOLEAN) && (7 < bit_cnt)) ||
        ((dc_type != DcType_t::DC_TYPE_BOOLEAN) && (0 < bit_cnt))) {
      // 1 byte is just filled out
      bit_cnt = 0;
      data_len++;
    }

    if (dc_type == DcType_t::DC_TYPE_BOOLEAN) {
      bit_cnt++;
    } else {
      data_len += dc_size;
    }
  }

  if (0 < bit_cnt) {
    data_len++;
  }

  return data_len;
}

DataCodeTranslator::Range_t DataCodeTranslator::getDataRange(const DcType_t dc_type) {
  Range_t range = {};
  if (dc_type == DcType_t::DC_TYPE_ANALOG_2BYTES) {
    range.min = INT16_MIN;
    range.max = INT16_MAX;
  } else if (dc_type == DcType_t::DC_TYPE_ANALOG_1BYTE) {
    range.min = INT8_MIN;
    range.max = INT8_MAX;
  } else if (dc_type == DcType_t::DC_TYPE_BINARY) {
    range.min = 0;
    range.max = UINT8_MAX;
  } else if (dc_type == DcType_t::DC_TYPE_BOOLEAN) {
    range.min = 0;
    range.max = 1;
  }
  return range;
}

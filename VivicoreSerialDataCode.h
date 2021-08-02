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
 * @file VivicoreSerialDataCode.h
 * @brief Data code translater library to encode/decode DCDT for VIVIWARE Cell Branch and Custom
 */

#ifndef VIVICORESERIAL_DATACODE_H
#define VIVICORESERIAL_DATACODE_H

#include "CommunicationProtocol.h"

/**
 * @brief DC information structure for each input or output of Branch
 *
 * Branch can have single or muptiple DC info which is shown on App as connector. The each connector has the properties
 * which can be configured with this structure.
 */
struct dcInfo_t {
  DcGroup_t  group_no;    /**< Group number to belong supported by @ref DcGroup_t */
  DcNature_t data_nature; /**< Direction from view point of Branch supported by @ref DcNature_t */
  DcType_t   data_type;   /**< Data type supported by @ref DcType_t */
  int16_t    data_min;    /**< Minimum value of range of the data */
  int16_t    data_max;    /**< Maximum value of range of the data */
  int16_t    data_ini;    /**< Default value of the data */
};

/** @cond */
#define NUM_DECODED_BUFF        (NUM_MAX_UART_PKT_BODY_DATA * 2) // maximum buffer for input of encode and output of decode
#define NUM_DATA_PKT_STRUCT_LEN (NUM_DECODED_BUFF > NUM_MAX_UART_PKT ? NUM_DECODED_BUFF : NUM_MAX_UART_PKT)

struct data_pkt {
  uint8_t data[NUM_DATA_PKT_STRUCT_LEN];
  uint8_t datalen;
};
/** @endcond */

/**
 * @brief This class is to encode and decode DCDT format data which is used between VIVIWARE Cell App, Core, and Branch.
 */
class DataCodeTranslator {
public:
  /**
   * Constructor which does not call @ref init. Caller needs to call @ref init by itself.
   */
  DataCodeTranslator(void) {}

  /**
   * Constructor which calls @ref init internally. Caller does not need to call @ref init by itself.
   *
   * @param [in] dc_info Pointer to @ref dcInfo_t array buffer
   * @param [in] dc_num dc_info array buffer's length
   * @param [out] initialized true if @ref init succeeded. false in another case.
   */
  DataCodeTranslator(const dcInfo_t *dc_info, const uint8_t dc_num, bool *initialized);

  /** @cond */
  virtual ~DataCodeTranslator(void) {}
  DataCodeTranslator(const DataCodeTranslator &) = delete;
  DataCodeTranslator &operator=(const DataCodeTranslator &) = delete;
  /** @endcond */

  /**
   * This API returns the number of DC. It is necessary to be called after @ref init.
   *
   * @return Number of DC.
   */
  inline uint8_t getDcNum(void) {
    return _dc_num;
  }

  /**
   * This API returns the pointer to DC info array. It is necessary to be called after @ref init.
   *
   * @return Pointer to DC info array.
   */
  inline const dcInfo_t *getDcInfo(void) {
    return _dc_info;
  }

  /**
   * This API returns if DC info has binary type. It is necessary to be called after @ref init.
   *
   * @return true if DC info has binary type. false in another case.
   */
  inline bool hasBinary(void) {
    return _has_binary;
  }

  /**
   * Return true if DC info has any of errors in the following.
   *
   *  - The number of DC is less or equal to maximum value @ref NUM_MAX_DC
   *  - Calculated maximum packet size is less or equal to maximum value @ref NUM_MAX_PKT_BODY_DATA_DC
   *  - DC min and max are correct as logical and meets range of data type
   *  - DC ini of @ref DC_NATURE_IN meets range of min and max
   *  - DC of binary is only a DC definition if it exists
   *
   * @return true if any error of above conditions. false in another case.
   */
  inline bool hasError(void) {
    return !(_is_dc_num_in_max && _is_dc_info_in_packet_length && _is_dc_minmax_valid && !_has_truncated_data &&
             (_has_binary ? (_dc_num == 1) : true));
  };

  /**
   * Return true if DC info has any of errors in the following.
   *
   *  - All DC type meets @ref DcType_t
   *
   * @return true if any error of above conditions. false in another case.
   */
  inline bool hasFatalError(void) {
    return !_is_known_dc_type;
  };

  /**
   * Initialize the class instance
   *
   * @param [in] dc_info Pointer to @ref dcInfo_t array buffer
   * @param [in] dc_num dc_info array length
   * @return true if no error in DC. false if DC has any of the errors in @ref hasFatalError or @ref hasError.
   */
  bool init(const dcInfo_t *dc_info, const uint8_t dc_num);

  /**
   * This API encodes all of data in encoding_values to be sent to core. This API does nothing and returns false if
   * @ref hasFatalError returns true, but can be used even if @ref hasError returns true. This API returns false if
   * there is at least one of the following errors in encoding process for each DC.
   *
   * - Encoding value is out of range between min and max which are specified in DC info at @ref init
   *
   * @param [in] encoding_values Array of values to be encoded as DC. The length of array should be equal to the number
   * of DC info array specified to @ref init.
   * @param [out] out            DC format data to be sent to core.
   * @return true if no error. false in another case.
   *
   * @warning It is better for caller of this method to check if no error in DC info by using @ref hasError method.
   */
  bool encode(const int16_t *encoding_values, data_pkt *out);

  /**
   * This API encodes data in encoding_values to be sent to core which are set as true in is_set. This API does nothing
   * and returns false if @ref hasFatalError returns true, but can be used even if @ref hasError returns true. This
   * API returns false if there is at least one of the following errors in encoding process for each DC.
   *
   * - Encoding value is out of range between min and max which are specified in DC info at @ref init
   * - DC nature is @ref DC_NATURE_IN (Skip this error checking if is_initial_data is true)
   * - DC type is @ref DC_TYPE_BINARY (Skip this error checking if is_initial_data is true)
   *
   * @param [in] encoding_values Array of values to be encoded as DC. The length of array should be less or equal to the
   * number of DC info array specified to @ref init.
   * @param [in] is_set          Array of true/false flag to be encoded to DC in encoding_array. The length of array
   * should be less or equal to the number of DC info array specified to @ref init.
   * @param [out] out            DC format data to be sent to core.
   * @param [in] is_initial_data Set true to encode DC min/max/ini.
   * @return true if no error. false in another case.
   *
   * @warning It is better for caller of this method to check if no error in DC info by using @ref hasError method.
   */
  bool encode(const int16_t *encoding_values, const bool *is_set, data_pkt *out, const bool is_initial_data = false);

  /**
   * This API decodes DC format data and corresponding DC numbers which is received from core. This API does nothing and
   * returns false if @ref hasFatalError returns true, but can be used even if @ref hasError returns true. This API
   * returns false if there is at least one of the following errors in encoding process for each DC.
   *
   * @param [in] in Data to be decoded. The first 2 bytes should be DC flags.
   * @param [out] values Array of values decoded from in data. The length of array should be less or equal to the number
   * of DC info array specified to @ref init.
   * @param [out] dc_nums Array of DC numbers decoded from in data. The length of array should be less or equal to the
   * number of DC info array specified to @ref init.
   * @param [out] dc_nums_count Actual number of DC stored in dc_nums.
   *
   * @return true if all DC data in the input data is valid. false in another case.
   *
   * @warning It is better for caller of this method to check if no error in DC info by using @ref hasError method.
   * @warning This method cannot decode data if any of the following errors was included in input data.
   * - Data for DC over the maximum number specified in DC info at @ref init
   * - Invalid or unknown DC type is included in DC info.
   */
  bool decode(const data_pkt *in, int16_t *values, uint8_t *dc_nums, uint8_t *dc_nums_count);

private:
  struct Range_t {
    int32_t min;
    int32_t max;
  };

  uint8_t  _dc_num                      = 0;
  bool     _is_dc_num_in_max            = false;
  bool     _is_dc_minmax_valid          = false;
  bool     _is_dc_info_in_packet_length = false;
  bool     _is_known_dc_type            = false;
  bool     _has_truncated_data          = false;
  bool     _has_binary                  = false;
  uint8_t  _max_data_body_length        = 0;
  dcInfo_t _dc_info[NUM_MAX_DC]         = {};

  uint8_t getDataBodyLength(void);
  Range_t getDataRange(const DcType_t dc_type);

  inline size_t getDataSize(const DcType_t dc_type) {
    if (dc_type == DcType_t::DC_TYPE_ANALOG_2BYTES) {
      return 2;
    } else if ((dc_type == DcType_t::DC_TYPE_ANALOG_1BYTE) || (dc_type == DcType_t::DC_TYPE_BOOLEAN) ||
               (dc_type == DcType_t::DC_TYPE_BINARY)) {
      return 1;
    }
    // unknown type and size
    return 0;
  };
};

#endif // VIVICORESERIAL_DATACODE_H

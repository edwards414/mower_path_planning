#include "modbus_rtu.hpp"

uint16_t ModbusRtu_Crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0U; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      if ((crc & 0x0001U) != 0U) {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

namespace {

void append_crc(uint8_t *frame, size_t len) {
  uint16_t crc = ModbusRtu_Crc16(frame, len);
  frame[len] = (uint8_t)(crc & 0xFFU);
  frame[len + 1U] = (uint8_t)(crc >> 8);
}

bool crc_matches(const uint8_t *frame, size_t len_with_crc) {
  uint16_t crc = ModbusRtu_Crc16(frame, len_with_crc - 2U);
  uint16_t wire = (uint16_t)(frame[len_with_crc - 2U] |
                             ((uint16_t)frame[len_with_crc - 1U] << 8));
  return crc == wire;
}

} // namespace

size_t ModbusRtu_BuildReadHolding(uint8_t *out, size_t out_size, uint8_t addr,
                                  uint16_t start_reg, uint16_t reg_count) {
  if ((out == nullptr) || (out_size < 8U)) {
    return 0U;
  }
  out[0] = addr;
  out[1] = MODBUS_FC_READ_HOLDING;
  out[2] = (uint8_t)(start_reg >> 8);
  out[3] = (uint8_t)(start_reg & 0xFFU);
  out[4] = (uint8_t)(reg_count >> 8);
  out[5] = (uint8_t)(reg_count & 0xFFU);
  append_crc(out, 6U);
  return 8U;
}

size_t ModbusRtu_BuildWriteMultiple(uint8_t *out, size_t out_size,
                                    uint8_t addr, uint16_t start_reg,
                                    const uint16_t *regs, uint16_t reg_count) {
  size_t data_bytes = (size_t)reg_count * 2U;
  size_t total = 7U + data_bytes + 2U;
  if ((out == nullptr) || (regs == nullptr) || (reg_count == 0U) ||
      (data_bytes > 0xFFU) || (out_size < total)) {
    return 0U;
  }
  out[0] = addr;
  out[1] = MODBUS_FC_WRITE_MULTIPLE;
  out[2] = (uint8_t)(start_reg >> 8);
  out[3] = (uint8_t)(start_reg & 0xFFU);
  out[4] = (uint8_t)(reg_count >> 8);
  out[5] = (uint8_t)(reg_count & 0xFFU);
  out[6] = (uint8_t)data_bytes;
  for (uint16_t i = 0U; i < reg_count; ++i) {
    out[7U + i * 2U] = (uint8_t)(regs[i] >> 8);
    out[8U + i * 2U] = (uint8_t)(regs[i] & 0xFFU);
  }
  append_crc(out, 7U + data_bytes);
  return total;
}

modbus_parse_result_t ModbusRtu_FindReadReply(const uint8_t *buf, size_t len,
                                              uint8_t addr, uint16_t *regs,
                                              uint16_t max_regs,
                                              modbus_read_reply_t *info) {
  if ((buf == nullptr) || (len < 5U)) {
    return MODBUS_PARSE_NONE;
  }

  for (size_t off = 0U; off + 5U <= len; ++off) {
    const uint8_t *f = buf + off;
    if (f[0] != addr) {
      continue;
    }

    if (f[1] == (MODBUS_FC_READ_HOLDING | MODBUS_EXCEPTION_BIT)) {
      /* addr, fc|0x80, exception, crc(2) */
      if (crc_matches(f, 5U)) {
        if (info != nullptr) {
          info->reg_count = 0U;
          info->exception_code = f[2];
          info->frame_offset = off;
        }
        return MODBUS_PARSE_EXCEPTION;
      }
      continue;
    }

    if (f[1] != MODBUS_FC_READ_HOLDING) {
      continue;
    }

    /* addr, fc, byte_count, data..., crc(2) */
    uint8_t byte_count = f[2];
    size_t frame_len = 3U + (size_t)byte_count + 2U;
    if ((byte_count == 0U) || ((byte_count & 1U) != 0U) ||
        (off + frame_len > len)) {
      continue;
    }
    if (!crc_matches(f, frame_len)) {
      continue;
    }

    uint16_t count = (uint16_t)(byte_count / 2U);
    if (regs != nullptr) {
      uint16_t n = (count < max_regs) ? count : max_regs;
      for (uint16_t i = 0U; i < n; ++i) {
        regs[i] = (uint16_t)(((uint16_t)f[3U + i * 2U] << 8) | f[4U + i * 2U]);
      }
    }
    if (info != nullptr) {
      info->reg_count = count;
      info->exception_code = 0U;
      info->frame_offset = off;
    }
    return MODBUS_PARSE_OK;
  }

  return MODBUS_PARSE_NONE;
}

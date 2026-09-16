#include "bl_protocol.h"
#include "bl_flash.h"
#include "boot_shared.h"
#include <string.h>

#define FRAME_SOF_0 0xA5U
#define FRAME_SOF_1 0x5AU
#define PROTOCOL_VERSION 0x01U
#define CRC_HEADER_SIZE 4U /* version, type, seq, payload_len */
/* Largest payload we accept: BL_WRITE header + data chunk. */
#define MAX_PAYLOAD (sizeof(boot_write_header_t) + BOOT_WRITE_CHUNK_MAX)

typedef enum {
  WAIT_SOF0 = 0,
  WAIT_SOF1,
  READ_VERSION,
  READ_TYPE,
  READ_SEQ,
  READ_LENGTH,
  READ_PAYLOAD,
  READ_CRC_LOW,
  READ_CRC_HIGH,
} parser_state_t;

typedef struct {
  parser_state_t state;
  uint8_t version;
  uint8_t type;
  uint8_t seq;
  uint8_t payload_len;
  uint8_t payload_index;
  uint8_t crc_low;
  uint8_t payload[MAX_PAYLOAD];
} parser_t;

static parser_t g_parser;
static bl_session_t *g_session;

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0U; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void send_frame(uint8_t type, uint8_t seq, const void *payload,
                       uint8_t payload_len) {
  uint8_t frame[8U + 32U];
  if (payload_len > 32U) {
    return;
  }

  frame[0] = FRAME_SOF_0;
  frame[1] = FRAME_SOF_1;
  frame[2] = PROTOCOL_VERSION;
  frame[3] = type;
  frame[4] = seq;
  frame[5] = payload_len;
  if (payload_len > 0U) {
    memcpy(&frame[6], payload, payload_len);
  }
  uint16_t crc = crc16_ccitt(&frame[2], (uint16_t)(CRC_HEADER_SIZE + payload_len));
  frame[6U + payload_len] = (uint8_t)(crc & 0xFFU);
  frame[7U + payload_len] = (uint8_t)(crc >> 8);

  BlUart_Send(frame, (uint16_t)(8U + payload_len));
}

static void send_ack(uint8_t command, uint8_t seq, uint8_t status,
                     uint32_t value) {
  boot_ack_payload_t ack;
  ack.command = command;
  ack.status = status;
  ack.value = value;
  send_frame(BOOT_FRAME_TYPE_BL_ACK, seq, &ack, (uint8_t)sizeof(ack));
}

static void send_info(uint8_t seq) {
  boot_info_payload_t info;
  memset(&info, 0, sizeof(info));
  info.app_start_address = BOOT_APP_START_ADDRESS;
  info.app_max_size = BOOT_APP_MAX_SIZE;
  info.write_chunk_max = BOOT_WRITE_CHUNK_MAX;
  info.bootloader_version = BOOT_VERSION;
  info.flags = 0U;
  if (g_session->app_valid) {
    info.flags |= BOOT_INFO_FLAG_APP_VALID;
  }
  if (g_session->entered_by_request) {
    info.flags |= BOOT_INFO_FLAG_ENTERED_BY_REQUEST;
  }
  if (g_session->erased) {
    info.flags |= BOOT_INFO_FLAG_ERASED;
  }
  send_frame(BOOT_FRAME_TYPE_BL_INFO, seq, &info, (uint8_t)sizeof(info));
}

static void handle_frame(uint8_t type, uint8_t seq, const uint8_t *payload,
                         uint8_t payload_len) {
  switch (type) {
  case BOOT_FRAME_TYPE_BL_PING:
    g_session->stay = true;
    send_info(seq);
    break;

  case BOOT_FRAME_TYPE_BL_ERASE: {
    g_session->stay = true;
    if (payload_len != sizeof(boot_erase_payload_t)) {
      send_ack(type, seq, BOOT_STATUS_BAD_ARGUMENT, 0U);
      break;
    }
    boot_erase_payload_t req;
    memcpy(&req, payload, sizeof(req));
    if ((req.image_size == 0U) || (req.image_size > BOOT_APP_MAX_SIZE)) {
      send_ack(type, seq, BOOT_STATUS_OUT_OF_RANGE, BOOT_APP_MAX_SIZE);
      break;
    }
    /* The app is gone from here on; never auto-jump into erased flash. */
    g_session->app_valid = false;
    g_session->erased = false;
    g_session->erased_size = 0U;
    if (!BlFlash_EraseForImage(req.image_size)) {
      send_ack(type, seq, BOOT_STATUS_FLASH_ERROR, 0U);
      break;
    }
    g_session->erased = true;
    g_session->erased_size = req.image_size;
    send_ack(type, seq, BOOT_STATUS_OK, req.image_size);
    break;
  }

  case BOOT_FRAME_TYPE_BL_WRITE: {
    g_session->stay = true;
    if (payload_len <= sizeof(boot_write_header_t)) {
      send_ack(type, seq, BOOT_STATUS_BAD_ARGUMENT, 0U);
      break;
    }
    boot_write_header_t header;
    memcpy(&header, payload, sizeof(header));
    const uint8_t *data = payload + sizeof(header);
    uint32_t data_len = (uint32_t)payload_len - sizeof(header);

    if (!g_session->erased) {
      send_ack(type, seq, BOOT_STATUS_NOT_ERASED, header.offset);
      break;
    }
    if (((data_len % 4U) != 0U) || (data_len > BOOT_WRITE_CHUNK_MAX) ||
        ((header.offset % 4U) != 0U)) {
      send_ack(type, seq, BOOT_STATUS_BAD_ARGUMENT, header.offset);
      break;
    }
    if ((header.offset >= g_session->erased_size) ||
        (data_len > (g_session->erased_size - header.offset))) {
      send_ack(type, seq, BOOT_STATUS_OUT_OF_RANGE, header.offset);
      break;
    }
    if (!BlFlash_Write(header.offset, data, data_len)) {
      send_ack(type, seq, BOOT_STATUS_FLASH_ERROR, header.offset);
      break;
    }
    send_ack(type, seq, BOOT_STATUS_OK, header.offset);
    break;
  }

  case BOOT_FRAME_TYPE_BL_VERIFY: {
    g_session->stay = true;
    if (payload_len != sizeof(boot_verify_payload_t)) {
      send_ack(type, seq, BOOT_STATUS_BAD_ARGUMENT, 0U);
      break;
    }
    boot_verify_payload_t req;
    memcpy(&req, payload, sizeof(req));
    if ((req.image_size == 0U) || (req.image_size > BOOT_APP_MAX_SIZE)) {
      send_ack(type, seq, BOOT_STATUS_OUT_OF_RANGE, 0U);
      break;
    }
    uint32_t crc = BlFlash_Crc32(BOOT_APP_START_ADDRESS, req.image_size);
    if (crc != req.crc32) {
      send_ack(type, seq, BOOT_STATUS_CRC_MISMATCH, crc);
      break;
    }
    g_session->app_valid = BlFlash_AppLooksValid();
    send_ack(type, seq,
             g_session->app_valid ? BOOT_STATUS_OK : BOOT_STATUS_NO_VALID_APP,
             crc);
    break;
  }

  case BOOT_FRAME_TYPE_BL_RUN_APP:
    if (!BlFlash_AppLooksValid()) {
      send_ack(type, seq, BOOT_STATUS_NO_VALID_APP, 0U);
      break;
    }
    send_ack(type, seq, BOOT_STATUS_OK, BOOT_APP_START_ADDRESS);
    g_session->run_app = true;
    break;

  default:
    /* Application frames (0x01-0x04 ...) are ignored while in the bootloader. */
    break;
  }
}

static void finalize_frame(uint8_t crc_high) {
  uint8_t crc_input[CRC_HEADER_SIZE + MAX_PAYLOAD];
  crc_input[0] = g_parser.version;
  crc_input[1] = g_parser.type;
  crc_input[2] = g_parser.seq;
  crc_input[3] = g_parser.payload_len;
  memcpy(&crc_input[CRC_HEADER_SIZE], g_parser.payload, g_parser.payload_len);

  uint16_t received = (uint16_t)g_parser.crc_low | ((uint16_t)crc_high << 8);
  uint16_t expected =
      crc16_ccitt(crc_input, (uint16_t)(CRC_HEADER_SIZE + g_parser.payload_len));

  if ((received == expected) && (g_parser.version == PROTOCOL_VERSION)) {
    handle_frame(g_parser.type, g_parser.seq, g_parser.payload,
                 g_parser.payload_len);
  }
}

void BlProtocol_Init(bl_session_t *session) {
  g_session = session;
  memset(&g_parser, 0, sizeof(g_parser));
  g_parser.state = WAIT_SOF0;
}

void BlProtocol_FeedByte(uint8_t byte) {
  switch (g_parser.state) {
  case WAIT_SOF0:
    if (byte == FRAME_SOF_0) {
      g_parser.state = WAIT_SOF1;
    }
    break;

  case WAIT_SOF1:
    if (byte == FRAME_SOF_1) {
      g_parser.state = READ_VERSION;
    } else if (byte != FRAME_SOF_0) {
      g_parser.state = WAIT_SOF0;
    }
    break;

  case READ_VERSION:
    g_parser.version = byte;
    g_parser.state = READ_TYPE;
    break;

  case READ_TYPE:
    g_parser.type = byte;
    g_parser.state = READ_SEQ;
    break;

  case READ_SEQ:
    g_parser.seq = byte;
    g_parser.state = READ_LENGTH;
    break;

  case READ_LENGTH:
    if (byte > MAX_PAYLOAD) {
      g_parser.state = WAIT_SOF0;
      break;
    }
    g_parser.payload_len = byte;
    g_parser.payload_index = 0U;
    g_parser.state = (byte == 0U) ? READ_CRC_LOW : READ_PAYLOAD;
    break;

  case READ_PAYLOAD:
    g_parser.payload[g_parser.payload_index++] = byte;
    if (g_parser.payload_index >= g_parser.payload_len) {
      g_parser.state = READ_CRC_LOW;
    }
    break;

  case READ_CRC_LOW:
    g_parser.crc_low = byte;
    g_parser.state = READ_CRC_HIGH;
    break;

  case READ_CRC_HIGH:
    finalize_frame(byte);
    g_parser.state = WAIT_SOF0;
    break;

  default:
    g_parser.state = WAIT_SOF0;
    break;
  }
}

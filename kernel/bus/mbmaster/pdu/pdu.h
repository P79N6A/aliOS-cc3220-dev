/*
 * Copyright (C) 2015-2018 Alibaba Group Holding Limited
 */

#ifndef PDU_H
#define PDU_H

#ifdef __cplusplus
extern "C" {
#endif

/* PDU len and payload offset define */
#define PDU_LENGTH_MAX     253
#define PDU_LENGTH_MIN     1
#define PDU_FUNC_FIELD_OFF 0
#define PDU_DATA_FIELD_OFF 1

/* slave addr define */
#define SLAVE_ADDR_BROADCAST 0
#define SLAVE_ADDR_MIN       1
#define SLAVE_ADDR_MAX       247

/* func code */
#define FUNC_CODE_READ_COILS             0x01
#define FUNC_CODE_READ_HOLDING_REGISTERS 0x03

/* exception code */
#define EXCEPTION_ILLEGAL_FUNCTION     0x01
#define EXCEPTION_ILLEGAL_DATA_ADDRESS 0x02
#define EXCEPTION_ILLEGAL_DATA_VALUE   0x03
#define EXCEPTION_SLAVE_DEVICE_FAILURE 0x04
#define EXCEPTION_ACKNOWLEDGE          0x05
#define EXCEPTION_SLAVE_DEVICE_BUSY    0x06
#define EXCEPTION_MEMORY_PARITY_ERROR  0x08
#define EXCEPTION_GATEWAY_PATH_FAILED  0x0A
#define EXCEPTION_GATEWAY_TGT_FAILED   0x0B

mb_status_t read_holding_reginster_assemble(mb_handler_t *req_handler, uint16_t start_addr, uint16_t quantity);
mb_status_t read_holding_reginster_disassemble(mb_handler_t *req_handler, uint8_t *respond_buf,
                                               uint8_t *respond_count);

#ifdef __cplusplus
}
#endif

#endif /* PDU_H */

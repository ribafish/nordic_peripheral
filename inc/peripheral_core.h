/*
 * peripheral_core.h
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 */

#ifndef PERIPHERAL_CORE_H_
#define PERIPHERAL_CORE_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	PERIPH_CORE_STATE_INIT,
	PERIPH_CORE_STATE_IDLE,
	PERIPH_CORE_STATE_NOTIF_SELFTEST,
	PERIPH_CORE_STATE_TEST_START,
	PERIPH_CORE_STATE_TEST_RUN,
	PERIPH_CORE_STATE_WAIT_NOTIF_DONE,
	PERIPH_CORE_STATE_TEST_TERMINATE,
	PERIPH_CORE_STATE_DELAY,
} periph_core_state_t;

typedef enum {
	PC_EVT_CTRL_WRITE,
	PC_EVT_DATA_NOTIF_SUBSCRIPTION,
	PC_EVT_CONNECTION,
	PC_EVT_DISCONNECTION,
	PC_EVT_DATA_WRITE,
	PC_EVT_SENT_DATA_READ_REPLY,
	PC_EVT_SENT_DATA_NOTIF,
}periph_core_evt_enum_t;

typedef struct {
	periph_core_evt_enum_t event;
	union {
		struct {
			uint16_t len;
			uint8_t * data;
		} ctrl;
		bool subscribed;
		struct{
			uint16_t len;
			uint8_t * data;
		} data_write;
		uint8_t data_read_reply_bytes;
	} params;
} periph_core_evt_t;

void peripheral_core_init();
void peripheral_core_update();

void peripheral_core_evt_handler(periph_core_evt_t evt);
void pc_ctrl_build_data(uint16_t uuid, uint8_t * data, uint8_t * len);

#endif /* PERIPHERAL_CORE_H_ */

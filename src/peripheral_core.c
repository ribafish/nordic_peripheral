/*
 * peripheral_core.c
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 */


#include "peripheral_core.h"
#include "utils.h"

// BLE header files
#include "ble_stack.h"
#include "peripheral_ble.h"
#include "control_commands.h"

#include "app_timer.h"
#include "test_params.h"

// Debug header files
#include "debug.h"
#include "app_error.h"

#include "bsp.h"

#ifdef DEBUG
#undef DEBUG
#endif

#define DEBUG	1
#define debug_line(...)  do { if (DEBUG>0) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_error(...)  do { if (DEBUG>0) { debug_errorline_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_L2(...)  do { if (DEBUG>1) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_data(...)  do { if (DEBUG>0) { debug_global(__VA_ARGS__); }} while (0)

#define DATALEN_MAX 512

// Variables
static periph_core_state_t state = PERIPH_CORE_STATE_INIT;
static periph_core_state_t state_old = 0xFF;
RINGBUF_U16_DECLARE_INIT(state_core_next, 16);

uint8_t data[DATALEN_MAX];
uint16_t datalen = 0;

test_params_t current_test;

uint32_t current_test_bytes_done = 0;
uint8_t last_sent_bytes_num = 0;
uint32_t output_counter = 0;
uint32_t delay_timestamp = 0;
uint32_t delay_time_ms = 0;

bool test_running = false;
bool notif_done = true;

// Forward function declarations
void timers_init();
periph_core_state_t get_next_state();
void queue_state(periph_core_state_t next_state);
void inject_state(periph_core_state_t next_state);
void delay_next_state(uint32_t ms, periph_core_state_t next_state);
void pc_ctrl_write_handler(uint16_t len, uint8_t * data);
void led_turn_on(bool on);


// Function bodies

void peripheral_core_init() {
	state = PERIPH_CORE_STATE_INIT;
}


void peripheral_core_update() {
	uint32_t ret_code;
	switch(state) {
	case PERIPH_CORE_STATE_INIT:
		// Initialize timer module
		timers_init();

		// Initialize BLE stack
		ble_stack_init();
		debug_line("Softdevice initialized");
		gap_params_init();
		debug_line("GAP params initialized");
		gatt_init();
		debug_line("GATT initialized");
		services_init();
		debug_line("Services initialized");

		conn_params_init();
		db_discovery_init();
		peer_manager_init();
		debug_line("BLE stack completely initialized\n");

		// Initialize timestamping clock
		clock_timer_init();

		peripheral_ble_init();

		ret_code = bsp_init(BSP_INIT_LED, NULL);
		APP_ERROR_CHECK(ret_code);

		// Initialize advertising module
		advertising_init();

		ble_stack_set_preferred_phy(BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED);

		debug_error("PERIPHERAL completely initialized\n");

		advertising_start(BLE_ADV_MODE_FAST, false);
		state = PERIPH_CORE_STATE_IDLE;
		break;
	case PERIPH_CORE_STATE_IDLE:
		led_turn_on(true);

		state = get_next_state();
		break;
	case PERIPH_CORE_STATE_TEST_START:
//		test_params_set_all(&current_test);

//		ret_code = ble_stack_set_phy(phy);
//	    if (ret_code != NRF_ERROR_BUSY) {
//			debug_line("Setting phy (rxtx) to %u: 0x%x", phy, err_code);
//			APP_ERROR_CHECK(err_code);
//	    }

		debug_line("Test started");
		test_running = true;
		current_test_bytes_done = 0;
		output_counter = 0;
		state = PERIPH_CORE_STATE_TEST_RUN;
		delay_next_state(current_test.conn_interval, PERIPH_CORE_STATE_TEST_RUN);
		break;
	case PERIPH_CORE_STATE_TEST_RUN:
		led_turn_on(false);
		if (current_test_bytes_done >= current_test.transfer_data_size) {
			debug_line("Test completed");
			delay_next_state(current_test.conn_interval*2+30, get_next_state());
			test_running = false;
		} else if (ringbuf_u16_peek(&state_core_next) == PERIPH_CORE_STATE_TEST_TERMINATE) {
			state = get_next_state();
		} else {
			switch(current_test.test_case) {
			case TEST_BLE_NOTIFY:
				test_params_build_data(&current_test, current_test_bytes_done, data, (uint8_t *)&datalen);
				last_sent_bytes_num = datalen;
				ret_code = peripheral_ble_notify(BLE_SERVICE_TEST, BLE_UUID_CHARA_DATA, data, datalen);
				if (ret_code == NRF_ERROR_BUSY) {
					notif_done = false;
					state = PERIPH_CORE_STATE_WAIT_NOTIF_DONE;
					inject_state(PERIPH_CORE_STATE_TEST_RUN);
				} else if (ret_code == NRF_SUCCESS) {
					current_test_bytes_done += last_sent_bytes_num;	// this will get sent
					if (current_test_bytes_done - output_counter >= current_test.transfer_data_size / 10) {
						debug_line("Notify tx %d/%d KB)", current_test_bytes_done/1024, current_test.transfer_data_size/1024);
						output_counter = current_test_bytes_done;
					}
				} else {
					debug_error("Notify error %x", ret_code);
					state = get_next_state();
				}
				break;
			default:
				// nothing do to, as it's controlled from central
				break;
			}
		}
		break;
	case PERIPH_CORE_STATE_NOTIF_SELFTEST:
		ret_code = peripheral_ble_notify(BLE_SERVICE_TEST, BLE_UUID_CHARA_DATA, data, datalen);
		if (ret_code == NRF_SUCCESS) {
			notif_done = false;
			state = PERIPH_CORE_STATE_WAIT_NOTIF_DONE;
		} else if (ret_code != NRF_ERROR_BUSY) {
			debug_error("Notify error %x", ret_code);
			state = get_next_state();
		}
		break;
	case PERIPH_CORE_STATE_WAIT_NOTIF_DONE:
		if (notif_done) {
			state = get_next_state();
		}
		break;
	case PERIPH_CORE_STATE_TEST_TERMINATE:
		debug_error("Terminate test. Done %d / %d KB", current_test_bytes_done, current_test.transfer_data_size);
		test_params_print(&current_test);
		state = PERIPH_CORE_STATE_IDLE;
		test_running = false;
		current_test_bytes_done = 0;
		output_counter = 0;
		//empty the queue
		while(ringbuf_u16_get_length(&state_core_next)) {
			ringbuf_u16_pop(&state_core_next);
		}
		break;
	case PERIPH_CORE_STATE_DELAY:
		if (clock_get_ms_since(delay_timestamp) > delay_time_ms) {
			state = get_next_state();
		}
		break;
	default:
		debug_error("Unknown periph state %d", state);
		break;
	}

	if (state_old != state) {
		state_old = state;
//		debug_line("State %d", state);
	}
}



void peripheral_core_evt_handler(periph_core_evt_t evt) {
	switch (evt.event) {
	case PC_EVT_CONNECTION:
		debug_line("Connected");
		break;
	case PC_EVT_DISCONNECTION:
		debug_line("Disconnected -> resetting the core");
		// Clean the state

		state = PERIPH_CORE_STATE_IDLE;
		test_running = false;
		current_test_bytes_done = 0;
		output_counter = 0;
		//empty the state queue
		while(ringbuf_u16_get_length(&state_core_next)) {
			ringbuf_u16_pop(&state_core_next);
		}
		break;
	case PC_EVT_DATA_NOTIF_SUBSCRIPTION:
		debug_line("Subscription to data char: %x", evt.params.subscribed);
		break;
	case PC_EVT_CTRL_WRITE:
		pc_ctrl_write_handler(evt.params.ctrl.len, evt.params.ctrl.data);
		break;
	case PC_EVT_DATA_WRITE:
//		debug_line("Data write received, len %d, current_test_bytes_done %d", evt.params.data_write.len, current_test_bytes_done);
		if (test_running) {
			current_test_bytes_done += evt.params.data_write.len;
			if (current_test_bytes_done - output_counter >= current_test.transfer_data_size / 10) {
				debug_line("Write rx %d/%d KB)", current_test_bytes_done/1024, current_test.transfer_data_size/1024);
				output_counter = current_test_bytes_done;
			}
		}
		break;
	case PC_EVT_SENT_DATA_READ_REPLY:
//		debug_line("Sent data read reply, len %d, current_test_bytes_done %d", evt.params.data_read_reply_bytes, current_test_bytes_done);
		if (test_running) {
			current_test_bytes_done += evt.params.data_read_reply_bytes;
			if (current_test_bytes_done - output_counter >= current_test.transfer_data_size / 10) {
				debug_line("Read tx %d/%d KB)", current_test_bytes_done/1024, current_test.transfer_data_size/1024);
				output_counter = current_test_bytes_done;
			}
		}
		break;
	case PC_EVT_SENT_DATA_NOTIF:
//		debug_line("Sent notif to data, len %d, current_test_bytes_done %d", last_sent_bytes_num, current_test_bytes_done);
		notif_done = true;
		break;
	default:
		debug_error("Unknown event %02x", evt.event);
		break;
	}
}

void pc_ctrl_write_handler(uint16_t len, uint8_t * datain) {
	uint8_t code = datain[0];
	len--;
	datain++;
	debug_line("Control write received. Code %02x", code);
	switch(code) {
	case CTRL_CMD_TEST_NOTIF:
		if (len > DATALEN_MAX) {
			debug_error("Incoming data too big (%d), truncating to %d", len, DATALEN_MAX);
			len = DATALEN_MAX;
		}
		memcpy(data,datain, len);	// copy what we got to notify back
		datalen = len;
		queue_state(PERIPH_CORE_STATE_NOTIF_SELFTEST);
		ringbuf_u16_printout(&state_core_next);
		break;
	case CTRL_CMD_WRITE_TEST_PARAMS:
		test_params_deserialize(datain, len, &current_test);
		debug_line("Received test params:");
		test_params_print(&current_test);
//		test_params_set_all(&current_test);

		break;
	case CTRL_CMD_START_TEST:
		queue_state(PERIPH_CORE_STATE_TEST_START);
		ringbuf_u16_printout(&state_core_next);
		break;
	case CTRL_CMD_TERMINATE_TEST:
		inject_state(PERIPH_CORE_STATE_TEST_TERMINATE);
		ringbuf_u16_printout(&state_core_next);
		break;
	default:
		debug_error("Unknown control code %02x", code);
		break;
	}
}

void pc_ctrl_build_data(uint16_t uuid, uint8_t * dataout, uint8_t * len) {
	switch(uuid) {
		case BLE_UUID_CHARA_CONTROL:
			debug_error("Reading from control!");
			*len = strlen("Test control!");
			strcpy((char *)dataout, "Test control!");
			break;
		case BLE_UUID_CHARA_DATA:
			if (test_running) {
				test_params_build_data(&current_test, current_test_bytes_done, dataout, len);
			} else {
				*len = strlen(TEST_READ_NOTIFY_STRING);
				strcpy((char *)dataout, TEST_READ_NOTIFY_STRING);
				debug_error("Building bogus test data");
			}
			break;
	}
	last_sent_bytes_num = *len;
}

// Helper functions ---------------------------------------------------------------------------

void timers_init()
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

periph_core_state_t get_next_state() {
	if (ringbuf_u16_get_length(&state_core_next) > 0) {
		return ringbuf_u16_pop(&state_core_next);
	} else {
		return PERIPH_CORE_STATE_IDLE;
	}
}

void queue_state(periph_core_state_t next_state) {
	ringbuf_u16_push(&state_core_next, next_state);
}

void inject_state(periph_core_state_t next_state) {
	ringbuf_u16_push_first(&state_core_next, next_state);
}


void delay_next_state(uint32_t ms, periph_core_state_t next_state) {
	delay_timestamp = clock_get_ms();
	delay_time_ms = ms;
	inject_state(next_state);
	state = PERIPH_CORE_STATE_DELAY;
}

void led_turn_on(bool on) {
	if (on) {
		bsp_board_led_off(BSP_BOARD_LED_0);
//		bsp_board_led_on(BSP_BOARD_LED_0);
//		bsp_board_led_on(BSP_BOARD_LED_1);
//		bsp_board_led_on(BSP_BOARD_LED_2);
//		bsp_board_led_on(BSP_BOARD_LED_3);
	} else {
		bsp_board_led_on(BSP_BOARD_LED_0);
//		bsp_board_led_off(BSP_BOARD_LED_0);
//		bsp_board_led_off(BSP_BOARD_LED_1);
//		bsp_board_led_off(BSP_BOARD_LED_2);
//		bsp_board_led_off(BSP_BOARD_LED_3);
	}
}


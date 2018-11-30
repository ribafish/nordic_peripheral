/*
 * peripheral_ble.c
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 */

#include <stdint.h>
#include "peripheral_ble.h"
#include "peripheral_core.h"
#include "ble_abstraction.h"
#include "nrf_log.h"
#include "app_error.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_dis.h"	// Device information service
#include "debug.h"


#define DEBUG	1
#define debug_line(...)  do { if (DEBUG>0) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_error(...)  do { if (DEBUG>0) { debug_errorline_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debugL2(...)  do { if (DEBUG>1) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_data(...)  do { if (DEBUG>2) { debug_global(__VA_ARGS__); }} while (0)

static ble_service_t				services[BLE_SERVICES_NUM];

static ble_gatts_char_handles_t 	test_chars[BLE_TEST_SERVICE_CHARA_NUM];

static bool							test_char_notify[BLE_TEST_SERVICE_CHARA_NUM];

static uint16_t						test_char_lookup[BLE_TEST_SERVICE_CHARA_NUM];

uint8_t request_data[255];


// Private function forward declarations
static void update_connection_handles(uint16_t conn_handle);
uint8_t find_service_by_uuid(uint16_t uuid);
static ret_code_t find_characteristic_with_cccd_handle(uint16_t cccd_handle, uint16_t * value_handle, uint16_t * uuid);
static ret_code_t find_characteristic_with_value_handle(uint16_t cccd_handle, uint16_t * uuid);
ret_code_t send_notification(ble_service_t * service, ble_gatts_char_handles_t * characteristic, uint8_t * data, uint16_t length, uint16_t uuid);

// Function bodies
void peripheral_ble_init() {
	uint8_t i;

	i = 0;
	ble_abstraction_service_init(&services[BLE_SERVICE_TEST], BLE_UUID_SERVICE_TEST, BLE_TEST_SERVICE_CHARA_NUM, test_chars, test_char_lookup, test_char_notify);
	ble_abstraction_chara_add(&services[BLE_SERVICE_TEST], BLE_UUID_CHARA_CONTROL, 			i++, BLE_READ|BLE_WRITE|BLE_VARIABLE_LEN, NRF_BLE_GATT_MAX_MTU_SIZE, 20, NULL);
	ble_abstraction_chara_add(&services[BLE_SERVICE_TEST], BLE_UUID_CHARA_DATA, 			i++, BLE_NOTIFY|BLE_READ|BLE_WRITE|BLE_VARIABLE_LEN, NRF_BLE_GATT_MAX_MTU_SIZE, 20, NULL);

}

/**@brief Function for building the data to be stored in a characteristic
 *
 * @details Use this function to parse received data and save it into the model.
 *
 * @param[in]	uuid					UUID of the characteristic to be built
 * @param[in]	p_data					Pointer to the data where the data will be saved
 * @param[in]	len						Length of the data
 *
 * @retval ::NRF_SUCCESS 				Successfully built the data
 * @retval ::NRF_ERROR_INVALID_PARAM	UUID does not belong to any supported characteristic
 */
static ret_code_t build_characteristic_data(uint16_t uuid, uint8_t * data, uint8_t * len) {
	switch(uuid) {
	case BLE_UUID_CHARA_CONTROL:
	case BLE_UUID_CHARA_DATA:
		pc_ctrl_build_data(uuid, data, len);
		return NRF_SUCCESS;
		break;
	default:
		debug_error("Couldn't build data for UUID 0x%04X", uuid);
		return NRF_ERROR_INVALID_PARAM;
	}
}

/**@brief Function for parsing data received over BLE.
 *
 * @details Use this function to parse received data and save it into the model.
 *
 * @param[in]	service_UUID			UUID of the service to which the data belongs to
 * @param[in]	char_UUID				UUID of the characteristic where the data was written to
 * @param[in]	p_data					Data to parse
 * @param[in]	len						Length of the data
 *
 * @retval ::NRF_SUCCESS 				Successfully parsed the data
 * @retval ::NRF_ERROR_INVALID_PARAM	UUID does not belong to any supported characteristic
 */
static ret_code_t parse_ble_data(uint16_t service_UUID, uint16_t char_UUID, uint8_t * p_data, uint16_t len) {
	periph_core_evt_t evt;
	debugL2("parse_ble_data service 0x%04x chara 0x%04x", service_UUID, char_UUID);
	switch(service_UUID) {
	case BLE_UUID_SERVICE_TEST:
		switch(char_UUID) {
		case BLE_UUID_CHARA_CONTROL:
#if DEBUG > 1
			debug_line("Control chara rx len %d data: ", len);
			for (uint8_t i=0; i<len; i++) {
				debug_global("%02x ", p_data[i]);
			}
			debug_global("\n");
#endif
			evt.event = PC_EVT_CTRL_WRITE;
			evt.params.ctrl.data = p_data;
			evt.params.ctrl.len = len;
			peripheral_core_evt_handler(evt);
			break;
		case BLE_UUID_CHARA_DATA:
#if DEBUG > 2
			debug_line("Data chara rx len %d data: ", len);
			for (uint8_t i=0; i<len; i++) {
				debug_global("%02x ", p_data[i]);
			}
			debug_global("\n");
#endif
			evt.event = PC_EVT_DATA_WRITE;
			evt.params.ctrl.data = p_data;
			evt.params.ctrl.len = len;
			peripheral_core_evt_handler(evt);
			break;
		default:
			return NRF_ERROR_INVALID_PARAM;
		}
		break;
	default:
		return NRF_ERROR_INVALID_PARAM;
	}

	return NRF_SUCCESS;

}

ret_code_t peripheral_ble_notify(ble_service_id_t service_id, uint16_t char_uuid, uint8_t * data, uint16_t length) {
	ret_code_t					err_code;


	ble_service_t * service = &services[service_id];
	uint8_t characteristic_index = ble_abstraction_find_chara_by_uuid(service, char_uuid);
	if (characteristic_index == 0xFF) {
		return NRF_ERROR_INVALID_PARAM;
	}

	ble_gatts_char_handles_t	* characteristic = &(service->char_handles[characteristic_index]);

	err_code = send_notification(service, characteristic, data, length, char_uuid);
	return err_code;
}

ret_code_t send_notification(ble_service_t * service, ble_gatts_char_handles_t * characteristic, uint8_t * data, uint16_t length, uint16_t uuid) {
	ret_code_t err_code = ble_abstraction_chara_notify(service, characteristic, length, data);
	if (err_code == NRF_SUCCESS) {
		debugL2("Notification queued");
//		debug_data("UUID: 0x%04X -> data = [ ", uuid);
//		for (uint16_t i = 0; i < length; i++) {
//			debug_data("%02X ", data[i]);
//		}
//		debug_data("]\n");
	} else if (err_code == NRF_ERROR_INVALID_STATE || err_code == BLE_ERROR_INVALID_CONN_HANDLE) {
		// Most likely the notification hasn't been enabled by the client (the app) or
		// the device is not connected to any central.
		debugL2("Send notification: UUID 0x%04X | Error = %d", uuid, err_code);
		return err_code;	// The GATT table was updated even though the notification failed

	} else if (err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
		err_code = sd_ble_gatts_sys_attr_set(service->conn_handle, NULL, 0, 0);
		debugL2("Notification error: Updating system attributes", err_code);
		return NRF_ERROR_BUSY;
	} else if (err_code == NRF_ERROR_RESOURCES) {
		debugL2("Notification error: Waiting for space in buffer", err_code);
		return NRF_ERROR_BUSY;
	}
	else if (err_code != NRF_SUCCESS) {
		debug_error("Notification error %d (UUID 0x%04X)", err_code, uuid);
	}

	return err_code;
}

// Event handlers -----------------------------------------------------------------------------

static void on_connect(ble_evt_t * p_ble_evt) {
	update_connection_handles(p_ble_evt->evt.gap_evt.conn_handle);
	periph_core_evt_t evt;
	evt.event = PC_EVT_CONNECTION;
	peripheral_core_evt_handler(evt);
}

static void on_disconnect(ble_evt_t * p_ble_evt) {
	update_connection_handles(BLE_CONN_HANDLE_INVALID);
	periph_core_evt_t evt;
	evt.event = PC_EVT_DISCONNECTION;
	peripheral_core_evt_handler(evt);
}

// For reads that need authorization from us
static void on_read_auth(ble_evt_t * p_ble_evt) {
	ret_code_t err_code = NRF_SUCCESS;
	ble_gatts_rw_authorize_reply_params_t auth_reply;
	uint8_t datalength = 0;
	static uint8_t * data = request_data;

	uint16_t charUUID = p_ble_evt->evt.gatts_evt.params.authorize_request.request.read.uuid.uuid;
	debugL2("on_read_auth - charUUID: 0x%04X", charUUID);

	build_characteristic_data(charUUID, data, &datalength);

	auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
	auth_reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
	auth_reply.params.read.update = 1;	// This has to be 1, otherwise the data is discarded
	auth_reply.params.read.offset = 0;
	auth_reply.params.read.len = datalength;
	auth_reply.params.read.p_data = data;
	err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
											   &auth_reply);
	APP_ERROR_CHECK(err_code);


	periph_core_evt_t evt;
	evt.event = PC_EVT_SENT_DATA_READ_REPLY;
	evt.params.data_read_reply_bytes = datalength;
	peripheral_core_evt_handler(evt);
}

// For writes that need authorization from us
static void on_write_auth(ble_evt_t * p_ble_evt) {
	ret_code_t err_code = NRF_SUCCESS;
	ble_gatts_rw_authorize_reply_params_t auth_reply;
	uint8_t * p_data = p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.data;
	uint16_t len = p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.len;

	debugL2("Got a write on UUID %04x: 0x%02x", p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.uuid.uuid, p_data[0]);

	uint16_t char_UUID = p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.uuid.uuid;
	uint16_t service_UUID = (char_UUID & 0xFF00) | 0x8000;

	err_code = parse_ble_data(service_UUID, char_UUID, p_data, len);

	if (err_code == NRF_SUCCESS) {
		auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
		auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
		auth_reply.params.write.update = 1;
		auth_reply.params.write.len = len;
		auth_reply.params.write.offset = p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.offset;
		auth_reply.params.write.p_data = p_data;
		err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
		APP_ERROR_CHECK(err_code);
	} else {
		debug_error("Write to UUID 0x%04X failed", char_UUID);
		auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
		auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
		err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
		APP_ERROR_CHECK(err_code);
	}
}

static void on_cccd_write(ble_evt_t * p_ble_evt) {
//	uint16_t value_handle, char_uuid, service_uuid;
//	if (find_characteristic_with_cccd_handle(p_ble_evt->evt.gatts_evt.params.write.handle, &value_handle, &char_uuid) != NRF_SUCCESS) {
//		return;
//	}

}

static void on_notification_subscription(ble_evt_t * p_ble_evt) {
	uint16_t value_handle, uuid;

	if (find_characteristic_with_cccd_handle(p_ble_evt->evt.gatts_evt.params.write.handle, &value_handle, &uuid) != NRF_SUCCESS) {
		debug_error("Characteristic for CCCD handle 0x%04X not found", p_ble_evt->evt.gatts_evt.params.write.handle);
		return;
	}

	debugL2("Subscribed UUID 0x%04X", uuid);

	uint16_t service_uuid = (uuid & 0x0F00) | 0x8000; 	// service UUIDs are marked by 1st two bytes, with first bit (MSB) always being 1 (0x8000)
	uint8_t service_id = find_service_by_uuid(service_uuid);

	ble_service_t * service = &services[service_id];

	uint8_t characteristic_index = ble_abstraction_find_chara_by_uuid(service, uuid);
	if (characteristic_index == 0xFF) {
		debug_error("Characteristic with UUID 0x%04X not found", uuid);
	}

	bool subscribed = p_ble_evt->evt.gatts_evt.params.write.data[0] == 0 ? false : true;

	if (uuid == BLE_UUID_CHARA_DATA) {
		periph_core_evt_t evt;
		evt.event = PC_EVT_DATA_NOTIF_SUBSCRIPTION;
		evt.params.subscribed = subscribed;
		peripheral_core_evt_handler(evt);
	}
	service->char_notify[characteristic_index] = subscribed;
	service->pending_notify = subscribed;
}

static void on_notification_sent(ble_evt_t * p_ble_evt) {
	periph_core_evt_t evt;
	evt.event = PC_EVT_SENT_DATA_NOTIF;
	peripheral_core_evt_handler(evt);
	debugL2("Notification sent");
}

void periph_on_ble_evt(ble_evt_t * p_ble_evt) {
    ret_code_t err_code = NRF_SUCCESS;

    uint16_t request_UUID = p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.uuid.uuid;

	switch(p_ble_evt->header.evt_id) {
		case BLE_GAP_EVT_CONNECTED:
			on_connect(p_ble_evt);
			break;
		case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_ble_evt);
			break;
		case BLE_GATTS_EVT_WRITE:;
			uint16_t char_UUID;
			if (find_characteristic_with_value_handle(p_ble_evt->evt.gatts_evt.params.write.handle, &char_UUID) == NRF_SUCCESS) {
				uint16_t service_UUID = (char_UUID & 0xFF00) | 0x8000;

				err_code = parse_ble_data(service_UUID, char_UUID, p_ble_evt->evt.gatts_evt.params.write.data, p_ble_evt->evt.gatts_evt.params.write.len);

				if (err_code != NRF_SUCCESS) {
					debug_error("error parse_ble_data service 0x%04x chara 0x%04x", service_UUID, char_UUID);
				}

			} else { // not a write to value, so probably to CCCD
				debug_line("Write to uuid %x handle %x",
						p_ble_evt->evt.gatts_evt.params.write.uuid.uuid,
						p_ble_evt->evt.gatts_evt.params.write.handle);

				// Here we can keep track of writes to CCCDs (enabling and disabling notifications)
				on_cccd_write(p_ble_evt);

				if (p_ble_evt->evt.gatts_evt.params.write.data[0] == 1) {
					// Notification was enabled
					on_notification_subscription(p_ble_evt);
				}
			}
			break;

		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:

			switch (p_ble_evt->evt.gatts_evt.params.authorize_request.type) {
				case BLE_GATTS_AUTHORIZE_TYPE_WRITE:
					on_write_auth(p_ble_evt);
					break;
				case BLE_GATTS_AUTHORIZE_TYPE_READ:
					on_read_auth(p_ble_evt);
					break;
				default:; // Intentional empty statement
					ble_gatts_rw_authorize_reply_params_t auth_reply;
					auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_INVALID;
					err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
															   &auth_reply);
					APP_ERROR_CHECK(err_code);
					break;
			}
			break;

		case BLE_GATTS_EVT_HVN_TX_COMPLETE:
			// A notification was successfully sent
			on_notification_sent(p_ble_evt);
			break;

		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
			err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
			if (err_code != NRF_SUCCESS) {
				debug_error("Failed setting system attributes with error code 0x%04X", err_code);
			} else {
				debug_line("System attributes set");
			}
			break;
	}
}

// End of event handlers ----------------------------------------------------------------------

// Helper functions ---------------------------------------------------------------------------

static void update_connection_handles(uint16_t conn_handle) {
	for (uint8_t i = 0; i < BLE_SERVICES_NUM; i++) {
		services[i].conn_handle = conn_handle;
	}
}

uint8_t find_service_by_uuid(uint16_t uuid) {
	for (uint8_t i = 0; i < BLE_SERVICES_NUM; i++) {
		if (services[i].uuid == uuid) {
			return i;
		}
	}

	if (uuid == 0x2A00 || uuid == 0xFF00) {
		// Most likely characteristics belonging to Device Information Service (0x180A)
		debugL2("DIS UUID?");
	} else {
		debug_error("ERROR: Can't find service with UUID 0x%04X", uuid);
	}
	return 0xFF;
}

static ret_code_t find_characteristic_with_cccd_handle(uint16_t cccd_handle, uint16_t * value_handle, uint16_t * uuid) {

	for (uint8_t i = 0; i < BLE_SERVICES_NUM; i++) {
		for (uint8_t j = 0; j < services[i].char_num; j++) {
			if (services[i].char_handles[j].cccd_handle == cccd_handle) {
				*value_handle = services[i].char_handles[j].value_handle;
				*uuid = services[i].char_lookup_table[j];
				return NRF_SUCCESS;
			}
		}
	}

	return NRF_ERROR_NOT_FOUND;
}

static ret_code_t find_characteristic_with_value_handle(uint16_t cccd_handle, uint16_t * uuid) {

	for (uint8_t i = 0; i < BLE_SERVICES_NUM; i++) {
		for (uint8_t j = 0; j < services[i].char_num; j++) {
			if (services[i].char_handles[j].value_handle == cccd_handle) {
				*uuid = services[i].char_lookup_table[j];
				return NRF_SUCCESS;
			}
		}
	}

	return NRF_ERROR_NOT_FOUND;
}


/*
 * peripheral_ble.h
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 */

#ifndef PERIPHERAL_BLE_H_
#define PERIPHERAL_BLE_H_

#include "ble.h"
#include "sdk_config.h"
#include "ble_uuid.h"
#include "sdk_errors.h"


#define OPCODE_LENGTH				1
#define HANDLE_LENGTH				2

// Maximum length of data (in bytes) that can be transmitted to the peer by the thumbnail service
#if defined(NRF_BLE_GATT_MAX_MTU_SIZE) && (NRF_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_THUMBNAIL_TX_CHAR_MAX_LEN (NRF_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_THUMBNAIL_TX_CHAR_MAX_LEN (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif



// Position in the BLE services array -> serves as priority for generic operations (e.g. look-ups)
typedef enum {
	BLE_SERVICE_TEST
} ble_service_id_t;


void peripheral_ble_init();

void periph_on_ble_evt(ble_evt_t * p_ble_ev);
void peripheral_ble_set_max_data_length(uint8_t new_att_mtu);

ret_code_t peripheral_ble_notify(ble_service_id_t service_id, uint16_t char_uuid, uint8_t * data, uint16_t length);
#endif /* PERIPHERAL_BLE_H_ */

/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <stdint.h>
#include <string.h>
#include "custom_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial.
 *
 * @param[in]   p_custom_service        Our Service structure.
 *
 */
static uint32_t custom_char_add(ble_os_t * p_custom_service)
{
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_CUSTOM_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_CUSTOM_CHARACTERISTIC;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    
    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
    
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    attr_char_value.max_len     = 4;
    attr_char_value.init_len    = 4;
    uint8_t value[4]            = {0x12,0x34,0x56,0x78};
    attr_char_value.p_value     = value;
    
    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_custom_service->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_custom_service->char_handles);
    APP_ERROR_CHECK(err_code);
    
    
    return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_custom_service        Our Service structure.
 *
 */
void custom_service_init(ble_os_t * p_custom_service)
{
    // STEP 3: Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table  - DONE
    
    uint32_t   err_code;
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_CUSTOM_BASE_UUID;
    service_uuid.uuid = BLE_UUID_CUSTOM_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);  // Add the service to the BLE stack, vs_add is because we add the vendor uuid
    if (err_code != NRF_SUCCESS) {
    	NRF_LOG_INFO("sd_ble_uuid_vs_add error 0x%x\r\n", err_code);
    	APP_ERROR_CHECK(err_code);
    }
    
    // STEP 4: Add our service - DONE
    
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_custom_service->service_handle);
    if (err_code != NRF_SUCCESS) {
        	NRF_LOG_INFO("sd_ble_gatts_service_add error 0x%x\r\n", err_code);
        	APP_ERROR_CHECK(err_code);
        }
    
    custom_char_add(p_custom_service);
    
    p_custom_service->conn_handle = BLE_CONN_HANDLE_INVALID;
    

    // Print messages to Segger Real Time Terminal
    // UNCOMMENT THE FOUR LINES BELOW AFTER INITIALIZING THE SERVICE OR THE EXAMPLE WILL NOT COMPILE.
    NRF_LOG_INFO("Executed our_service_init().\n"); // Print message to RTT to the application flow
    NRF_LOG_INFO("Service UUID: 0x%#04x\n", service_uuid.uuid); // Print service UUID should match definition BLE_UUID_OUR_SERVICE
    NRF_LOG_INFO("Service UUID type: 0x%#02x\n", service_uuid.type); // Print UUID type. Should match BLE_UUID_TYPE_VENDOR_BEGIN. Search for BLE_UUID_TYPES in ble_types.h for more info
    NRF_LOG_INFO("Service handle: 0x%#04x\n", p_custom_service->service_handle); // Print out the service handle. Should match service handle shown in MCP under Attribute values
}

/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_os_t * p_custom_service, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_custom_service->char_handles.value_handle) &&
        (p_custom_service->write_handler != NULL))
    {
        p_custom_service->write_handler(p_custom_service, p_evt_write->len, p_evt_write->data);
    }
}

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_custom_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_custom_service_on_ble_evt(ble_os_t * p_custom_service, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_custom_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_custom_service->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_custom_service, p_ble_evt);
            break;
        default:
            // No implementation needed.
            break;
    }

}

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_custom_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void custom_uart_characteristic_update(ble_os_t *p_custom_service, int32_t *temperature_value)
{
    if (p_custom_service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t               len = 4;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));
        
        hvx_params.handle = p_custom_service->char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)temperature_value;
        
        sd_ble_gatts_hvx(p_custom_service->conn_handle, &hvx_params);
    }
}


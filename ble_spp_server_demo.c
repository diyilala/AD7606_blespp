/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"


#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "ble_spp_server_demo.h"

#include "time.h"
#include "esp_timer.h"
#include "sys/time.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "driver/spi_master.h"
#include "ad7606.h"

#define GATTS_TABLE_TAG  "GATTS_SPP_DEMO"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          "ESP_SPP_SERVER"    //The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID	            0

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1 //SPP_DATA_RECV_CHAR     READ&WRITE_NR
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2 //SPP_DATA_NOTIFY_CHAR   READ&NOTIFY
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3 //SPP_COMMAND_CHAR       READ&WRITE_NR
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4 //SPP_STATUS_CHAR        READ&WRITE_NR&NOTIFY

#ifdef SUPPORT_HEARTBEAT
#define ESP_GATT_UUID_SPP_HEARTBEAT         0xABF5 //SPP_HEARTBEAT_CHAR     READ&WRITE_NR&NOTIFY
#endif

static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0F,0x09, 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R','V', 'E', 'R'  //?????????????????????????????????????????????????????????
};

// static uint16_t spp_mtu_size = 23; //MTU???????????????????????????????????????Protocol Data Unit, PDU) ?????????????????????Byte/
static uint16_t spp_mtu_size = 43;

static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static xQueueHandle cmd_cmd_queue = NULL;

esp_err_t ret;
spi_device_handle_t spi;    //spi??????????????????
QueueHandle_t XQueue_SPI;   //spi????????????????????????????????????????????????
esp_timer_handle_t timer1;  //?????????1

/*???????????????????????????*/
static xQueueHandle lc_cmd_queue = NULL;
/*???????????????????????????*/

/*
  * spp_uart_queue       - Uart data messages received from the Uart    //???Uart?????????Uart????????????
  * cmd_cmd_queue        - commands received from the client    //??????????????????????????????
  * cmd_heartbeat_queue  - heartbeat received, if supported  //??????
*/

#ifdef SUPPORT_HEARTBEAT
static xQueueHandle cmd_heartbeat_queue = NULL;
static uint8_t  heartbeat_s[9] = {'E','s','p','r','e','s','s','i','f'};
static bool enable_heart_ntf = false;
static uint8_t heartbeat_count_num = 0;
#endif

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};//????????????

static uint16_t spp_handle_table[SPP_IDX_NB];//????????????

static esp_ble_adv_params_t spp_adv_params = { //????????????????????????
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;//????????????
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;//?????????
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT 
 *????????????gatts????????????????????????app id?????????gatts if???????????????????????????ESP gatts REG EVT?????????gatts
 */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_HEARTBEAT
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

#ifdef SUPPORT_HEARTBEAT
///SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t  spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t  spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

#ifdef SUPPORT_HEARTBEAT
    //SPP -  Heart beat characteristic Declaration
    [SPP_IDX_SPP_HEARTBEAT_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    //SPP -  Heart beat characteristic Value
    [SPP_IDX_SPP_HEARTBEAT_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_heart_beat_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(spp_heart_beat_val), sizeof(spp_heart_beat_val), (uint8_t *)spp_heart_beat_val}},

    //SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_HEARTBEAT_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_heart_beat_ccc}},
#endif
};

static uint8_t find_char_and_desr_index(uint16_t handle)//??????????????????????????????
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if(temp_spp_recv_data_node_p1 == NULL){
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if(temp_spp_recv_data_node_p2 != NULL){
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff,p_data->write.value,p_data->write.len);
    if(SppRecvDataBuff.node_num == 0){
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }else{
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

void uart_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t total_num = 0;//?????????
    uint8_t current_num = 0;//????????????

    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {//(???????????????????????????buffer?????????????????????)
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                if ((event.size)&&(is_connected)) {
                    uint8_t * temp = NULL;
                    uint8_t * ntf_value_p = NULL;
#ifdef SUPPORT_HEARTBEAT
                    if(!enable_heart_ntf){
                        ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable heartbeat Notify\n", __func__);
                        break;
                    }
#endif
                    if(!enable_data_ntf){
                        ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable data Notify\n", __func__);
                        break;
                    }
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    if(temp == NULL){
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.1 failed\n", __func__);
                        break;
                    }
                    memset(temp,0x0,event.size);//?????????????????????n??????????????????????????????
                    uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);//UART???UART????????????????????????
                    if(event.size <= (spp_mtu_size - 3)){
                        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],event.size, temp, false);
                    }else if(event.size > (spp_mtu_size - 3)){
                        if((event.size%(spp_mtu_size - 7)) == 0){
                            total_num = event.size/(spp_mtu_size - 7);
                        }else{
                            total_num = event.size/(spp_mtu_size - 7) + 1;
                        }
                        current_num = 1;
                        ntf_value_p = (uint8_t *)malloc((spp_mtu_size-3)*sizeof(uint8_t));
                        if(ntf_value_p == NULL){
                            ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.2 failed\n", __func__);
                            free(temp);
                            break;
                        }
                        while(current_num <= total_num){
                            if(current_num < total_num){
                                ntf_value_p[0] = '#';
                                ntf_value_p[1] = '#';
                                ntf_value_p[2] = total_num;
                                ntf_value_p[3] = current_num;
                                memcpy(ntf_value_p + 4,temp + (current_num - 1)*(spp_mtu_size-7),(spp_mtu_size-7));
                                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],(spp_mtu_size-3), ntf_value_p, false);
                            }
                            else if(current_num == total_num){
                                ntf_value_p[0] = '#';
                                ntf_value_p[1] = '#';
                                ntf_value_p[2] = total_num;
                                ntf_value_p[3] = current_num;
                                memcpy(ntf_value_p + 4,temp + (current_num - 1)*(spp_mtu_size-7),(event.size - (current_num - 1)*(spp_mtu_size - 7)));
                                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],(event.size - (current_num - 1)*(spp_mtu_size - 7) + 4), ntf_value_p, false);
                            }
                            vTaskDelay(2 / portTICK_PERIOD_MS);//???20MS
                            current_num++;
                        }
                        free(ntf_value_p);
                    }
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        // .baud_rate = 115200,//?????????
        .baud_rate = 1500000,
        .data_bits = UART_DATA_8_BITS,//?????????
        .parity = UART_PARITY_DISABLE,//??????????????????
        .stop_bits = UART_STOP_BITS_1,//????????????1
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,//????????????
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,//APB??????
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10,&spp_uart_queue,0);
    //(uart_port_tuart_num, int rx_buffer_size, int tx_buffer_size, int queue_size, QueueHandle_t *uart_queue, int intr_alloc_flags)
    //???????????????????????????????????????????????????????????????????????????????????????????????????????????????

    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);//??????uart0?????????
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //???uart????????????????????????GPIO?????????UART_PIN_NO_CHANGE???????????????????????????
    xTaskCreate(uart_task, "uTask", 2048, (void*)UART_NUM_0,tskIDLE_PRIORITY, NULL);
}

#ifdef SUPPORT_HEARTBEAT
void spp_heartbeat_task(void * arg)
{
    uint16_t cmd_id;

    for(;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_heartbeat_queue, &cmd_id, portMAX_DELAY)) {
            while(1){
                heartbeat_count_num++;
                vTaskDelay(5000/ portTICK_PERIOD_MS);
                if((heartbeat_count_num >3)&&(is_connected)){
                    esp_ble_gap_disconnect(spp_remote_bda);
                }
                if(is_connected && enable_heart_ntf){
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_HEARTBEAT_VAL],sizeof(heartbeat_s), heartbeat_s, false);
                }else if(!is_connected){
                    break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}
#endif

void AD7606_IOset()
{
    gpio_set_direction(PIN_NUM_RST,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_CVAB,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_CS,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BUSY,GPIO_MODE_INPUT);
   
    gpio_pullup_en(PIN_NUM_BUSY);
    gpio_pulldown_dis(PIN_NUM_BUSY);

    gpio_set_direction(PIN_NUM_RAGE,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_OS0,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_OS1,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_OS2,GPIO_MODE_OUTPUT);
   
    gpio_set_level(PIN_NUM_CS,1);
    gpio_set_level(PIN_NUM_CVAB,1);
    gpio_set_level(PIN_NUM_RST,0);
}

void AD7606_Init()
{
    AD7606_IOset();
    AD7606_SetInputRange(1);
    // AD7606_SetInputRange(1);//?????????-10~+10???

    AD7606_OSset();
    AD7606_Reset();
}

//?????????????????????0????????5V, 1????????10V.
void AD7606_SetInputRange(int range)
{
    gpio_set_level(PIN_NUM_RAGE,range);
}

/*??????????????????
 *OS[2:0]:
 *000???????????????????????????200Ksps????????????:????????????
 *001??????2??????????????? ???????????????????????????2??????????????????:????????????
 *010??????4??????????????? ???????????????????????????4??????????????????:???????????????
 *011??????8??????????????? ???????????????????????????8??????????????????:???????????????
 *100??????16??????????????? ???????????????????????????16??????????????????:???????????????
 *101??????32??????????????? ???????????????????????????32??????????????????:???????????????
 *110??????64??????????????? ???????????????????????????64??????????????????:???????????????
 */
void AD7606_OSset()
{
    gpio_set_level(PIN_NUM_OS0,0);
    gpio_set_level(PIN_NUM_OS1,0);
    gpio_set_level(PIN_NUM_OS2,0);
}

/*ESP32??????????????????????????????300ns*/
void AD7606_Reset()
{
    gpio_set_level(PIN_NUM_CS,1);
    gpio_set_level(PIN_NUM_CVAB,1);
    gpio_set_level(PIN_NUM_RST,0);
    gpio_set_level(PIN_NUM_RST,1);
    gpio_set_level(PIN_NUM_RST,0);//reset?????????????????????50ns
}

void AD7606_StartConv()
{
    // gpio_set_level(PIN_NUM_CVAB,0);
    // gpio_set_level(PIN_NUM_CVAB,0);
    gpio_set_level(PIN_NUM_CVAB,0);
    gpio_set_level(PIN_NUM_CVAB,1);
}

//ESP32???spi??????
esp_err_t  spi_read(spi_device_handle_t spi, uint8_t *data)
{
    spi_transaction_t t;
    gpio_set_level(PIN_NUM_CS, 0);
    memset(&t, 0, sizeof(t));
    t.length=8;
    // t.length=8;//??????????????????????????????
    t.rxlength=8;
    t.flags = SPI_TRANS_USE_RXDATA;
    // t.user = (void*)1;
    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );
    *data = t.rx_data[0];
    gpio_set_level(PIN_NUM_CS, 1);
    return ret;
}

// spi????????????
// void TaskSPI(void *pvParameters)
// // void TaskSPI()
// {
//     AD7606_Init();//ad?????????
   
//     while (1)
//     {      
//         /*??????????????????*/
//         if( gpio_get_level(PIN_NUM_BUSY)==0 )
//             AD7606_StartConv();
    
//         for (int i = 0; i < CH_NUM*2; i++) 
//             {
                
//                 ret = spi_read(spi, &ADC_val[i]);
//                 ESP_ERROR_CHECK(ret);
//             }
   
//         xQueueSend( XQueue_SPI, ADC_val, 0 );
//         /*??????????????????*/

//             // ESP_LOGI(TAG, "Read: %s", ADC_val);
//             // vTaskDelay(1);//??????????????????????????????????????????????????????freeRTOS??????1us??????
               
//     }    
   
// }


void timer1Interrupt_SPI(void *args){ 
    if( gpio_get_level(PIN_NUM_BUSY)==0 )
            AD7606_StartConv();
    
        for (int i = 0; i < CH_NUM*2; i++) 
            {
                
                ret = spi_read(spi, &ADC_val[i]);
                ESP_ERROR_CHECK(ret);
            }
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],16*sizeof(uint8_t), ADC_val, false);
    
}


// void TaskRev(void *pvParameters)//??????????????????????????????????????????????????????
// {
    
//     while (1)
//     {     
//         xQueueReceive( XQueue_SPI, spp_data, portMAX_DELAY );//??????
//     }    
   
// }

// void spp_cmd_task(void * arg)//???????????????????????????????????????????????????
// {
//     uint8_t * cmd_id;

//     for(;;){
//         vTaskDelay(50 / portTICK_PERIOD_MS);
//         if(xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY)) {
//             esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(cmd_id),strlen((char *)cmd_id));

//             // if(&cmd_id==888){
//             //   xQueueSend(spp_uart_queue, &cmd_id, (portTickType)portMAX_DELAY);     
//             // }//?????????????????????????????????service?????????888????????????service??????????????????888 
  
            
//             free(cmd_id);
            
//         }
//     }
//     vTaskDelete(NULL);
// }

/*???????????????????????????*/
void lc_cmd_task(void * arg)//???????????????????????????????????????????????????
{
    uint32_t *lc_cmd_id=NULL;
    // uint_64t *lc_cmd_id=NULL;
    // int lc_cmd[20];
    

    while (1)
    {
        // vTaskDelay(50 / portTICK_PERIOD_MS);

        // xQueueReceive( XQueue_SPI, spp_data, portMAX_DELAY );//??????
        
        // printf("??????????????????1"); 
                
        if(xQueueReceive(lc_cmd_queue, (void *)&lc_cmd_id, 0)) { //??????wait???????????????0????????????????????????????????????
            printf("??????????????????2");
            uint32_t *tmp = NULL;
            tmp= (uint32_t *)malloc(4*sizeof(uint32_t));
                if(tmp == NULL){
                    ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                    break;
                }
            memset(tmp,0x0,4);
            memcpy(tmp,lc_cmd_id,sizeof(&lc_cmd_id));
    
            if(*tmp == 49u)//??????1???ASCII???
            // if(!strcmp(lc_cmd_id,"li"))
            {
                /*????????????????????????*/
                // uint16_t tmp1[8];
                // float disp[8];
                // float *disp1;
                // disp1 = (float*)malloc(sizeof(float)*8);
                // memset(disp1,0,sizeof(float)*8);
                // for (int i=0;i<CH_NUM;i++)
                // {
                //     tmp1[i]=spp_data[2 * i + 1] + (spp_data[2 * i] << 8);
                //     disp[i] = 1.0 * tmp1[i] * 20 / 65535; 
                //     if(disp[i] >= 10)
                //         {
                //             disp[i] = 20 - disp[i];
                //             disp[i] = -disp[i];
                //         }
                
                // }
                // memcpy(disp1,disp,sizeof(float)*8);
                /*????????????????????????*/

                esp_timer_start_periodic(timer1, 1000);//?????????1?????????????????????????????????????????????????????????us
                // esp_timer_start_periodic(timer1, 2000);//?????????1?????????????????????????????????????????????????????????us

                // esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],16*sizeof(uint8_t), spp_data, false);
                //??????????????????????????????????????????
                // esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],8*sizeof(disp1), disp1, false);
                
                // free(spp_2data);
                // free(disp1);


                // esp_log_buffer_hex("?????????", disp, sizeof(disp));
                // printf("??????????????????client??????4\n");

                // float a[8] = {6.91,02u,03u,04u,05u,6.91,30u,18u};
                // float *p = NULL;
                // p = a;
                // float *disp = NULL;
                // disp=(float *)malloc(8*sizeof(float));
                //     if(disp == NULL){
                //         ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                //         break;
                //     }
                // memset(disp,0x0,8*sizeof(float));
                // memcpy(disp,p,8*sizeof(float));
                //????????????????????????????????????????????????????????????20220715
            }

            else if(*tmp == 50u)
            {
                esp_timer_stop(timer1);
            }

            else if(*tmp == 51u)
            {
                if( gpio_get_level(PIN_NUM_BUSY)==0 )
                    AD7606_StartConv();
    
                for (int i = 0; i < CH_NUM*2; i++) 
                    {
                
                         ret = spi_read(spi, &ADC_val[i]);
                        ESP_ERROR_CHECK(ret);
                    }
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],16*sizeof(uint8_t), ADC_val, false);
            }

            else
            {   
                printf("lc_cmd_id?????????????????????\n");
                printf("*tmp:%d\n",*tmp);

                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],2*sizeof(&lc_cmd_id), lc_cmd_id, false);

            }
  
            
            free(lc_cmd_id);
            free(tmp);
            
        }
        else 
        {
            // printf("????????????????????????");
        }
    }
    // vTaskDelete(NULL);
}
/*???????????????????????????*/

static void spp_task_init(void) //??????cmd_cmd_queue???spp_cmd_task
{
    spp_uart_init();

#ifdef SUPPORT_HEARTBEAT
    cmd_heartbeat_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_heartbeat_task, "spp_heartbeat_task", 2048, NULL, 10, NULL);
#endif

    // cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    // xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, tskIDLE_PRIORITY, NULL);

    /*???????????????????????????*/
    lc_cmd_queue = xQueueCreate(500, sizeof(uint32_t));
    xTaskCreate(lc_cmd_task, "lc_cmd_task", 2048, NULL, tskIDLE_PRIORITY , NULL);
    /*???????????????????????????*/
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)//??????????????????
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    //ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n",event);
    //???????????????????????????????????????
    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);//??????????????????????????????
            if(res == SPP_IDX_SPP_STATUS_VAL){
                //TODO:client read the status characteristic
            }
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);//?????????????????????????????????handle==7??????SPP_IDX_SPP_COMMAND_VAL??????????????????
            if(p_data->write.is_prep == false){
                //ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
                //????????????????????????????????????
                if(res == SPP_IDX_SPP_COMMAND_VAL){
                    uint8_t * spp_cmd_buff = NULL;
                    spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                    if(spp_cmd_buff == NULL){
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                        break;
                    }
                    memset(spp_cmd_buff,0x0,(spp_mtu_size - 3));
                    memcpy(spp_cmd_buff,p_data->write.value,p_data->write.len);
                    xQueueSend(cmd_cmd_queue,&spp_cmd_buff,10/portTICK_PERIOD_MS);//????????????
                }else if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = false;
                    }
                }
#ifdef SUPPORT_HEARTBEAT
                else if(res == SPP_IDX_SPP_HEARTBEAT_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_heart_ntf = false;
                    }
                }else if(res == SPP_IDX_SPP_HEARTBEAT_VAL){
                    if((p_data->write.len == sizeof(heartbeat_s))&&(memcmp(heartbeat_s,p_data->write.value,sizeof(heartbeat_s)) == 0)){
                        heartbeat_count_num = 0;
                    }
                }
#endif
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){//????????????????????????
#ifdef SPP_DEBUG_MODE
                    esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(p_data->write.value),p_data->write.len);
#else

                    /*?????????????????????????????????*/
                    uint32_t * lc_cmd_buff = NULL;
                    lc_cmd_buff = (uint32_t *)malloc(8*sizeof(uint32_t));
                    if(lc_cmd_buff == NULL){
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                        break;
                    }
                    memset(lc_cmd_buff,0x0,8);
                    memcpy(lc_cmd_buff,p_data->write.value,p_data->write.len);
                    xQueueSend(lc_cmd_queue,(void *)&lc_cmd_buff,0);//???portMAX_DELAY
                    /*?????????????????????????????????*/

                    uart_write_bytes(UART_NUM_0, (char *)(p_data->write.value), p_data->write.len);
#endif
                }else{
                    //TODO:
                }
            }else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)){
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
                store_wr_buffer(p_data);
            }
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
    	    if(p_data->exec_write.exec_write_flag){
    	        print_write_buffer();
    	        free_write_buffer();
    	    }
    	    break;
    	}
    	case ESP_GATTS_MTU_EVT:
    	    spp_mtu_size = p_data->mtu.mtu;
    	    break;
    	case ESP_GATTS_CONF_EVT:
        /*?????????????????????????????????*/
            printf("server???????????????\n");//???server???client????????????????????????server???
            

        /*?????????????????????????????????*/
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    	    spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;
    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
#ifdef SUPPORT_HEARTBEAT
    	    uint16_t cmd = 0;
            xQueueSend(cmd_heartbeat_queue,&cmd,10/portTICK_PERIOD_MS);
#endif
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
    	    is_connected = false;
    	    enable_data_ntf = false;
#ifdef SUPPORT_HEARTBEAT
    	    enable_heart_ntf = false;
    	    heartbeat_count_num = 0;
#endif
    	    esp_ble_gap_start_advertising(&spp_adv_params);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
    	    }
    	    else {
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    //ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);
    //???????????????????????????????????????

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    esp_err_t ret;

    AD7606_Init();//ad?????????
    esp_timer_init();//??????????????????
    esp_timer_create_args_t args = {
        .callback = timer1Interrupt_SPI,//?????????????????????
        .arg = (void*)1,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Tiemr1",
        .skip_unhandled_events = false,
    };
    //???????????????
   

    XQueue_SPI = xQueueCreate(500,16);
    // XQueue_SPI = xQueueCreate(10,32); 

    // xTaskCreate(TaskSPI, "SPIdata", 4096, NULL, tskIDLE_PRIORITY, NULL);
    // xTaskCreate(TaskRev, "Revdata", 4096, NULL, tskIDLE_PRIORITY, NULL);


    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,                // MISO?????????
        .mosi_io_num = -1,                          // MOSI?????????
        .sclk_io_num = PIN_NUM_CLK,                 // SCLK?????????
        .quadwp_io_num = -1,                        // WP?????????????????????QSPI???D2
        .quadhd_io_num = -1,                        // HD?????????????????????QSPI???D3
        .max_transfer_sz = 64*8,                    // ????????????????????????
    };

    //spi_device_interface_config_t????????????SPI????????????
    //????????????????????????????????????????????????
    spi_device_interface_config_t devcfg={
        .clock_speed_hz = SPI_MASTER_FREQ_10M,      // Clock out at 10 MHz,
        .mode = 2,                                  // SPI mode 1???CPOL = 1, CPHA = 0???
        .spics_io_num = -1,
        .queue_size = 7,                            // ?????????????????????????????????????????????????????????
    };

    //?????????SPI??????
    ret = spi_bus_initialize(AD_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    
    //???????????????
    ret = spi_bus_add_device(AD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    
    //??????SPI

    esp_timer_create(&args, &timer1);//???????????????1
    // esp_timer_start_periodic(timer1, 500e3);//?????????1?????????????????????????????????????????????????????????us


    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    spp_task_init();



    return;
}

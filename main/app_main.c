/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include "stdlib.h"
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "time.h"
#include "esp_wifi.h"
#include "esp_netif_ip_addr.h"
#include "math.h"
#include "driver/gpio.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

uint16_t send_packet_count = 0;
uint16_t receive_packet_count = 0;
uint16_t error_count = 0;
uint16_t cycle_count = 0;


#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)

#define PUBLISH_INTERVAL 1000

#define PIN_START          2

#define PIN_LED_G           12
#define PIN_LED_R           15
static const char *TAG = "LoRa_Gateway";
#define BROADCAST_LISTEN_INTERVAL_MS 300  // Short delay to avoid overloading
#define ONE_DATA_PACKET_SEND_INTERVAL_MS 1000
#define TIMEOUT_ASSIGN_TASK_MS 6000
#define TIMEOUT_REQUEST_DATA_TASK_MS 6000
#define CYCLE_MS 20000
#define ACK_LISTEN_TIMEOUT_MS 1000
#define MAX_NODES 20
#define JOIN_REQUEST_BUF "Open"
#define MAX_RETRIES 3
float T_min= 15.0;
float T_max= 30.0;
float H_min= 40.0;
float H_max= 60.0;

void send_telemetry_data();

typedef struct {
    uint8_t id;
    float latitude;
    float longitude;
    float t; // Temperature
    float d; // Humidity
    TickType_t last_seen;
} node_info_t;

static node_info_t nodes[MAX_NODES];
static int node_count = 0;
static node_info_t previous_nodes[MAX_NODES];
static int previous_node_count = 0;
static node_info_t out_nodes[MAX_NODES];
static int out_node_count = 0;
static node_info_t new_nodes[MAX_NODES];
static int new_node_count = 0;
static void reset_nodes() {
    memset(previous_nodes, 0, sizeof(previous_nodes));
    previous_node_count=0;
    for (int i = 0; i < node_count; i++) {
        previous_nodes[i] = nodes[i];
        ESP_LOGW(TAG, "add Node %d to preivious node", previous_nodes[i].id);
        previous_node_count++;
    }
    // Clear nodes and reset counters
    memset(nodes, 0, sizeof(nodes));
    node_count = 0;

    // Clear new_nodes and reset counter
    memset(new_nodes, 0, sizeof(new_nodes));
    new_node_count = 0;
}
static void out_nodes_write() {
// Identify nodes that are no longer active
    out_node_count = 0;
    int active;
    for (int i = 0; i < previous_node_count; i++) {
        active=0;
        for (int j = 0; j < node_count; j++) {
            if (previous_nodes[i].id == nodes[j].id) {
                active = 1;
                break;
            }
        }
        if (!active && out_node_count < MAX_NODES) {
            out_nodes[out_node_count++] = previous_nodes[i];
            ESP_LOGW(TAG, "Write node %d is out of network", out_nodes[i].id);

        }
    }
}

static void add_node(uint8_t id, float latitude, float longitude, float t, float d) {
    // Check if node is already in the current cycle
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].id == id) {
            nodes[i].latitude = latitude;
            nodes[i].longitude = longitude;
            nodes[i].t = t;
            nodes[i].d = d;
            nodes[i].last_seen = xTaskGetTickCount();
            ESP_LOGI(TAG, "Node %d updated. Lat: %.5f, Lon: %.5f, Temp: %.1f, Humidity: %.1f", id, latitude, longitude, t, d);
            return;
        }
    }

    // Check if node is new
    int is_new = 1;
    for (int i = 0; i < previous_node_count; i++) {
        if (previous_nodes[i].id == id) {
            is_new = 0;
            break;
        }
    }

    // Add node to the new list if it is new
    if (is_new && new_node_count < MAX_NODES) {
        new_nodes[new_node_count].id = id;
        new_nodes[new_node_count].latitude = latitude;
        new_nodes[new_node_count].longitude = longitude;
        new_nodes[new_node_count].t = t;
        new_nodes[new_node_count].d = d;
        new_nodes[new_node_count].last_seen = xTaskGetTickCount();
        new_node_count++;
    }

    // Add node to the current cycle
    if (node_count < MAX_NODES) {
        nodes[node_count].id = id;
        nodes[node_count].latitude = latitude;
        nodes[node_count].longitude = longitude;
        nodes[node_count].t = t;
        nodes[node_count].d = d;
        nodes[node_count].last_seen = xTaskGetTickCount();
        node_count++;
        ESP_LOGI(TAG, "Node %d added to the network. Lat: %.1f, Lon: %.1f, Temp: %.1f, Humidity: %.1f", id, latitude, longitude, t, d);
    } else {
        ESP_LOGW(TAG, "Node list full, cannot add node %d.", id);
    }
}

static int send_with_ack(const char *message, uint8_t expected_ack_id) {
    uint8_t buf[256];
    int retries = 0;

    while (retries <= MAX_RETRIES) {
        lora_send_packet((uint8_t *)message, strlen(message));
        return 1;//test----------------------------------------------
        TickType_t start_wait = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start_wait) < pdMS_TO_TICKS(ACK_LISTEN_TIMEOUT_MS)) {
            lora_receive();
            if (lora_received()) {
                int rxLen = lora_receive_packet(buf, sizeof(buf));
                buf[rxLen] = '\0';  // Đảm bảo chuỗi hợp lệ
                ESP_LOGI(TAG, "Received: %s", buf);

                // Trích xuất giá trị id và kiểm tra phần "ACK"
                unsigned char received_ack_id;
                if (sscanf((char *)buf, "%hhu ACK", &received_ack_id) == 1) {
                    // Kiểm tra nếu id trong buf khớp với expected_ack_id
                    if (received_ack_id == expected_ack_id) {
                        return 1; // ACK hợp lệ đã nhận
                    }
                }
            }
        }

        retries++;
        ESP_LOGW(TAG, "Retry %d for message: %s", retries, message);
    }

    ESP_LOGE(TAG, "Failed to receive ACK after %d retries: %s", MAX_RETRIES, message);
    return 0;
}

static void send_ack(uint8_t id) {
    uint8_t buf[256];
    int send_len = sprintf((char *)buf, "%d ACK", id);
    lora_send_packet(buf, send_len);
    ESP_LOGI(TAG, "Sent ACK to node %d.", id);
}

static void send_accept_packet(uint8_t id) {
    uint8_t buf[256];
    sprintf((char *)buf, "%d %d %.1f %.1f %.1f %.1f", id, node_count, T_min, T_max, H_min, H_max);
    ESP_LOGI(TAG, "Sent %s.", buf);
    if (send_with_ack((char *)buf, id)){
        ESP_LOGI(TAG, "Sent accept packet to node %d.", id);
    }
    else{
        ESP_LOGW(TAG, "Failed to send accept packet to node %d.", id);
    }  
}
static void send_request_packet(uint8_t id) {
    uint8_t buf[256];
    sprintf((char *)buf, "%d R", id);
    ESP_LOGI(TAG, "Sent request to node %d: %s", id, buf);
    send_with_ack((char *)buf, id);
    if (send_with_ack((char *)buf, id)){
        ESP_LOGI(TAG, "Sent request packet to node %d.", id);
    }
    else{
        ESP_LOGW(TAG, "Failed to send request packet to node %d.", id);
    }
}

static void send_ok_packet(uint8_t id) {
    uint8_t buf[256];
    sprintf((char *)buf, "%d Ok", id);
    ESP_LOGI(TAG, "Sent Ok to node %d: %s", id, buf);
    send_with_ack((char *)buf, id);
    if (send_with_ack((char *)buf, id)){
        ESP_LOGI(TAG, "Sent Ok packet to node %d.", id);
    }
    else{
        ESP_LOGW(TAG, "Failed to send Ok packet to node %d.", id);
    }
}

void task_lora_gateway(void *pvParameters) {
    ESP_LOGI(TAG, "Gateway task started.");
    // Lưu lại thời gian bắt đầu để tính chu kỳ
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {    
        ESP_LOGI(TAG, "Timeout reached. Resetting node lists.");
        reset_nodes();


        // -------------------------------------------------------Assign phase-------------------------------------------------------
        uint8_t buf[256];
        TickType_t start_time = xTaskGetTickCount();
        while (xTaskGetTickCount() - start_time < pdMS_TO_TICKS(TIMEOUT_REQUEST_DATA_TASK_MS)) {
            // Start one sub assign phase
            TickType_t sub_start_time = xTaskGetTickCount();

            // Broadcast message
            int send_len = sprintf((char *)buf, JOIN_REQUEST_BUF);
            lora_send_packet((uint8_t *)buf, send_len);
            ESP_LOGI(TAG, "Broadcasted: %s (length: %d bytes)", buf, send_len);

            // Listen for assign packets
            while ((xTaskGetTickCount() - sub_start_time) < pdMS_TO_TICKS(BROADCAST_LISTEN_INTERVAL_MS)) {
                // ESP_LOGI(TAG, "Listening for assign packets...");
                lora_receive();
                if (lora_received()) {
                    int rxLen = lora_receive_packet(buf, sizeof(buf));
                    buf[rxLen] = '\0'; // Null-terminate for safe string handling
                    ESP_LOGI(TAG, "Received: %s", buf);

                    
                    uint8_t node_id;
                    float latitude, longitude, t = -1, d = -1;
                    if (sscanf((char *)buf, "%hhd %f %f", &node_id, &latitude, &longitude) >= 3) {
                        // send_ack(node_id);
                        add_node(node_id, latitude, longitude, t, d);
                        send_accept_packet(node_id);
                        break;
                    }
                }
                
                vTaskDelay(1); // Avoid WatchDog alerts
            }
        }
        // -------------------------------------------------------ENd assign phase-------------------------------------------------------


        // -------------------------------------------------------Request data phase-------------------------------------------------------
        start_time = xTaskGetTickCount();
        for (int i = 0; (i < node_count)&(xTaskGetTickCount() - start_time < pdMS_TO_TICKS(TIMEOUT_REQUEST_DATA_TASK_MS)); i++) {
            
            TickType_t sub_start_time = xTaskGetTickCount();

            uint8_t buf[256];
            uint8_t node_id = nodes[i].id;

            // Send "id R" request to node
            send_request_packet(node_id);

            // Listen for data packet
            int data_received = 0;
            float t = -1, d = -1;

            
            while ((xTaskGetTickCount() - sub_start_time) < pdMS_TO_TICKS(ONE_DATA_PACKET_SEND_INTERVAL_MS)) {
                // ESP_LOGI(TAG, "Listening for data packets...");
                lora_receive();
                if (lora_received()) {
                    int rxLen = lora_receive_packet(buf, sizeof(buf));
                    buf[rxLen] = '\0'; // Null-terminate for safe string handling
                    if (sscanf((char *)buf, "%hhu %f %f", &node_id, &t, &d) == 3) {
                        if (node_id == nodes[i].id) {
                            nodes[i].t = t;
                            nodes[i].d = d;
                            data_received = 1;
                            ESP_LOGI(TAG, "Data received from node %d: Temp=%.1f, Humidity=%.1f", node_id, t, d);
                        
                            // send_ack(node_id);
                            break;
                        }                                              
                    }
                }
                vTaskDelay(10); // Avoid WatchDog alerts
            }

            if (!data_received) {
                ESP_LOGW(TAG, "No data received from node %d within timeout.", node_id);
                continue;
            }

            // Send "Ok" packet
            send_ok_packet(node_id);

        }
        // -------------------------------------------------------End request data phase-------------------------------------------------------
        out_nodes_write();
        if (receive_packet_count)
        {
            ESP_LOGW(TAG, "Cycle %d ---------- Error receive paket rate = %.2f ------------- receive_packet_count = %d", cycle_count,(float) error_count/receive_packet_count,receive_packet_count);
        }
        else{
            ESP_LOGW(TAG, "Cycle %d ---------- Error receive paket rate = NaN ------------- receive_packet_count = %d", cycle_count, receive_packet_count );
        }
        send_telemetry_data();
        cycle_count++;
        vTaskDelayUntil(&xLastWakeTime, CYCLE_MS / portTICK_PERIOD_MS);
    }
}

//=====================MQTT==============================

static esp_mqtt5_user_property_item_t user_property_arr[] = {
        {"board", "esp32"},
        {"u", "user"},
        {"p", "password"}
    };

static esp_mqtt5_publish_property_config_t publish_property = {
    .payload_format_indicator = 1,
    .message_expiry_interval = 1000,
    .topic_alias = 0,
    .response_topic = "/topic/test/response",
    .correlation_data = "123456",
    .correlation_data_len = 6,
};

esp_mqtt_client_handle_t mqtt_client;     // Biến toàn cục để giữ client MQTT

// Create random data funciton

// Create and add data to json field
void send_telemetry_data() {
    // Create JSON
    cJSON *root = cJSON_CreateObject();

    // Add the field to json
    for(int i=0; i<node_count;i++){
        char t_key[16], d_key[16],node_key[8],x_key[4],y_key[4];
        snprintf(t_key,sizeof(t_key),"Temperature%d",nodes[i].id);
        snprintf(d_key,sizeof(d_key),"Humidity%d",nodes[i].id);
        snprintf(node_key, sizeof(node_key), "Node%d", nodes[i].id);
        snprintf(x_key, sizeof(node_key), "X%d", nodes[i].id);
        snprintf(y_key, sizeof(node_key), "Y%d", nodes[i].id);
        cJSON_AddNumberToObject(root,t_key, nodes[i].t);
        cJSON_AddNumberToObject(root, d_key, nodes[i].d);
        cJSON_AddNumberToObject(root, x_key, nodes[i].latitude);
        cJSON_AddNumberToObject(root, y_key, nodes[i].longitude);
        if (nodes[i].t < T_min || nodes[i].t > T_max || nodes[i].d < H_min || nodes[i].d > H_max){
            cJSON_AddStringToObject(root, node_key, "Warning");
        }
        else{
            cJSON_AddStringToObject(root, node_key, "Joined");
        }
    }
    for(int i=0; i<out_node_count;i++){
        char out_node_key[8];
        snprintf(out_node_key, sizeof(out_node_key), "Node%d", out_nodes[i].id);
        cJSON_AddStringToObject(root, out_node_key, "Out");
        ESP_LOGE(TAG, "Node %d is out of network", out_nodes[i].id);

    }
    

    // Convert json data to string
    char *json_string = cJSON_Print(root);

    // Send data to ThingsBoard passing over MQTT
    esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry", json_string, 0, 1, 0);

    // free the memory
    cJSON_Delete(root);
    free(json_string);

    ESP_LOGI("Telemetry Task", "Data is sent");
}

static void print_user_property(mqtt5_user_property_handle_t user_property)
{
    if (user_property) {
        uint8_t count = esp_mqtt5_client_get_user_property_count(user_property);
        if (count) {
            esp_mqtt5_user_property_item_t *item = malloc(count * sizeof(esp_mqtt5_user_property_item_t));
            if (esp_mqtt5_client_get_user_property(user_property, item, &count) == ESP_OK) {
                for (int i = 0; i < count; i ++) {
                    esp_mqtt5_user_property_item_t *t = &item[i];
                    ESP_LOGI(TAG, "key is %s, value is %s", t->key, t->value);
                    free((char *)t->key);
                    free((char *)t->value);
                }
            }
            free(item);
        }
    }
}

// Response function MQTT type {"method":"setState","params":true/false}
void send_mqtt_set_state(esp_mqtt_client_handle_t client, int request_id, bool state) {
    // Create topic response
    char response_topic[50];
    snprintf(response_topic, sizeof(response_topic), "v1/devices/me/rpc/response/%d", request_id);

    // Create json
    cJSON *response_json = cJSON_CreateObject();
    cJSON_AddStringToObject(response_json, "method", "getState");
    cJSON_AddBoolToObject(response_json, "params", state);
    char *response_data = cJSON_PrintUnformatted(response_json);

    // Send data to ThingsBoard
    int msg_id = esp_mqtt_client_publish(client, response_topic, response_data, 0, 1, 0);
    ESP_LOGI(TAG, "Sent response to %s, msg_id=%d, data=%s", response_topic, msg_id, response_data);

    // Free memory
    cJSON_Delete(response_json);
    free(response_data);
}

// Process data from MQTT
void process_mqtt_data(esp_mqtt_client_handle_t client, const char *topic, int topic_len, const char *data, int data_len) {
    ESP_LOGI(TAG, "Received Data:");
    ESP_LOGI(TAG, "TOPIC=%.*s", topic_len, topic);
    ESP_LOGI(TAG, "DATA=%.*s", data_len, data);

    // copy json str
    char *json_str = strndup(data, data_len);
    if (json_str == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for JSON data");
        return;
    }

    //  Parse json string
    cJSON *json = cJSON_Parse(json_str);
    if (json == NULL) {
        ESP_LOGE(TAG, "Invalid JSON format");
        free(json_str);
        return;
    }
    // check "method" field JSON
    cJSON *method = cJSON_GetObjectItem(json, "method");
    if (cJSON_IsString(method) && strcmp(method->valuestring, "getState") == 0) {
        ESP_LOGI(TAG, "Received getState method. Processing...");

        // Boolen status
        bool current_state = false;

        // Get id from topic
        int request_id;
        sscanf(topic + strlen("v1/devices/me/rpc/request/"), "%d", &request_id);

        // Respon to thingsboard
        send_mqtt_set_state(client, request_id, current_state);
    }

    if (strcmp(method->valuestring, "setState") == 0) {
        //Get param info
        cJSON *params = cJSON_GetObjectItem(json, "params");
        if (cJSON_IsBool(params)) {
            // Turn on led
            bool state = cJSON_IsTrue(params);
            gpio_set_level(PIN_START, state ? 1 : 0);
            ESP_LOGI(TAG, "Network is %s", state ? "ON" : "OFF");
        } else {
            ESP_LOGE(TAG, "Invalid 'params' field");
        }
    }
     if (strcmp(method->valuestring, "setValue1") == 0) {
        //Get param info
        cJSON *params = cJSON_GetObjectItem(json, "params");
        T_min = (float)params->valuedouble;
        ESP_LOGI(TAG, "Low level temperature: %.1f", T_min);
    }
    if (strcmp(method->valuestring, "setValue2") == 0) {
        //Get param info
        cJSON *params = cJSON_GetObjectItem(json, "params");
        T_max = (float)params->valuedouble;
        ESP_LOGI(TAG, "High level temperature: %.1f", T_max);
    }
    if (strcmp(method->valuestring, "setValue3") == 0) {
        //Get param info
        cJSON *params = cJSON_GetObjectItem(json, "params");
        H_min = (float)params->valuedouble;
        ESP_LOGI(TAG, "Low level humidity: %.1f", H_min);
    }
    if (strcmp(method->valuestring, "setValue4") == 0) {
        //Get param info
        cJSON *params = cJSON_GetObjectItem(json, "params");
        H_max = (float)params->valuedouble;
        ESP_LOGI(TAG, "High level humidity: %.1f", H_max);
    }
    
    // free memory
    cJSON_Delete(json);
    free(json_str);
}

static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    ESP_LOGD(TAG, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "v1/devices/me/rpc/request/+", 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        esp_mqtt_client_reconnect(mqtt_client);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        esp_mqtt5_client_set_publish_property(client, &publish_property);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        print_user_property(event->property->user_property);
        // ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
        // ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
        process_mqtt_data(mqtt_client, event->topic, event->topic_len, event->data, event->data_len);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt5_app_start(void)
{
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .payload_format_indicator = true,
        .message_expiry_interval = 10,
        .response_topic = "/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = "mqtt://mqtt.thingsboard.cloud:1883",
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = true,
        .credentials.username = "cQxfx91sylRI5XclT3AP",
        .credentials.authentication.password = "",
        .session.last_will.topic = "/topic/will",
        .session.last_will.msg = "i will leave",
        .session.last_will.msg_len = 12,
        .session.last_will.qos = 1,
        .network.timeout_ms = -1,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt5_cfg);
    mqtt_client = client;

    esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_connect_property(client, &connect_property);

    esp_mqtt5_client_delete_user_property(connect_property.user_property);
    esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

     if (lora_init() == 0) {
        ESP_LOGE(TAG, "LoRa module not recognized.");
        while (1) {
            vTaskDelay(1);
        }
    }

    ESP_LOGI(TAG, "LoRa initialized. Setting parameters...");
    lora_set_frequency(433e6); // 433MHz
    lora_enable_crc();
    lora_set_coding_rate(1);
    lora_set_bandwidth(7);
    lora_set_spreading_factor(7);
    ESP_LOGW(TAG, "LoRa parameters spreading_factor set: %d", lora_get_spreading_factor());


    xTaskCreate(&task_lora_gateway, "LoRa_Gateway", 1024 * 4, NULL, 5, NULL);

    ESP_ERROR_CHECK(example_connect());

    mqtt5_app_start();
}

/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/addr.h>

#include <bluetooth/services/nus.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
#include <cJSON.h>
#include "ble_gps_weather.h"

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define JSON_BUFF_SIZE 2000

uint8_t gps_status_sent = false;
uint32_t max_send_len = 20;

double latitude, longitude, altitude, Speed, Time, Hdop;
int Satellites;

/*open meteo reply is large( approximate 2kb so use dynamic allocation for temporary biffers)*/
#define HTTP_REQUEST_URL_FMT "https://api.open-meteo.com/v1/forecast?latitude=%f&longitude=%f&current=wind_speed_10m,wind_direction_10m,relative_humidity_2m,temperature_2m,apparent_temperature,weather_code&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max&wind_speed_unit=ms&timezone=auto&forecast_days=%d"

#define GB_HTTP_REQUEST_FMT "{\"t\":\"http\",\"url\":\"%s\",\"id\":\"%u\"}\r\n"

#define WEATHER_UI_NUM_FORECASTS 2

typedef enum
{
	BLE_HTTP_STATUS_OK,
	BLE_HTTP_STATUS_TIMEOUT,
	BLE_HTTP_STATUS_BUSY,
	BLE_HTTP_STATUS_ERROR,
} ble_http_status_code_t;

typedef enum parse_state
{
	WAIT_GB,
	WAIT_END,
	PARSE_STATE_DONE,
} parse_state_t;

typedef struct {
    double temperature;
    double apparent_temperature;
    double wind_speed;
    char *text;
} current_weather_data_t;

typedef struct {
    double temperature;
    int rain_percent;
    double low_temp;
    double high_temp;
    char day[4]; // 3 Letter day name
    char *text;
} weather_forecast_data_t;

#define MAX_GB_PACKET_LENGTH 2000

static uint8_t num_parsed_brackets;
static parse_state_t parse_state = WAIT_GB;
static uint16_t parsed_data_index = 0;
static uint8_t receive_buf[MAX_GB_PACKET_LENGTH + 1];

#define HTTP_MAX_PENDING_REQUESTS 5
#define HTTP_TIMEOUT_SECONDS 10

static uint16_t request_id;

uint16_t weather_request_pending = false;

int gps_data_receive;

/*callback invoked when http get request completed/or response received

 status    The status of the HTTP GET request
 response  The response data.*/
typedef void (*ble_http_callback)(ble_http_status_code_t status, char *response);

static void ble_http_timeout_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(ble_http_timeout_work, ble_http_timeout_handler); /*dont poll for timeout, execute handler if timeout occur, schedule it to run after timeout*/
void weather_request_handler(struct k_work *work);
K_WORK_DEFINE(weather_request_work, weather_request_handler);

char *extract_value_str(char *key, char *data, int *value_len);
int parse_gps_data(char *data, uint16_t len);
void weather_http_req_send();
int ble_http_send(char *url, ble_http_callback callback);
int ble_send_app_data(char *data, int len);
void ble_gadgetbridge_input(const uint8_t *const data, uint16_t len);
static void parse_data(char *data, int len);
static int parse_httpstate(char *data, int len);
static char *find_js_value(char *data, const char *key);
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
              struct bt_gatt_exchange_params *params);

ble_http_callback call_back; /*callback function pointer to be invoked when http response is received or timeout occurs.
						execute callback if user defined it*/
/*Create variable that holds callback for MTU negotiation */
static struct bt_gatt_exchange_params exchange_params;

	
void ble_write_data(bool enable);


static void ble_http_timeout_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_WRN("HTTP request timed out");
	weather_request_pending = false;

	//  if user defined callback , Invoke the callback with timeout status and NULL rsponse
	if(call_back)
	{
		call_back(BLE_HTTP_STATUS_TIMEOUT, NULL);
	}
}

void http_rsp_cb(ble_http_status_code_t status, char *response)

{
	current_weather_data_t current_weather;
	weather_forecast_data_t forecasts[WEATHER_UI_NUM_FORECASTS] = {0};

	if (status == BLE_HTTP_STATUS_OK)
	{
		cJSON *parsed_response = cJSON_Parse(response);
		if (parsed_response == NULL)
		{
			LOG_ERR("Failed to parse weather JSON response");
			goto invalid_response;
		}

		cJSON *current = cJSON_GetObjectItem(parsed_response, "current");
		cJSON *current_temperature_2m = cJSON_GetObjectItem(current, "temperature_2m");
		cJSON *current_weather_code = cJSON_GetObjectItem(current, "weather_code");
		cJSON *current_wind_speed = cJSON_GetObjectItem(current, "wind_speed_10m");
		cJSON *humidity = cJSON_GetObjectItem(current, "relative_humidity_2m");
		cJSON *wind_direction = cJSON_GetObjectItem(current, "wind_direction_10m");
		cJSON *apparent_temperature = cJSON_GetObjectItem(current, "apparent_temperature");

		cJSON *daily_forecasts = cJSON_GetObjectItem(parsed_response, "daily");

		cJSON *weather_code_list = cJSON_GetObjectItem(daily_forecasts, "weather_code");
		cJSON *temperature_2m_max_list = cJSON_GetObjectItem(daily_forecasts, "temperature_2m_max");
		cJSON *temperature_2m_min_list = cJSON_GetObjectItem(daily_forecasts, "temperature_2m_min");
		cJSON *precipitation_probability_max_list = cJSON_GetObjectItem(daily_forecasts, "precipitation_probability_max");

		int num_forecasts = cJSON_GetArraySize(weather_code_list);
		if(!cJSON_IsNumber(current_temperature_2m) || !cJSON_IsNumber(current_weather_code) ||
			!cJSON_IsNumber(current_wind_speed) || !cJSON_IsNumber(apparent_temperature) ||
			num_forecasts < 1 || num_forecasts > WEATHER_UI_NUM_FORECASTS ||
			!cJSON_IsArray(weather_code_list) || !cJSON_IsArray(temperature_2m_max_list) ||
			!cJSON_IsArray(temperature_2m_min_list) || !cJSON_IsArray(precipitation_probability_max_list) ||
			cJSON_GetArraySize(temperature_2m_max_list) != num_forecasts ||
			cJSON_GetArraySize(temperature_2m_min_list) != num_forecasts ||
			cJSON_GetArraySize(precipitation_probability_max_list) != num_forecasts)
		{
			cJSON_Delete(parsed_response);
			goto invalid_response;
		}

		current_weather.temperature = current_temperature_2m->valuedouble;
		current_weather.wind_speed = current_wind_speed->valuedouble;
		current_weather.apparent_temperature = apparent_temperature->valuedouble;

		for (int i = 0; i < num_forecasts; i++)
		{
			cJSON *high = cJSON_GetArrayItem(temperature_2m_max_list, i);
			cJSON *low = cJSON_GetArrayItem(temperature_2m_min_list, i);
			cJSON *rain = cJSON_GetArrayItem(precipitation_probability_max_list, i);
			cJSON *code = cJSON_GetArrayItem(weather_code_list, i);
			if (!cJSON_IsNumber(high) || !cJSON_IsNumber(low) ||
				!cJSON_IsNumber(rain) || !cJSON_IsNumber(code))
			{
				cJSON_Delete(parsed_response);
				goto invalid_response;
			}

			forecasts[i].temperature = high->valuedouble;
			forecasts[i].low_temp = low->valuedouble;
			forecasts[i].high_temp = high->valuedouble;
			forecasts[i].rain_percent = rain->valueint;

			// TODO: diaplay week days with time, fetch time data from GB
		}

		ble_write_data(false);
		cJSON_Delete(parsed_response);
		LOG_INF("parsed http response  temp %f , wind speed %f, apparent temp %f, forecast 1st day temp %f , rain_percent %d", 
		current_weather.temperature, current_weather.wind_speed, current_weather.apparent_temperature, forecasts[0].temperature, forecasts[0].rain_percent);
		k_work_cancel_delayable(&ble_http_timeout_work);
	}
	else {
        LOG_ERR("HTTP request failed");
    }
    return;
	
	invalid_response:
		LOG_ERR("Invalid weather response");
	
}


static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
						  uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	ble_gadgetbridge_input(data, len);
}

struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};


static void exchange_func(struct bt_conn *conn, uint8_t att_err,
              struct bt_gatt_exchange_params *params)
{
    LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if (!att_err) {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
		//max_send_len = payload_mtu;
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}

 void update_mtu(struct bt_conn *conn)
{
    int err;
    exchange_params.func = exchange_func;    //assign callback/ assign function to function pointer

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}

void update_data_length(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    err = bt_conn_le_data_len_update(conn, &my_data_len);
    if (err) {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

/*TODO: make http request work , http request currently not send with k_work, but can be exected with main loop
  see if seperate thread, work queue needs*/
void weather_request_handler(struct k_work *work)
{
	uint8_t weather_done = false;

		if (gps_data_receive == 1 && !weather_done)
		{
			ble_write_data(false);
			weather_http_req_send(); /*send weather request after receiving gps coordinates*/
			weather_done = true;
		}
	
}

/*call the function after bt_conn_ref*/
void ble_write_data(bool enable)
{
	char gps_status[50];


	int len = snprintf(gps_status, sizeof(gps_status), "{\"t\":\"gps_power\", \"status\":%s} \n",
					   enable ? "true" : "false");

	LOG_INF("Sending GPS power status: %s", gps_status);

	ble_send_app_data(gps_status, len); // passing local buffer is safe before function returns
}

int parse_gps_data(char *data, uint16_t len)
{
	cJSON *root = cJSON_Parse(data);
	if (root == NULL)
	{
		LOG_ERR("Failed to parse GPS data");
		return -EINVAL;
	}

	cJSON *lat = cJSON_GetObjectItem(root, "lat");
	cJSON *lon = cJSON_GetObjectItem(root, "lon");
	cJSON *alt = cJSON_GetObjectItem(root, "alt");
	cJSON *speed = cJSON_GetObjectItem(root, "speed");
	cJSON *time = cJSON_GetObjectItem(root, "time");
	cJSON *satellites = cJSON_GetObjectItem(root, "satellites");
	cJSON *hdop = cJSON_GetObjectItem(root, "hdop");

	latitude = lat->valuedouble;
	longitude = lon->valuedouble;
	altitude = alt->valuedouble;
	Speed = speed->valuedouble;
	Time = time->valuedouble;
	Satellites = satellites->valueint;
	Hdop = hdop->valuedouble;

	LOG_INF("Parsed GPS data: lat=%f, lon=%f, alt=%f, speed=%f, time=%f, satellites=%d, hdop=%f",
			latitude, longitude, altitude, Speed, Time, Satellites, Hdop);

	cJSON_Delete(root);
	return 0;
}

// 20 bytes of data recieved from ble in bt_reveive callbacks

char *extract_value_str(char *key, char *data, int *value_len)
{
	char *start;
	char *end;
	char *str = strstr(data, key);
	*value_len = 0;
	if (str == NULL)
	{
		return NULL;
	}
	str += strlen(key); /*increment the key and point to value*/
	if (*str != '\"')
	{
		return NULL; // if no quotation mark at start , the value in json may be int
	}

	str++;

	if (*str == '\0')
	{
		return NULL; // Got end of data
	}
	end = str;

	while (*end != '\0' && *end != '\"')
	{ // loop continus until end of string or closing quote is found

		if (*end == '\\' && end[1] != '\0')
		{
			end++; // Skip escaped character so \" does not end the value, if \ present then skip the next character(double qoutation) as well
		}
		end++;
	}

	if (*end == '\0')
	{
		return NULL; // No end of value, couldnt find the closing quote
	}

	start = str;

	*value_len = end - start; /*calculate the length of the value*/

	return start;
}

/*weather app*/

/*send http weather request via ble */
void weather_http_req_send()
{
	char http_request[512];
	snprintf(http_request, sizeof(http_request), HTTP_REQUEST_URL_FMT, latitude, longitude, WEATHER_UI_NUM_FORECASTS);
	ble_http_send(http_request, http_rsp_cb);
}

/*send an http request to url and invoke callback when response is received
 The callback function is passed the status code and the response data in
 * JSON format. dDo not retain or free the pointer, copy its contents in seperate bufferif you need the response later*/
int ble_http_send(char *url, ble_http_callback callback)
{

	int ret = 0;

	if (!url || !callback)
	{
		return -EINVAL;
	}

	// register the user callback, store the function address in function pointer

	call_back = callback;
	/* Five digits for the uint16_t ID, even after the request counter grows.
	 memory allocation to construct the HTTP request */
	size_t capacity = strlen(url) + sizeof(GB_HTTP_REQUEST_FMT) + 5;

	char *request = k_malloc(capacity);
	if (!request)
	{
		return -ENOMEM;
	}

	if (weather_request_pending) // todo : set this flag in resp callback
	{
		k_free(request);
		LOG_WRN("Another HTTP request is already pending");
		return -EBUSY;
	}
	uint16_t id = ++request_id;

	k_work_schedule(&ble_http_timeout_work, K_SECONDS(HTTP_TIMEOUT_SECONDS));

	int len = snprintf(request, capacity, GB_HTTP_REQUEST_FMT, url, (unsigned int)id); /*copy the http request in request buffer and send */
	LOG_INF("http request : %s", request);
	ret = ble_send_app_data(request, len);

	k_free(request);
	if (ret)
	{
		LOG_ERR("Failed to send HTTP request over BLE (err: %d)", ret);
		return -EIO;
	}

	weather_request_pending = true;
	return ret;
}

int ble_send_app_data(char *data, int len)
{
	uint16_t offset = 0;
	int ret = 0;

	while (offset < len)
	{
		uint16_t chunk_len = len - offset;
		if (chunk_len > max_send_len)
		{
			chunk_len = max_send_len;
		}

		ret = bt_nus_send(current_conn, data + offset, chunk_len);
		if (ret)
		{
			LOG_ERR("Failed to send data over BLE via nus (err: %d)", ret);
			return -EIO;
		}

		offset += chunk_len;
	}
	return ret;
}

void ble_gadgetbridge_input(const uint8_t *const data, uint16_t len)
{
	// LOG_HEXDUMP_DBG(data, len, "RX");

	// NULL terminate the data to make string parsing easier and safer.
	char *safe_buf = k_malloc(len + 1);
	if (safe_buf == NULL)
	{
		LOG_ERR("Failed to allocate %u bytes for NUS parse buffer", len + 1);
		return;
	}
	memcpy(safe_buf, data, len);
	safe_buf[len] = '\0';

	char *gb_start = strstr(safe_buf, "GB(");
	if (gb_start && parse_state != WAIT_GB)
	{
		LOG_ERR("Parsing error, was waiting end, but got GB");
		parse_state = WAIT_GB;
	}

	char *time_start = strstr(safe_buf, "setTime(");
	if (time_start && parse_state == WAIT_GB)
	{
		time_start += strlen("setTime(");
		goto done;
	}

	// ie. ;E.setTimeZone(1.0);
	char *offset = strstr(safe_buf, ";E.setTimeZone(");
	if (offset && parse_state == WAIT_GB)
	{
		offset += strlen(";E.setTimeZone(");
		goto done;
	}

	size_t receive_buf_capacity = MAX_GB_PACKET_LENGTH;

	switch (parse_state)
	{
	case WAIT_GB:
	{
		if (gb_start)
		{
			gb_start += 3;
			parse_state = WAIT_END;
			num_parsed_brackets = 0;
			parsed_data_index = 0;
			memset(receive_buf, 0, sizeof(receive_buf));
			uint32_t index = gb_start - safe_buf;
			for (int i = index; i < len; i++)
			{
				if (parsed_data_index >= receive_buf_capacity)
				{
					LOG_ERR("Data from Gadgetbridge does not fit in receive_buf (%zu)", receive_buf_capacity);
					parse_state = WAIT_GB;
					break;
				}
				receive_buf[parsed_data_index] = safe_buf[i];
				parsed_data_index++;
				if (safe_buf[i] == '{')
				{
					num_parsed_brackets++;
				}
				else if (safe_buf[i] == '}')
				{
					num_parsed_brackets--;
					if (num_parsed_brackets == 0)
					{
						parse_state = PARSE_STATE_DONE;
						break;
					}
				}
			}
		}
		break;
	}
	case WAIT_END:
	{
		for (int i = 0; i < len; i++)
		{
			if (parsed_data_index >= receive_buf_capacity)
			{
				LOG_ERR("Data from Gadgetbridge does not fit in receive_buf (%zu)", receive_buf_capacity);
				parse_state = WAIT_GB;
				break;
			}
			receive_buf[parsed_data_index] = safe_buf[i];
			parsed_data_index++;
			if (safe_buf[i] == '{')
			{
				num_parsed_brackets++;
			}
			else if (safe_buf[i] == '}')
			{
				num_parsed_brackets--;
				if (num_parsed_brackets == 0)
				{
					parse_state = PARSE_STATE_DONE;
					break;
				}
			}
		}
		break;
	}
	case PARSE_STATE_DONE:
	{
		LOG_WRN("Busy parsing, ingnoring...");
		break;
	}
	default:
		LOG_WRN("Unhandled state");
		break;
	}
	if (parse_state == PARSE_STATE_DONE)
	{
		parse_state = WAIT_GB;
		receive_buf[parsed_data_index] = '\0';
		LOG_DBG("%s", receive_buf);
		parse_data((char *)receive_buf, parsed_data_index);
	}

done:
	k_free(safe_buf);
}

static void parse_data(char *data, int len)
{
	int type_len;
	char *type;

	type = extract_value_str("\"t\":", data, &type_len);
	if (type == NULL)
	{
		LOG_ERR("Failed to extract type from JSON data");
		return;
	}

	if (strlen("is_gps_active") == type_len && strncmp(type, "is_gps_active", type_len) == 0)
	{

		if (!gps_status_sent)
		{ /*send gps coordinates request just once*/

			ble_write_data(true);
			gps_status_sent = true;
		}
	}

	if (strlen("http") == type_len && strncmp(type, "http", type_len) == 0)
	{	
		LOG_INF("Received JSON data (%d bytes)" ,len);
		parse_httpstate(data,len);
		//call_back(BLE_HTTP_STATUS_OK, data);
		
	}

	if (strlen("gps") == type_len && strncmp(type, "gps", type_len) == 0)
	{
		int err = parse_gps_data(data, len);

		if (err)
		{
			LOG_ERR("Failed to parse GPS data");
			return;
		}

		gps_data_receive++;
		k_work_submit(&weather_request_work);  /*use to off load task and run it in system work queue*/
		if (gps_data_receive == 40)
		{
			gps_data_receive = 0; // Reset the counter
			
			
		}
	}

	
		LOG_INF("Received JSON data with type: %.*s", type_len, type);
	

}


/* Return the value following key (leading spaces skipped), or NULL when key is absent. */
static char *find_js_value(char *data, const char *key)
{
    char *value = strstr(data, key);
    if (value == NULL) {
        return NULL;
    }
    value += strlen(key);
    while (*value == ' ') {
        value++;
    }
    return value;
}

/*conversion from hex value to uint32 int*/
static int hex_value(const char *src , int digits, uint32_t* value)
{
	*value =0;
	
	for(int i=0; i<digits; i++)
	{
		char c = src[i]; 
		int v = (c >='0' && c<='9')? c - '0' : (c >= 'a' && c<= 'f')? c-'a'+10:
		            (c >= 'A' && c<='F')? c- 'A'+10 : -1; 

		 if (v < 0) {
            return -EINVAL;
        }
        *value = (*value << 4) | v;
	}

	return 0;
}

/*convert from uint32_t int to utf8 hex bytes*/
static int utf8_put(uint32_t cp, char *dst, size_t *pos, size_t cap)
{
    uint8_t bytes[4];
    size_t n;
    if (cp < 0x80) {
        bytes[0] = cp;
        n = 1;
    } else if (cp < 0x800) {
        bytes[0] = 0xC0 | (cp >> 6);       /*2 bytes in unicode hold 11 bits 5+6 bits*/
        bytes[1] = 0x80 | (cp & 0x3F);
        n = 2;
    } else if (cp < 0x10000) {             /* 3 bytes hold 16 bits*/
        bytes[0] = 0xE0 | (cp >> 12);
        bytes[1] = 0x80 | ((cp >> 6) & 0x3F);
        bytes[2] = 0x80 | (cp & 0x3F);
        n = 3;
    } else {
        bytes[0] = 0xF0 | (cp >> 18);             /* 4 bytes hold 21 bits*/    
        bytes[1] = 0x80 | ((cp >> 12) & 0x3F);
        bytes[2] = 0x80 | ((cp >> 6) & 0x3F);
        bytes[3] = 0x80 | (cp & 0x3F);
        n = 4;
    }
    if (*pos + n >= cap) {
        return -ENOSPC;
    }
    memcpy(&dst[*pos], bytes, n);
    *pos += n;
    return 0;
}

/* Decode the JS string literal at src into dst as UTF-8. Gadgetbridge (Bangle.js) sends
 * JavaScript, not strict JSON: strings may use \xNN, \v and octal escapes, or be
 * replaced by atob("base64"). Strict JSON from the companion apps is a subset.
 * dst may be src, output is never longer than input. With truncate, an overlong result is
 * cut on a UTF-8 character boundary instead of failing with -ENOSPC. */
static int decode_js_literal(const char *src, char *dst, size_t cap, bool truncate)
{
	size_t pos = 0;
	size_t char_start = 0;
	if (cap == 0)
	{
		return -ENOSPC;
	}

	if (*src++ != '"')
	{ /*if string is not started with double quote , return invalid string*/
		return -EINVAL;
	}
	while (*src && *src != '"')
	{ /*read string until end, decoder convert the escape sequences into thoir actual value*/
		uint32_t cp = (uint8_t)src[0];
		src++;
		if (cp == '\\')
		{ /*increment is character is backslash*/
			char esc = *src++;
			switch (esc)
			{
			case 'b': /*backspace*/
				cp = '\b';
				break;
			case 'f': /*form feed*/
				cp = '\f';
				break;
			case 'n':
				cp = '\n';
				break;
			case 'r':
				cp = '\r';
				break;
			case 't': /*tab*/
				cp = '\t';
				break;
			case 'v': /*vertical tab*/
				cp = '\v';
				break;
			case 'x':
				if (hex_value(src, 2, &cp))
				{
					return -EINVAL;
				}
				src += 2;
				break;
			case 'u': /*unicode characters, pair of two 16-bit values*/
				if (hex_value(src, 4, &cp))
				{ /*check for 16-bit hex value*/
					return -EINVAL;
				}
				src += 4;
				if (cp >= 0xD800 && cp <= 0xDBFF && src[0] == '\\' && src[1] == 'u')
				{ /*check for high surrogate */
					uint32_t low;
					if (hex_value(src + 2, 4, &low) == 0 && low >= 0xDC00 && low <= 0xDFFF)
					{														   /*check for low surrogate */
						cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00); /*calculate actual value of character*/
						src += 6;
					}
				}
				break;
			case '\0':
				return -EINVAL;
			default:
				if (esc >= '0' && esc <= '7')
				{ /*check for octal characters*/
					cp = esc - '0';
				}
				else
				{
					cp = (uint8_t)esc; /* \" \\ \/ and unknown escapes map to the character. */
				}
				break;
			}
			if (utf8_put(cp, dst, &pos, cap))
			{
				if (truncate)
				{
					goto done;
				}
				return -ENOSPC;
			}
			char_start = pos;
		}
		else
		{
			/* Raw bytes are already UTF-8 (or converted before parsing). */
			if (pos + 1 >= cap)
			{
				if (!truncate)
				{
					return -ENOSPC;
				}
				if ((cp & 0xC0) == 0x80)
				{
					pos = char_start;
				}
				goto done;
			}
			if ((cp & 0xC0) != 0x80)
			{
				char_start = pos;
			}
			dst[pos++] = cp;
		}
	}
	if (*src != '"')
	{
		return -EINVAL;
	}
done:
	dst[pos] = '\0';
	return pos;
}


static int parse_httpstate(char *data, int len)
{
    ARG_UNUSED(len);
    // {"t":"http","id":"5","resp":"{\"current\":...}"} or {"t":"http","id":"5","err":"..."}

    /* The request sends the id as a string; accept a plain number too. */
    char *id = strstr(data, "\"id\":");
    if (!id) {
        return -EINVAL;
    }
    id += strlen("\"id\":");
    if (*id == '"') {
        id++;
    }
    char *end;
    errno = 0;
    long request_id = strtol(id, &end, 10);
    if (end == id || errno || request_id < 0 || request_id > UINT16_MAX) {
        return -EINVAL;
    }
    

    /* Decode the body in place: the decoded string is never longer than the escaped one.
     * A NULL response (error or malformed relay result) still finishes the matching request.
     */
    char *resp = strstr(data, "\"err\":") ? NULL : find_js_value(data, "\"resp\":");  /* resp is a local pointer point to to receive data buffer*/
    if (resp && decode_js_literal(resp, resp, strlen(resp) + 1, false) > 0) {
        call_back(BLE_HTTP_STATUS_OK, resp);
    }
    
    return 0;
}



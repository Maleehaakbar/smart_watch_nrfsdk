#ifndef BLE_GPS_WEATHER_H
#define BLE_GPS_WEATHER_H

#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>

void update_data_length(struct bt_conn *conn);
void update_mtu(struct bt_conn *conn);

extern struct bt_nus_cb nus_cb;
extern struct bt_conn *current_conn;



#endif
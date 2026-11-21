#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <string.h>
#include "packet.h"
#include "config.h"

static QueueHandle_t q_rx;
static volatile bool got_data = false;

// Corre en la tarea de WiFi. Solo encola, nada de Serial aqui.
static void on_recv(const uint8_t *mac, const uint8_t *data, int len)
{
  if (len != (int)sizeof(imu_pkt_t)) return;
  if (data[0] != PKT_MAGIC) return;

  rx_item_t it;
  it.rx_us = (uint32_t)esp_timer_get_time();
  memcpy(&it.pkt, data, sizeof(imu_pkt_t));
  xQueueSend(q_rx, &it, 0);
  got_data = true;
}

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CH_ESPNOW, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    while (1) {
      Serial.println("esp_now_init fallo");
      delay(1000);
    }
  }

  q_rx = xQueueCreate(64, sizeof(rx_item_t));
  esp_now_register_recv_cb(on_recv);
}

void loop()
{
  static uint32_t t_mac_ms = 0;
  rx_item_t it;

  if (xQueueReceive(q_rx, &it, pdMS_TO_TICKS(10)) == pdTRUE) {
    uint8_t fr[3 + sizeof(rx_item_t) + 1];
    fr[0] = FR_SYNC0;
    fr[1] = FR_SYNC1;
    fr[2] = sizeof(rx_item_t);
    memcpy(&fr[3], &it, sizeof(rx_item_t));
    fr[3 + sizeof(rx_item_t)] = crc8((const uint8_t *)&it, sizeof(rx_item_t));
    Serial.write(fr, sizeof(fr));
  }

  // Mientras no llegue nada, imprime la MAC para usar unicast si se quiere.
  if (!got_data && millis() - t_mac_ms > 2000) {
    t_mac_ms = millis();
    Serial.print("GW_MAC ");
    Serial.println(WiFi.macAddress());
  }
}

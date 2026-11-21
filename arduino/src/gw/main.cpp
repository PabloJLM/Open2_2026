#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <string.h>
#include "packet.h"
#include "link.h"
#include "trunk.h"
#include "config.h"

static QueueHandle_t q_rx;
static volatile bool got_data = false;
static volatile uint32_t n_rx_any = 0;   // todo lo que llego por ESP-NOW
static volatile uint32_t n_rx_bad = 0;   // lo que se rechazo por largo o por magic
static tk_t tk;
static lk_parser_t lkp;

// Corre en la tarea de WiFi. Solo encola, nada de Serial aqui.
static void on_recv(const uint8_t *mac, const uint8_t *data, int len)
{
  n_rx_any++;
  if (len != (int)sizeof(imu_pkt_t) || data[0] != PKT_MAGIC) {
    n_rx_bad++;
    return;
  }

  rx_item_t it;
  it.rx_us = (uint32_t)esp_timer_get_time();
  memcpy(&it.pkt, data, sizeof(imu_pkt_t));
  xQueueSend(q_rx, &it, 0);
  got_data = true;
}

// Marco crudo A5 5A, el que lee tools/logger.py
static void send_raw(const rx_item_t *it)
{
  uint8_t fr[3 + sizeof(rx_item_t) + 1];
  fr[0] = FR_SYNC0;
  fr[1] = FR_SYNC1;
  fr[2] = sizeof(rx_item_t);
  memcpy(&fr[3], it, sizeof(rx_item_t));
  fr[3 + sizeof(rx_item_t)] = crc8((const uint8_t *)it, sizeof(rx_item_t));
  Serial.write(fr, sizeof(fr));
}

// Marco A5 5B, el que lee el dashboard
static void send_link(uint8_t type, const void *pl, uint8_t n)
{
  uint8_t fr[LK_MAX_PL + 5];
  size_t len = lk_encode(fr, type, pl, n);
  Serial.write(fr, len);
}

static void send_status(uint32_t now_ms)
{
  status_t st;
  tk_make_status(&tk, now_ms, &st);
  send_link(LK_T_STATUS, &st, sizeof(st));
}

// Comandos de la PC. Despues de cada uno responde con un STATUS.
static void read_commands()
{
  while (Serial.available() > 0) {
    uint8_t type, n;
    uint8_t pl[LK_MAX_PL];
    if (lk_feed(&lkp, (uint8_t)Serial.read(), &type, pl, &n)) {
      uint32_t now = millis();
      tk_cmd(&tk, type, pl, n, now);
      send_status(now);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  tk_init(&tk);
  memset(&lkp, 0, sizeof(lkp));

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
  static uint32_t t_status_ms = 0;
  rx_item_t it;

  read_commands();

  if (xQueueReceive(q_rx, &it, pdMS_TO_TICKS(5)) == pdTRUE) {
    uint32_t now = millis();
    if (tk.mode & LK_MODE_RAW) send_raw(&it);
    if (it.pkt.node_id <= 1) {
      tk_feed(&tk, &it.pkt, now);
      if (tk.mode & LK_MODE_LIVE) {
        live_t lv;
        tk_make_live(&tk, it.pkt.node_id, now, &lv);
        send_link(LK_T_LIVE, &lv, sizeof(lv));
      }
    }
  }

  tk_tick(&tk, millis());
  if (tk_take_evt(&tk)) send_status(millis());

  if (millis() - t_status_ms >= 1000) {
    t_status_ms = millis();
    send_status(t_status_ms);
  }

  // Mientras no llegue nada, imprime la MAC para usar unicast si se quiere.
  if (!got_data && millis() - t_mac_ms > 2000) {
    t_mac_ms = millis();
    uint8_t ch;
    wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);
    Serial.print("GW_MAC ");
    Serial.print(WiFi.macAddress());
    Serial.print(" CH ");
    Serial.print(ch);
    Serial.print(" RX ");
    Serial.print(n_rx_any);
    Serial.print(" RECHAZADOS ");
    Serial.println(n_rx_bad);
  }
}

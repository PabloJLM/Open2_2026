#ifndef ROLE_GW

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "packet.h"
#include "config.h"
#include "imu.h"

#define T_TICK_US (1000000UL / FS_HZ)   // 7812 us

static const char *TAG = "node";
static const uint8_t gw_mac[6] = GW_MAC;
static TaskHandle_t h_samp = NULL;
static imu_pkt_t pkt_buf[2];            // doble buffer, ESP-NOW envia en segundo plano
static volatile uint32_t t_tick_us = 0;
static volatile uint32_t n_send_fail = 0;
static volatile uint32_t n_miss_tot = 0;
static volatile uint32_t seq_dbg = 0;

// Corre en la tarea de esp_timer cada 1/FS_HZ. Solo marca el tick y despierta la tarea.
static void on_tick(void *arg)
{
  t_tick_us = (uint32_t)esp_timer_get_time();
  xTaskNotifyGive(h_samp);
}

static void on_sent(const uint8_t *mac, esp_now_send_status_t st)
{
  if (st != ESP_NOW_SEND_SUCCESS) n_send_fail++;
}

static void task_samp(void *arg)
{
  uint8_t idx = 0;
  uint8_t buf_sel = 0;
  uint32_t seq = 0;
  uint32_t n_miss = 0;

  for (;;) {
    uint32_t n_tk = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (n_tk > 1) n_miss += n_tk - 1;          // ticks que no alcanzamos a atender

    imu_pkt_t *p = &pkt_buf[buf_sel];
    if (idx == 0) p->t0_us = t_tick_us;
    imu_read(&p->s[idx]);
    idx++;

    if (idx == N_SAMP) {
      p->magic = PKT_MAGIC;
      p->node_id = NODE_ID;
      p->n_miss = (n_miss > 255) ? 255 : (uint8_t)n_miss;
      p->rsv = 0;
      p->seq = seq++;
      esp_now_send(gw_mac, (const uint8_t *)p, sizeof(imu_pkt_t));

      n_miss_tot += n_miss;
      seq_dbg = seq;
      n_miss = 0;
      idx = 0;
      buf_sel ^= 1;
    }
  }
}

void role_run(void)
{
  esp_now_register_send_cb(on_sent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, gw_mac, 6);
  peer.channel = CH_ESPNOW;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  ESP_ERROR_CHECK(esp_now_add_peer(&peer));

  if (!imu_init()) {
    ESP_LOGE(TAG, "imu_init fallo");
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  xTaskCreate(task_samp, "samp", 4096, NULL, 5, &h_samp);

  esp_timer_create_args_t args = {};
  args.callback = on_tick;
  args.name = "tick";
  esp_timer_handle_t h_tmr;
  ESP_ERROR_CHECK(esp_timer_create(&args, &h_tmr));
  ESP_ERROR_CHECK(esp_timer_start_periodic(h_tmr, T_TICK_US));

  // Solo diagnostico
  for (;;) {
    ESP_LOGI(TAG, "node %d seq %lu fail %lu miss %lu", NODE_ID,
             (unsigned long)seq_dbg, (unsigned long)n_send_fail,
             (unsigned long)n_miss_tot);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

#endif  // ROLE_GW

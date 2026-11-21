#pragma once
// Protocolo serial entre el gateway (master) y la PC, para el dashboard en PyQt.
// C estandar, sin Arduino ni IDF. Se copia a las dos versiones con tools/sync.sh
//
// Marco, igual en los dos sentidos
//   A5 5B len type payload[len] crc8
// len es solo el tamano del payload. crc8 (poly 0x07) cubre len, type y payload.
// Los marcos crudos de packet.h usan A5 5A, son otro tipo de marco y conviven en el mismo cable.

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "packet.h"

#define LK_SYNC0     0xA5
#define LK_SYNC1     0x5B
#define LK_MAX_PL    32
#define LK_PROTO_VER 2

// Gateway a PC
#define LK_T_LIVE    0x01   // angulos en vivo, uno por cada paquete que llega
#define LK_T_STATUS  0x02   // estado, uno por segundo y uno despues de cada comando

// PC al gateway
#define LK_C_CAL        0x10   // calibrar, el usuario debe estar erguido y quieto
#define LK_C_CAL_CLEAR  0x11   // borra la calibracion
#define LK_C_MODE       0x12   // payload de 1 byte con LK_MODE_*
#define LK_C_STATUS     0x13   // pide un STATUS
#define LK_C_AXIS       0x14   // eje de giro de la flexion, 0 X, 1 Y, 2 Z, y opcional inversion de signo por nodo

// Que marcos manda el gateway
#define LK_MODE_RAW   0x01   // marcos crudos A5 5A, los que lee logger.py
#define LK_MODE_LIVE  0x02   // marcos LIVE

// Bits de flags
#define LK_F_ALIVE0   0x01   // el nodo 0 mando algo en los ultimos 500 ms
#define LK_F_ALIVE1   0x02
#define LK_F_CAL0     0x04   // el nodo 0 tiene calibracion
#define LK_F_CAL1     0x08
#define LK_F_CALIBRANDO 0x10 // calibracion en curso

// Resultado del ultimo comando
#define LK_RES_OK        0
#define LK_RES_NO_NODES  1   // calibrar sin ningun nodo vivo
#define LK_RES_BAD_CMD   2
#define LK_RES_BAD_ARG   3
#define LK_RES_CAL_TIMEOUT 4 // un nodo no completo la calibracion a tiempo
#define LK_RES_CAL_MOVED   5 // el usuario se movio durante la calibracion, se conserva la anterior

// Angulos en vivo. 0 grados es erguido calibrado, positivo es flexion hacia adelante.
typedef struct __attribute__((packed)) {
  uint32_t t_ms;       // reloj del gateway
  uint8_t  flags;      // LK_F_*
  uint8_t  node;       // nodo cuyo paquete disparo este marco
  int16_t  ang0;       // lumbar, centesimas de grado
  int16_t  ang1;       // espalda alta, centesimas de grado
  int16_t  dif;        // ang1 menos ang0, centesimas de grado, indicador de espalda redondeada
  int16_t  w0;         // velocidad angular lumbar, decimas de grado por segundo
  int16_t  w1;         // velocidad angular espalda alta
  uint16_t lost0;      // paquetes perdidos acumulados por nodo
  uint16_t lost1;
} live_t;

typedef struct __attribute__((packed)) {
  uint8_t proto;       // LK_PROTO_VER
  uint8_t mode;        // LK_MODE_*
  uint8_t flags;       // LK_F_*
  uint8_t last_cmd;
  uint8_t last_res;    // LK_RES_*
  uint8_t fs_hz;       // FS_HZ
  uint8_t axis;        // eje de giro de la flexion, 0 X, 1 Y, 2 Z
  uint8_t inv;         // bit0 invierte el signo del nodo 0, bit1 el del nodo 1
} status_t;

#ifdef __cplusplus
static_assert(sizeof(live_t) == 20, "live_t debe medir 20 bytes");
static_assert(sizeof(status_t) == 8, "status_t debe medir 8 bytes");
#else
_Static_assert(sizeof(live_t) == 20, "live_t debe medir 20 bytes");
_Static_assert(sizeof(status_t) == 8, "status_t debe medir 8 bytes");
#endif

// Arma un marco completo en out, que debe tener LK_MAX_PL + 5 bytes. Devuelve su largo.
static inline size_t lk_encode(uint8_t *out, uint8_t type, const void *pl, uint8_t n)
{
  out[0] = LK_SYNC0;
  out[1] = LK_SYNC1;
  out[2] = n;
  out[3] = type;
  if (n) memcpy(&out[4], pl, n);
  out[4 + n] = crc8(&out[2], (size_t)n + 2);
  return (size_t)n + 5;
}

// Receptor byte a byte, para los comandos que llegan de la PC.
typedef struct {
  uint8_t st;                       // 0 espera A5, 1 espera 5B, 2 len, 3 type, 4 payload, 5 crc
  uint8_t i;
  uint8_t buf[LK_MAX_PL + 2];       // len, type, payload
  uint32_t n_crc_err;
} lk_parser_t;

// Devuelve 1 cuando completa un marco valido. Entonces type, pl y n quedan llenos.
static inline int lk_feed(lk_parser_t *p, uint8_t b, uint8_t *type, uint8_t *pl, uint8_t *n)
{
  switch (p->st) {
  case 0:
    if (b == LK_SYNC0) p->st = 1;
    break;
  case 1:
    if (b == LK_SYNC1) p->st = 2;
    else if (b != LK_SYNC0) p->st = 0;
    break;
  case 2:
    if (b > LK_MAX_PL) { p->st = 0; break; }
    p->buf[0] = b;
    p->st = 3;
    break;
  case 3:
    p->buf[1] = b;
    p->i = 0;
    p->st = (p->buf[0] == 0) ? 5 : 4;
    break;
  case 4:
    p->buf[2 + p->i] = b;
    if (++p->i >= p->buf[0]) p->st = 5;
    break;
  default:
    p->st = 0;
    if (crc8(p->buf, (size_t)p->buf[0] + 2) != b) { p->n_crc_err++; break; }
    *type = p->buf[1];
    *n = p->buf[0];
    if (*n) memcpy(pl, &p->buf[2], *n);
    return 1;
  }
  return 0;
}

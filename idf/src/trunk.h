#pragma once
// Inclinacion del tronco en el plano sagital, flexion hacia adelante. Corre en el gateway.
// C estandar, sin Arduino ni IDF. Se copia a las dos versiones con tools/sync.sh
//
// Idea
//   El acelerometro da el angulo respecto a la gravedad, pero se ensucia cuando te mueves.
//   El giroscopio da cambios suaves, pero se va desviando con el tiempo.
//   El filtro complementario toma el giroscopio a corto plazo y corrige con el acelerometro.
//   ang = a * (ang + w * dt) + (1 - a) * ang_acc

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "packet.h"
#include "link.h"

// Escalas de la MPU6050, deben coincidir con el rango que fijes en imu.cpp.
// Por defecto son las del reset del chip, +-2 g y +-250 grados/s.
#ifndef ACC_LSB_PER_G
#define ACC_LSB_PER_G    16384.0f
#endif
#ifndef GYR_LSB_PER_DPS
#define GYR_LSB_PER_DPS  131.0f
#endif

// Eje de giro de la flexion. Solo se monitorea un angulo por nodo, el que gira alrededor de este eje.
//   0 = X, 1 = Y, 2 = Z. Por defecto Z, con la placa pegada de lado al cuerpo y Z de izquierda a derecha.
// Se cambia desde la PC con el comando LK_C_AXIS, sin volver a compilar.
// Los otros dos ejes forman el plano donde se ve la gravedad. No importa cual de ellos apunta hacia arriba,
// porque la calibracion en posicion erguida deja ese punto en cero.
// Para otro valor de arranque, compila con -DTK_AXIS_DEFAULT=0 o 1.
#ifndef TK_AXIS_DEFAULT
#define TK_AXIS_DEFAULT  2
#endif
// Inversion de signo por nodo, bit0 nodo 0 y bit1 nodo 1. Por defecto ninguna.
// Prueba de signo. Calibra erguido e inclinate hacia adelante. El angulo debe subir.
// Si baja, invierte el signo de ese nodo con LK_C_AXIS.
#ifndef TK_INV_DEFAULT
#define TK_INV_DEFAULT   0
#endif

#ifndef TK_ALPHA
#define TK_ALPHA        0.98f   // peso del giroscopio, constante de tiempo de unos 0.4 s a 128 Hz
#endif
#define TK_ACC_TOL_G    0.25f   // si |a| se aleja tanto de 1 g, el acelerometro no se usa
#define TK_CAL_N        64      // muestras que se promedian al calibrar, 0.5 s
#define TK_CAL_MAX_RANGE_DEG 3.0f  // si el angulo varia mas que esto durante la calibracion, se rechaza
#define TK_CAL_TIMEOUT_MS 3000
#define TK_ALIVE_MS     500

typedef struct {
  float    ang;        // grados en (-180, 180], sin restar la calibracion
  float    w;          // grados por segundo
  float    off;        // angulo erguido, sale de la calibracion
  float    cal_ref, cal_sum, cal_min, cal_max;   // la calibracion se mide contra cal_ref para no romperse en +-180
  uint32_t last_seq;
  uint32_t lost;
  uint32_t last_rx_ms;
  uint16_t cal_n;
  uint8_t  init, have_seq, cal_on, cal_ok;
} tk_node_t;

typedef struct {
  tk_node_t n[2];
  uint32_t  cal_deadline_ms;
  uint8_t   mode;
  uint8_t   axis;        // 0 X, 1 Y, 2 Z, eje de giro de la flexion
  uint8_t   inv;         // bit n invierte el signo del nodo n
  uint8_t   last_cmd, last_res;
  uint8_t   cal_moved;   // algun nodo se movio durante la calibracion
  uint8_t   evt;         // la calibracion termino, hay que mandar un STATUS
} tk_t;

static inline void tk_init(tk_t *t)
{
  memset(t, 0, sizeof(*t));
  t->mode = LK_MODE_RAW | LK_MODE_LIVE;
  t->axis = TK_AXIS_DEFAULT;
  t->inv  = TK_INV_DEFAULT;
}

// Lleva un angulo en grados al rango (-180, 180]
static inline float tk_wrap(float d)
{
  while (d > 180.0f) d -= 360.0f;
  while (d <= -180.0f) d += 360.0f;
  return d;
}

static inline int tk_alive(const tk_node_t *e, uint32_t now_ms)
{
  return e->init && (uint32_t)(now_ms - e->last_rx_ms) < TK_ALIVE_MS;
}

// Una muestra de un nodo. q y p son los dos ejes del plano de la flexion en g,
// gyr es el giro alrededor del eje de la flexion en grados por segundo y norm es |a| en g.
// El angulo es atan2(p, q). Si la placa gira +a alrededor del eje, ese angulo cambia -a,
// por eso quien llama pasa gyr con signo menos.
static inline void tk_step(tk_node_t *e, float q, float p, float gyr, float norm)
{
  float ang_acc = atan2f(p, q) * 57.2957795f;
  if (!e->init) {
    e->ang = ang_acc;
    e->w = gyr;
    e->init = 1;
    return;
  }
  float a = TK_ALPHA;
  if (fabsf(norm - 1.0f) > TK_ACC_TOL_G) a = 1.0f;
  float pred = e->ang + gyr * (1.0f / FS_HZ);
  e->ang = tk_wrap(pred + (1.0f - a) * tk_wrap(ang_acc - pred));
  e->w = gyr;
}

// Cuando el ultimo nodo termina de calibrar, fija el resultado y avisa para mandar un STATUS.
static inline void tk_cal_end(tk_t *t)
{
  if (t->n[0].cal_on || t->n[1].cal_on) return;
  t->last_res = t->cal_moved ? LK_RES_CAL_MOVED : LK_RES_OK;
  t->evt = 1;
}

// Devuelve 1 una vez cuando termino una calibracion, para que el lazo principal mande un STATUS.
static inline int tk_take_evt(tk_t *t)
{
  int e = t->evt;
  t->evt = 0;
  return e;
}

// Procesa un paquete completo. Cuenta perdidos con seq y avanza la calibracion si esta activa.
static inline void tk_feed(tk_t *t, const imu_pkt_t *p, uint32_t now_ms)
{
  if (p->node_id > 1) return;
  tk_node_t *e = &t->n[p->node_id];

  if (e->have_seq) {
    uint32_t d = p->seq - e->last_seq;
    if (d == 0) return;                 // paquete repetido
    if (d >= 1000) e->init = 0;         // el nodo reinicio, arranca de nuevo
    else if (d > 1) {
      e->lost += d - 1;
      // Los paquetes perdidos se llevan su giroscopio. Se estima con la ultima velocidad.
      if (e->init) e->ang = tk_wrap(e->ang + e->w * ((d - 1) * N_SAMP) * (1.0f / FS_HZ));
    }
  }
  e->last_seq = p->seq;
  e->have_seq = 1;
  e->last_rx_ms = now_ms;

  // Eje de giro k. El plano de la flexion lo forman q = k+1 y p = k+2 modulo 3, con k = q x p.
  const int k = t->axis;
  const float sign = (t->inv & (1u << p->node_id)) ? -1.0f : 1.0f;

  for (int i = 0; i < N_SAMP; i++) {
    const imu_sample_t *s = &p->s[i];
    float acc[3] = { s->ax / ACC_LSB_PER_G, s->ay / ACC_LSB_PER_G, s->az / ACC_LSB_PER_G };
    float gyo[3] = { s->gx / GYR_LSB_PER_DPS, s->gy / GYR_LSB_PER_DPS, s->gz / GYR_LSB_PER_DPS };
    float norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);

    tk_step(e, acc[(k + 1) % 3], sign * acc[(k + 2) % 3], -sign * gyo[k], norm);

    if (e->cal_on) {
      if (e->cal_n == 0) {
        e->cal_ref = e->ang;
        e->cal_sum = 0;
        e->cal_min = e->cal_max = 0;
      }
      float d = tk_wrap(e->ang - e->cal_ref);
      if (d < e->cal_min) e->cal_min = d;
      if (d > e->cal_max) e->cal_max = d;
      e->cal_sum += d;
      if (++e->cal_n >= TK_CAL_N) {
        e->cal_on = 0;
        if (e->cal_max - e->cal_min > TK_CAL_MAX_RANGE_DEG) {
          t->cal_moved = 1;             // se conserva la calibracion anterior
        } else {
          e->off = tk_wrap(e->cal_ref + e->cal_sum / e->cal_n);
          e->cal_ok = 1;
        }
        tk_cal_end(t);
      }
    }
  }
}

static inline uint8_t tk_flags(const tk_t *t, uint32_t now_ms)
{
  uint8_t f = 0;
  if (tk_alive(&t->n[0], now_ms)) f |= LK_F_ALIVE0;
  if (tk_alive(&t->n[1], now_ms)) f |= LK_F_ALIVE1;
  if (t->n[0].cal_ok) f |= LK_F_CAL0;
  if (t->n[1].cal_ok) f |= LK_F_CAL1;
  if (t->n[0].cal_on || t->n[1].cal_on) f |= LK_F_CALIBRANDO;
  return f;
}

// Cierra una calibracion que no termino a tiempo. Llamala seguido desde el lazo principal.
static inline void tk_tick(tk_t *t, uint32_t now_ms)
{
  if (!(t->n[0].cal_on || t->n[1].cal_on)) return;
  if ((int32_t)(now_ms - t->cal_deadline_ms) >= 0) {
    t->n[0].cal_on = 0;
    t->n[1].cal_on = 0;
    t->last_res = LK_RES_CAL_TIMEOUT;
    t->evt = 1;
  }
}

// Ejecuta un comando que llego de la PC. Devuelve LK_RES_*.
static inline uint8_t tk_cmd(tk_t *t, uint8_t type, const uint8_t *pl, uint8_t n, uint32_t now_ms)
{
  uint8_t res = LK_RES_OK;
  switch (type) {
  case LK_C_CAL: {
    int any = 0;
    t->cal_moved = 0;
    for (int i = 0; i < 2; i++) {
      tk_node_t *e = &t->n[i];
      e->cal_n = 0;
      e->cal_sum = 0;
      e->cal_on = tk_alive(e, now_ms) ? 1 : 0;
      any |= e->cal_on;
    }
    if (any) t->cal_deadline_ms = now_ms + TK_CAL_TIMEOUT_MS;
    else res = LK_RES_NO_NODES;
    break;
  }
  case LK_C_CAL_CLEAR:
    for (int i = 0; i < 2; i++) {
      t->n[i].off = 0;
      t->n[i].cal_ok = 0;
      t->n[i].cal_on = 0;
    }
    break;
  case LK_C_MODE:
    if (n >= 1) t->mode = pl[0] & (LK_MODE_RAW | LK_MODE_LIVE);
    else res = LK_RES_BAD_ARG;
    break;
  case LK_C_STATUS:
    break;
  case LK_C_AXIS:
    // payload, byte 0 eje (0 X, 1 Y, 2 Z), byte 1 opcional con la inversion de signo por nodo
    if (n >= 1 && pl[0] <= 2) {
      t->axis = pl[0];
      if (n >= 2) t->inv = pl[1] & 0x03;
      // el angulo de antes no sirve con otro eje ni otro signo, se reinicia el filtro y la calibracion
      for (int i = 0; i < 2; i++) {
        t->n[i].init = 0;
        t->n[i].off = 0;
        t->n[i].cal_ok = 0;
        t->n[i].cal_on = 0;
      }
    } else res = LK_RES_BAD_ARG;
    break;
  default:
    res = LK_RES_BAD_CMD;
    break;
  }
  t->last_cmd = type;
  t->last_res = res;
  return res;
}

static inline int16_t tk_i16(float x)
{
  x += (x >= 0.0f) ? 0.5f : -0.5f;
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)x;
}

static inline uint16_t tk_u16(uint32_t x)
{
  return (x > 65535u) ? 65535u : (uint16_t)x;
}

static inline void tk_make_live(const tk_t *t, uint8_t node, uint32_t now_ms, live_t *o)
{
  const tk_node_t *a = &t->n[0];
  const tk_node_t *b = &t->n[1];
  float ang0 = tk_wrap(a->ang - a->off);
  float ang1 = tk_wrap(b->ang - b->off);
  o->t_ms  = now_ms;
  o->flags = tk_flags(t, now_ms);
  o->node  = node;
  o->ang0  = tk_i16(ang0 * 100.0f);
  o->ang1  = tk_i16(ang1 * 100.0f);
  o->dif   = tk_i16(tk_wrap(ang1 - ang0) * 100.0f);
  o->w0    = tk_i16(a->w * 10.0f);
  o->w1    = tk_i16(b->w * 10.0f);
  o->lost0 = tk_u16(a->lost);
  o->lost1 = tk_u16(b->lost);
}

static inline void tk_make_status(const tk_t *t, uint32_t now_ms, status_t *o)
{
  o->proto    = LK_PROTO_VER;
  o->mode     = t->mode;
  o->flags    = tk_flags(t, now_ms);
  o->last_cmd = t->last_cmd;
  o->last_res = t->last_res;
  o->fs_hz    = FS_HZ;
  o->axis     = t->axis;
  o->inv      = t->inv;
}

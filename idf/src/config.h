#pragma once

// Canal WiFi que usan nodos y gateway. Debe ser el mismo en todos.
#define CH_ESPNOW 1

// MAC del gateway. Con FF en los seis bytes se manda en broadcast,
// no hay ACK ni reintentos, pero no hace falta conocer la MAC.
// Para unicast pon la MAC que imprime el gateway al arrancar.
#define GW_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

#ifndef NODE_ID
#define NODE_ID 0
#endif

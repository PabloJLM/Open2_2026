# Protocolo del gateway con la PC

Definido en `common/link.h`. Lado PC en `tools/gwlink.py`.

## Marco

Igual en los dos sentidos.

```
A5 5B  len  type  payload[len]  crc8
```

`len` es solo el tamaño del payload, máximo 32. `crc8` usa el polinomio 0x07 y cubre `len`, `type` y `payload`. Todos los enteros son little endian.

Los marcos crudos del registrador (`A5 5A`, ver `packet.h`) van por el mismo cable. El texto `GW_MAC` también. El parser de la PC debe ignorar lo que no reconozca.

## Gateway a PC

| type | nombre | payload |
|---|---|---|
| 0x01 | LIVE | `live_t`, 20 bytes, uno por cada paquete que llega de un nodo |
| 0x02 | STATUS | `status_t`, 8 bytes, uno por segundo, uno después de cada comando y uno al terminar una calibración |

`live_t`

| campo | tipo | significado |
|---|---|---|
| t_ms | u32 | reloj del gateway |
| flags | u8 | bit0 nodo 0 vivo, bit1 nodo 1 vivo, bit2 nodo 0 calibrado, bit3 nodo 1 calibrado, bit4 calibrando |
| node | u8 | nodo cuyo paquete disparó el marco |
| ang0 | i16 | lumbar, centésimas de grado |
| ang1 | i16 | espalda alta, centésimas de grado |
| dif | i16 | ang1 menos ang0, centésimas de grado |
| w0, w1 | i16 | velocidad angular, décimas de grado por segundo |
| lost0, lost1 | u16 | paquetes perdidos acumulados por nodo |

Cero grados es erguido calibrado y positivo es flexión hacia adelante. Sin calibrar, el ángulo es el de la placa y no el del cuerpo, así que calibra antes de leerlo.

`status_t`

| campo | tipo | significado |
|---|---|---|
| proto | u8 | versión del protocolo, hoy 2 |
| mode | u8 | bit0 manda marcos crudos, bit1 manda marcos LIVE |
| flags | u8 | igual que en LIVE |
| last_cmd | u8 | último comando recibido |
| last_res | u8 | 0 ok, 1 sin nodos vivos, 2 comando inválido, 3 argumento inválido, 4 la calibración no terminó a tiempo, 5 te moviste durante la calibración |
| fs_hz | u8 | 128 |
| axis | u8 | eje de giro de la flexión, 0 X, 1 Y, 2 Z |
| inv | u8 | bit0 invierte el signo del nodo 0, bit1 el del nodo 1 |

## PC a gateway

| type | nombre | payload | efecto |
|---|---|---|---|
| 0x10 | CAL | ninguno | calibra. El usuario debe estar erguido y quieto 0.5 s. Se promedia el ángulo de cada nodo vivo y se toma como cero. Si el ángulo varía más de 3 grados en esa ventana se rechaza y se conserva la calibración anterior. Hay 3 s de plazo. |
| 0x11 | CAL_CLEAR | ninguno | borra la calibración |
| 0x12 | MODE | 1 byte | bit0 marcos crudos, bit1 marcos LIVE. Por defecto 3, los dos |
| 0x13 | STATUS | ninguno | pide un STATUS |
| 0x14 | AXIS | 1 o 2 bytes | byte 0 es el eje de giro, 0 X, 1 Y, 2 Z. Byte 1, opcional, es la inversión de signo por nodo, bit0 nodo 0 y bit1 nodo 1. Reinicia el filtro y borra la calibración. |

Cada comando se responde con un STATUS. Al terminar una calibración, bien o mal, el gateway manda otro STATUS en el momento. Para saber que terminó, espera ese STATUS con el bit de calibrando en cero y lee `last_res`.

## Ejemplo en Python

```python
import serial, gwlink
ser = serial.Serial("COM16", 115200, timeout=0.05)
parser = gwlink.LinkParser()
ser.write(gwlink.cmd_cal())
while True:
    for kind, d in parser.feed(ser.read(512)):
        if kind == "live":
            print(d["ang0"], d["ang1"], d["dif"])
```

## Eje y signo

Solo se monitorea un ángulo por nodo, el de la flexión hacia adelante. Es el giro alrededor de un eje de la placa, que por defecto es Z. Se cambia desde la PC con el comando AXIS, sin recompilar. Para otro valor de arranque, compila con `-DTK_AXIS_DEFAULT=0` o `1`.

Con Z como eje, la placa va pegada de lado al cuerpo, con Z apuntando de izquierda a derecha, como los discos del video. X e Y quedan en el plano de perfil.

Cómo se calcula. Los otros dos ejes forman el plano donde se ve la gravedad, y el ángulo es `atan2` de ellos. El giroscopio del eje elegido suaviza el movimiento rápido. No importa cuál de los dos ejes apunta hacia arriba, porque la calibración en posición erguida deja ese punto en cero.

Prueba de signo. Calibra erguido, inclínate hacia adelante y el ángulo debe subir. Si baja, invierte el signo de ese nodo con AXIS, o con la casilla de la GUI.

Si el eje configurado no es el eje real de giro, el ángulo no sigue al movimiento y se queda cerca de cero o se va a valores sin sentido. Prueba los tres ejes y quédate con el que sube al inclinarte.

Las escalas `ACC_LSB_PER_G` y `GYR_LSB_PER_DPS` de `common/trunk.h` deben coincidir con el rango que fijes en `imu.cpp`. Por defecto son las del reset del chip, 16384 y 131, que corresponden a más o menos 2 g y más o menos 250 grados por segundo. Si el giroscopio se satura en movimientos rápidos, sube el rango a 500 grados por segundo, pon 65.5 en `GYR_LSB_PER_DPS` y registra 0x08 en GYRO_CONFIG.

## Dashboard

`tools/gui.py` es la ventana en PyQt5. Muestra los dos ángulos, la diferencia, una gráfica de 15 s, una vista de perfil, calibración con cuenta de 3 s, cambio de eje y umbrales editables. Con `--demo` corre sin hardware.

```
py -m pip install PyQt5 pyserial
py tools\gui.py --port COM16
```

La GUI y el registrador no pueden usar el mismo puerto a la vez. Para grabar CSV cierra la GUI y usa `logger.py`.

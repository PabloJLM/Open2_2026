# lumbar

Dos nodos con MPU6050 mandan por ESP-NOW a 128 Hz a un gateway. El gateway reenvia por USB serial a un registrador en Python que guarda CSV etiquetado.

Estado actual. El firmware NO se ha compilado con PlatformIO ni probado con los sensores reales. Lo probado, en la PC, es el protocolo, el filtro de angulo y la calibracion de `trunk.h` con trayectorias simuladas, los parsers de Python y la GUI en modo demo.

## Estructura

```
common/packet.h      contrato de datos de los nodos, es la fuente unica
common/link.h        protocolo serial del gateway con la PC, ver PROTOCOLO.md
common/trunk.h       angulo de flexion, calibracion y comandos, corre en el gateway
arduino/             version Arduino, entornos node_s3mini node_xiao gw_nano
idf/                 version ESP-IDF, mismos entornos
tools/logger.py      registrador en la PC, guarda CSV crudo etiquetado
tools/gwlink.py      lado PC del protocolo, para la GUI y pruebas por terminal
tools/gui.py         dashboard en PyQt5
tools/sync.sh        copia packet.h, link.h y trunk.h a arduino/include y a idf/src
```

## Roles

- node_s3mini (ESP32-S3 Super Mini) es NODE_ID 0, va en la zona lumbar.
- node_xiao (XIAO ESP32C3) es NODE_ID 1, va en la espalda alta o el esternon.
- gw_nano es el gateway.

Los NODE_ID son arbitrarios. Se cambian con -DNODE_ID en platformio.ini.

## Lo que falta escribir

En `imu.cpp` de cada version hay un stub. `imu_init` y `imu_read` generan una rampa para probar el enlace. Reemplazalos por la lectura real de la MPU6050 en el orden ax ay az gx gy gz, valores crudos int16.

## Primer arranque con la version Arduino

```
cd arduino
pio run -e gw_nano -t upload
pio run -e node_s3mini -t upload
pio run -e node_xiao -t upload
```

Si la Nano no sube, pon la placa en modo bootloader con doble toque a reset.

Para probar el enlace con la rampa y sin sensor, en otra terminal.

```
pip install -r tools/requirements.txt
python3 tools/logger.py /dev/ttyACM0 --persona prueba --postura buena --carga ligera --kg 0 --serie 1 --seg 30
```

La linea de estado muestra por nodo paquetes recibidos, paquetes perdidos y ticks perdidos. Con el enlace sano las perdidas deben quedar cerca de cero.

## Version IDF

Misma estructura de entornos dentro de `idf/`. El rol se decide con el flag ROLE_GW, que ya esta puesto en el entorno gw_nano. Sin ese flag se compila como nodo.

El gateway saca los marcos por USB Serial JTAG con el driver. Subir IDF a la Nano ESP32 reemplaza el bootloader de Arduino de la placa. Se puede restaurar despues con la herramienta de bootloader de Arduino. Esto es de memoria, confirmalo antes.

## Notas del protocolo

- Cada paquete lleva 8 muestras, 62.5 ms. Un paquete perdido son 8 muestras perdidas.
- `t0_us` es el reloj del nodo, de 32 bits, se desborda cada 71 minutos. El registrador lo desenvuelve.
- Los relojes de los dos nodos no estan sincronizados. Para alinear usa un golpe en ambos sensores al inicio de cada serie.
- Por defecto el envio es broadcast, sin ACK ni reintentos. Para unicast pon la MAC del gateway en `GW_MAC` de config.h. El gateway imprime su MAC cada 2 s mientras no recibe nada.
- Si cambias algo en `common/`, corre `sh tools/sync.sh`. Si cambias `packet.h`, actualiza tambien `tools/logger.py`.

## Etiquetas del CSV

persona, postura (buena o mala), carga (ligera o pesada), kg y serie. El nombre del archivo es persona_postura_carga_sN.csv y el registrador no sobrescribe archivos existentes.

#!/bin/sh
# Copia los encabezados comunes a las dos versiones del firmware.
# Correlo despues de editar cualquier archivo de common/
set -e
cd "$(dirname "$0")/.."
for f in packet.h link.h trunk.h; do
  cp common/$f arduino/include/$f
  cp common/$f idf/src/$f
done
echo "encabezados sincronizados"

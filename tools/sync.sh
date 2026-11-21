#!/bin/sh
# Copia common/packet.h a las dos versiones del firmware.
# Correlo despues de editar common/packet.h
set -e
cd "$(dirname "$0")/.."
cp common/packet.h arduino/include/packet.h
cp common/packet.h idf/src/packet.h
echo "packet.h sincronizado"

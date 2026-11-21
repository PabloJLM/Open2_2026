#!/usr/bin/env python3
"""Registrador del gateway. Lee marcos binarios por serial, verifica CRC,
cuenta paquetes perdidos por nodo y guarda un CSV por muestra con etiquetas.

Uso
  python3 tools/logger.py /dev/ttyACM0 --persona ana --postura buena --carga pesada --kg 10 --serie 1
"""
import argparse
import csv
import os
import struct
import sys
import time

# Debe coincidir con common/packet.h
FS_HZ = 128
N_SAMP = 8
PKT_MAGIC = 0xB7
SYNC = b"\xA5\x5A"
PKT_FMT = "<BBBBII" + "6h" * N_SAMP     # magic node_id n_miss rsv seq t0_us muestras
ITEM_FMT = "<I" + PKT_FMT[1:]           # rx_us seguido del paquete
ITEM_LEN = struct.calcsize(ITEM_FMT)    # 112

COLS = ["persona", "postura", "carga", "kg", "serie", "node", "seq", "idx",
        "t_us", "rx_us", "ax", "ay", "az", "gx", "gy", "gz"]


def crc8(data):
    c = 0
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


class FrameParser:
    """Acumula bytes y entrega tuplas (rx_us, campos del paquete).
    Ignora texto y basura entre marcos, por ejemplo la linea GW_MAC."""

    def __init__(self):
        self.buf = bytearray()
        self.n_crc_err = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # conserva el ultimo byte por si es la mitad de SYNC
                del self.buf[:-1]
                break
            del self.buf[:i]
            if len(self.buf) < 3:
                break
            n = self.buf[2]
            if n != ITEM_LEN:
                del self.buf[:2]
                continue
            if len(self.buf) < 3 + n + 1:
                break
            body = bytes(self.buf[3:3 + n])
            crc_rx = self.buf[3 + n]
            if crc8(body) != crc_rx:
                self.n_crc_err += 1
                del self.buf[:2]
                continue
            del self.buf[:3 + n + 1]
            f = struct.unpack(ITEM_FMT, body)
            if f[1] != PKT_MAGIC:
                continue
            out.append(f)
        return out


class NodeState:
    def __init__(self):
        self.last_seq = None
        self.last_t0 = None
        self.n_wrap = 0
        self.n_pkt = 0
        self.n_lost = 0
        self.n_miss = 0

    def update(self, seq, t0_us, n_miss):
        """Cuenta perdidas por salto de seq y desenvuelve t0_us de 32 bits."""
        if self.last_seq is not None and seq > self.last_seq:
            self.n_lost += seq - self.last_seq - 1
        self.last_seq = seq
        if self.last_t0 is not None and t0_us < self.last_t0 - 2**31:
            self.n_wrap += 1
        self.last_t0 = t0_us
        self.n_pkt += 1
        self.n_miss += n_miss
        return t0_us + self.n_wrap * 2**32


def pkt_rows(f, st, labels):
    """Convierte un paquete en N_SAMP filas. Devuelve la lista de filas."""
    rx_us, _magic, node, n_miss, _rsv, seq, t0_us = f[:7]
    s = f[7:]
    t0 = st.update(seq, t0_us, n_miss)
    rows = []
    for k in range(N_SAMP):
        t_us = t0 + round(k * 1e6 / FS_HZ)
        a = s[6 * k:6 * k + 6]
        rows.append(labels + [node, seq, k, t_us, rx_us] + list(a))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--persona", required=True)
    ap.add_argument("--postura", required=True, choices=["buena", "mala"])
    ap.add_argument("--carga", required=True, choices=["ligera", "pesada"])
    ap.add_argument("--kg", required=True, type=float)
    ap.add_argument("--serie", required=True, type=int)
    ap.add_argument("--out", default="data")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seg", type=float, default=0, help="duracion en segundos, 0 sin limite")
    a = ap.parse_args()

    import serial  # pyserial

    os.makedirs(a.out, exist_ok=True)
    name = f"{a.persona}_{a.postura}_{a.carga}_s{a.serie}.csv"
    path = os.path.join(a.out, name)
    if os.path.exists(path):
        sys.exit(f"{path} ya existe, cambia --serie o borralo")

    labels = [a.persona, a.postura, a.carga, a.kg, a.serie]
    ser = serial.Serial(a.port, a.baud, timeout=0.05)
    ser.reset_input_buffer()
    parser = FrameParser()
    nodes = {}
    t_start = time.time()
    t_stat = t_start
    n_rows = 0

    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(COLS)
        print(f"Grabando en {path}, Ctrl-C para terminar")
        try:
            while True:
                data = ser.read(4096)
                for f in parser.feed(data):
                    st = nodes.setdefault(f[2], NodeState())
                    rows = pkt_rows(f, st, labels)
                    w.writerows(rows)
                    n_rows += len(rows)
                now = time.time()
                if now - t_stat >= 1.0:
                    t_stat = now
                    fh.flush()
                    txt = " ".join(
                        f"n{k} pkt {s.n_pkt} perd {s.n_lost} miss {s.n_miss}"
                        for k, s in sorted(nodes.items()))
                    print(f"\r{now - t_start:6.1f}s filas {n_rows} crc_err {parser.n_crc_err} {txt}   ",
                          end="", flush=True)
                if a.seg and now - t_start >= a.seg:
                    break
        except KeyboardInterrupt:
            pass
    print(f"\nListo, {n_rows} filas en {path}")


if __name__ == "__main__":
    main()

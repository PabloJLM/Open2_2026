#!/usr/bin/env python3
"""Lado PC del protocolo del gateway, definido en common/link.h.
Sirve para el dashboard en PyQt y tambien como prueba rapida por terminal.

Uso por terminal
  py tools\\gwlink.py COM16            muestra los angulos en vivo
  py tools\\gwlink.py COM16 --cal      pide calibrar al inicio, usuario erguido y quieto
  py tools\\gwlink.py COM16 --axis x   cambia el eje de giro de la flexion, x y o z, por defecto z
  py tools\\gwlink.py COM16 --axis z --inv0   invierte el signo del nodo 0 si el angulo baja al inclinarte
"""
import argparse
import struct
import sys
import time

# Debe coincidir con common/link.h
SYNC = b"\xA5\x5B"
MAX_PL = 32
T_LIVE, T_STATUS = 0x01, 0x02
C_CAL, C_CAL_CLEAR, C_MODE, C_STATUS, C_AXIS = 0x10, 0x11, 0x12, 0x13, 0x14
MODE_RAW, MODE_LIVE = 0x01, 0x02
LIVE_FMT = "<IBBhhhhhHH"       # t_ms flags node ang0 ang1 dif w0 w1 lost0 lost1
STATUS_FMT = "<BBBBBBBB"       # proto mode flags last_cmd last_res fs_hz axis inv
AXES = "XYZ"
RES = {0: "ok", 1: "sin nodos vivos", 2: "comando invalido", 3: "argumento invalido",
       4: "calibracion no termino a tiempo", 5: "te moviste durante la calibracion"}


def crc8(data):
    c = 0
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def encode(tipo, payload=b""):
    cuerpo = bytes([len(payload), tipo]) + payload
    return SYNC + cuerpo + bytes([crc8(cuerpo)])


def cmd_cal():
    return encode(C_CAL)


def cmd_cal_clear():
    return encode(C_CAL_CLEAR)


def cmd_mode(raw=True, live=True):
    return encode(C_MODE, bytes([(MODE_RAW if raw else 0) | (MODE_LIVE if live else 0)]))


def cmd_status():
    return encode(C_STATUS)


def cmd_axis(axis="z", inv0=False, inv1=False):
    """Elige el eje de giro de la flexion, "x", "y" o "z". Reinicia el filtro y la calibracion.
    inv0 e inv1 invierten el signo del nodo si el angulo baja al inclinarte hacia adelante."""
    n = "xyz".index(axis.lower())
    return encode(C_AXIS, bytes([n, (1 if inv0 else 0) | (2 if inv1 else 0)]))


def _live(p):
    t_ms, flags, node, a0, a1, dif, w0, w1, l0, l1 = struct.unpack(LIVE_FMT, p)
    return {"t_ms": t_ms, "node": node,
            "alive0": bool(flags & 1), "alive1": bool(flags & 2),
            "cal0": bool(flags & 4), "cal1": bool(flags & 8), "calibrando": bool(flags & 16),
            "ang0": a0 / 100.0, "ang1": a1 / 100.0, "dif": dif / 100.0,
            "w0": w0 / 10.0, "w1": w1 / 10.0, "lost0": l0, "lost1": l1}


def _status(p):
    proto, mode, flags, last_cmd, last_res, fs, axis, inv = struct.unpack(STATUS_FMT, p)
    return {"proto": proto, "raw": bool(mode & MODE_RAW), "live": bool(mode & MODE_LIVE),
            "alive0": bool(flags & 1), "alive1": bool(flags & 2),
            "cal0": bool(flags & 4), "cal1": bool(flags & 8), "calibrando": bool(flags & 16),
            "last_cmd": last_cmd, "last_res": last_res, "last_res_txt": RES.get(last_res, "?"),
            "fs_hz": fs, "axis": AXES[axis] if axis < 3 else "?", "inv0": bool(inv & 1), "inv1": bool(inv & 2)}


class LinkParser:
    """Acumula bytes y devuelve lista de ("live", dict) o ("status", dict).
    Ignora el texto y los marcos crudos A5 5A que van por el mismo cable."""

    def __init__(self):
        self.buf = bytearray()
        self.n_crc_err = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]
                break
            del self.buf[:i]
            if len(self.buf) < 4:
                break
            n = self.buf[2]
            if n > MAX_PL:
                del self.buf[:2]
                continue
            if len(self.buf) < n + 5:
                break
            cuerpo = bytes(self.buf[2:4 + n])
            if crc8(cuerpo) != self.buf[4 + n]:
                self.n_crc_err += 1
                del self.buf[:2]
                continue
            tipo, pl = cuerpo[1], cuerpo[2:]
            del self.buf[:n + 5]
            if tipo == T_LIVE and len(pl) == struct.calcsize(LIVE_FMT):
                out.append(("live", _live(pl)))
            elif tipo == T_STATUS and len(pl) == struct.calcsize(STATUS_FMT):
                out.append(("status", _status(pl)))
        return out


def main():
    import serial
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cal", action="store_true", help="pide calibrar al empezar")
    ap.add_argument("--axis", choices=["x", "y", "z"], help="eje de giro de la flexion")
    ap.add_argument("--inv0", action="store_true", help="invierte el signo del nodo 0")
    ap.add_argument("--inv1", action="store_true", help="invierte el signo del nodo 1")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    parser = LinkParser()
    if args.axis or args.inv0 or args.inv1:
        ser.write(cmd_axis(args.axis or "z", args.inv0, args.inv1))
        print("eje", args.axis or "z", "inv0", args.inv0, "inv1", args.inv1, "la calibracion se borro")
    if args.cal:
        time.sleep(1.5)                     # deja que el nodo llegue a estar vivo
        ser.write(cmd_cal())
        print("calibrando, quedate erguido y quieto")
    t_print = 0.0
    try:
        while True:
            for kind, d in parser.feed(ser.read(512)):
                if kind == "status":
                    print("estado", d)
                elif time.time() - t_print > 0.1:
                    t_print = time.time()
                    print(f"lumbar {d['ang0']:7.1f}  espalda {d['ang1']:7.1f}  dif {d['dif']:7.1f}  "
                          f"perdidos {d['lost0']}/{d['lost1']}  cal {int(d['cal0'])}{int(d['cal1'])}")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())

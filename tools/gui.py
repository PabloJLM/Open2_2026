#!/usr/bin/env python3
"""Dashboard en PyQt5 del gateway. Usa el protocolo de common/link.h a traves de gwlink.py.

Uso
  py tools\\gui.py                  abre la ventana, elige el puerto y pulsa Conectar
  py tools\\gui.py --port COM16     conecta al abrir
  py tools\\gui.py --demo           datos simulados, sin hardware

Dependencias
  py -m pip install PyQt5 pyserial
"""
import argparse
import math
import sys
import time
from collections import deque

from PyQt5 import QtCore, QtGui, QtWidgets

import gwlink

PLOT_SECONDS = 15.0
COL_OK, COL_WARN, COL_BAD = "#2e9e5b", "#e0a100", "#d64545"
COL_LUMBAR, COL_UPPER, COL_DIF = "#2f6fd6", "#e07a1f", "#7a7a7a"


def zone_color(ang, warn, alarm):
    if ang >= alarm:
        return COL_BAD
    if ang >= warn:
        return COL_WARN
    return COL_OK


# ---------------------------------------------------------------- fuentes de datos

class SerialSource:
    """Lee el puerto del gateway y entrega los marcos del protocolo."""

    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0)
        self.parser = gwlink.LinkParser()

    def poll(self):
        n = self.ser.in_waiting
        return self.parser.feed(self.ser.read(n)) if n else []

    def send(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.close()


class DemoSource:
    """Imita al gateway para probar la ventana sin hardware. Levanta una carga cada 6 s."""

    def __init__(self):
        self.t0 = time.monotonic()
        self.t_last = 0.0
        self.t_status = 0.0
        self.axis, self.inv = 2, 0
        self.cal = [False, False]
        self.cal_until = None
        self.mode = 3
        self.last_cmd, self.last_res = 0, 0
        self.pending = []

    def _angle(self, t):
        c = t % 6.0
        if c < 2.0:
            return 0.0
        if c < 3.2:
            return 62.0 * 0.5 * (1 - math.cos(math.pi * (c - 2.0) / 1.2))
        if c < 3.8:
            return 62.0
        if c < 5.0:
            return 62.0 * 0.5 * (1 + math.cos(math.pi * (c - 3.8) / 1.2))
        return 0.0

    def send(self, data):
        if len(data) < 5 or data[:2] != gwlink.SYNC:
            return
        n, tipo, pl = data[2], data[3], data[4:4 + data[2]]
        self.last_cmd, self.last_res = tipo, 0
        if tipo == gwlink.C_CAL:
            self.cal_until = time.monotonic() + 0.5
        elif tipo == gwlink.C_CAL_CLEAR:
            self.cal = [False, False]
        elif tipo == gwlink.C_AXIS and n >= 1 and pl[0] <= 2:
            self.axis = pl[0]
            self.inv = pl[1] & 3 if n >= 2 else self.inv
            self.cal = [False, False]
        elif tipo == gwlink.C_MODE and n >= 1:
            self.mode = pl[0] & 3
        self.pending.append(("status", self._status()))

    def _flags(self):
        return {"alive0": True, "alive1": True, "cal0": self.cal[0], "cal1": self.cal[1],
                "calibrando": self.cal_until is not None}

    def _status(self):
        d = {"proto": 2, "raw": bool(self.mode & 1), "live": bool(self.mode & 2),
             "last_cmd": self.last_cmd, "last_res": self.last_res,
             "last_res_txt": gwlink.RES.get(self.last_res, "?"), "fs_hz": 128,
             "axis": gwlink.AXES[self.axis], "inv0": bool(self.inv & 1), "inv1": bool(self.inv & 2)}
        d.update(self._flags())
        return d

    def poll(self):
        out, self.pending = self.pending, []
        now = time.monotonic()
        t = now - self.t0
        if self.cal_until is not None and now >= self.cal_until:
            self.cal_until = None
            self.cal = [True, True]
            self.last_res = 0
            out.append(("status", self._status()))
        while self.t_last + 0.0625 <= t:
            self.t_last += 0.0625
            a0 = self._angle(self.t_last) + 0.3 * math.sin(self.t_last * 2.0)
            a1 = a0 * 1.2 + 0.4 * math.sin(self.t_last * 3.1)
            d = {"t_ms": int(self.t_last * 1000), "node": 0, "ang0": a0, "ang1": a1, "dif": a1 - a0,
                 "w0": 0.0, "w1": 0.0, "lost0": 0, "lost1": 0}
            d.update(self._flags())
            if self.mode & 2:
                out.append(("live", d))
        if t - self.t_status >= 1.0:
            self.t_status = t
            out.append(("status", self._status()))
        return out

    def close(self):
        pass


# ---------------------------------------------------------------- widgets de dibujo

class Plot(QtWidgets.QWidget):
    """Grafica de los ultimos segundos. Lumbar, espalda alta y la diferencia."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(420, 240)
        self.buf = deque()
        self.warn, self.alarm = 30.0, 60.0
        self.show_dif = True

    def add(self, t, a0, a1, dif):
        self.buf.append((t, a0, a1, dif))
        while self.buf and t - self.buf[0][0] > PLOT_SECONDS + 1:
            self.buf.popleft()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        ml, mr, mt, mb = 44, 10, 10, 24
        r = QtCore.QRectF(ml, mt, w - ml - mr, h - mt - mb)
        p.fillRect(self.rect(), QtGui.QColor("#fafafa"))
        vals = [v for b in self.buf for v in b[1:3]]
        top = max(100.0, (max(vals) if vals else 0) + 10, self.alarm + 15)
        bot = min(-20.0, (min(vals) if vals else 0) - 10)

        def y(v):
            return r.bottom() - (v - bot) / (top - bot) * r.height()

        p.setPen(QtGui.QPen(QtGui.QColor("#dddddd"), 1))
        g = 20
        v = math.ceil(bot / g) * g
        while v <= top:
            p.drawLine(QtCore.QPointF(r.left(), y(v)), QtCore.QPointF(r.right(), y(v)))
            p.setPen(QtGui.QColor("#666666"))
            p.drawText(QtCore.QRectF(0, y(v) - 8, ml - 4, 16), QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, f"{v:.0f}°")
            p.setPen(QtGui.QPen(QtGui.QColor("#dddddd"), 1))
            v += g
        for val, col in ((self.warn, COL_WARN), (self.alarm, COL_BAD)):
            p.setPen(QtGui.QPen(QtGui.QColor(col), 1.5, QtCore.Qt.DashLine))
            p.drawLine(QtCore.QPointF(r.left(), y(val)), QtCore.QPointF(r.right(), y(val)))
        p.setPen(QtGui.QPen(QtGui.QColor("#999999"), 1))
        p.drawLine(QtCore.QPointF(r.left(), y(0)), QtCore.QPointF(r.right(), y(0)))
        if len(self.buf) > 1:
            tn = self.buf[-1][0]

            def x(t):
                return r.right() - (tn - t) / PLOT_SECONDS * r.width()

            series = [(1, COL_LUMBAR, 2.2), (2, COL_UPPER, 2.2)]
            if self.show_dif:
                series.append((3, COL_DIF, 1.2))
            p.setClipRect(r)
            for idx, col, wd in series:
                p.setPen(QtGui.QPen(QtGui.QColor(col), wd))
                pts = [QtCore.QPointF(x(b[0]), y(b[idx])) for b in self.buf if tn - b[0] <= PLOT_SECONDS]
                p.drawPolyline(QtGui.QPolygonF(pts))
            p.setClipping(False)
        p.setPen(QtGui.QColor("#666666"))
        p.drawText(QtCore.QRectF(r.left(), h - mb + 4, r.width(), 16), QtCore.Qt.AlignRight, f"últimos {PLOT_SECONDS:.0f} s")
        p.setPen(QtGui.QPen(QtGui.QColor("#bbbbbb"), 1))
        p.drawRect(r)


class Trunk(QtWidgets.QWidget):
    """Vista de perfil. El tramo inferior sigue al sensor lumbar y el superior al de espalda alta."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(260, 300)
        self.a0 = self.a1 = 0.0
        self.col = QtGui.QColor(COL_OK)
        self.live = False

    def set_angles(self, a0, a1, col, live):
        self.a0, self.a1, self.col, self.live = a0, a1, QtGui.QColor(col), live
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QtGui.QColor("#fafafa"))
        sc = min(w, h) / 300.0
        hip = QtCore.QPointF(w * 0.42, h * 0.62)
        p.setPen(QtGui.QPen(QtGui.QColor("#cccccc"), 2))
        p.drawLine(QtCore.QPointF(10, hip.y() + 110 * sc), QtCore.QPointF(w - 10, hip.y() + 110 * sc))
        col = self.col if self.live else QtGui.QColor("#aaaaaa")
        # piernas
        pen = QtGui.QPen(QtGui.QColor("#555555"), 9 * sc, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
        p.setPen(pen)
        p.drawLine(hip, QtCore.QPointF(hip.x(), hip.y() + 110 * sc))
        # referencia vertical
        p.setPen(QtGui.QPen(QtGui.QColor("#bbbbbb"), 1.5, QtCore.Qt.DashLine))
        p.drawLine(hip, QtCore.QPointF(hip.x(), hip.y() - 170 * sc))

        def tip(o, ang, ln):
            a = math.radians(ang)
            return QtCore.QPointF(o.x() + ln * sc * math.sin(a), o.y() - ln * sc * math.cos(a))

        mid = tip(hip, self.a0, 70)
        top = tip(mid, self.a1, 70)
        p.setPen(QtGui.QPen(col, 12 * sc, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        p.drawLine(hip, mid)
        p.drawLine(mid, top)
        head = tip(top, self.a1, 22)
        p.setBrush(QtGui.QBrush(col))
        p.setPen(QtCore.Qt.NoPen)
        p.drawEllipse(head, 17 * sc, 17 * sc)
        for pt, c in ((hip, COL_LUMBAR), (mid, COL_UPPER)):
            p.setBrush(QtGui.QBrush(QtGui.QColor(c)))
            p.setPen(QtGui.QPen(QtGui.QColor("white"), 2))
            p.drawEllipse(pt, 7 * sc, 7 * sc)
        p.setPen(QtGui.QColor("#888888"))
        p.drawText(QtCore.QRectF(0, h - 20, w, 18), QtCore.Qt.AlignCenter, "perfil, adelante a la derecha")


# ---------------------------------------------------------------- ventana principal

class Window(QtWidgets.QMainWindow):
    def __init__(self, port=None, demo=False):
        super().__init__()
        self.setWindowTitle("Lumbar, dashboard")
        self.resize(1180, 760)
        self.src = None
        self.last = None          # ultimo marco LIVE
        self.t_last_live = 0.0
        self.status = None
        self.cal_pending = False
        self.cal_count = 0
        self.cal_wait_result = False
        self.build()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(15)
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self.refresh)
        self.ui_timer.start(33)
        self.refresh_ports()
        if demo:
            self.chk_demo.setChecked(True)
            self.connect_toggle()
        elif port:
            self.cmb_port.setEditText(port)
            self.connect_toggle()

    # ---- construccion
    def build(self):
        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)
        root = QtWidgets.QVBoxLayout(cw)

        top = QtWidgets.QHBoxLayout()
        self.cmb_port = QtWidgets.QComboBox()
        self.cmb_port.setEditable(True)
        self.cmb_port.setMinimumWidth(140)
        btn_ref = QtWidgets.QPushButton("Actualizar")
        btn_ref.clicked.connect(self.refresh_ports)
        self.chk_demo = QtWidgets.QCheckBox("Demo sin hardware")
        self.btn_conn = QtWidgets.QPushButton("Conectar")
        self.btn_conn.clicked.connect(self.connect_toggle)
        self.lbl_link = QtWidgets.QLabel("Desconectado")
        self.lbl_link.setStyleSheet("font-weight: 600;")
        for wdg in (QtWidgets.QLabel("Puerto"), self.cmb_port, btn_ref, self.chk_demo, self.btn_conn):
            top.addWidget(wdg)
        top.addStretch(1)
        top.addWidget(self.lbl_link)
        root.addLayout(top)

        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)

        left = QtWidgets.QVBoxLayout()
        body.addLayout(left, 0)
        self.trunk = Trunk()
        left.addWidget(self.trunk, 1)

        self.lbl_state = QtWidgets.QLabel("Sin datos")
        self.lbl_state.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_state.setStyleSheet("font-size: 20px; font-weight: 700; color: white; background: #aaaaaa; padding: 8px; border-radius: 6px;")
        left.addWidget(self.lbl_state)

        right = QtWidgets.QVBoxLayout()
        body.addLayout(right, 1)

        grid = QtWidgets.QGridLayout()
        self.big = {}
        for col, (key, title, color) in enumerate((("a0", "Lumbar", COL_LUMBAR), ("a1", "Espalda alta", COL_UPPER), ("dif", "Diferencia, redondeo", COL_DIF))):
            box = QtWidgets.QGroupBox(title)
            lay = QtWidgets.QVBoxLayout(box)
            val = QtWidgets.QLabel("--")
            val.setAlignment(QtCore.Qt.AlignCenter)
            val.setStyleSheet(f"font-size: 40px; font-weight: 700; color: {color};")
            sub = QtWidgets.QLabel("")
            sub.setAlignment(QtCore.Qt.AlignCenter)
            sub.setStyleSheet("color: #777777;")
            lay.addWidget(val)
            lay.addWidget(sub)
            grid.addWidget(box, 0, col)
            self.big[key] = (val, sub)
        right.addLayout(grid)

        self.plot = Plot()
        right.addWidget(self.plot, 1)

        ctl = QtWidgets.QHBoxLayout()
        right.addLayout(ctl)

        g_cal = QtWidgets.QGroupBox("Calibración")
        lc = QtWidgets.QVBoxLayout(g_cal)
        self.btn_cal = QtWidgets.QPushButton("Calibrar erguido")
        self.btn_cal.clicked.connect(self.start_cal)
        btn_clr = QtWidgets.QPushButton("Borrar calibración")
        btn_clr.clicked.connect(lambda: self.send(gwlink.cmd_cal_clear()))
        self.lbl_cal = QtWidgets.QLabel("Párate recto, quieto, y pulsa el botón. Hay 3 s de cuenta.")
        self.lbl_cal.setWordWrap(True)
        lc.addWidget(self.btn_cal)
        lc.addWidget(btn_clr)
        lc.addWidget(self.lbl_cal)
        ctl.addWidget(g_cal, 1)

        g_ax = QtWidgets.QGroupBox("Eje de giro de la flexión")
        la = QtWidgets.QGridLayout(g_ax)
        self.cmb_axis = QtWidgets.QComboBox()
        self.cmb_axis.addItems(["X", "Y", "Z"])
        self.cmb_axis.setCurrentIndex(2)
        self.chk_inv0 = QtWidgets.QCheckBox("Invertir signo nodo 0")
        self.chk_inv1 = QtWidgets.QCheckBox("Invertir signo nodo 1")
        btn_ax = QtWidgets.QPushButton("Aplicar")
        btn_ax.clicked.connect(self.apply_axis)
        hint = QtWidgets.QLabel("Prueba de signo. Calibra erguido e inclínate hacia adelante, el ángulo debe subir. Si baja, invierte ese nodo. Aplicar borra la calibración.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #777777;")
        la.addWidget(QtWidgets.QLabel("Eje"), 0, 0)
        la.addWidget(self.cmb_axis, 0, 1)
        la.addWidget(btn_ax, 0, 2)
        la.addWidget(self.chk_inv0, 1, 0, 1, 3)
        la.addWidget(self.chk_inv1, 2, 0, 1, 3)
        la.addWidget(hint, 3, 0, 1, 3)
        ctl.addWidget(g_ax, 1)

        g_th = QtWidgets.QGroupBox("Umbrales, pendientes de los videos")
        lt = QtWidgets.QFormLayout(g_th)
        self.sp_warn = QtWidgets.QDoubleSpinBox()
        self.sp_alarm = QtWidgets.QDoubleSpinBox()
        for sp, v in ((self.sp_warn, 30.0), (self.sp_alarm, 60.0)):
            sp.setRange(0, 170)
            sp.setDecimals(0)
            sp.setSuffix("°")
            sp.setValue(v)
        lt.addRow("Cuidado desde", self.sp_warn)
        lt.addRow("Riesgo desde", self.sp_alarm)
        note = QtWidgets.QLabel("Valores de ejemplo. Se reemplazan con los rangos que salgan de los clips.")
        note.setWordWrap(True)
        note.setStyleSheet("color: #777777;")
        lt.addRow(note)
        self.chk_dif = QtWidgets.QCheckBox("Mostrar diferencia en la gráfica")
        self.chk_dif.setChecked(True)
        lt.addRow(self.chk_dif)
        ctl.addWidget(g_th, 1)

        self.lbl_link2 = QtWidgets.QLabel("")
        self.lbl_link2.setStyleSheet("color: #555555;")
        root.addWidget(self.lbl_link2)

    # ---- conexion
    def refresh_ports(self):
        cur = self.cmb_port.currentText()
        self.cmb_port.clear()
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                self.cmb_port.addItem(p.device)
        except Exception:
            pass
        if cur:
            self.cmb_port.setEditText(cur)

    def connect_toggle(self):
        if self.src is not None:
            self.src.close()
            self.src = None
            self.btn_conn.setText("Conectar")
            self.lbl_link.setText("Desconectado")
            return
        try:
            self.src = DemoSource() if self.chk_demo.isChecked() else SerialSource(self.cmb_port.currentText())
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "No se pudo abrir el puerto", str(e))
            return
        self.btn_conn.setText("Desconectar")
        self.lbl_link.setText("Conectado, esperando datos")
        self.last = None
        self.plot.buf.clear()
        self.send(gwlink.cmd_status())

    def send(self, data):
        if self.src is None:
            return
        try:
            self.src.send(data)
        except Exception as e:
            self.lbl_link.setText(f"Error de envío, {e}")

    # ---- comandos
    def start_cal(self):
        if self.src is None:
            return
        self.cal_count = 3
        self.btn_cal.setEnabled(False)
        self.cal_tick()

    def cal_tick(self):
        if self.cal_count > 0:
            self.lbl_cal.setStyleSheet("font-weight: 700; color: #b36b00;")
            self.lbl_cal.setText(f"Quédate erguido y quieto, calibrando en {self.cal_count}")
            self.cal_count -= 1
            QtCore.QTimer.singleShot(1000, self.cal_tick)
        else:
            self.lbl_cal.setText("Midiendo, no te muevas")
            self.cal_wait_result = True
            self.send(gwlink.cmd_cal())

    def apply_axis(self):
        self.send(gwlink.cmd_axis("xyz"[self.cmb_axis.currentIndex()], self.chk_inv0.isChecked(), self.chk_inv1.isChecked()))

    # ---- datos
    def tick(self):
        if self.src is None:
            return
        try:
            events = self.src.poll()
        except Exception as e:
            self.lbl_link.setText(f"Error de lectura, {e}")
            self.connect_toggle()
            return
        for kind, d in events:
            if kind == "live":
                self.last = d
                self.t_last_live = time.monotonic()
                self.plot.add(self.t_last_live, d["ang0"], d["ang1"], d["dif"])
            else:
                self.on_status(d)

    def on_status(self, d):
        self.status = d
        if not self.cmb_axis.hasFocus():
            self.cmb_axis.setCurrentIndex("XYZ".index(d["axis"]) if d["axis"] in "XYZ" else 2)
        for chk, key in ((self.chk_inv0, "inv0"), (self.chk_inv1, "inv1")):
            if not chk.hasFocus():
                chk.setChecked(d[key])
        if self.cal_wait_result and not d["calibrando"] and d["last_cmd"] == gwlink.C_CAL:
            self.cal_wait_result = False
            self.btn_cal.setEnabled(True)
            if d["last_res"] == 0:
                self.lbl_cal.setStyleSheet("font-weight: 700; color: " + COL_OK + ";")
                self.lbl_cal.setText("Calibrado. Cero es la posición erguida.")
            else:
                self.lbl_cal.setStyleSheet("font-weight: 700; color: " + COL_BAD + ";")
                self.lbl_cal.setText("No se calibró, " + d["last_res_txt"] + ". Inténtalo de nuevo.")
        if d["last_cmd"] == gwlink.C_CAL and d["last_res"] == 1 and self.cal_wait_result:
            self.cal_wait_result = False
            self.btn_cal.setEnabled(True)
            self.lbl_cal.setStyleSheet("font-weight: 700; color: " + COL_BAD + ";")
            self.lbl_cal.setText("No se calibró, " + d["last_res_txt"] + ".")

    # ---- pantalla
    def refresh(self):
        warn, alarm = self.sp_warn.value(), self.sp_alarm.value()
        self.plot.warn, self.plot.alarm, self.plot.show_dif = warn, alarm, self.chk_dif.isChecked()
        fresh = self.last is not None and time.monotonic() - self.t_last_live < 1.5
        d = self.last
        if not fresh:
            for key in self.big:
                self.big[key][0].setText("--")
                self.big[key][1].setText("")
            self.trunk.set_angles(0, 0, "#aaaaaa", False)
            self.lbl_state.setText("Sin datos")
            self.lbl_state.setStyleSheet("font-size: 20px; font-weight: 700; color: white; background: #aaaaaa; padding: 8px; border-radius: 6px;")
            if self.src is not None:
                self.lbl_link.setText("Conectado, sin datos del gateway")
            self.plot.update()
            return
        a0, a1, dif = d["ang0"], d["ang1"], d["dif"]
        col = zone_color(a0, warn, alarm)
        self.big["a0"][0].setText(f"{a0:.1f}°")
        self.big["a1"][0].setText(f"{a1:.1f}°")
        self.big["dif"][0].setText(f"{dif:+.1f}°")
        self.big["a0"][1].setText(("calibrado" if d["cal0"] else "sin calibrar") + ("" if d["alive0"] else ", nodo sin señal"))
        self.big["a1"][1].setText(("calibrado" if d["cal1"] else "sin calibrar") + ("" if d["alive1"] else ", nodo sin señal"))
        self.big["dif"][1].setText("espalda alta menos lumbar")
        self.trunk.set_angles(a0, a1, col, d["alive0"] or d["alive1"])
        txt = "Bien" if col == COL_OK else ("Cuidado" if col == COL_WARN else "Riesgo")
        self.lbl_state.setText(f"{txt}, lumbar {a0:.0f}°")
        self.lbl_state.setStyleSheet(f"font-size: 20px; font-weight: 700; color: white; background: {col}; padding: 8px; border-radius: 6px;")
        ok = d["alive0"] and d["alive1"]
        self.lbl_link.setText("Enlace bien, dos nodos" if ok else "Falta un nodo")
        self.lbl_link.setStyleSheet(f"font-weight: 600; color: {COL_OK if ok else COL_BAD};")
        extra = ""
        if self.status:
            extra = f"   eje {self.status['axis']}   marcos crudos {'sí' if self.status['raw'] else 'no'}"
        self.lbl_link2.setText(f"Paquetes perdidos, lumbar {d['lost0']}, espalda alta {d['lost1']}{extra}")
        self.plot.update()

    def closeEvent(self, e):
        if self.src is not None:
            self.src.close()
        super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="puerto del gateway, por ejemplo COM16")
    ap.add_argument("--demo", action="store_true", help="datos simulados")
    args = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = Window(port=args.port, demo=args.demo)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())

"""
drone_director.py
=================
Sistema integrado del Drone Filmador Autonomo.

Une los 3 modulos en un flujo completo:
  1. Escaneo inicial del area
  2. Loop de ordenes en español
  3. Tracking visual en background

Uso:
    python3 drone_director.py
    python3 drone_director.py --sin-drone --video test_personas.mp4
"""

import asyncio
import argparse
import cv2
import json
import math
import glob
import time
import threading
import numpy as np
from datetime import datetime
from ultralytics import YOLO
from evitacion_obstaculos import SistemaEvitacion
from parser_llm import ParserLLM


# ============================================================
# CONFIGURACION GLOBAL
# ============================================================
CONFIG = {
    # vuelo
    "altura_despegue":      3.0,
    "altura_filmacion":     5.0,
    "altura_escaneo":       8.0,
    "altura_cenital":       12.0,
    "velocidad_ms":         2.0,

    # escaneo
    "radio_inicial":        3.0,
    "radio_maximo":         15.0,
    "incremento_radio":     3.0,
    "puntos_por_anillo":    8,

    # vision
    "modelo_yolo":          "yolov8n.pt",
    "confianza_minima":     0.45,
    "resolucion":           (640, 480),
    "fps_vision":           15,
    "zona_muerta_px":       50,
    "ganancia_lateral":     0.012,
    "max_correccion_m":     1.5,

    # colores UI
    "color_deteccion":      (0, 255, 0),
    "color_tracking":       (0, 100, 255),
    "color_orden":          (255, 255, 0),
}

COLORES_DESCRIPTOR = {
    "roja": (0, 0, 255), "rojo": (0, 0, 255),
    "azul": (255, 100, 0), "azules": (255, 100, 0),
    "verde": (0, 200, 0), "verdes": (0, 200, 0),
    "blanca": (200, 200, 200), "blanco": (200, 200, 200),
    "negra": (50, 50, 50), "negro": (50, 50, 50),
    "amarilla": (0, 220, 220), "amarillo": (0, 220, 220),
}


# ============================================================
# ESTADO COMPARTIDO (entre threads)
# ============================================================
class EstadoSistema:
    def __init__(self):
        self.lock = threading.Lock()

        # vision
        self.frame_actual = None
        self.personas_detectadas = []
        self.sujeto_activo = None
        self.correccion_actual = {"tipo": "sin_sujeto", "norte": 0.0, "este": 0.0}

        # drone
        self.posicion_ned = {"norte": 0.0, "este": 0.0, "altura": 0.0}
        self.modo_actual = "idle"          # idle | escaneo | orden | tracking
        self.tracking_activo = False
        self.corriendo = True

        # log
        self.ultima_orden = ""
        self.ultimo_log = ""
        self.mapa = None

    def log(self, msg: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        linea = f"[{timestamp}] {msg}"
        self.ultimo_log = linea
        print(linea)

    def set_modo(self, modo: str):
        with self.lock:
            self.modo_actual = modo
        self.log(f"Modo → {modo.upper()}")


# ============================================================
# MODULO 1: ESCANER
# ============================================================
class Escaner:
    def __init__(self, estado: EstadoSistema, config: dict):
        self.estado = estado
        self.config = config

    def generar_espiral(self) -> list:
        waypoints = []
        radio = self.config["radio_inicial"]
        puntos = self.config["puntos_por_anillo"]
        altura = self.config["altura_escaneo"]

        while radio <= self.config["radio_maximo"]:
            for i in range(puntos):
                angulo = (2 * math.pi * i) / puntos
                norte = radio * math.cos(angulo)
                este = radio * math.sin(angulo)
                yaw = math.degrees(math.atan2(-este, -norte)) % 360
                waypoints.append((norte, este, altura, yaw))
            radio += self.config["incremento_radio"]

        return waypoints

    async def ejecutar(self, drone=None) -> dict:
        self.estado.set_modo("escaneo")
        waypoints = self.generar_espiral()
        self.estado.log(f"Escaneo: {len(waypoints)} waypoints, radio {self.config['radio_maximo']}m")

        if drone:
            from mavsdk.offboard import PositionNedYaw, OffboardError
            await drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -self.config["altura_escaneo"], 0.0)
            )
            try:
                await drone.offboard.start()
            except OffboardError:
                pass

            for norte, este, altura, yaw in waypoints:
                await drone.offboard.set_position_ned(
                    PositionNedYaw(norte, este, -altura, yaw)
                )
                dist = math.sqrt(norte**2 + este**2)
                await asyncio.sleep(max(1.0, dist / self.config["velocidad_ms"] * 0.3))

            await drone.offboard.stop()
            self.estado.log("Escaneo completado.")
        else:
            # simulacion sin drone
            self.estado.log("Escaneo simulado (sin drone)...")
            await asyncio.sleep(2)

        # construir mapa
        mapa = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "area_radio_m": self.config["radio_maximo"],
            "altura_escaneo_m": self.config["altura_escaneo"],
            "waypoints": [
                {"idx": i, "norte": round(wp[0], 2), "este": round(wp[1], 2),
                 "altura": wp[2], "yaw": round(wp[3], 1)}
                for i, wp in enumerate(waypoints)
            ],
            "estado": "completado",
            "total_waypoints": len(waypoints)
        }

        # guardar
        archivo = f"mapa_evento_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(archivo, "w") as f:
            json.dump(mapa, f, indent=2)
        self.estado.log(f"Mapa guardado: {archivo}")
        self.estado.mapa = mapa
        return mapa


# ============================================================
# MODULO 2: PARSER DE ORDENES
# ============================================================
class ParserOrdenes:
    COMANDOS = {
        "panoramica":     ["panoramica", "panorama", "360", "vuelta completa"],
        "cenital":        ["cenital", "desde arriba", "plano cenital"],
        "norte":          ["norte", "fondo"],
        "sur":            ["sur", "entrada"],
        "este":           ["este", "derecha"],
        "oeste":          ["oeste", "izquierda"],
        "centro":         ["centro", "medio", "volver", "regresa"],
        "hover":          ["hover", "queda quieto", "para", "espera"],
        "bajar":          ["baja", "bajar", "descende", "mas bajo"],
        "subir":          ["sube", "subir", "asciende", "mas alto"],
        "aterrizar":      ["aterriza", "aterrizar", "tierra", "termina"],
        "seguir":         ["segui", "sigue", "seguir", "enfoca", "apunta"],
        "dejar_seguir":   ["deja de seguir", "dejar de seguir", "stop tracking", "suelta", "libre"],
        "escanear":       ["escanea", "escaneá", "rescaneá", "nuevo escaneo"],
        "estado":         ["estado", "donde estas", "posicion", "info"],
    }

    def parsear(self, texto: str) -> dict:
        texto_lower = texto.lower().strip()
        for comando, keywords in self.COMANDOS.items():
            for kw in keywords:
                if kw in texto_lower:
                    return {
                        "comando": comando,
                        "texto": texto,
                        "params": self._params(texto_lower, comando)
                    }
        return {"comando": "desconocido", "texto": texto, "params": {}}

    def _params(self, texto: str, comando: str) -> dict:
        params = {}
        palabras = texto.split()
        for p in palabras:
            if p.isdigit():
                params["numero"] = int(p)
                break
        if comando == "seguir":
            for color in COLORES_DESCRIPTOR:
                if color in texto:
                    params["color"] = color
                    break
        return params


# ============================================================
# MODULO 3: VISION Y TRACKING
# ============================================================
class SistemaVision:
    def __init__(self, estado: EstadoSistema, config: dict, fuente_video):
        self.estado = estado
        self.config = config
        self.fuente_video = fuente_video
        self.modelo = None
        self.cap = None
        self.frames_sin_deteccion = 0
        self.max_perdida = 15

    def iniciar(self):
        self.estado.log("Cargando YOLOv8...")
        self.modelo = YOLO(self.config["modelo_yolo"])
        self.cap = cv2.VideoCapture(self.fuente_video)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config["resolucion"][0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config["resolucion"][1])
        self.estado.log("Vision activa.")

    def _detectar(self, frame) -> list:
        resultados = self.modelo(frame, conf=self.config["confianza_minima"],
                                  classes=[0], verbose=False)
        personas = []
        for r in resultados:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                personas.append({
                    "bbox": (x1, y1, x2, y2), "centro": (cx, cy),
                    "confianza": conf, "area": (x2-x1) * (y2-y1),
                    "ancho": x2-x1, "alto": y2-y1,
                })
        personas.sort(key=lambda p: p["area"], reverse=True)
        return personas

    def _actualizar_sujeto(self, personas: list):
        if not personas:
            self.frames_sin_deteccion += 1
            if self.frames_sin_deteccion > self.max_perdida:
                self.estado.sujeto_activo = None
            return

        self.frames_sin_deteccion = 0
        sujeto_prev = self.estado.sujeto_activo

        if sujeto_prev is None:
            self.estado.sujeto_activo = personas[0]
        else:
            cx_p, cy_p = sujeto_prev["centro"]
            mejor = min(personas, key=lambda p: math.sqrt(
                (p["centro"][0]-cx_p)**2 + (p["centro"][1]-cy_p)**2))
            self.estado.sujeto_activo = mejor

    def _calcular_correccion(self, sujeto) -> dict:
        if sujeto is None:
            return {"tipo": "sin_sujeto", "norte": 0.0, "este": 0.0}

        w, h = self.config["resolucion"]
        cx_f, cy_f = w // 2, h // 2
        cx, cy = sujeto["centro"]
        off_x = cx - cx_f
        off_y = cy - cy_f
        zona = self.config["zona_muerta_px"]

        if abs(off_x) < zona and abs(off_y) < zona:
            return {"tipo": "en_zona_muerta", "norte": 0.0, "este": 0.0,
                    "off_x": off_x, "off_y": off_y}

        g = self.config["ganancia_lateral"]
        max_c = self.config["max_correccion_m"]
        corr_este = max(-max_c, min(max_c, off_x * g))
        corr_norte = max(-max_c, min(max_c, -off_y * g))

        return {"tipo": "correccion", "norte": round(corr_norte, 3),
                "este": round(corr_este, 3), "off_x": off_x, "off_y": off_y}

    def _dibujar(self, frame, personas, sujeto, correccion) -> np.ndarray:
        h, w = frame.shape[:2]
        cx_f, cy_f = w // 2, h // 2

        # cruz y zona muerta
        cv2.line(frame, (cx_f-25, cy_f), (cx_f+25, cy_f), (255,255,255), 1)
        cv2.line(frame, (cx_f, cy_f-25), (cx_f, cy_f+25), (255,255,255), 1)
        zona = self.config["zona_muerta_px"]
        cv2.rectangle(frame, (cx_f-zona, cy_f-zona),
                      (cx_f+zona, cy_f+zona), (60,60,60), 1)

        # detecciones
        for p in personas:
            x1,y1,x2,y2 = p["bbox"]
            cv2.rectangle(frame, (x1,y1), (x2,y2),
                          self.config["color_deteccion"], 1)

        # sujeto
        if sujeto:
            x1,y1,x2,y2 = sujeto["bbox"]
            col = self.config["color_tracking"]
            cv2.rectangle(frame, (x1,y1), (x2,y2), col, 2)
            cv2.putText(frame, "TRACKING", (x1, y1-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            scx, scy = sujeto["centro"]
            cv2.line(frame, (cx_f, cy_f), (scx, scy), col, 1)

        # info
        tipo = correccion.get("tipo", "")
        modo = self.estado.modo_actual.upper()
        if tipo == "correccion":
            txt = f"N:{correccion['norte']:+.2f}m E:{correccion['este']:+.2f}m"
            cv2.putText(frame, txt, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        self.config["color_tracking"], 2)
        elif tipo == "en_zona_muerta":
            cv2.putText(frame, "CENTRADO", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        self.config["color_deteccion"], 2)
        else:
            cv2.putText(frame, "Buscando...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,200,255), 2)

        # modo y ultima orden
        cv2.putText(frame, f"MODO: {modo}", (10, h-40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)
        cv2.putText(frame, f"Orden: {self.estado.ultima_orden[:40]}",
                    (10, h-20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, self.config["color_orden"], 1)
        cv2.putText(frame, "R:reset Q:salir",
                    (w-130, h-10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (150,150,150), 1)
        return frame

    async def loop(self):
        """Loop de vision — corre en paralelo con el loop de ordenes."""
        self.iniciar()
        interval = 1.0 / self.config["fps_vision"]

        while self.estado.corriendo:
            ret, frame = self.cap.read()
            if not ret:
                # reiniciar video si llega al final
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                await asyncio.sleep(0.1)
                continue

            personas = self._detectar(frame)

            with self.estado.lock:
                self.estado.personas_detectadas = personas
                if self.estado.tracking_activo:
                    self._actualizar_sujeto(personas)
                correccion = self._calcular_correccion(
                    self.estado.sujeto_activo
                    if self.estado.tracking_activo else None
                )
                self.estado.correccion_actual = correccion

            frame = self._dibujar(frame, personas,
                                   self.estado.sujeto_activo, correccion)

            with self.estado.lock:
                self.estado.frame_actual = frame.copy()

            cv2.imshow("Drone Director", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.estado.corriendo = False
                break
            elif key == ord('r'):
                with self.estado.lock:
                    self.estado.sujeto_activo = None
                self.estado.log("Sujeto reseteado")

            await asyncio.sleep(interval)

        self.cap.release()
        cv2.destroyAllWindows()


# ============================================================
# CONTROLADOR DE VUELO
# ============================================================
class ControladorVuelo:
    def __init__(self, estado: EstadoSistema, config: dict, drone=None):
        self.estado = estado
        self.config = config
        self.drone = drone
        self.offboard_activo = False
        self.evitacion = None

    async def _offboard_on(self):
        if self.drone and not self.offboard_activo:
            from mavsdk.offboard import PositionNedYaw, OffboardError
            pos = self.estado.posicion_ned
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(pos["norte"], pos["este"],
                               -pos["altura"], 0.0))
            try:
                await self.drone.offboard.start()
                self.offboard_activo = True
            except OffboardError:
                pass

    async def ir_a(self, norte, este, altura, yaw=0.0, espera=None):
        pos = self.estado.posicion_ned

        # si hay sistema de evitacion, calcular ruta A*
        if self.evitacion:
            origen  = (pos["norte"], pos["este"])
            destino = (norte, este)
            dist = ((norte-pos["norte"])**2 + (este-pos["este"])**2)**0.5

            # solo usar A* si el movimiento es mayor a 1.5m
            if dist > 1.5:
                # actualizar obstaculos dinamicos con personas detectadas
                personas = [
                    {"pos": (p["centro"][0]*0.02, p["centro"][1]*0.02)}
                    for p in self.estado.personas_detectadas
                ]
                self.evitacion.actualizar_obstaculos_dinamicos(personas)

                ruta = self.evitacion.planificar_ruta(origen, destino)

                if len(ruta) > 1:
                    print(f"   [A*] Ruta: {len(ruta)} waypoints")
                    for wp in ruta[1:]:
                        await self._mover_a_waypoint(wp[0], wp[1], altura, yaw)
                    with self.estado.lock:
                        self.estado.posicion_ned = {"norte": norte, "este": este, "altura": altura}
                    return

        await self._mover_a_waypoint(norte, este, altura, yaw, espera)

    async def _mover_a_waypoint(self, norte, este, altura, yaw=0.0, espera=None):
        if self.drone:
            from mavsdk.offboard import PositionNedYaw
            await self._offboard_on()
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(norte, este, -altura, yaw))

        pos = self.estado.posicion_ned
        if espera is None:
            dist = math.sqrt((norte-pos["norte"])**2 + (este-pos["este"])**2)
            espera = max(0.8, dist / self.config["velocidad_ms"])

        await asyncio.sleep(espera)
        with self.estado.lock:
            self.estado.posicion_ned = {"norte": norte, "este": este, "altura": altura}

    async def aplicar_tracking(self):
        """Aplica correcciones de vision al vuelo — corre periodicamente."""
        while self.estado.corriendo:
            if self.estado.tracking_activo and self.estado.modo_actual == "tracking":
                with self.estado.lock:
                    corr = self.estado.correccion_actual
                    pos = self.estado.posicion_ned.copy()

                if corr["tipo"] == "correccion":
                    nuevo_norte = pos["norte"] + corr["norte"]
                    nuevo_este = pos["este"] + corr["este"]
                    await self.ir_a(nuevo_norte, nuevo_este,
                                    pos["altura"], 0.0, espera=0.1)

            await asyncio.sleep(0.15)


# ============================================================
# EJECUTOR DE COMANDOS
# ============================================================
class EjecutorComandos:
    def __init__(self, estado: EstadoSistema, config: dict,
                 controlador: ControladorVuelo):
        self.estado = estado
        self.config = config
        self.ctrl = controlador

    async def ejecutar(self, orden: dict) -> bool:
        cmd = orden["comando"]
        params = orden["params"]
        self.estado.ultima_orden = orden["texto"]

        pos = self.estado.posicion_ned

        if cmd == "panoramica":
            await self._panoramica()
        elif cmd == "cenital":
            await self._cenital()
        elif cmd in ["norte", "sur", "este", "oeste"]:
            await self._direccion(cmd)
        elif cmd == "centro":
            self.estado.set_modo("orden")
            await self.ctrl.ir_a(0.0, 0.0, pos["altura"])
            self.estado.log("En el centro.")
        elif cmd == "hover":
            self.estado.set_modo("hover")
            self.estado.log("Hover activo.")
        elif cmd == "bajar":
            h = float(params.get("numero", self.config["altura_filmacion"]))
            await self.ctrl.ir_a(pos["norte"], pos["este"], h, espera=2.0)
            self.estado.log(f"Altura {h}m.")
        elif cmd == "subir":
            h = float(params.get("numero", self.config["altura_cenital"]))
            await self.ctrl.ir_a(pos["norte"], pos["este"], h, espera=2.0)
            self.estado.log(f"Altura {h}m.")
        elif cmd == "seguir":
            color = params.get("color", "")
            self.estado.set_modo("tracking")
            with self.estado.lock:
                self.estado.tracking_activo = True
                self.estado.sujeto_activo = None
            self.estado.log(f"Tracking activo. Buscando persona {color}.")
        elif cmd == "dejar_seguir":
            with self.estado.lock:
                self.estado.tracking_activo = False
                self.estado.sujeto_activo = None
            self.estado.set_modo("hover")
            self.estado.log("Tracking desactivado.")
        elif cmd == "escanear":
            escaner = Escaner(self.estado, self.config)
            await escaner.ejecutar(self.ctrl.drone)
        elif cmd == "estado":
            pos = self.estado.posicion_ned
            self.estado.log(
                f"Pos: N={pos['norte']:.1f}m E={pos['este']:.1f}m "
                f"Alt={pos['altura']:.1f}m | Modo: {self.estado.modo_actual} | "
                f"Tracking: {self.estado.tracking_activo} | "
                f"Personas: {len(self.estado.personas_detectadas)}"
            )
        elif cmd == "aterrizar":
            self.estado.log("Aterrizando...")
            self.estado.corriendo = False
            return False
        else:
            self.estado.log(f"Orden no reconocida: '{orden['texto']}'")
            self.estado.log("Intentá: panoramica | cenital | norte | sur | este | oeste |")
            self.estado.log("         centro | hover | sube N | baja N | seguir |")
            self.estado.log("         dejar seguir | estado | aterrizar")

        return True

    async def _panoramica(self):
        self.estado.set_modo("orden")
        radio = 8.0
        puntos = 12
        altura = self.estado.posicion_ned["altura"]
        self.estado.log("Panoramica 360°...")
        for i in range(puntos + 1):
            angulo = (2 * math.pi * i) / puntos
            norte = radio * math.cos(angulo)
            este = radio * math.sin(angulo)
            yaw = math.degrees(math.atan2(-este, -norte)) % 360
            await self.ctrl.ir_a(norte, este, altura, yaw, espera=1.5)
        self.estado.log("Panoramica completada.")

    async def _cenital(self):
        self.estado.set_modo("orden")
        pos = self.estado.posicion_ned
        await self.ctrl.ir_a(pos["norte"], pos["este"],
                              self.config["altura_cenital"], 0.0, espera=3.0)
        self.estado.log("Toma cenital activa.")

    async def _direccion(self, direccion: str):
        self.estado.set_modo("orden")
        radio = (self.estado.mapa["area_radio_m"] * 0.7
                 if self.estado.mapa else 10.0)
        altura = self.estado.posicion_ned["altura"]
        destinos = {
            "norte": (radio, 0.0, 180.0),
            "sur":   (-radio, 0.0, 0.0),
            "este":  (0.0, radio, 270.0),
            "oeste": (0.0, -radio, 90.0),
        }
        norte, este, yaw = destinos[direccion]
        await self.ctrl.ir_a(norte, este, altura, yaw)
        self.estado.log(f"Posicion {direccion} alcanzada.")


# ============================================================
# LOOP DE ORDENES (input en consola)
# ============================================================
async def loop_ordenes(ejecutor: EjecutorComandos, estado: EstadoSistema):
    parser = ParserLLM()
    print('  [LLM] Parser inteligente activo')
    print("\n" + "─"*50)
    print("  DRONE DIRECTOR — Listo para recibir ordenes")
    print("  Ordenes: panoramica | cenital | norte/sur/este/oeste")
    print("           centro | hover | sube N | baja N | seguir")
    print("           dejar seguir | estado | aterrizar")
    print("─"*50 + "\n")

    loop = asyncio.get_event_loop()

    while estado.corriendo:
        try:
            texto = await loop.run_in_executor(None, lambda: input(">> "))
            if not texto.strip():
                continue
            orden = await parser.parsear(texto.strip())
            continuar = await ejecutor.ejecutar(orden)
            if not continuar:
                break
        except (EOFError, KeyboardInterrupt):
            estado.corriendo = False
            break


# ============================================================
# MAIN
# ============================================================
async def main(sin_drone: bool, fuente_video):
    print("=" * 55)
    print("   DRONE DIRECTOR — SISTEMA INTEGRADO v1.0")
    print("=" * 55)

    estado = EstadoSistema()

    # conectar drone
    drone = None
    if not sin_drone:
        try:
            from mavsdk import System
            drone = System()
            await drone.connect(system_address="udp://:14540")
            estado.log("Conectando con PX4 SITL...")
            async for s in drone.core.connection_state():
                if s.is_connected:
                    estado.log("Drone conectado OK")
                    break
            async for h in drone.telemetry.health():
                if h.is_global_position_ok:
                    estado.log("GPS OK")
                    break
            await drone.action.arm()
            await drone.action.set_takeoff_altitude(CONFIG["altura_filmacion"])
            await drone.action.takeoff()
            await asyncio.sleep(4)
            estado.posicion_ned["altura"] = CONFIG["altura_filmacion"]
            estado.log("Drone en posicion.")
        except Exception as e:
            estado.log(f"Sin drone ({e})")
            drone = None
    else:
        estado.log("Modo sin drone activo.")

    # buscar mapa existente o escanear
    mapas = sorted(glob.glob("mapa_evento_*.json"))
    if mapas:
        with open(mapas[-1]) as f:
            estado.mapa = json.load(f)
        estado.log(f"Mapa cargado: {mapas[-1]}")
    else:
        escaner = Escaner(estado, CONFIG)
        await escaner.ejecutar(drone)

    # inicializar modulos
    evitacion = SistemaEvitacion({
        "area_metros": int(estado.mapa["area_radio_m"] * 2),
        "resolucion_grid": 0.5,
        "margen_seguridad": 1.0,
        "obstaculos_estaticos": [],
        "num_personas_dinamicas": 0,
        "velocidad_persona": 0.8,
    })
    controlador = ControladorVuelo(estado, CONFIG, drone)
    controlador.evitacion = evitacion
    ejecutor    = EjecutorComandos(estado, CONFIG, controlador)
    vision      = SistemaVision(estado, CONFIG, fuente_video)

    # lanzar todo en paralelo
    await asyncio.gather(
        vision.loop(),
        loop_ordenes(ejecutor, estado),
        controlador.aplicar_tracking(),
    )

    # cleanup
    if drone:
        try:
            if controlador.offboard_activo:
                await drone.offboard.stop()
            await drone.action.return_to_launch()
        except Exception:
            pass

    estado.log("Sistema terminado.")


# ============================================================
# ENTRY POINT
# ============================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone Director — Sistema Integrado")
    parser.add_argument("--sin-drone", action="store_true",
                        help="Correr sin conectar PX4 SITL")
    parser.add_argument("--video", type=str, default="test_personas.mp4",
                        help="Fuente de video (archivo o indice de camara)")
    args = parser.parse_args()

    # fuente de video: numero = camara, string = archivo
    fuente = int(args.video) if args.video.isdigit() else args.video

    asyncio.run(main(sin_drone=args.sin_drone, fuente_video=fuente))

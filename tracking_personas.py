"""
tracking_personas.py
====================
Modulo 3 del sistema de filmacion autonoma.

Usa la webcam de la PC para detectar personas en tiempo real
con YOLOv8 y genera comandos de movimiento para que el drone
mantenga el sujeto centrado en el encuadre.

Logica:
  - Detecta personas con YOLOv8 nano (rapido, liviano)
  - Calcula offset del sujeto respecto al centro del frame
  - Genera correccion NED proporcional al offset
  - Envia correcciones al drone via MAVLink offboard

Uso:
    python3 tracking_personas.py
    python3 tracking_personas.py --sin-drone   (solo camara, sin PX4)
"""

import asyncio
import argparse
import cv2
import math
import time
import threading
import numpy as np
from ultralytics import YOLO


# ------------------------------------------------------------
# CONFIGURACION
# ------------------------------------------------------------
CONFIG = {
    "modelo_yolo":        "yolov8n.pt",    # nano = mas rapido
    "confianza_minima":   0.5,             # 0-1, filtrar detecciones debiles
    "camara_id":          0,               # 0 = webcam principal
    "resolucion":         (640, 480),
    "fps_objetivo":       15,

    # control del drone
    "altura_tracking":    5.0,             # metros
    "ganancia_lateral":   0.015,           # cuanto mover por pixel de offset
    "ganancia_profundidad": 0.008,         # ajuste de distancia segun tamaño
    "zona_muerta_px":     40,              # pixeles — no corregir si offset < esto
    "max_correccion_m":   2.0,             # metros maximos de correccion por frame

    # visualizacion
    "mostrar_ventana":    True,
    "color_deteccion":    (0, 255, 0),     # verde
    "color_tracking":     (0, 100, 255),   # naranja — sujeto seleccionado
}


# ------------------------------------------------------------
# DETECTOR DE PERSONAS
# ------------------------------------------------------------
class DetectorPersonas:
    def __init__(self, config: dict):
        print("[Vision] Cargando modelo YOLOv8...")
        self.modelo = YOLO(config["modelo_yolo"])
        self.config = config
        self.ultimo_frame_procesado = None
        print("[Vision] Modelo listo.")

    def detectar(self, frame: np.ndarray) -> list:
        """
        Detecta personas en el frame.
        Retorna lista de dicts con bbox, centro y confianza.
        """
        resultados = self.modelo(
            frame,
            conf=self.config["confianza_minima"],
            classes=[0],        # clase 0 = persona en COCO
            verbose=False
        )

        personas = []
        for r in resultados:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                ancho = x2 - x1
                alto = y2 - y1

                personas.append({
                    "bbox": (x1, y1, x2, y2),
                    "centro": (cx, cy),
                    "confianza": conf,
                    "ancho": ancho,
                    "alto": alto,
                    "area": ancho * alto,
                })

        # ordenar por area descendente (persona mas cercana primero)
        personas.sort(key=lambda p: p["area"], reverse=True)
        return personas


# ------------------------------------------------------------
# SELECTOR DE SUJETO
# ------------------------------------------------------------
class SelectorSujeto:
    """
    Mantiene el tracking de un sujeto seleccionado
    entre frames sucesivos.
    """
    def __init__(self):
        self.sujeto_activo = None
        self.frames_sin_deteccion = 0
        self.max_frames_perdida = 10

    def actualizar(self, personas: list):
        if not personas:
            self.frames_sin_deteccion += 1
            if self.frames_sin_deteccion > self.max_frames_perdida:
                self.sujeto_activo = None
            return self.sujeto_activo

        self.frames_sin_deteccion = 0

        if self.sujeto_activo is None:
            # tomar la persona mas grande (mas cercana)
            self.sujeto_activo = personas[0]
        else:
            # encontrar la deteccion mas cercana al sujeto previo
            cx_prev, cy_prev = self.sujeto_activo["centro"]
            mejor = min(personas, key=lambda p: math.sqrt(
                (p["centro"][0] - cx_prev)**2 +
                (p["centro"][1] - cy_prev)**2
            ))
            self.sujeto_activo = mejor

        return self.sujeto_activo

    def resetear(self):
        self.sujeto_activo = None
        self.frames_sin_deteccion = 0


# ------------------------------------------------------------
# CALCULADOR DE CORRECCION DE VUELO
# ------------------------------------------------------------
class CalculadorCorreccion:
    """
    Calcula cuanto debe moverse el drone para centrar
    al sujeto en el encuadre.
    """
    def __init__(self, resolucion: tuple, config: dict):
        self.ancho, self.alto = resolucion
        self.cx_frame = self.ancho // 2
        self.cy_frame = self.alto // 2
        self.config = config

    def calcular(self, sujeto: dict) -> dict:
        """
        Retorna correcciones en metros para el sistema NED.
        """
        if sujeto is None:
            return {"norte": 0.0, "este": 0.0, "tipo": "sin_sujeto"}

        cx, cy = sujeto["centro"]

        # offset en pixeles desde el centro del frame
        offset_x = cx - self.cx_frame   # positivo = sujeto a la derecha
        offset_y = cy - self.cy_frame   # positivo = sujeto abajo

        # zona muerta — no corregir si el offset es pequeño
        zona = self.config["zona_muerta_px"]
        if abs(offset_x) < zona and abs(offset_y) < zona:
            return {"norte": 0.0, "este": 0.0, "tipo": "en_zona_muerta",
                    "offset_x": offset_x, "offset_y": offset_y}

        # convertir pixels a metros
        # offset_x → movimiento ESTE (lateral)
        # offset_y → movimiento SUR/NORTE (adelante/atras) — invertido
        corr_este = offset_x * self.config["ganancia_lateral"]
        corr_norte = -offset_y * self.config["ganancia_lateral"]

        # limitar correccion maxima
        max_c = self.config["max_correccion_m"]
        corr_este = max(-max_c, min(max_c, corr_este))
        corr_norte = max(-max_c, min(max_c, corr_norte))

        return {
            "norte": round(corr_norte, 3),
            "este": round(corr_este, 3),
            "tipo": "correccion",
            "offset_x": offset_x,
            "offset_y": offset_y,
        }


# ------------------------------------------------------------
# CONTROLADOR DEL DRONE (opcional — se puede correr sin drone)
# ------------------------------------------------------------
class ControladorDrone:
    def __init__(self, config: dict):
        self.config = config
        self.posicion = {"norte": 0.0, "este": 0.0}
        self.conectado = False
        self.drone = None

    async def conectar(self):
        try:
            from mavsdk import System
            from mavsdk.offboard import PositionNedYaw
            self.PositionNedYaw = PositionNedYaw

            self.drone = System()
            await self.drone.connect(system_address="udp://:14540")

            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.conectado = True
                    print("[Drone] Conectado OK")
                    break

            # iniciar offboard
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -self.config["altura_tracking"], 0.0)
            )
            await self.drone.offboard.start()
            print("[Drone] Modo offboard activo")

        except Exception as e:
            print(f"[Drone] Sin conexion ({e}) — modo solo vision")
            self.conectado = False

    async def aplicar_correccion(self, correccion: dict):
        if not self.conectado or correccion["tipo"] != "correccion":
            return

        self.posicion["norte"] += correccion["norte"]
        self.posicion["este"] += correccion["este"]

        try:
            await self.drone.offboard.set_position_ned(
                self.PositionNedYaw(
                    self.posicion["norte"],
                    self.posicion["este"],
                    -self.config["altura_tracking"],
                    0.0
                )
            )
        except Exception as e:
            print(f"[Drone] Error al aplicar correccion: {e}")

    async def aterrizar(self):
        if self.conectado:
            await self.drone.offboard.stop()
            await self.drone.action.return_to_launch()


# ------------------------------------------------------------
# VISUALIZADOR
# ------------------------------------------------------------
class Visualizador:
    def __init__(self, config: dict):
        self.config = config

    def dibujar(self, frame: np.ndarray, personas: list,
                sujeto: dict, correccion: dict) -> np.ndarray:
        h, w = frame.shape[:2]
        cx = w // 2
        cy = h // 2

        # cruz central
        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1)

        # zona muerta
        zona = self.config["zona_muerta_px"]
        cv2.rectangle(frame,
                      (cx - zona, cy - zona),
                      (cx + zona, cy + zona),
                      (80, 80, 80), 1)

        # todas las detecciones
        for p in personas:
            x1, y1, x2, y2 = p["bbox"]
            color = self.config["color_deteccion"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
            cv2.putText(frame, f"{p['confianza']:.2f}",
                        (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, color, 1)

        # sujeto seleccionado
        if sujeto:
            x1, y1, x2, y2 = sujeto["bbox"]
            color = self.config["color_tracking"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, "TRACKING",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2)

            # linea desde centro del frame al sujeto
            scx, scy = sujeto["centro"]
            cv2.line(frame, (cx, cy), (scx, scy), color, 1)

        # info de correccion
        tipo = correccion.get("tipo", "")
        if tipo == "correccion":
            texto = (f"N:{correccion['norte']:+.2f}m  "
                     f"E:{correccion['este']:+.2f}m")
            cv2.putText(frame, texto, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        self.config["color_tracking"], 2)
        elif tipo == "en_zona_muerta":
            cv2.putText(frame, "CENTRADO", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        self.config["color_deteccion"], 2)
        elif tipo == "sin_sujeto":
            cv2.putText(frame, "Buscando persona...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 200, 255), 2)

        # instrucciones
        cv2.putText(frame, "R: resetear sujeto | Q: salir",
                    (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (180, 180, 180), 1)

        return frame


# ------------------------------------------------------------
# SISTEMA PRINCIPAL
# ------------------------------------------------------------
async def main(sin_drone: bool = False):
    print("=" * 50)
    print("  TRACKING DE PERSONAS — DRONE FILMADOR")
    print("=" * 50)

    # inicializar componentes
    detector = DetectorPersonas(CONFIG)
    selector = SelectorSujeto()
    calculador = CalculadorCorreccion(CONFIG["resolucion"], CONFIG)
    visualizador = Visualizador(CONFIG)
    controlador = ControladorDrone(CONFIG)

    # conectar camara
    print(f"\n[Camara] Abriendo camara {CONFIG['camara_id']}...")
    cap = cv2.VideoCapture("test_personas.mp4")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CONFIG["resolucion"][0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CONFIG["resolucion"][1])

    if not cap.isOpened():
        print("[ERROR] No se pudo abrir la camara.")
        return

    print("[Camara] OK")

    # conectar drone
    if not sin_drone:
        await controlador.conectar()
    else:
        print("[Drone] Modo solo-vision (--sin-drone)")

    print("\n[OK] Sistema activo. Apuntá la camara a una persona.")
    print("     R = resetear sujeto | Q = salir\n")

    correccion = {"tipo": "sin_sujeto", "norte": 0.0, "este": 0.0}
    fps_timer = time.time()
    frames = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] No se pudo leer frame de la camara.")
            break

        # detectar personas
        personas = detector.detectar(frame)

        # seleccionar sujeto a seguir
        sujeto = selector.actualizar(personas)

        # calcular correccion de vuelo
        correccion = calculador.calcular(sujeto)

        # aplicar al drone
        if not sin_drone and correccion["tipo"] == "correccion":
            await controlador.aplicar_correccion(correccion)

        # visualizar
        frame = visualizador.dibujar(frame, personas, sujeto, correccion)

        # fps counter
        frames += 1
        if time.time() - fps_timer >= 1.0:
            fps = frames
            frames = 0
            fps_timer = time.time()
            cv2.putText(frame, f"FPS: {fps}",
                        (frame.shape[1] - 80, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (180, 180, 180), 1)

        if CONFIG["mostrar_ventana"]:
            cv2.imshow("Drone Tracker", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n[!] Saliendo...")
                break
            elif key == ord('r'):
                selector.resetear()
                print("[Vision] Sujeto reseteado — buscando nueva persona")

        await asyncio.sleep(1.0 / CONFIG["fps_objetivo"])

    # cleanup
    cap.release()
    cv2.destroyAllWindows()

    if not sin_drone:
        await controlador.aterrizar()

    print("[OK] Tracking terminado.")


# ------------------------------------------------------------
# ENTRY POINT
# ------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sin-drone", action="store_true",
                        help="Correr solo vision sin conectar PX4")
    args = parser.parse_args()

    asyncio.run(main(sin_drone=args.sin_drone))

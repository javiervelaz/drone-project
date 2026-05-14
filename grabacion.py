"""
grabacion.py
============
Modulo de grabacion del sistema Drone Director.

Graba en paralelo:
  1. VIDEO  — frames procesados con HUD superpuesto
  2. EVENTOS — log JSON de ordenes, modos y tracking
  3. TELEMETRIA — posicion del drone cada segundo

Genera por sesion:
  grabacion_YYYYMMDD_HHMMSS.mp4
  grabacion_YYYYMMDD_HHMMSS.json

Uso standalone (prueba sin drone_director):
    python3 grabacion.py --video test_personas.mp4

Integracion en drone_director.py:
    from grabacion import ModuloGrabacion
    grabador = ModuloGrabacion(estado)
    await grabador.iniciar()
    # ... sistema corre ...
    grabador.finalizar()
"""

import asyncio
import argparse
import cv2
import json
import os
import time
import threading
import numpy as np
from datetime import datetime
from pathlib import Path


# ------------------------------------------------------------
# CONFIGURACION
# ------------------------------------------------------------
CONFIG_GRABACION = {
    "fps":          15,
    "resolucion":   (640, 480),
    "codec":        "mp4v",
    "directorio":   "grabaciones",
    "telemetria_intervalo": 1.0,   # segundos entre registros de telemetria
}


# ------------------------------------------------------------
# MODULO DE GRABACION
# ------------------------------------------------------------
class ModuloGrabacion:
    def __init__(self, config: dict = None):
        self.config = config or CONFIG_GRABACION
        self.activo = False
        self.writer = None
        self.sesion_id = ""
        self.ruta_video = ""
        self.ruta_log = ""
        self.log_data = {
            "sesion_id": "",
            "inicio": "",
            "fin": "",
            "duracion_seg": 0,
            "total_frames": 0,
            "eventos": [],
            "telemetria": [],
        }
        self.frames_grabados = 0
        self.tiempo_inicio = 0.0
        self._lock = threading.Lock()

        # estado del sistema (se inyecta desde drone_director)
        self.estado = None

    # --------------------------------------------------------
    # INICIAR SESION
    # --------------------------------------------------------
    def iniciar(self, estado=None):
        """
        Inicia una nueva sesion de grabacion.
        estado: objeto EstadoSistema de drone_director (opcional)
        """
        self.estado = estado
        self.sesion_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.tiempo_inicio = time.time()

        # crear directorio
        Path(self.config["directorio"]).mkdir(exist_ok=True)

        # rutas de archivos
        self.ruta_video = os.path.join(
            self.config["directorio"],
            f"grabacion_{self.sesion_id}.mp4"
        )
        self.ruta_log = os.path.join(
            self.config["directorio"],
            f"grabacion_{self.sesion_id}.json"
        )

        # inicializar VideoWriter
        fourcc = cv2.VideoWriter_fourcc(*self.config["codec"])
        self.writer = cv2.VideoWriter(
            self.ruta_video,
            fourcc,
            self.config["fps"],
            self.config["resolucion"]
        )

        if not self.writer.isOpened():
            raise RuntimeError(f"No se pudo crear el archivo de video: {self.ruta_video}")

        # inicializar log
        self.log_data["sesion_id"] = self.sesion_id
        self.log_data["inicio"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_data["eventos"] = []
        self.log_data["telemetria"] = []

        self.activo = True
        self.frames_grabados = 0

        self._registrar_evento("sistema", "grabacion_iniciada", {
            "video": self.ruta_video,
            "log": self.ruta_log,
        })

        print(f"[Grabacion] Sesion iniciada: {self.sesion_id}")
        print(f"[Grabacion] Video: {self.ruta_video}")
        print(f"[Grabacion] Log:   {self.ruta_log}")

    # --------------------------------------------------------
    # GRABAR FRAME
    # --------------------------------------------------------
    def grabar_frame(self, frame: np.ndarray,
                     info: dict = None) -> np.ndarray:
        """
        Graba un frame con HUD de grabacion superpuesto.
        Retorna el frame con el indicador REC visible.
        """
        if not self.activo or self.writer is None:
            return frame

        # asegurar resolucion correcta
        h_target, w_target = self.config["resolucion"][1], self.config["resolucion"][0]
        if frame.shape[1] != w_target or frame.shape[0] != h_target:
            frame = cv2.resize(frame, self.config["resolucion"])

        frame_con_hud = self._dibujar_hud_grabacion(frame.copy(), info)

        with self._lock:
            self.writer.write(frame_con_hud)
            self.frames_grabados += 1

        return frame_con_hud

    # --------------------------------------------------------
    # HUD DE GRABACION
    # --------------------------------------------------------
    def _dibujar_hud_grabacion(self, frame: np.ndarray,
                                info: dict = None) -> np.ndarray:
        h, w = frame.shape[:2]
        ahora = datetime.now()
        elapsed = time.time() - self.tiempo_inicio

        # --- indicador REC parpadeante ---
        if int(elapsed * 2) % 2 == 0:
            cv2.circle(frame, (w - 25, 18), 7, (0, 0, 255), -1)
            cv2.putText(frame, "REC", (w - 55, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # --- timestamp ---
        ts = ahora.strftime("%H:%M:%S.") + f"{ahora.microsecond // 10000:02d}"
        cv2.putText(frame, ts, (w - 130, h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # --- duracion de sesion ---
        mins = int(elapsed) // 60
        segs = int(elapsed) % 60
        dur = f"{mins:02d}:{segs:02d}"
        cv2.putText(frame, dur, (w - 55, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # --- frame counter ---
        cv2.putText(frame, f"F:{self.frames_grabados:05d}",
                    (w - 130, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        # --- info del sistema si está disponible ---
        if info:
            modo = info.get("modo", "")
            orden = info.get("ultima_orden", "")[:35]
            personas = info.get("personas", 0)
            tracking = info.get("tracking", False)
            pos = info.get("posicion", {})

            # barra de estado superior
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, 22), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

            cv2.putText(frame,
                        f"MODO:{modo.upper()}  PERSONAS:{personas}"
                        f"  TRACKING:{'ON' if tracking else 'OFF'}",
                        (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (200, 255, 200), 1)

            # posicion del drone
            if pos:
                pos_txt = (f"N:{pos.get('norte', 0):.1f}m "
                           f"E:{pos.get('este', 0):.1f}m "
                           f"Alt:{pos.get('altura', 0):.1f}m")
                cv2.putText(frame, pos_txt, (5, h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (180, 220, 255), 1)

        return frame

    # --------------------------------------------------------
    # REGISTRO DE EVENTOS
    # --------------------------------------------------------
    def _registrar_evento(self, tipo: str, evento: str,
                          detalle: dict = None):
        entrada = {
            "ts":     datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "tipo":   tipo,
            "evento": evento,
        }
        if detalle:
            entrada["detalle"] = detalle

        with self._lock:
            self.log_data["eventos"].append(entrada)

    def registrar_orden(self, texto: str, comando: str, params: dict = None):
        self._registrar_evento("orden", texto, {
            "comando": comando,
            "params": params or {}
        })

    def registrar_modo(self, modo: str):
        self._registrar_evento("modo", f"cambio_a_{modo}")

    def registrar_tracking(self, activo: bool, detalle: str = ""):
        self._registrar_evento(
            "tracking",
            "activado" if activo else "desactivado",
            {"detalle": detalle} if detalle else None
        )

    def registrar_telemetria(self, norte: float, este: float,
                              altura: float, personas: int = 0,
                              tracking: bool = False):
        entrada = {
            "ts":       datetime.now().strftime("%H:%M:%S"),
            "norte":    round(norte, 2),
            "este":     round(este, 2),
            "altura":   round(altura, 2),
            "personas": personas,
            "tracking": tracking,
        }
        with self._lock:
            self.log_data["telemetria"].append(entrada)

    # --------------------------------------------------------
    # LOOP DE TELEMETRIA (corre en paralelo)
    # --------------------------------------------------------
    async def loop_telemetria(self):
        """Registra telemetria periodicamente desde el estado del sistema."""
        while self.activo:
            if self.estado:
                with self.estado.lock:
                    pos = self.estado.posicion_ned.copy()
                    personas = len(self.estado.personas_detectadas)
                    tracking = self.estado.tracking_activo

                self.registrar_telemetria(
                    pos["norte"], pos["este"], pos["altura"],
                    personas, tracking
                )

            await asyncio.sleep(self.config["telemetria_intervalo"])

    # --------------------------------------------------------
    # FINALIZAR SESION
    # --------------------------------------------------------
    def finalizar(self):
        if not self.activo:
            return

        self.activo = False
        elapsed = time.time() - self.tiempo_inicio

        # cerrar video
        with self._lock:
            if self.writer:
                self.writer.release()

        # completar log
        self.log_data["fin"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_data["duracion_seg"] = round(elapsed, 1)
        self.log_data["total_frames"] = self.frames_grabados

        self._registrar_evento("sistema", "grabacion_finalizada", {
            "duracion_seg": round(elapsed, 1),
            "total_frames": self.frames_grabados,
            "fps_real": round(self.frames_grabados / elapsed, 1) if elapsed > 0 else 0,
        })

        # guardar JSON
        with open(self.ruta_log, "w", encoding="utf-8") as f:
            json.dump(self.log_data, f, indent=2, ensure_ascii=False)

        print(f"\n[Grabacion] Sesion finalizada.")
        print(f"[Grabacion] Duracion:  {int(elapsed//60):02d}:{int(elapsed%60):02d}")
        print(f"[Grabacion] Frames:    {self.frames_grabados}")
        print(f"[Grabacion] FPS real:  {self.frames_grabados/elapsed:.1f}")
        print(f"[Grabacion] Video:     {self.ruta_video}")
        print(f"[Grabacion] Log:       {self.ruta_log}")

        return {
            "video": self.ruta_video,
            "log": self.ruta_log,
            "duracion_seg": round(elapsed, 1),
            "total_frames": self.frames_grabados,
        }


# ============================================================
# TEST STANDALONE
# ============================================================
async def test_grabacion(fuente_video: str):
    """
    Prueba el modulo de grabacion procesando un video existente.
    Simula ordenes y telemetria para poblar el log.
    """
    from ultralytics import YOLO

    print("=" * 55)
    print("  TEST MODULO DE GRABACION")
    print("=" * 55)

    # inicializar
    grabador = ModuloGrabacion()
    grabador.iniciar()

    modelo = YOLO("yolov8n.pt")
    cap = cv2.VideoCapture(fuente_video)

    if not cap.isOpened():
        print(f"[ERROR] No se pudo abrir: {fuente_video}")
        return

    # simular estado basico
    class EstadoSimulado:
        def __init__(self):
            self.lock = threading.Lock()
            self.posicion_ned = {"norte": 0.0, "este": 0.0, "altura": 5.0}
            self.personas_detectadas = []
            self.tracking_activo = False
            self.modo_actual = "filmacion"
            self.ultima_orden = ""

    estado_sim = EstadoSimulado()
    grabador.estado = estado_sim

    # simular ordenes para el log
    ordenes_simuladas = [
        (2.0,  "panoramica",  "hacé una vuelta completa"),
        (5.0,  "seguir",      "seguí a la persona del centro"),
        (10.0, "cenital",     "subí y filmá desde arriba"),
        (15.0, "hover",       "quedate quieto ahí"),
    ]
    tiempo_inicio = time.time()

    print(f"\n[Test] Procesando video: {fuente_video}")
    print("[Test] Presioná Q para detener\n")

    frame_num = 0
    fps_timer = time.time()
    fps_count = 0
    fps_actual = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        elapsed = time.time() - tiempo_inicio
        frame_num += 1
        fps_count += 1

        # calcular fps
        if time.time() - fps_timer >= 1.0:
            fps_actual = fps_count
            fps_count = 0
            fps_timer = time.time()

        # simular ordenes en momentos especificos
        for t_orden, cmd, texto in ordenes_simuladas:
            if abs(elapsed - t_orden) < 0.1:
                grabador.registrar_orden(texto, cmd)
                estado_sim.ultima_orden = texto
                estado_sim.modo_actual = cmd
                print(f"[Sim] Orden: {texto}")

        # simular movimiento del drone
        estado_sim.posicion_ned = {
            "norte": round(5.0 * np.sin(elapsed * 0.3), 2),
            "este":  round(5.0 * np.cos(elapsed * 0.3), 2),
            "altura": 5.0 + np.sin(elapsed * 0.1) * 2,
        }

        # detectar personas
        resultados = modelo(frame, conf=0.45, classes=[0], verbose=False)
        personas = []
        for r in resultados:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cx, cy = (x1+x2)//2, (y1+y2)//2
                personas.append({"bbox": (x1,y1,x2,y2), "centro": (cx,cy),
                                  "confianza": conf})
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 1)
                if len(personas) == 1:
                    cv2.rectangle(frame, (x1,y1), (x2,y2), (0,100,255), 2)
                    cv2.putText(frame, "TRACKING", (x1, y1-8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,100,255), 2)

        estado_sim.personas_detectadas = personas
        estado_sim.tracking_activo = len(personas) > 0

        # registrar telemetria cada segundo
        if frame_num % grabador.config["fps"] == 0:
            grabador.registrar_telemetria(
                estado_sim.posicion_ned["norte"],
                estado_sim.posicion_ned["este"],
                estado_sim.posicion_ned["altura"],
                len(personas),
                estado_sim.tracking_activo,
            )

        # info para el HUD
        info = {
            "modo":         estado_sim.modo_actual,
            "ultima_orden": estado_sim.ultima_orden,
            "personas":     len(personas),
            "tracking":     estado_sim.tracking_activo,
            "posicion":     estado_sim.posicion_ned,
        }

        # GRABAR el frame
        frame_grabado = grabador.grabar_frame(frame, info)

        # mostrar en pantalla
        cv2.putText(frame_grabado, f"FPS:{fps_actual}",
                    (5, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (180,180,180), 1)
        cv2.imshow("Drone Director — Grabacion", frame_grabado)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        await asyncio.sleep(1.0 / grabador.config["fps"])

    cap.release()
    cv2.destroyAllWindows()

    # finalizar y mostrar resumen
    resultado = grabador.finalizar()

    # mostrar resumen del log
    print("\n--- RESUMEN DEL LOG ---")
    print(f"Eventos registrados: {len(grabador.log_data['eventos'])}")
    print(f"Registros telemetria: {len(grabador.log_data['telemetria'])}")
    print("\nPrimeros 5 eventos:")
    for ev in grabador.log_data["eventos"][:5]:
        print(f"  [{ev['ts']}] {ev['tipo']} → {ev['evento']}")

    print(f"\n✅ Archivos listos en: ./{grabador.config['directorio']}/")


# ============================================================
# ENTRY POINT
# ============================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", default="test_personas.mp4",
                        help="Video de prueba")
    args = parser.parse_args()

    asyncio.run(test_grabacion(args.video))

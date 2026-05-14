"""
evitacion_obstaculos.py
=======================
Modulo de evitacion de obstaculos para el Drone Director.

Implementa:
  - Mapa de ocupacion 2D (grid) del area del evento
  - Obstaculos estaticos (paredes, arboles, mesas)
  - Obstaculos dinamicos (personas que se mueven)
  - Algoritmo A* para replanificacion de ruta
  - Visualizacion del mapa y rutas en tiempo real

Uso standalone:
    python3 evitacion_obstaculos.py

Integracion en drone_director.py:
    from evitacion_obstaculos import SistemaEvitacion
"""

import asyncio
import argparse
import cv2
import math
import heapq
import random
import time
import numpy as np
from datetime import datetime


# ------------------------------------------------------------
# CONFIGURACION
# ------------------------------------------------------------
CONFIG = {
    # mapa
    "area_metros":      30,        # area cuadrada de 30x30 metros
    "resolucion_grid":  0.5,       # 1 celda = 0.5 metros
    "margen_seguridad": 1.0,       # metros de margen alrededor de obstaculos

    # visualizacion
    "escala_visual":    20,        # pixeles por celda en la ventana
    "fps":              10,

    # obstaculos estaticos (x_m, y_m, radio_m, nombre)
    "obstaculos_estaticos": [
        (5.0,   5.0,  1.5, "Arbol"),
        (-7.0,  3.0,  1.0, "Arbol"),
        (3.0,  -8.0,  2.0, "Toldo"),
        (-4.0, -5.0,  1.5, "Mesa"),
        (8.0,  -3.0,  1.0, "Columna"),
        (-9.0,  8.0,  1.5, "Arbol"),
        (0.0,   9.0,  2.5, "Escenario"),
    ],

    # obstaculos dinamicos — personas moviendose
    "num_personas_dinamicas": 4,
    "velocidad_persona":      0.8,   # m/s
}


# ------------------------------------------------------------
# MAPA DE OCUPACION
# ------------------------------------------------------------
class MapaOcupacion:
    """
    Grid 2D donde cada celda puede estar:
      0 = libre
      1 = obstaculo estatico
      2 = obstaculo dinamico (persona)
      3 = zona de margen de seguridad
    """
    def __init__(self, config: dict):
        self.config = config
        self.area = config["area_metros"]
        self.res = config["resolucion_grid"]
        self.margen = config["margen_seguridad"]

        # tamaño del grid
        self.celdas = int(self.area / self.res)
        self.offset = self.celdas // 2   # (0,0) en el centro

        # grids separados
        self.grid_estatico  = np.zeros((self.celdas, self.celdas), dtype=np.uint8)
        self.grid_dinamico  = np.zeros((self.celdas, self.celdas), dtype=np.uint8)

        # cargar obstaculos estaticos
        self._cargar_estaticos()

    def _metros_a_celda(self, x_m: float, y_m: float) -> tuple:
        """Convierte metros (NED) a indices del grid."""
        col = int(x_m / self.res) + self.offset
        row = int(y_m / self.res) + self.offset
        col = max(0, min(self.celdas - 1, col))
        row = max(0, min(self.celdas - 1, row))
        return row, col

    def _celda_a_metros(self, row: int, col: int) -> tuple:
        """Convierte indices del grid a metros."""
        x_m = (col - self.offset) * self.res
        y_m = (row - self.offset) * self.res
        return x_m, y_m

    def _marcar_circulo(self, grid: np.ndarray, cx: float, cy: float,
                         radio: float, valor: int):
        """Marca un circulo en el grid."""
        radio_celdas = int((radio + self.margen) / self.res)
        r0, c0 = self._metros_a_celda(cx, cy)
        for dr in range(-radio_celdas, radio_celdas + 1):
            for dc in range(-radio_celdas, radio_celdas + 1):
                if dr**2 + dc**2 <= radio_celdas**2:
                    r = max(0, min(self.celdas-1, r0 + dr))
                    c = max(0, min(self.celdas-1, c0 + dc))
                    grid[r, c] = valor

    def _cargar_estaticos(self):
        for x, y, radio, nombre in self.config["obstaculos_estaticos"]:
            self._marcar_circulo(self.grid_estatico, x, y, radio, 1)

    def actualizar_dinamicos(self, personas: list):
        """Actualiza el grid con posiciones actuales de personas."""
        self.grid_dinamico.fill(0)
        for persona in personas:
            x, y = persona["pos"]
            self._marcar_circulo(self.grid_dinamico, x, y, 0.6, 2)

    def es_libre(self, row: int, col: int) -> bool:
        """Retorna True si la celda está libre de obstaculos."""
        if row < 0 or row >= self.celdas or col < 0 or col >= self.celdas:
            return False
        return (self.grid_estatico[row, col] == 0 and
                self.grid_dinamico[row, col] == 0)

    def grid_combinado(self) -> np.ndarray:
        return np.maximum(self.grid_estatico, self.grid_dinamico)


# ------------------------------------------------------------
# ALGORITMO A*
# ------------------------------------------------------------
class PlanificadorAstar:
    def __init__(self, mapa: MapaOcupacion):
        self.mapa = mapa

    def _heuristica(self, a: tuple, b: tuple) -> float:
        """Distancia euclidiana entre dos celdas."""
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def _vecinos(self, row: int, col: int) -> list:
        """Retorna celdas vecinas libres (8 direcciones)."""
        vecinos = []
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                r, c = row + dr, col + dc
                if self.mapa.es_libre(r, c):
                    # movimiento diagonal cuesta mas
                    costo = 1.414 if (dr != 0 and dc != 0) else 1.0
                    vecinos.append((r, c, costo))
        return vecinos

    def planificar(self, origen_m: tuple, destino_m: tuple) -> list:
        """
        Calcula la ruta optima de origen a destino evitando obstaculos.
        Retorna lista de puntos en metros [(x1,y1), (x2,y2), ...]
        o lista vacia si no hay ruta.
        """
        inicio = self.mapa._metros_a_celda(*origen_m)
        fin    = self.mapa._metros_a_celda(*destino_m)

        # si el destino esta bloqueado, buscar celda libre cercana
        if not self.mapa.es_libre(*fin):
            fin = self._celda_libre_cercana(*fin)
            if fin is None:
                return []

        # A* clasico
        open_set = []
        heapq.heappush(open_set, (0, inicio))
        came_from = {}
        g_score = {inicio: 0}
        f_score = {inicio: self._heuristica(inicio, fin)}

        while open_set:
            _, actual = heapq.heappop(open_set)

            if actual == fin:
                return self._reconstruir_ruta(came_from, actual)

            for r, c, costo in self._vecinos(*actual):
                vecino = (r, c)
                g_nuevo = g_score[actual] + costo

                if vecino not in g_score or g_nuevo < g_score[vecino]:
                    came_from[vecino] = actual
                    g_score[vecino] = g_nuevo
                    f = g_nuevo + self._heuristica(vecino, fin)
                    f_score[vecino] = f
                    heapq.heappush(open_set, (f, vecino))

        return []  # sin ruta

    def _celda_libre_cercana(self, row: int, col: int, max_radio: int = 5):
        """Busca la celda libre mas cercana a una posicion bloqueada."""
        for r in range(1, max_radio + 1):
            for dr in range(-r, r+1):
                for dc in range(-r, r+1):
                    nr, nc = row+dr, col+dc
                    if self.mapa.es_libre(nr, nc):
                        return (nr, nc)
        return None

    def _reconstruir_ruta(self, came_from: dict, actual: tuple) -> list:
        """Reconstruye la ruta desde el diccionario came_from."""
        ruta_celdas = [actual]
        while actual in came_from:
            actual = came_from[actual]
            ruta_celdas.append(actual)
        ruta_celdas.reverse()

        # convertir a metros y simplificar
        ruta_metros = [self.mapa._celda_a_metros(r, c)
                       for r, c in ruta_celdas]
        return self._simplificar_ruta(ruta_metros)

    def _simplificar_ruta(self, ruta: list, tolerancia: float = 1.0) -> list:
        """
        Elimina puntos intermedios colineales para rutas mas suaves.
        """
        if len(ruta) <= 2:
            return ruta

        simplificada = [ruta[0]]
        for i in range(1, len(ruta) - 1):
            # calcular angulo entre segmentos consecutivos
            p1, p2, p3 = ruta[i-1], ruta[i], ruta[i+1]
            v1 = (p2[0]-p1[0], p2[1]-p1[1])
            v2 = (p3[0]-p2[0], p3[1]-p2[1])
            cross = abs(v1[0]*v2[1] - v1[1]*v2[0])
            if cross > tolerancia * 0.1:
                simplificada.append(p2)

        simplificada.append(ruta[-1])
        return simplificada


# ------------------------------------------------------------
# PERSONAS DINAMICAS (simuladas)
# ------------------------------------------------------------
class PersonaDinamica:
    def __init__(self, area: float, velocidad: float):
        self.area = area * 0.7
        self.velocidad = velocidad
        self.pos = [
            random.uniform(-self.area/2, self.area/2),
            random.uniform(-self.area/2, self.area/2)
        ]
        self.destino = self._nuevo_destino()
        self.nombre = f"P{random.randint(1,99)}"

    def _nuevo_destino(self) -> list:
        return [
            random.uniform(-self.area/2, self.area/2),
            random.uniform(-self.area/2, self.area/2)
        ]

    def actualizar(self, dt: float):
        dx = self.destino[0] - self.pos[0]
        dy = self.destino[1] - self.pos[1]
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 1.0:
            self.destino = self._nuevo_destino()
            return

        velocidad = self.velocidad * dt
        self.pos[0] += (dx/dist) * velocidad
        self.pos[1] += (dy/dist) * velocidad


# ------------------------------------------------------------
# VISUALIZADOR DEL MAPA
# ------------------------------------------------------------
class VisualizadorMapa:
    def __init__(self, mapa: MapaOcupacion, config: dict):
        self.mapa = mapa
        self.config = config
        self.escala = config["escala_visual"]
        self.tam = mapa.celdas * self.escala

        # colores
        self.COLOR_LIBRE      = (30, 30, 30)
        self.COLOR_ESTATICO   = (50, 50, 180)
        self.COLOR_DINAMICO   = (50, 180, 180)
        self.COLOR_DRONE      = (0, 255, 100)
        self.COLOR_DESTINO    = (0, 100, 255)
        self.COLOR_RUTA       = (0, 200, 255)
        self.COLOR_RUTA_PREV  = (100, 100, 100)
        self.COLOR_WAYPOINT   = (255, 200, 0)
        self.COLOR_PERSONA    = (200, 100, 0)

    def _m_a_px(self, x_m: float, y_m: float) -> tuple:
        """Convierte metros a pixeles en la ventana."""
        px = int((x_m / self.config["area_metros"] + 0.5) * self.tam)
        py = int((y_m / self.config["area_metros"] + 0.5) * self.tam)
        px = max(0, min(self.tam-1, px))
        py = max(0, min(self.tam-1, py))
        return px, py

    def dibujar(self, drone_pos: tuple, destino: tuple,
                ruta_actual: list, ruta_anterior: list,
                personas: list, stats: dict) -> np.ndarray:
        frame = np.zeros((self.tam, self.tam, 3), dtype=np.uint8)
        frame[:] = self.COLOR_LIBRE

        # grid combinado
        grid = self.mapa.grid_combinado()
        for r in range(self.mapa.celdas):
            for c in range(self.mapa.celdas):
                if grid[r, c] > 0:
                    x = c * self.escala
                    y = r * self.escala
                    color = (self.COLOR_ESTATICO if grid[r,c] == 1
                             else self.COLOR_DINAMICO)
                    cv2.rectangle(frame, (x, y),
                                  (x+self.escala, y+self.escala),
                                  color, -1)

        # grid lines cada 5 metros
        paso = int(5.0 / self.config["resolucion_grid"]) * self.escala
        for i in range(0, self.tam, paso):
            cv2.line(frame, (i, 0), (i, self.tam), (50,50,50), 1)
            cv2.line(frame, (0, i), (self.tam, i), (50,50,50), 1)

        # ejes centrales
        centro = self.tam // 2
        cv2.line(frame, (centro,0), (centro,self.tam), (60,60,60), 1)
        cv2.line(frame, (0,centro), (self.tam,centro), (60,60,60), 1)

        # ruta anterior (gris)
        if len(ruta_anterior) > 1:
            for i in range(len(ruta_anterior)-1):
                p1 = self._m_a_px(*ruta_anterior[i])
                p2 = self._m_a_px(*ruta_anterior[i+1])
                cv2.line(frame, p1, p2, self.COLOR_RUTA_PREV, 1)

        # ruta actual (cyan)
        if len(ruta_actual) > 1:
            for i in range(len(ruta_actual)-1):
                p1 = self._m_a_px(*ruta_actual[i])
                p2 = self._m_a_px(*ruta_actual[i+1])
                cv2.line(frame, p1, p2, self.COLOR_RUTA, 2)
            # waypoints
            for wp in ruta_actual[1:-1]:
                cv2.circle(frame, self._m_a_px(*wp), 3,
                           self.COLOR_WAYPOINT, -1)

        # obstaculos estaticos (etiquetas)
        for x, y, radio, nombre in self.config["obstaculos_estaticos"]:
            px, py = self._m_a_px(x, y)
            cv2.putText(frame, nombre, (px-15, py+3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150,150,255), 1)

        # personas dinamicas
        for p in personas:
            px, py = self._m_a_px(*p["pos"])
            cv2.circle(frame, (px, py), 8, self.COLOR_PERSONA, -1)
            cv2.putText(frame, p["nombre"], (px+8, py-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                        self.COLOR_PERSONA, 1)

        # destino
        if destino:
            dx, dy = self._m_a_px(*destino)
            cv2.drawMarker(frame, (dx, dy), self.COLOR_DESTINO,
                           cv2.MARKER_CROSS, 16, 2)
            cv2.putText(frame, "DEST", (dx+8, dy-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        self.COLOR_DESTINO, 1)

        # drone
        dpx, dpy = self._m_a_px(*drone_pos)
        cv2.circle(frame, (dpx, dpy), 8, self.COLOR_DRONE, -1)
        cv2.circle(frame, (dpx, dpy), 12, self.COLOR_DRONE, 1)
        cv2.putText(frame, "DRONE", (dpx+10, dpy-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    self.COLOR_DRONE, 1)

        # panel de stats
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, self.tam-70),
                      (260, self.tam), (0,0,0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        lineas = [
            f"Drone: ({drone_pos[0]:.1f}m, {drone_pos[1]:.1f}m)",
            f"Replanificaciones: {stats.get('replan', 0)}",
            f"Waypoints ruta: {len(ruta_actual)}",
            f"Personas: {len(personas)}",
        ]
        for i, linea in enumerate(lineas):
            cv2.putText(frame, linea,
                        (5, self.tam - 55 + i*14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        (200,200,200), 1)

        # leyenda
        leyenda = [
            ((50,50,180), "Obstaculo estatico"),
            ((50,180,180), "Persona dinamica"),
            ((0,200,255), "Ruta A*"),
            ((0,255,100), "Drone"),
        ]
        for i, (color, texto) in enumerate(leyenda):
            y = 15 + i * 16
            cv2.rectangle(frame, (self.tam-130, y-8),
                          (self.tam-118, y+4), color, -1)
            cv2.putText(frame, texto, (self.tam-112, y+2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                        (180,180,180), 1)

        return frame


# ------------------------------------------------------------
# SISTEMA DE EVITACION
# ------------------------------------------------------------
class SistemaEvitacion:
    def __init__(self, config: dict = None):
        self.config = config or CONFIG
        self.mapa = MapaOcupacion(self.config)
        self.planificador = PlanificadorAstar(self.mapa)
        self.ruta_actual = []
        self.ruta_anterior = []
        self.replanificaciones = 0
        self.ultimo_replan = 0.0
        self.intervalo_replan = 1.5    # segundos entre replanificaciones

    def planificar_ruta(self, origen: tuple, destino: tuple) -> list:
        """Calcula ruta A* desde origen a destino."""
        t0 = time.time()
        ruta = self.planificador.planificar(origen, destino)
        t1 = time.time()

        if ruta:
            self.ruta_anterior = self.ruta_actual.copy()
            self.ruta_actual = ruta
            self.replanificaciones += 1
            print(f"[Evitacion] Ruta calculada: {len(ruta)} waypoints "
                  f"en {(t1-t0)*1000:.1f}ms")
        else:
            print(f"[Evitacion] Sin ruta disponible de {origen} a {destino}")

        return ruta

    def necesita_replanificar(self, pos_drone: tuple) -> bool:
        """
        Verifica si la ruta actual tiene obstaculos en el camino.
        """
        if not self.ruta_actual:
            return False
        if time.time() - self.ultimo_replan < self.intervalo_replan:
            return False

        # revisar proximos 3 waypoints
        for wp in self.ruta_actual[:3]:
            row, col = self.mapa._metros_a_celda(*wp)
            if not self.mapa.es_libre(row, col):
                self.ultimo_replan = time.time()
                print(f"[Evitacion] Obstaculo en ruta — replanificando...")
                return True

        return False

    def actualizar_obstaculos_dinamicos(self, personas: list):
        self.mapa.actualizar_dinamicos(personas)


# ------------------------------------------------------------
# TEST STANDALONE — SIMULACION COMPLETA
# ------------------------------------------------------------
async def test_evitacion():
    print("=" * 55)
    print("  TEST EVITACION DE OBSTACULOS — A*")
    print("=" * 55)
    print()
    print("  Click izquierdo en el mapa para setear destino")
    print("  ESPACIO para mover el drone al siguiente waypoint")
    print("  R para generar nueva ruta aleatoria")
    print("  Q para salir")
    print()

    config = CONFIG
    sistema = SistemaEvitacion(config)
    visualizador = VisualizadorMapa(sistema.mapa, config)

    # personas dinamicas
    personas_sim = [
        PersonaDinamica(config["area_metros"], config["velocidad_persona"])
        for _ in range(config["num_personas_dinamicas"])
    ]

    # estado del drone
    drone_pos = [0.0, 0.0]
    destino = (10.0, 8.0)
    waypoint_idx = 0
    stats = {"replan": 0}

    # calcular ruta inicial
    sistema.planificar_ruta(tuple(drone_pos), destino)
    stats["replan"] = sistema.replanificaciones

    # callback de click del mouse
    click_pos = [None]
    escala = visualizador.escala
    tam = visualizador.tam
    area = config["area_metros"]

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            x_m = (x / tam - 0.5) * area
            y_m = (y / tam - 0.5) * area
            click_pos[0] = (x_m, y_m)

    cv2.namedWindow("Evitacion de Obstaculos A*")
    cv2.setMouseCallback("Evitacion de Obstaculos A*", on_mouse)

    t_ultimo = time.time()
    t_ultimo_replan = time.time()
    intervalo_replan = 2.0

    while True:
        t_ahora = time.time()
        dt = t_ahora - t_ultimo
        t_ultimo = t_ahora

        # actualizar personas
        for p in personas_sim:
            p.actualizar(dt)

        personas_data = [{"pos": tuple(p.pos), "nombre": p.nombre}
                         for p in personas_sim]

        # actualizar mapa con personas
        sistema.actualizar_obstaculos_dinamicos(personas_data)

        # replanificar si hay obstaculo en ruta
        if (t_ahora - t_ultimo_replan > intervalo_replan and
                sistema.necesita_replanificar(tuple(drone_pos))):
            sistema.planificar_ruta(tuple(drone_pos), destino)
            stats["replan"] = sistema.replanificaciones
            waypoint_idx = 0
            t_ultimo_replan = t_ahora

        # procesar click — nuevo destino
        if click_pos[0]:
            destino = click_pos[0]
            click_pos[0] = None
            print(f"[Sim] Nuevo destino: ({destino[0]:.1f}m, {destino[1]:.1f}m)")
            sistema.planificar_ruta(tuple(drone_pos), destino)
            stats["replan"] = sistema.replanificaciones
            waypoint_idx = 0

        # dibujar
        frame = visualizador.dibujar(
            tuple(drone_pos), destino,
            sistema.ruta_actual, sistema.ruta_anterior,
            personas_data, stats
        )

        cv2.imshow("Evitacion de Obstaculos A*", frame)

        key = cv2.waitKey(int(1000 / config["fps"])) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):
            # mover drone al siguiente waypoint
            if sistema.ruta_actual and waypoint_idx < len(sistema.ruta_actual) - 1:
                waypoint_idx += 1
                wp = sistema.ruta_actual[waypoint_idx]
                drone_pos[0] = wp[0]
                drone_pos[1] = wp[1]
                print(f"[Drone] Waypoint {waypoint_idx}: "
                      f"({drone_pos[0]:.1f}m, {drone_pos[1]:.1f}m)")

                # verificar si llegamos
                dist_dest = math.sqrt(
                    (drone_pos[0]-destino[0])**2 +
                    (drone_pos[1]-destino[1])**2
                )
                if dist_dest < 1.5:
                    print(f"[Drone] Destino alcanzado!")

        elif key == ord('r'):
            # destino aleatorio
            destino = (
                random.uniform(-12, 12),
                random.uniform(-12, 12)
            )
            print(f"[Sim] Destino aleatorio: ({destino[0]:.1f}m, {destino[1]:.1f}m)")
            sistema.planificar_ruta(tuple(drone_pos), destino)
            stats["replan"] = sistema.replanificaciones
            waypoint_idx = 0

        await asyncio.sleep(1.0 / config["fps"])

    cv2.destroyAllWindows()
    print(f"\n[OK] Test completado.")
    print(f"     Total replanificaciones: {sistema.replanificaciones}")


# ------------------------------------------------------------
# ENTRY POINT
# ------------------------------------------------------------
if __name__ == "__main__":
    try:
        asyncio.run(test_evitacion())
    except KeyboardInterrupt:
        print("\n[!] Interrumpido.")
        cv2.destroyAllWindows()

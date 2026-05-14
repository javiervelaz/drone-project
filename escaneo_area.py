"""
escaneo_area.py
===============
Modulo 1 del sistema de filmacion autonoma.

El drone ejecuta un vuelo de reconocimiento en espiral
sobre el area del evento, registrando:
  - Dimensiones del area operativa
  - Posiciones clave (centro, bordes)
  - Alturas seguras de vuelo
  - Log de waypoints recorridos

Uso:
    python3 escaneo_area.py
"""

import asyncio
import math
import json
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
)


# ------------------------------------------------------------
# CONFIGURACION DEL ESCANEO
# ------------------------------------------------------------
CONFIG = {
    "altura_escaneo": 8.0,       # metros — altura de vuelo durante escaneo
    "altura_despegue": 3.0,      # metros — altura inicial antes de moverse
    "radio_inicial": 3.0,        # metros — primer anillo de la espiral
    "radio_maximo": 15.0,        # metros — radio total del area a cubrir
    "incremento_radio": 3.0,     # metros — expansion entre anillos
    "puntos_por_anillo": 8,      # waypoints por vuelta de la espiral
    "velocidad_ms": 2.0,         # m/s — velocidad de crucero
    "pausa_waypoint": 0.5,       # segundos — pausa en cada punto
}


# ------------------------------------------------------------
# CLASE PRINCIPAL
# ------------------------------------------------------------
class EscaneoArea:
    def __init__(self, config: dict):
        self.config = config
        self.drone = System()
        self.waypoints_recorridos = []
        self.mapa = {
            "timestamp": "",
            "area_radio_m": config["radio_maximo"],
            "altura_escaneo_m": config["altura_escaneo"],
            "waypoints": [],
            "estado": "pendiente",
        }

    # --------------------------------------------------------
    # CONEXION Y SETUP
    # --------------------------------------------------------
    async def conectar(self):
        print("[*] Conectando con PX4 SITL...")
        await self.drone.connect(system_address="udp://:14540")

        print("[*] Esperando conexion", end="", flush=True)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(" OK")
                break
            print(".", end="", flush=True)

        print("[*] Esperando GPS", end="", flush=True)
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(" OK")
                break
            print(".", end="", flush=True)

    # --------------------------------------------------------
    # GENERADOR DE WAYPOINTS EN ESPIRAL
    # --------------------------------------------------------
    def generar_espiral(self) -> list:
        """
        Genera waypoints en espiral cuadrada desde el centro hacia afuera.
        Devuelve lista de (norte_m, este_m, altura_m, yaw_deg)
        """
        waypoints = []
        radio = self.config["radio_inicial"]
        puntos = self.config["puntos_por_anillo"]
        altura = self.config["altura_escaneo"]

        while radio <= self.config["radio_maximo"]:
            for i in range(puntos):
                angulo = (2 * math.pi * i) / puntos
                norte = radio * math.cos(angulo)
                este = radio * math.sin(angulo)
                # yaw apunta siempre al centro para filmar hacia adentro
                yaw = math.degrees(math.atan2(-este, -norte)) % 360
                waypoints.append((norte, este, -altura, yaw))

            radio += self.config["incremento_radio"]

        return waypoints

    # --------------------------------------------------------
    # DESPEGUE
    # --------------------------------------------------------
    async def despegar(self):
        print(f"\n[1/4] Armando y despegando a "
              f"{self.config['altura_despegue']}m...")

        await self.drone.action.arm()
        await self.drone.action.set_takeoff_altitude(
            self.config["altura_despegue"]
        )
        await self.drone.action.takeoff()

        # esperar altura
        print("      Subiendo", end="", flush=True)
        async for pos in self.drone.telemetry.position():
            if pos.relative_altitude_m >= self.config["altura_despegue"] * 0.9:
                print(f" OK — {pos.relative_altitude_m:.1f}m")
                break
            print(".", end="", flush=True)
            await asyncio.sleep(0.3)

    # --------------------------------------------------------
    # ESCANEO EN ESPIRAL
    # --------------------------------------------------------
    async def ejecutar_escaneo(self):
        waypoints = self.generar_espiral()
        total = len(waypoints)

        print(f"\n[2/4] Iniciando escaneo espiral...")
        print(f"      Radio: {self.config['radio_maximo']}m | "
              f"Waypoints: {total} | "
              f"Altura: {self.config['altura_escaneo']}m")

        # activar modo offboard
        print("\n      Activando modo offboard...")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0,
                           -self.config["altura_escaneo"], 0.0)
        )

        try:
            await self.drone.offboard.start()
            print("      Modo offboard OK")
        except OffboardError as e:
            print(f"      Error offboard: {e}")
            return

        # recorrer waypoints
        print(f"\n      Recorriendo area", end="", flush=True)
        for idx, (norte, este, alt, yaw) in enumerate(waypoints):
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(norte, este, alt, yaw)
            )

            # esperar a que llegue al punto
            await asyncio.sleep(
                self._tiempo_vuelo(norte, este, idx, waypoints) +
                self.config["pausa_waypoint"]
            )

            # registrar waypoint
            self.waypoints_recorridos.append({
                "idx": idx,
                "norte": round(norte, 2),
                "este": round(este, 2),
                "altura": self.config["altura_escaneo"],
                "yaw": round(yaw, 1),
            })

            if idx % 4 == 0:
                print(f".", end="", flush=True)

        print(f" OK ({total} puntos)")

        # detener offboard
        await self.drone.offboard.stop()

    # --------------------------------------------------------
    # RETORNO AL HOME Y ATERRIZAJE
    # --------------------------------------------------------
    async def retornar_y_aterrizar(self):
        print("\n[3/4] Retornando al punto de inicio...")
        await self.drone.action.return_to_launch()

        print("      Aterrizando", end="", flush=True)
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                print(" OK")
                break
            print(".", end="", flush=True)
            await asyncio.sleep(0.5)

    # --------------------------------------------------------
    # GUARDAR MAPA DEL AREA
    # --------------------------------------------------------
    async def guardar_mapa(self):
        print("\n[4/4] Guardando mapa del area...")

        self.mapa["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.mapa["waypoints"] = self.waypoints_recorridos
        self.mapa["total_waypoints"] = len(self.waypoints_recorridos)
        self.mapa["estado"] = "completado"

        archivo = f"mapa_evento_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(archivo, "w") as f:
            json.dump(self.mapa, f, indent=2)

        print(f"      Mapa guardado: {archivo}")
        print(f"      Waypoints registrados: {len(self.waypoints_recorridos)}")
        return archivo

    # --------------------------------------------------------
    # HELPER — tiempo estimado de vuelo entre waypoints
    # --------------------------------------------------------
    def _tiempo_vuelo(self, norte, este, idx, waypoints) -> float:
        if idx == 0:
            distancia = math.sqrt(norte**2 + este**2)
        else:
            prev = waypoints[idx - 1]
            distancia = math.sqrt(
                (norte - prev[0])**2 + (este - prev[1])**2
            )
        return max(0.5, distancia / self.config["velocidad_ms"])

    # --------------------------------------------------------
    # FLUJO PRINCIPAL
    # --------------------------------------------------------
    async def ejecutar(self):
        print("=" * 50)
        print("  ESCANEO INICIAL DEL AREA DEL EVENTO")
        print("=" * 50)

        await self.conectar()
        await self.despegar()
        await self.ejecutar_escaneo()
        await self.retornar_y_aterrizar()
        archivo = await self.guardar_mapa()

        print("\n" + "=" * 50)
        print("  ESCANEO COMPLETADO")
        print(f"  Area cubierta: {math.pi * self.config['radio_maximo']**2:.0f} m²")
        print(f"  Mapa listo para el sistema de filmacion")
        print("=" * 50 + "\n")

        return archivo


# ------------------------------------------------------------
# ENTRY POINT
# ------------------------------------------------------------
if __name__ == "__main__":
    escaner = EscaneoArea(CONFIG)
    asyncio.run(escaner.ejecutar())

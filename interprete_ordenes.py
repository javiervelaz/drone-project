"""
interprete_ordenes.py
=====================
Modulo 2 del sistema de filmacion autonoma.

Recibe ordenes en texto natural en español y las convierte
en comandos de vuelo concretos usando el mapa del area.

Ordenes soportadas:
  - "panoramica"        → arco de 360° sobre el area
  - "toma cenital"      → subir y apuntar hacia abajo
  - "sobrevolar norte"  → volar hacia el norte del area
  - "volver al centro"  → retornar al centro del area
  - "bajar altura"      → descender a altura de filmacion
  - "subir altura"      → subir a altura de escaneo
  - "posicion X Y"      → ir a coordenada NED especifica
  - "hover"             → quedarse quieto en posicion actual
  - "aterrizar"         → retorno y aterrizaje

Uso:
    python3 interprete_ordenes.py mapa_evento_XXXXXX.json
"""

import asyncio
import json
import math
import sys
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


# ------------------------------------------------------------
# CONFIGURACION
# ------------------------------------------------------------
CONFIG = {
    "altura_filmacion": 5.0,    # metros — altura normal de filmacion
    "altura_cenital": 10.0,     # metros — altura para toma cenital
    "velocidad_ms": 2.0,        # m/s — para calcular tiempos
    "radio_panoramica": 8.0,    # metros — radio del arco panoramico
    "puntos_panoramica": 12,    # waypoints para el arco 360°
}


# ------------------------------------------------------------
# PARSER DE ORDENES
# ------------------------------------------------------------
class ParserOrdenes:
    """
    Convierte texto en español a comandos estructurados.
    Sin LLM externo — reglas simples para el MVP.
    Fácil de reemplazar por LLM más adelante.
    """

    COMANDOS = {
        "panoramica":     ["panoramica", "panorama", "360", "vuelta completa", "recorre todo"],
        "cenital":        ["cenital", "desde arriba", "toma aerea", "plano cenital", "arriba del todo"],
        "norte":          ["norte", "arriba del area", "fondo"],
        "sur":            ["sur", "abajo del area", "entrada"],
        "este":           ["este", "derecha"],
        "oeste":          ["oeste", "izquierda"],
        "centro":         ["centro", "medio", "en el medio", "volver", "regresa"],
        "hover":          ["hover", "queda quieto", "para", "mantente", "espera"],
        "bajar":          ["baja", "bajar", "descende", "mas bajo", "altura filmacion"],
        "subir":          ["sube", "subir", "asciende", "mas alto", "altura escaneo"],
        "aterrizar":      ["aterriza", "aterrizar", "tierra", "baja y aterriza", "termina"],
        "seguir_persona": ["segui", "sigue", "seguir", "enfoca", "apunta a"],
    }

    def parsear(self, texto: str) -> dict:
        texto_lower = texto.lower().strip()

        for comando, keywords in self.COMANDOS.items():
            for kw in keywords:
                if kw in texto_lower:
                    return {
                        "comando": comando,
                        "texto_original": texto,
                        "parametros": self._extraer_parametros(texto_lower, comando)
                    }

        return {
            "comando": "desconocido",
            "texto_original": texto,
            "parametros": {}
        }

    def _extraer_parametros(self, texto: str, comando: str) -> dict:
        params = {}

        # extraer altura si se menciona
        palabras = texto.split()
        for i, p in enumerate(palabras):
            if p.isdigit():
                params["valor_numerico"] = int(p)
                break

        # extraer nombre de persona si se menciona
        if comando == "seguir_persona":
            # buscar descriptor: "remera roja", "de azul", etc.
            for color in ["roja", "rojo", "azul", "verde", "blanca", "blanco",
                         "negro", "negra", "amarilla", "amarillo"]:
                if color in texto:
                    params["descriptor"] = color
                    break

        return params


# ------------------------------------------------------------
# EJECUTOR DE COMANDOS
# ------------------------------------------------------------
class EjecutorComandos:
    def __init__(self, drone: System, mapa: dict, config: dict):
        self.drone = drone
        self.mapa = mapa
        self.config = config
        self.posicion_actual = {"norte": 0.0, "este": 0.0, "altura": config["altura_filmacion"]}
        self.offboard_activo = False

    async def iniciar_offboard(self):
        if not self.offboard_activo:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, -self.config["altura_filmacion"], 0.0)
            )
            try:
                await self.drone.offboard.start()
                self.offboard_activo = True
            except OffboardError as e:
                print(f"   Error offboard: {e}")

    async def ir_a(self, norte: float, este: float, altura: float, yaw: float = 0.0, espera: float = None):
        """Mueve el drone a una posicion NED."""
        await self.iniciar_offboard()

        await self.drone.offboard.set_position_ned(
            PositionNedYaw(norte, este, -altura, yaw)
        )

        # calcular tiempo de espera segun distancia
        if espera is None:
            dist = math.sqrt(
                (norte - self.posicion_actual["norte"])**2 +
                (este - self.posicion_actual["este"])**2
            )
            espera = max(1.5, dist / self.config["velocidad_ms"])

        await asyncio.sleep(espera)
        self.posicion_actual = {"norte": norte, "este": este, "altura": altura}

    # --------------------------------------------------------
    # IMPLEMENTACION DE CADA COMANDO
    # --------------------------------------------------------
    async def cmd_panoramica(self, params: dict):
        """Arco de 360° alrededor del centro del area."""
        print("   Ejecutando panoramica 360°...")
        radio = self.config["radio_panoramica"]
        puntos = self.config["puntos_panoramica"]
        altura = self.posicion_actual["altura"]

        for i in range(puntos + 1):  # +1 para cerrar el circulo
            angulo = (2 * math.pi * i) / puntos
            norte = radio * math.cos(angulo)
            este = radio * math.sin(angulo)
            # yaw siempre apunta al centro
            yaw = math.degrees(math.atan2(-este, -norte)) % 360
            await self.ir_a(norte, este, altura, yaw, espera=2.0)
            print(f"   Punto {i+1}/{puntos+1} — ({norte:.1f}m N, {este:.1f}m E)", end="\r")

        print(f"\n   Panoramica completada.")

    async def cmd_cenital(self, params: dict):
        """Sube y apunta hacia abajo para toma cenital."""
        print(f"   Subiendo a {self.config['altura_cenital']}m para toma cenital...")
        norte = self.posicion_actual["norte"]
        este = self.posicion_actual["este"]
        await self.ir_a(norte, este, self.config["altura_cenital"], 0.0, espera=3.0)
        print("   Toma cenital activa — drone en posicion.")

    async def cmd_direccion(self, direccion: str, params: dict):
        """Vuela hacia un punto cardinal del area."""
        radio = self.mapa["area_radio_m"] * 0.7
        altura = self.posicion_actual["altura"]

        destinos = {
            "norte": (radio, 0.0, 180.0),
            "sur":   (-radio, 0.0, 0.0),
            "este":  (0.0, radio, 270.0),
            "oeste": (0.0, -radio, 90.0),
        }

        norte, este, yaw = destinos[direccion]
        print(f"   Volando hacia el {direccion} del area ({radio:.0f}m)...")
        await self.ir_a(norte, este, altura, yaw)
        print(f"   Posicion {direccion} alcanzada.")

    async def cmd_centro(self, params: dict):
        """Retorna al centro del area."""
        print("   Retornando al centro del area...")
        await self.ir_a(0.0, 0.0, self.posicion_actual["altura"], 0.0)
        print("   En el centro.")

    async def cmd_hover(self, params: dict):
        """Mantiene la posicion actual."""
        print("   Hover — manteniendo posicion...")
        await self.ir_a(
            self.posicion_actual["norte"],
            self.posicion_actual["este"],
            self.posicion_actual["altura"],
            0.0, espera=1.0
        )

    async def cmd_bajar(self, params: dict):
        """Desciende a altura de filmacion."""
        altura = params.get("valor_numerico", self.config["altura_filmacion"])
        print(f"   Bajando a {altura}m...")
        await self.ir_a(
            self.posicion_actual["norte"],
            self.posicion_actual["este"],
            float(altura), 0.0, espera=2.0
        )
        print(f"   Altura {altura}m alcanzada.")

    async def cmd_subir(self, params: dict):
        """Asciende a altura de escaneo."""
        altura = params.get("valor_numerico", self.config["altura_cenital"])
        print(f"   Subiendo a {altura}m...")
        await self.ir_a(
            self.posicion_actual["norte"],
            self.posicion_actual["este"],
            float(altura), 0.0, espera=2.0
        )
        print(f"   Altura {altura}m alcanzada.")

    async def cmd_aterrizar(self, params: dict):
        """Detiene offboard y aterriza."""
        print("   Retornando al home y aterrizando...")
        if self.offboard_activo:
            await self.drone.offboard.stop()
            self.offboard_activo = False
        await self.drone.action.return_to_launch()
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                break
            await asyncio.sleep(0.5)
        print("   Drone en tierra.")
        return False  # señal para salir del loop

    async def cmd_seguir_persona(self, params: dict):
        """Placeholder — se implementa con vision en modulo 3."""
        descriptor = params.get("descriptor", "desconocida")
        print(f"   [Modulo Vision] Seguimiento de persona ({descriptor}) — proximamente.")
        print(f"   Por ahora el drone mantiene posicion actual.")

    async def cmd_desconocido(self, texto: str):
        print(f"   No entendi la orden: '{texto}'")
        print(f"   Ordenes validas: panoramica, cenital, norte/sur/este/oeste,")
        print(f"   centro, hover, bajar, subir, aterrizar, segui [color]")

    # --------------------------------------------------------
    # DISPATCHER
    # --------------------------------------------------------
    async def ejecutar(self, orden: dict) -> bool:
        """
        Ejecuta una orden parseada.
        Retorna False si se debe terminar el programa.
        """
        cmd = orden["comando"]
        params = orden["parametros"]

        if cmd == "panoramica":
            await self.cmd_panoramica(params)
        elif cmd == "cenital":
            await self.cmd_cenital(params)
        elif cmd in ["norte", "sur", "este", "oeste"]:
            await self.cmd_direccion(cmd, params)
        elif cmd == "centro":
            await self.cmd_centro(params)
        elif cmd == "hover":
            await self.cmd_hover(params)
        elif cmd == "bajar":
            await self.cmd_bajar(params)
        elif cmd == "subir":
            await self.cmd_subir(params)
        elif cmd == "aterrizar":
            return await self.cmd_aterrizar(params)
        elif cmd == "seguir_persona":
            await self.cmd_seguir_persona(params)
        elif cmd == "desconocido":
            await self.cmd_desconocido(orden["texto_original"])

        return True  # continuar


# ------------------------------------------------------------
# LOOP PRINCIPAL
# ------------------------------------------------------------
async def main(archivo_mapa: str):
    # cargar mapa
    with open(archivo_mapa) as f:
        mapa = json.load(f)

    print("=" * 50)
    print("  DRONE FILMADOR — MODO DIRECTOR")
    print(f"  Mapa: {archivo_mapa}")
    print(f"  Area: radio {mapa['area_radio_m']}m")
    print("=" * 50)
    print()

    # conectar drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("[*] Conectando", end="", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(" OK")
            break
        print(".", end="", flush=True)

    print("[*] Esperando GPS", end="", flush=True)
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(" OK")
            break
        print(".", end="", flush=True)

    # despegue
    print("\n[*] Despegando...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(CONFIG["altura_filmacion"])
    await drone.action.takeoff()
    await asyncio.sleep(4)
    print("[*] Drone en posicion. Listo para recibir ordenes.\n")

    # instanciar modulos
    parser = ParserOrdenes()
    ejecutor = EjecutorComandos(drone, mapa, CONFIG)

    # loop de ordenes
    print("─" * 50)
    print("  Escribi una orden en español (o 'aterrizar' para terminar)")
    print("  Ejemplos: panoramica | cenital | norte | sigue remera roja")
    print("─" * 50 + "\n")

    continuar = True
    while continuar:
        try:
            orden_texto = input(">> Orden: ").strip()
            if not orden_texto:
                continue

            orden = parser.parsear(orden_texto)
            print(f"   [{orden['comando'].upper()}] ", end="")

            continuar = await ejecutor.ejecutar(orden)

        except KeyboardInterrupt:
            print("\n[!] Interrumpido — aterrizando...")
            await drone.action.return_to_launch()
            break

    print("\n[OK] Sesion de filmacion terminada.")


# ------------------------------------------------------------
# ENTRY POINT
# ------------------------------------------------------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        # buscar el mapa mas reciente automaticamente
        import glob
        mapas = sorted(glob.glob("mapa_evento_*.json"))
        if mapas:
            archivo = mapas[-1]
            print(f"[*] Usando mapa: {archivo}")
        else:
            print("Uso: python3 interprete_ordenes.py mapa_evento_XXXXXX.json")
            sys.exit(1)
    else:
        archivo = sys.argv[1]

    asyncio.run(main(archivo))

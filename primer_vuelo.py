"""
primer_vuelo.py
===============
Valida que todo el stack funciona:
  - Conexion con PX4 SITL
  - Armado del drone
  - Despegue autonomo
  - Hover en posicion
  - Aterrizaje

Ejecutar con PX4 SITL corriendo en otra terminal.
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw


async def conectar_drone() -> System:
    """Conecta con PX4 SITL y espera hasta tener telemetria."""
    drone = System()
    print("[*] Conectando con PX4 SITL...")
    await drone.connect(system_address="udp://:14540")

    print("[*] Esperando conexion", end="", flush=True)
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

    return drone


async def primer_vuelo():
    """
    Flujo completo de prueba:
    despegue → hover 5 seg → aterrizaje
    """
    drone = await conectar_drone()

    # --- Estado inicial ---
    print("\n[INFO] Estado del drone:")
    async for battery in drone.telemetry.battery():
        print(f"  Bateria: {battery.remaining_percent:.0%}")
        break

    async for pos in drone.telemetry.position():
        print(f"  Posicion inicial: lat={pos.latitude_deg:.6f} "
              f"lon={pos.longitude_deg:.6f} "
              f"alt={pos.relative_altitude_m:.1f}m")
        break

    # --- Armar ---
    print("\n[1/4] Armando motores...")
    await drone.action.arm()
    print("  Motores armados OK")

    # --- Despegue ---
    altura_objetivo = 5.0  # metros
    print(f"\n[2/4] Despegando a {altura_objetivo}m...")
    await drone.action.set_takeoff_altitude(altura_objetivo)
    await drone.action.takeoff()

    # esperar a alcanzar la altura
    print("  Subiendo", end="", flush=True)
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= altura_objetivo * 0.95:
            print(f" OK — {pos.relative_altitude_m:.1f}m")
            break
        print(".", end="", flush=True)
        await asyncio.sleep(0.5)

    # --- Hover ---
    segundos_hover = 5
    print(f"\n[3/4] Hover por {segundos_hover} segundos...")
    for i in range(segundos_hover, 0, -1):
        async for pos in drone.telemetry.position():
            print(f"  t-{i}s | altura: {pos.relative_altitude_m:.2f}m")
            break
        await asyncio.sleep(1)

    # --- Aterrizar ---
    print("\n[4/4] Aterrizando...")
    await drone.action.land()

    print("  Aterrizando", end="", flush=True)
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print(" OK — en tierra")
            break
        print(".", end="", flush=True)
        await asyncio.sleep(0.5)

    print("\n[OK] Primer vuelo completado exitosamente.")
    print("     El stack PX4 + MAVSDK funciona correctamente.")
    print("     Siguiente paso: modulo de escaneo del area.\n")


if __name__ == "__main__":
    asyncio.run(primer_vuelo())


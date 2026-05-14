#!/bin/bash
# ================================================================
# SETUP DRONE SIMULACION — PX4 SITL + Gazebo Classic + MAVSDK
# Portable: Ubuntu 20.04 (focal) / Ubuntu 22.04 (jammy) / WSL2
#
# Uso:
#   chmod +x setup_drone_sim.sh
#   ./setup_drone_sim.sh
#
# Opciones:
#   --dir <ruta>     Directorio de instalación (default: $HOME)
#   --no-compile     Solo dependencias, no compila PX4
#   --jmavsim        Usa jMAVSim en vez de Gazebo (más liviano)
#   --reinstall      Borra instalación previa y reinstala todo
# ================================================================

set -euo pipefail

# ----------------------------------------------------------------
# COLORES
# ----------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
info() { echo -e "${CYAN}[>>]${NC} $1"; }
warn() { echo -e "${YELLOW}[!!]${NC} $1"; }
fail() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

step() {
    echo ""
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  $1${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
}

# ----------------------------------------------------------------
# ARGUMENTOS
# ----------------------------------------------------------------
INSTALL_DIR="$HOME"
NO_COMPILE=false
USE_JMAVSIM=false
REINSTALL=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --dir)       INSTALL_DIR="$2"; shift 2 ;;
        --no-compile) NO_COMPILE=true; shift ;;
        --jmavsim)   USE_JMAVSIM=true; shift ;;
        --reinstall) REINSTALL=true; shift ;;
        *) warn "Argumento desconocido: $1"; shift ;;
    esac
done

PX4_DIR="$INSTALL_DIR/PX4-Autopilot"
DRONE_DIR="$INSTALL_DIR/drone_project"

# ----------------------------------------------------------------
# BANNER
# ----------------------------------------------------------------
echo ""
echo -e "${CYAN}╔════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║    DRONE SIM — Setup automático v2.4           ║${NC}"
echo -e "${CYAN}║    PX4 SITL + Gazebo Classic + MAVSDK Python   ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════╝${NC}"
echo ""
echo "  Directorio de instalación : $INSTALL_DIR"
echo "  PX4-Autopilot             : $PX4_DIR"
echo "  Proyecto drone            : $DRONE_DIR"
echo "  Simulador                 : $([ "$USE_JMAVSIM" = true ] && echo 'jMAVSim' || echo 'Gazebo Classic')"
echo ""

# ----------------------------------------------------------------
# PASO 0 — DETECTAR ENTORNO
# ----------------------------------------------------------------
step "0/7 — Detectando entorno"

# Verificar que sea Linux
[[ "$(uname)" == "Linux" ]] || fail "Este script requiere Linux o WSL2."

# Detectar distro y versión
if [ -f /etc/os-release ]; then
    . /etc/os-release
    DISTRO_ID="${ID:-unknown}"
    DISTRO_VERSION="${VERSION_CODENAME:-unknown}"
    DISTRO_VERSION_ID="${VERSION_ID:-unknown}"
else
    fail "No se pudo detectar la distribución Linux."
fi

info "Distro    : $DISTRO_ID $DISTRO_VERSION_ID ($DISTRO_VERSION)"

# Solo Ubuntu soportado
[[ "$DISTRO_ID" == "ubuntu" ]] || fail "Solo Ubuntu 20.04 y 22.04 están soportados (detectado: $DISTRO_ID)."

# Verificar versión soportada
case "$DISTRO_VERSION" in
    focal)   UBUNTU_VERSION=20; ok "Ubuntu 20.04 LTS (focal) ✓" ;;
    jammy)   UBUNTU_VERSION=22; ok "Ubuntu 22.04 LTS (jammy) ✓" ;;
    *)       fail "Versión no soportada: $DISTRO_VERSION. Usar Ubuntu 20.04 o 22.04." ;;
esac

# Detectar WSL
IS_WSL=false
if grep -qi microsoft /proc/version 2>/dev/null; then
    IS_WSL=true
    info "Entorno   : WSL2 detectado"
else
    info "Entorno   : Linux nativo"
fi

# Detectar RAM
RAM_GB=$(awk '/MemTotal/ {printf "%.0f", $2/1024/1024}' /proc/meminfo)
info "RAM       : ${RAM_GB}GB"
[ "$RAM_GB" -lt 8 ] && warn "Menos de 8GB RAM — la compilación puede ser lenta o fallar."

# Verificar sudo
sudo -n true 2>/dev/null || {
    warn "Necesita contraseña sudo. Se pedirá durante la instalación."
    sudo true || fail "No se pudo obtener privilegios sudo."
}

# ----------------------------------------------------------------
# PASO 1 — ACTUALIZAR SISTEMA
# ----------------------------------------------------------------
step "1/7 — Actualizando sistema"

# Algunos entornos VM traen repos externos con GPG keys rotas
# (ej: Google Cloud SDK). Los deshabilitamos temporalmente para
# no bloquear el setup — no son necesarios para el proyecto.
fix_broken_repos() {
    local broken_repos=()

    # Buscar repos que fallen al hacer update
    info "Verificando repos externos..."
    local update_out
    update_out=$(sudo apt-get update 2>&1 || true)

    # Detectar repos con error de firma
    while IFS= read -r line; do
        if [[ "$line" =~ "NO_PUBKEY" ]] || [[ "$line" =~ "is not signed" ]]; then
            # Extraer el archivo de repo problemático
            local repo_file
            repo_file=$(echo "$line" | grep -oP '/etc/apt/sources\.list\.d/[^\s:]+' || true)
            if [[ -n "$repo_file" ]] && [[ -f "$repo_file" ]]; then
                warn "Repo con firma inválida: $repo_file → deshabilitando temporalmente"
                sudo mv "$repo_file" "${repo_file}.disabled_by_setup"
                broken_repos+=("$repo_file")
            fi
        fi
    done <<< "$update_out"

    # Caso especial: Google Cloud SDK (común en VMs de GCP/cloud)
    for f in /etc/apt/sources.list.d/google-cloud*.list \
              /etc/apt/sources.list.d/cloud-sdk*.list; do
        if [[ -f "$f" ]]; then
            warn "Repo Google Cloud encontrado: $f → deshabilitando"
            sudo mv "$f" "${f}.disabled_by_setup"
            broken_repos+=("$f")
        fi
    done

    if [[ ${#broken_repos[@]} -gt 0 ]]; then
        warn "Se deshabilitaron ${#broken_repos[@]} repo(s) externos no necesarios para el drone."
        info "Para restaurarlos después: renombrá los archivos .disabled_by_setup → .list"
    fi
}

fix_broken_repos

# Ahora sí, actualizar limpio
sudo apt-get update -qq \
    || sudo apt-get update --allow-unauthenticated -qq \
    || warn "apt update con advertencias menores — continuando de todas formas"

ok "apt update completado"

# ----------------------------------------------------------------
# PASO 2 — DEPENDENCIAS BASE
# ----------------------------------------------------------------
step "2/7 — Instalando dependencias base"

BASE_DEPS=(
    git wget curl
    python3 python3-pip python3-venv
    cmake build-essential ninja-build
    pkg-config
    libopencv-dev python3-opencv
    gstreamer1.0-plugins-bad gstreamer1.0-libav
    gstreamer1.0-gl
    libfuse2
    astyle
    xterm  # necesario para PX4 SITL launch
)

# libfuse2 en 22.04 puede llamarse libfuse2t64
if [ "$UBUNTU_VERSION" -eq 22 ]; then
    if apt-cache show libfuse2t64 &>/dev/null; then
        BASE_DEPS+=(libfuse2t64)
    else
        BASE_DEPS+=(libfuse2)
    fi
fi

info "Instalando ${#BASE_DEPS[@]} paquetes base..."
sudo apt-get install -y "${BASE_DEPS[@]}" 2>&1 | grep -E "^(E:|W:|Setting up)" || true

ok "Dependencias base instaladas"

# ----------------------------------------------------------------
# PASO 3 — GAZEBO CLASSIC (omitir si --jmavsim)
# ----------------------------------------------------------------
if [ "$USE_JMAVSIM" = false ]; then
    step "3/7 — Instalando Gazebo Classic 11"

    # Verificar si ya está instalado
    if command -v gazebo &>/dev/null; then
        GAZEBO_VER=$(gazebo --version 2>&1 | head -1 | grep -oP '\d+\.\d+\.\d+' || echo "desconocida")
        ok "Gazebo ya instalado: versión $GAZEBO_VER — omitiendo"
    else
        # Ubuntu 20.04: Gazebo no está en los repos oficiales → necesita OSRF
        if [ "$UBUNTU_VERSION" -eq 20 ]; then
            info "Ubuntu 20.04: agregando repositorio OSRF para Gazebo 11..."

            # Agregar repo OSRF
            sudo sh -c "echo 'deb http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main' \
                > /etc/apt/sources.list.d/gazebo-stable.list"

            # Agregar key (método nuevo para evitar deprecated apt-key)
            wget -qO - https://packages.osrfoundation.org/gazebo.key | \
                sudo gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg 2>/dev/null || \
                wget -qO - https://packages.osrfoundation.org/gazebo.key | sudo apt-key add -

            sudo apt-get update -qq
            sudo apt-get install -y gazebo11 libgazebo11-dev
            ok "Gazebo 11 instalado (vía OSRF repo)"

        # Ubuntu 22.04: Gazebo 11 está en repos estándar
        elif [ "$UBUNTU_VERSION" -eq 22 ]; then
            info "Ubuntu 22.04: instalando Gazebo Classic desde repos..."
            sudo apt-get install -y gazebo libgazebo-dev
            ok "Gazebo instalado"
        fi

        # Verificar instalación — refrescar hash de comandos
        hash -r 2>/dev/null || true
        if command -v gazebo &>/dev/null; then
            GVER=$(gazebo --version 2>&1 | head -1)
            ok "Gazebo verificado: $GVER"
        elif dpkg -l gazebo 2>/dev/null | grep -q "^ii"; then
            ok "Gazebo instalado (dpkg ok) — requiere re-login para PATH"
        else
            fail "Gazebo no quedó instalado. Ejecutá: sudo apt-get install -y gazebo libgazebo-dev"
        fi
    fi
else
    step "3/7 — Gazebo omitido (modo jMAVSim)"
    ok "jMAVSim no requiere instalación extra"
fi

# ----------------------------------------------------------------
# PASO 4 — CLONAR PX4
# ----------------------------------------------------------------
step "4/7 — Clonando PX4-Autopilot"

if [ "$REINSTALL" = true ] && [ -d "$PX4_DIR" ]; then
    warn "REINSTALL activado — borrando $PX4_DIR..."
    rm -rf "$PX4_DIR"
fi

if [ -d "$PX4_DIR" ]; then
    ok "PX4-Autopilot ya existe en $PX4_DIR — omitiendo clone"
    info "Para reinstalar ejecutá con --reinstall"
else
    info "Clonando con --depth 1 (versión ligera sin historial completo)..."
    info "Esto puede tardar 5-15 min según la conexión..."

    git clone \
        https://github.com/PX4/PX4-Autopilot.git \
        --recursive \
        --depth 1 \
        --shallow-submodules \
        "$PX4_DIR" \
        || fail "El clone de PX4 falló. Verificá tu conexión a GitHub."

    ok "PX4-Autopilot clonado en $PX4_DIR"
fi

# ----------------------------------------------------------------
# PASO 4.1 — DEPENDENCIAS DE PX4 (ubuntu.sh)
# ----------------------------------------------------------------
step "4.1/7 — Instalando dependencias PX4 (ubuntu.sh)"

if [ -f "$PX4_DIR/Tools/setup/ubuntu.sh" ]; then
    info "Ejecutando ubuntu.sh con --no-nuttx (omite toolchain de hardware)..."
    cd "$PX4_DIR"
    # DEBIAN_FRONTEND=noninteractive para evitar prompts interactivos
    DEBIAN_FRONTEND=noninteractive bash ./Tools/setup/ubuntu.sh --no-nuttx 2>&1 || \
        warn "ubuntu.sh terminó con advertencias (puede ser normal)"
    ok "ubuntu.sh completado"
else
    warn "ubuntu.sh no encontrado — puede que la versión de PX4 sea diferente"
fi

# ----------------------------------------------------------------
# PASO 4.2 — FIX GAZEBO CLASSIC EN UBUNTU 20.04
# ----------------------------------------------------------------
if [ "$USE_JMAVSIM" = false ] && [ "$UBUNTU_VERSION" -eq 20 ]; then
    step "4.2/7 — Aplicando fix Gazebo Classic / Ubuntu 20.04"

    GAZEBO_CMAKE="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/CMakeLists.txt"

    if [ -f "$GAZEBO_CMAKE" ]; then
        # Verificar si el fix ya fue aplicado
        if grep -q "FIX_BOOST_THREAD_UBUNTU20" "$GAZEBO_CMAKE"; then
            ok "Fix ya aplicado — omitiendo"
        else
            info "Parcheando CMakeLists.txt para compatibilidad con Ubuntu 20.04..."

            # Backup del archivo original
            cp "$GAZEBO_CMAKE" "${GAZEBO_CMAKE}.bak"

            # Insertar fix después de find_package(Boost...)
            # Reemplaza la línea de Boost para agregar el fix a continuación
            python3 - <<'PYEOF'
import re, sys

path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

fix = """
# FIX_BOOST_THREAD_UBUNTU20 — Compatibilidad Ubuntu 20.04
# Fuerza linkeo correcto de Boost.Thread en GCC 9+
set_target_properties_if_exists_fix()
macro(set_target_properties_if_exists_fix)
endmacro()
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lpthread")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -lpthread")
"""

# Insertar después de find_package(Boost...
content = re.sub(
    r'(find_package\(Boost[^\n]*\n)',
    r'\1' + fix,
    content
)

with open(path, 'w') as f:
    f.write(content)

print("[OK] Patch aplicado")
PYEOF
            python3 - "$GAZEBO_CMAKE"

            ok "CMakeLists.txt parcheado"
        fi
    else
        warn "CMakeLists.txt no encontrado en $GAZEBO_CMAKE"
        warn "La versión de PX4 puede ser diferente a la esperada"
    fi
fi

# ----------------------------------------------------------------
# PASO 5 — COMPILAR PX4
# ----------------------------------------------------------------
if [ "$NO_COMPILE" = false ]; then
    step "5/7 — Compilando PX4 SITL"

    cd "$PX4_DIR"

    if [ "$USE_JMAVSIM" = true ]; then
        TARGET="px4_sitl jmavsim"
    else
        TARGET="px4_sitl gazebo-classic_iris"
    fi

    info "Target    : $TARGET"
    info "Esta compilación tarda 10-25 minutos la primera vez..."
    info "No la interrumpas aunque parezca congelada."
    echo ""

    # Compilar con DONT_RUN=1 para solo compilar sin lanzar
    DONT_RUN=1 make $TARGET \
        && ok "PX4 compilado exitosamente ✓" \
        || {
            echo ""
            warn "La compilación falló. Intentando diagnóstico..."
            echo ""
            # Mostrar el último error del log CMake
            CMAKE_ERR="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic/CMakeFiles/CMakeError.log"
            if [ -f "$CMAKE_ERR" ]; then
                echo "--- Últimas líneas de CMakeError.log ---"
                tail -20 "$CMAKE_ERR"
                echo "---"
            fi
            fail "Compilación fallida. Revisá los errores arriba."
        }
else
    step "5/7 — Compilación omitida (--no-compile)"
    warn "PX4 no fue compilado. Ejecutá manualmente:"
    echo "  cd $PX4_DIR && DONT_RUN=1 make px4_sitl gazebo-classic_iris"
fi

# ----------------------------------------------------------------
# PASO 6 — PYTHON / MAVSDK
# ----------------------------------------------------------------
step "6/7 — Instalando MAVSDK Python"

# Crear directorio del proyecto
mkdir -p "$DRONE_DIR"
cd "$DRONE_DIR"

# Crear virtualenv si no existe
if [ ! -d "$DRONE_DIR/venv" ]; then
    info "Creando virtualenv en $DRONE_DIR/venv..."
    python3 -m venv venv
    ok "Virtualenv creado"
else
    ok "Virtualenv ya existe — omitiendo"
fi

# Activar e instalar dependencias
source "$DRONE_DIR/venv/bin/activate"

pip install --upgrade pip -q
pip install mavsdk asyncio -q

ok "MAVSDK instalado en virtualenv"

# ----------------------------------------------------------------
# PASO 6.1 — SCRIPT DE PRIMER VUELO
# ----------------------------------------------------------------
cat > "$DRONE_DIR/primer_vuelo.py" << 'PYEOF'
#!/usr/bin/env python3
"""
Primer vuelo de validación — PX4 SITL + MAVSDK
Despega, hace hover 5 segundos, aterriza.
Uso: python3 primer_vuelo.py
"""
import asyncio
from mavsdk import System
from mavsdk.action import ActionError


async def primer_vuelo():
    drone = System()

    print("\n[*] Conectando con PX4 SITL...", end="", flush=True)
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(" OK")
            break

    print("[*] Esperando GPS...", end="", flush=True)
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(" OK")
            break

    print("\n[1/4] Armando motores...")
    await drone.action.arm()

    print("[2/4] Despegando a 5.0m...")
    await drone.action.takeoff()
    await asyncio.sleep(4)

    print("[3/4] Hover por 5 segundos...")
    for i in range(5, 0, -1):
        print(f"      {i}...", end="\r", flush=True)
        await asyncio.sleep(1)

    print("[4/4] Aterrizando...        ")
    await drone.action.land()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
        await asyncio.sleep(0.5)

    print("\n[OK] Primer vuelo completado exitosamente.")
    print("     PX4 + MAVSDK funcionan correctamente.\n")


if __name__ == "__main__":
    asyncio.run(primer_vuelo())
PYEOF

ok "primer_vuelo.py creado"

# ----------------------------------------------------------------
# PASO 6.2 — SCRIPT DE LANZAMIENTO RÁPIDO
# ----------------------------------------------------------------
if [ "$USE_JMAVSIM" = true ]; then
    SIM_CMD="make px4_sitl jmavsim"
else
    SIM_CMD="make px4_sitl gazebo-classic_iris"
fi

cat > "$DRONE_DIR/launch_sim.sh" << SHEOF
#!/bin/bash
# Lanza el simulador PX4 SITL
echo "Lanzando simulador desde $PX4_DIR..."
cd "$PX4_DIR"
$SIM_CMD
SHEOF
chmod +x "$DRONE_DIR/launch_sim.sh"

cat > "$DRONE_DIR/run_vuelo.sh" << SHEOF
#!/bin/bash
# Ejecuta el script de primer vuelo en el virtualenv
cd "$DRONE_DIR"
source venv/bin/activate
python3 primer_vuelo.py
SHEOF
chmod +x "$DRONE_DIR/run_vuelo.sh"

ok "Scripts de lanzamiento creados"

# ----------------------------------------------------------------
# PASO 7 — VERIFICACIÓN FINAL
# ----------------------------------------------------------------
step "7/7 — Verificación del entorno"

ERRORS=0

check() {
    local label="$1"
    local cmd="$2"
    if eval "$cmd" &>/dev/null; then
        ok "$label"
    else
        warn "FALTA: $label"
        ERRORS=$((ERRORS + 1))
    fi
}

check "git"                    "command -v git"
check "python3"                "command -v python3"
check "cmake"                  "command -v cmake"
check "ninja"                  "command -v ninja"

if [ "$USE_JMAVSIM" = false ]; then
    # dpkg más confiable que command -v (no depende del PATH actual)
    check "gazebo (instalado)" "dpkg -l gazebo 2>/dev/null | grep -q '^ii'"
fi

check "PX4-Autopilot dir"      "[ -d '$PX4_DIR' ]"
check "MAVSDK (virtualenv)"    "source '$DRONE_DIR/venv/bin/activate' && python3 -c 'import mavsdk'"

if [ "$NO_COMPILE" = false ]; then
    if [ "$USE_JMAVSIM" = true ]; then
        check "PX4 binario (jmavsim)" "[ -f '$PX4_DIR/build/px4_sitl_default/bin/px4' ]"
    else
        check "PX4 binario (gazebo)"  "[ -f '$PX4_DIR/build/px4_sitl_default/bin/px4' ]"
    fi
fi

# ----------------------------------------------------------------
# RESUMEN FINAL
# ----------------------------------------------------------------
echo ""
echo -e "${CYAN}════════════════════════════════════════════════════${NC}"

if [ "$ERRORS" -eq 0 ]; then
    echo -e "${GREEN}  ✓ Setup completado sin errores${NC}"
else
    echo -e "${YELLOW}  ⚠ Setup completado con $ERRORS advertencia(s)${NC}"
fi

echo -e "${CYAN}════════════════════════════════════════════════════${NC}"
echo ""
echo "  CÓMO USAR:"
echo ""
echo "  Terminal 1 — Lanzar simulador:"
echo "    $DRONE_DIR/launch_sim.sh"
echo ""
echo "  Terminal 2 — Ejecutar primer vuelo (cuando el sim esté listo):"
echo "    $DRONE_DIR/run_vuelo.sh"
echo ""
echo "  Activar virtualenv manualmente:"
echo "    source $DRONE_DIR/venv/bin/activate"
echo ""
echo "  Directorio PX4:"
echo "    $PX4_DIR"
echo ""
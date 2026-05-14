"""
parser_llm.py
=============
Reemplaza el ParserOrdenes de reglas por un LLM local via Ollama.

Uso standalone:
    python3 parser_llm.py

Integración en drone_director.py:
    from parser_llm import ParserLLM
    parser = ParserLLM()
    orden = await parser.parsear("subí un poco y hacé una panorámica")
"""

import asyncio
import aiohttp
import json
import re


# ------------------------------------------------------------
# CONFIGURACION
# ------------------------------------------------------------
OLLAMA_URL  = "http://172.17.16.1:11434/api/generate"
MODELO      = "qwen2.5-coder:7b"
TIMEOUT_SEG = 15

SYSTEM_PROMPT = """Sos el intérprete de órdenes de un drone filmador autónomo.
Recibís órdenes en español natural y las convertís a JSON estructurado.

Comandos válidos:
  panoramica    - vuelo 360° alrededor del área
  cenital       - subir y filmar desde arriba
  norte         - volar al norte del área
  sur           - volar al sur del área
  este          - volar al este del área
  oeste         - volar al oeste del área
  centro        - volver al centro del área
  hover         - quedarse quieto en posición actual
  subir         - ascender (puede tener número de metros)
  bajar         - descender (puede tener número de metros)
  seguir        - seguir a una persona (puede tener color de ropa)
  dejar_seguir  - dejar de seguir, modo libre
  aterrizar     - terminar sesión y aterrizar
  estado        - mostrar posición y estado actual
  escanear      - nuevo escaneo del área

Formato de respuesta — SOLO JSON, sin texto adicional, sin markdown:
{"comando": "nombre_comando", "parametros": {"numero": N, "color": "color"}}

Si no hay número, omití el campo. Si no hay color, omití el campo.
Si la orden no corresponde a ningún comando, usá "desconocido".

Ejemplos:
  "hacé una vuelta completa"   → {"comando": "panoramica", "parametros": {}}
  "subí 5 metros"              → {"comando": "subir", "parametros": {"numero": 5}}
  "seguí al de remera roja"    → {"comando": "seguir", "parametros": {"color": "roja"}}
  "quédate quieto"             → {"comando": "hover", "parametros": {}}
  "volvé al medio"             → {"comando": "centro", "parametros": {}}
"""


# ------------------------------------------------------------
# PARSER LLM
# ------------------------------------------------------------
class ParserLLM:
    def __init__(self,
                 url: str = OLLAMA_URL,
                 modelo: str = MODELO,
                 timeout: int = TIMEOUT_SEG):
        self.url    = url
        self.modelo = modelo
        self.timeout = timeout
        self._fallback = ParserFallback()

    async def parsear(self, texto: str) -> dict:
        """
        Envía el texto al LLM y retorna el comando estructurado.
        Si falla, usa el parser de reglas como fallback.
        """
        try:
            respuesta = await self._consultar_llm(texto)
            orden = self._extraer_json(respuesta)
            orden["texto"] = texto
            orden["origen"] = "llm"
            return orden

        except Exception as e:
            print(f"   [LLM] Error: {e} — usando fallback")
            orden = self._fallback.parsear(texto)
            orden["origen"] = "fallback"
            return orden

    async def _consultar_llm(self, texto: str) -> str:
        prompt = f"{SYSTEM_PROMPT}\n\nOrden del operador: \"{texto}\"\n\nJSON:"

        payload = {
            "model":  self.modelo,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": 0.1,   # baja temperatura = respuestas consistentes
                "num_predict": 100,   # máximo tokens — los JSON son cortos
            }
        }

        timeout = aiohttp.ClientTimeout(total=self.timeout)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.post(self.url, json=payload) as resp:
                if resp.status != 200:
                    raise Exception(f"HTTP {resp.status}")
                data = await resp.json()
                return data.get("response", "")

    def _extraer_json(self, texto: str) -> dict:
        """
        Extrae el JSON de la respuesta del LLM.
        Maneja casos donde el LLM agrega markdown o texto extra.
        """
        # limpiar markdown
        texto = re.sub(r"```json\s*", "", texto)
        texto = re.sub(r"```\s*", "", texto)
        texto = texto.strip()

        # buscar el primer objeto JSON válido
        # buscar JSON con llaves anidadas
        try:
            return {"comando": json.loads(texto).get("comando","desconocido"), "parametros": json.loads(texto).get("parametros",{}), "params": json.loads(texto).get("parametros",{})}
        except Exception:
            pass
        match = re.search(r"\{.*\}", texto, re.DOTALL)
        if match:
            try:
                data = json.loads(match.group())
                # normalizar estructura
                return {
                    "comando":    data.get("comando", "desconocido"),
                    "parametros": data.get("parametros", {}),
                    "params":     data.get("parametros", {}),  # alias
                }
            except json.JSONDecodeError:
                pass

        raise Exception(f"No se pudo extraer JSON de: '{texto[:80]}'")


# ------------------------------------------------------------
# FALLBACK — parser de reglas (por si Ollama no responde)
# ------------------------------------------------------------
class ParserFallback:
    COMANDOS = {
        "panoramica":  ["panoramica", "panorama", "360", "vuelta completa", "vuelta"],
        "cenital":     ["cenital", "desde arriba", "plano cenital"],
        "norte":       ["norte", "fondo"],
        "sur":         ["sur", "entrada"],
        "este":        ["este", "derecha"],
        "oeste":       ["oeste", "izquierda"],
        "centro":      ["centro", "medio", "volver", "regresa"],
        "hover":       ["hover", "queda quieto", "para", "espera", "quieto"],
        "bajar":       ["baja", "bajar", "descende", "mas bajo"],
        "subir":       ["sube", "subir", "asciende", "mas alto"],
        "aterrizar":   ["aterriza", "aterrizar", "tierra", "termina"],
        "seguir":      ["segui", "sigue", "seguir", "enfoca", "apunta"],
        "dejar_seguir":["deja de seguir", "dejar de seguir", "suelta", "libre"],
        "escanear":    ["escanea", "nuevo escaneo"],
        "estado":      ["estado", "posicion", "info", "donde"],
    }

    def parsear(self, texto: str) -> dict:
        t = texto.lower().strip()
        for cmd, kws in self.COMANDOS.items():
            for kw in kws:
                if kw in t:
                    return {"comando": cmd, "texto": texto,
                            "params": self._params(t, cmd),
                            "parametros": self._params(t, cmd)}
        return {"comando": "desconocido", "texto": texto,
                "params": {}, "parametros": {}}

    def _params(self, texto: str, comando: str) -> dict:
        params = {}
        for p in texto.split():
            if p.isdigit():
                params["numero"] = int(p)
                break
        if comando == "seguir":
            for color in ["roja","rojo","azul","verde","blanca",
                          "blanco","negro","negra","amarilla","amarillo"]:
                if color in texto:
                    params["color"] = color
                    break
        return params


# ------------------------------------------------------------
# TEST STANDALONE
# ------------------------------------------------------------
async def test():
    parser = ParserLLM()

    ordenes_prueba = [
        "hacé una vuelta completa",
        "subí 8 metros",
        "seguí al de remera azul",
        "quédate quieto ahí",
        "volvé al centro",
        "tomá una panorámica del evento",
        "aterrizá que terminamos",
        "estás bien? dónde estás?",
        "bajá un poco, está muy alto",
        "filmá desde arriba",
    ]

    print("=" * 55)
    print("  TEST PARSER LLM — qwen2.5-coder:7b")
    print("=" * 55)

    for orden_texto in ordenes_prueba:
        print(f"\n>> \"{orden_texto}\"")
        orden = await parser.parsear(orden_texto)
        origen = orden.get("origen", "?")
        print(f"   [{origen.upper()}] comando={orden['comando']} "
              f"params={orden.get('params', {})}")

    print("\n" + "=" * 55)
    print("  Test completado.")
    print("=" * 55)


if __name__ == "__main__":
    asyncio.run(test())

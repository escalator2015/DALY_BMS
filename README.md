# BMS Monitor

Cliente BLE + servidor web para baterías **Daly BMS**, ejecutándose en un **ESP32-S3** con ESP-IDF v5.x.  
Conecta al BMS por Bluetooth Low Energy, publica los datos en tiempo real mediante WebSocket y opcionalmente los envía a un servidor remoto vía HTTPS.

---

## Características

- **BLE** – conexión automática al BMS Daly por dirección MAC; reconexión automática en caso de pérdida de enlace.
- **WiFi con provisioning** – al primer arranque (o si las credenciales guardadas no funcionan) se levanta un **SoftAP** (`DALY-BMS-Setup`) con captive portal para introducir la red WiFi; las credenciales se guardan en NVS y persisten entre reinicios.
- **HTTP + WebSocket** – servidor embebido en el ESP32; la UI se sirve desde flash y se actualiza vía WS cada segundo.
- **mDNS** – accesible como `http://bms.local/` en la red local.
- **Push remoto** – envío periódico de estado (JSON) a un servidor externo por HTTPS con autenticación por API key.
- **Interfaz web local** (*BMS Monitor*) – visualización completa: voltajes por celda, balanceo, temperaturas, alarmas, estado MOS, SOC, etc. Incluye controles de MOS de carga/descarga y reset de SOC.
- **Interfaz web nube** – misma visualización en servidor externo (solo lectura, sin controles).

---

## Tramas Daly BMS soportadas

| Cmd  | Contenido                                    |
|------|----------------------------------------------|
| 0x90 | Voltaje total, corriente, SOC                |
| 0x91 | Celda máx/mín (voltaje y número)             |
| 0x92 | Temperatura máx/mín                          |
| 0x93 | Modo, MOS carga/descarga, capacidad restante |
| 0x94 | Nº celdas, sensores de temp, ciclos, estado  |
| 0x95 | Voltajes individuales por celda (hasta 48)   |
| 0x96 | Temperaturas individuales por sensor         |
| 0x97 | Estado de balanceo por celda                 |
| 0x98 | Flags de alarma (13 tipos)                   |
| 0x44 | Comando: activar/desactivar MOS de carga     |
| 0x45 | Comando: activar/desactivar MOS de descarga  |
| 0x00 | Comando: reset SOC al 100%                   |

---

## Estructura del proyecto

```
├── CMakeLists.txt
├── partitions.csv
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── main.cpp          # Lógica principal: BLE, WiFi, HTTP, WS, push remoto
│   └── bms_ui.html       # UI web embebida (servida desde flash)
├── bms/
│   ├── index.html        # UI web para servidor remoto (solo lectura)
│   ├── data.php          # Endpoint que devuelve el JSON de estado
│   └── update.php        # Endpoint que recibe el push del ESP32
└── DALY_BMS_PROTOCOL.md  # Documentación del protocolo Daly BMS
```

---

## Configuración

Edita las constantes en `main/main.cpp`:

| Constante         | Descripción                                      |
|-------------------|--------------------------------------------------|
| `BMS_MAC_ADDRESS` | Dirección MAC BLE del BMS (minúsculas)           |
| `REMOTE_PUSH_URL` | URL HTTPS del servidor de push remoto            |
| `REMOTE_API_KEY`  | API key para autenticar el push                  |
| `REMOTE_PUSH_MS`  | Intervalo del push remoto en ms (por defecto 5 s)|

Las credenciales WiFi **no se configuran en código** — se introducen en el captive portal al primer arranque.

---

## Provisioning WiFi

1. Al arrancar sin credenciales (o si la red guardada falla), el ESP32 crea la red abierta **`DALY-BMS-Setup`**.
2. Conecta desde cualquier dispositivo a esa red — el captive portal se abre automáticamente (o navega a `http://192.168.4.1/`).
3. Pulsa **Escanear redes**, selecciona tu WiFi, introduce la contraseña y pulsa **Guardar y reiniciar**.
4. El ESP32 guarda las credenciales en NVS y se reinicia en modo normal.

---

## Compilar y flashear

```bash
# Activar entorno ESP-IDF
source ~/.espressif/v5.5.3/esp-idf/export.sh

# Compilar
idf.py build

# Flashear y monitorizar
idf.py -p /dev/ttyUSB0 flash monitor
```

Target: **ESP32-S3** · ESP-IDF: **v5.5.x**

---

## Dependencias (managed components)

- `espressif/mdns`
- `h2zero/esp-nimble-cpp`

---

## Licencia

CC0-1.0 — sin restricciones de uso.

# Daly BMS — Protocolo de comunicación BLE

## Estructura de trama (13 bytes fijos)

```
Byte 0    Byte 1    Byte 2    Byte 3    Bytes 4–11   Byte 12
0xA5      address   CMD       0x08      payload[8]   checksum
```

| Campo      | Valor                               | Descripción              |
|------------|-------------------------------------|--------------------------|
| `0xA5`     | fijo                                | Start-of-frame           |
| `address`  | `0x01` host→BMS / `0x40` BMS→host   | Dirección del emisor     |
| `CMD`      | ver tabla                           | Código de comando        |
| `0x08`     | fijo                                | Longitud payload = 8     |
| `payload`  | 8 bytes                             | Datos del comando        |
| `checksum` | suma bytes 0–11 mod 256             | Verificación integridad  |

### Checksum

```c
uint8_t sum = 0;
for (int i = 0; i < 12; i++) sum += pkt[i];
pkt[12] = sum;
```

---

## Comandos de lectura (Host → BMS, luego BMS → Host)

### `0x90` — Estado básico · polling recomendado cada 1 s

| Bytes payload | Campo            | Fórmula                           | Unidad | Notas                                    |
|---------------|------------------|-----------------------------------|--------|------------------------------------------|
| [0–1]         | `total_voltage`  | raw / 10                          | V      | 0–655.3 V                                |
| [2–3]         | `accum_voltage`  | raw / 10                          | V      | Voltaje acumulado histórico              |
| [4–5]         | `current`        | (raw − 30000) / 10                | A      | Offset 30000 = cero; <0 carga, >0 descarga |
| [6–7]         | `soc`            | raw / 10                          | %      | 0.0 – 100.0 %                            |

---

### `0x91` — Tensión mínima/máxima de celda

| Bytes payload | Campo               | Fórmula    | Unidad | Notas                                |
|---------------|---------------------|------------|--------|--------------------------------------|
| [0–1]         | `max_cell_voltage`  | raw / 1000 | V      |                                      |
| [2]           | `max_cell_number`   | directo    | —      | Número de celda, 1-based             |
| [3–4]         | `min_cell_voltage`  | raw / 1000 | V      |                                      |
| [5]           | `min_cell_number`   | directo    | —      | Número de celda, 1-based             |

> Diferencia máx−mín > 50 mV indica desequilibrio relevante del pack.

---

### `0x92` — Temperatura

| Bytes payload | Campo              | Fórmula    | Unidad | Notas                     |
|---------------|--------------------|------------|--------|---------------------------|
| [0]           | `max_temperature`  | raw − 40   | °C     | raw=40 → 0 °C, raw=65 → 25 °C |
| [1]           | `max_temp_sensor`  | directo    | —      | Sensor 1-based            |
| [2]           | `min_temperature`  | raw − 40   | °C     |                           |
| [3]           | `min_temp_sensor`  | directo    | —      |                           |

---

### `0x93` — Estado del BMS

| Bytes payload | Campo                     | Valores                                | Descripción                              |
|---------------|---------------------------|----------------------------------------|------------------------------------------|
| [0]           | `charge_discharge_status` | 0=Stationary, 1=Charging, 2=Discharging | Estado de operación                     |
| [1]           | `charge_mos`              | 0=OFF, 1=ON                            | MOSFET de carga                          |
| [2]           | `discharge_mos`           | 0=OFF, 1=ON                            | MOSFET de descarga                       |
| [3]           | `bms_life`                | 0–255                                  | Contador cíclico heartbeat               |
| [4–7]         | `remaining_capacity`      | uint32 mAh                             | Capacidad restante                       |

> Si `charge_mos=OFF` y `discharge_mos=OFF` el BMS está en modo protección.

---

### `0x94` — Configuración del pack · leer al inicio

| Bytes payload | Campo              | Descripción                                    |
|---------------|--------------------|------------------------------------------------|
| [0]           | `cell_count`       | Número de celdas en serie (1–48)               |
| [1]           | `temp_sensor_count`| Número de sensores de temperatura              |
| [2]           | `charger_status`   | 0=Desconectado, 1=Conectado                    |
| [3]           | `load_status`      | 0=Desconectado, 1=Conectado                    |
| [4]           | `dio_states`       | Bits de I/O digitales (varía por modelo)       |
| [5–6]         | `charge_cycles`    | Ciclos de carga completados                    |
| [7]           | reservado          | —                                              |

> **Debe solicitarse primero** para conocer `cell_count` antes de interpretar `0x95`.

---

### `0x95` — Tensiones individuales de celda · multi-trama

Caben 3 celdas por trama (2 bytes/celda + 1 byte índice):

| Bytes payload | Campo              | Descripción                                        |
|---------------|--------------------|----------------------------------------------------|
| [0]           | `frame_number`     | Índice de trama (0-based o 1-based según firmware) |
| [1–2]         | `cell_voltage_A`   | raw / 1000 → V · celda `frame*3 + 0`              |
| [3–4]         | `cell_voltage_B`   | raw / 1000 → V · celda `frame*3 + 1`              |
| [5–6]         | `cell_voltage_C`   | raw / 1000 → V · celda `frame*3 + 2`              |
| [7]           | reservado / 0x00   | —                                                  |

- Tramas necesarias: `⌈cell_count / 3⌉`
- `0xFFFF` y `0x0000` = posición de celda no instalada (ignorar)
- El firmware detecta automáticamente si `frame_number` empieza en 0 o en 1

---

### `0x97` — Estado de balanceo de celdas

Los 8 bytes de payload son un **mapa de bits** (1 bit por celda):

```
payload[0] bits 0–7  → celdas  1–8
payload[1] bits 0–7  → celdas  9–16
payload[2] bits 0–7  → celdas 17–24
...
```

| Bit | Significado                                    |
|-----|------------------------------------------------|
| 0   | Balanceador **inactivo** (Closed)              |
| 1   | Balanceador **activo** (Open — derivando corriente) |

---

### `0x96` — Temperaturas individuales por sensor · multi-trama

Caben 7 sensores por trama:

| Bytes payload | Campo             | Descripción                                       |
|---------------|-------------------|---------------------------------------------------|
| [0]           | `frame_number`    | Índice de trama, 1-based                          |
| [1]–[7]       | `temp_sensor_N`   | raw − 40 → °C · sensores base+(0…6)               |

- Tramas necesarias: `⌈temp_sensor_count / 7⌉`
- Fórmula: `°C = raw − 40` (raw=40 → 0 °C; raw=1 → −39 °C)
- Usar `0x94.temp_sensor_count` para saber cuántos sensores existen

---

### `0x98` — Flags de alarma · polling cada 2 s

Los bytes del payload son un mapa de bits; los más relevantes:

| Byte | Bit | Campo               | Descripción                     |
|------|-----|---------------------|---------------------------------|
| [0]  | 7   | `cell_overvolt`     | Sobretensión de celda           |
| [0]  | 6   | `cell_undervolt`    | Subtensión de celda             |
| [0]  | 5   | `pack_overvolt`     | Sobretensión del pack           |
| [0]  | 4   | `pack_undervolt`    | Subtensión del pack             |
| [0]  | 3   | `chg_overtemp`      | Sobretemperatura en carga       |
| [0]  | 2   | `chg_undertemp`     | Subtemperatura en carga         |
| [0]  | 1   | `dis_overtemp`      | Sobretemperatura en descarga    |
| [0]  | 0   | `dis_undertemp`     | Subtemperatura en descarga      |
| [1]  | 7   | `chg_overcurrent`   | Sobrecorriente en carga         |
| [1]  | 6   | `dis_overcurrent`   | Sobrecorriente en descarga      |
| [1]  | 5   | `short_circuit`     | Cortocircuito                   |
| [1]  | 4   | `ic_error`          | Error interno de IC             |
| [1]  | 3   | `mos_lock`          | MOS bloqueado por software      |

> El firmware almacena los 4 bytes como `uint32_t alarm_flags` y expone cada bit por nombre en el JSON.

---

## Comandos de escritura (Host → BMS)

### `0x44` — Activar/desactivar MOSFET de carga

| Bytes payload | Campo    | Valores         | Descripción               |
|---------------|----------|-----------------|---------------------------|
| [0]           | `enable` | `0x01`=ON, `0x00`=OFF | Control MOSFET carga |
| [1–7]         | reservado | `0x00`         |                           |

Comando WebSocket: `{ "cmd": "set_charge_mos", "enable": true|false }`

---

### `0x45` — Activar/desactivar MOSFET de descarga

| Bytes payload | Campo    | Valores         | Descripción               |
|---------------|----------|-----------------|---------------------------|
| [0]           | `enable` | `0x01`=ON, `0x00`=OFF | Control MOSFET descarga |
| [1–7]         | reservado | `0x00`         |                           |

Comando WebSocket: `{ "cmd": "set_discharge_mos", "enable": true|false }`

---

### `0x00` — Reset SOC al 100%

Sin payload significativo (8 bytes a `0x00`). El BMS recalcula el SOC al 100%.

Comando WebSocket: `{ "cmd": "reset_soc" }`

> **Advertencia**: estos comandos actúan directamente sobre el hardware del BMS. Usarlos solo con intención deliberada.

---

## Flujo de polling recomendado

```
Arranque:     0x94 (config) → esperar 200 ms
              → 0x90, 0x91, 0x92, 0x93, 0x95, 0x96, 0x97, 0x98

Cada 1 s:     0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97
Cada 2 s:     0x98
```

---

## API JSON del firmware

### WebSocket `ws://<ip>/ws` — push del servidor cada 1 s

```json
{
  "t": "status",
  "ble":     { "connected": true },
  "wifi":    { "ip": "192.168.1.100" },
  "basic":   { "v": 48.5, "i": -12.3, "soc": 87.4 },
  "minmax":  { "max_v": 4.125, "max_n": 3, "min_v": 4.098, "min_n": 7, "diff_mv": 27 },
  "temp":    { "max": 28.0, "max_s": 1, "min": 25.0, "min_s": 2 },
  "state":   { "mode": 0, "mode_str": "Stationary", "c_mos": true, "d_mos": true, "rem_mah": 45000 },
  "cfg":     { "cells": 16, "temps": 2, "charger": false, "load": true, "cycles": 42 },
  "voltages": [4.125, 4.120, 4.110, 4.098, ...],
  "cell_temps": [28.0, 25.5],
  "balance":  [false, false, true, false, ...],
  "alarms": {
    "flags": 0,
    "cell_overvolt": false, "cell_undervolt": false,
    "pack_overvolt": false, "pack_undervolt": false,
    "chg_overtemp": false, "chg_undertemp": false,
    "dis_overtemp": false, "dis_undertemp": false,
    "chg_overcurrent": false, "dis_overcurrent": false,
    "short_circuit": false, "ic_error": false, "mos_lock": false
  },
  "uptime":  3600,
  "heap":    180000
}
```

### WebSocket — comandos del cliente al servidor

```json
{ "cmd": "connect" }
{ "cmd": "disconnect" }
{ "cmd": "scan" }
{ "cmd": "request", "code": 144 }
{ "cmd": "set_charge_mos",    "enable": true }
{ "cmd": "set_discharge_mos", "enable": false }
{ "cmd": "reset_soc" }
```

> `code` es el byte del comando en decimal: 0x90=144, 0x91=145, 0x95=149, 0x97=151, etc.

### HTTP GET `/` — sirve el frontend HTML

### Campos JSON: referencia rápida

| Campo JSON        | Fuente  | Unidad  | Notas                              |
|-------------------|---------|---------|------------------------------------|
| `basic.v`         | 0x90[0–1] | V     | Tensión total del pack             |
| `basic.i`         | 0x90[4–5] | A     | Negativo = cargando                |
| `basic.soc`       | 0x90[6–7] | %     | Estado de carga                    |
| `minmax.max_v`    | 0x91[0–1] | V     | Celda más cargada                  |
| `minmax.min_v`    | 0x91[3–4] | V     | Celda menos cargada                |
| `minmax.diff_mv`  | calculado | mV    | Desequilibrio del pack             |
| `temp.max`        | 0x92[0]   | °C    | Temperatura máxima                 |
| `state.mode`      | 0x93[0]   | —     | 0=reposo, 1=cargando, 2=descargando |
| `state.c_mos`     | 0x93[1]   | bool  | MOSFET carga activo                |
| `state.d_mos`     | 0x93[2]   | bool  | MOSFET descarga activo             |
| `state.rem_mah`   | 0x93[4–7] | mAh   | Capacidad restante                 |
| `cfg.cells`       | 0x94[0]   | —     | Número de celdas                   |
| `cfg.cycles`      | 0x94[5–6] | —     | Ciclos de carga                    |
| `voltages[i]`     | 0x95      | V     | Tensión celda i+1                  |
| `balance[i]`      | 0x97      | bool  | true = balanceador activo celda i+1 |

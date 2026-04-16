# Acceso remoto al ESP32-S3 via Synology Reverse Proxy

## Requisitos previos
- NAS Synology DS218+ con DSM 7.x
- Acceso admin al router
- El ESP32 tiene IP fija (o reserva DHCP) en la red local

---

## Paso 1 — IP fija para el ESP32 (ZTE ZXHN H3600P)

Para que el reverse proxy siempre apunte al mismo destino, reserva la IP del ESP32 en el router:

1. Accede a `http://192.168.1.1` (credenciales en la etiqueta del router, normalmente `admin` / `admin` o las impresas)
2. Ve a **Configuración avanzada → Red → LAN → Configuración DHCP**
3. Baja hasta **Asignación de IP estática** (Static IP Assignment)
4. Haz clic en **Añadir** e introduce:
   - MAC: `dc:b4:d9:0b:52:98`
   - IP: `192.168.1.157`
5. Guarda y reinicia el ESP32 para confirmar que mantiene la IP

> Si no encuentras esa ruta exacta, busca el menú **DHCP** dentro de **Red Local** o **LAN**.

---

## Paso 2 — DDNS Synology

1. En DSM ve a **Control Panel → External Access → DDNS**
2. Haz clic en **Add**
3. Selecciona proveedor: **Synology**
4. Elige un hostname, por ejemplo: `mi-bms.synology.me`
5. Haz clic en **Test Connection** → debe aparecer **Normal**
6. Guarda

> Synology actualizará automáticamente el DDNS cada vez que cambie tu IP pública.

---

## Paso 3 — Certificado SSL (Let's Encrypt)

Para que el acceso sea HTTPS sin advertencias del navegador:

1. Ve a **Control Panel → Security → Certificate**
2. Haz clic en **Add → Add a new certificate**
3. Selecciona **Get a certificate from Let's Encrypt**
4. Rellena:
   - Domain name: `mi-bms.synology.me`
   - Email: tu correo
5. Haz clic en **Apply**

> Synology renueva el certificado automáticamente cada 90 días.

---

## Paso 4 — Abrir puerto en el router (ZTE ZXHN H3600P)

El router debe redirigir el tráfico HTTPS externo al NAS:

| Puerto externo | IP destino        | Puerto destino |
|:--------------:|:-----------------:|:--------------:|
| 443            | IP del NAS (ej. `192.168.1.X`) | 443 |

1. Accede a `http://192.168.1.1`
2. Ve a **Configuración avanzada → Seguridad → NAT → Reenvío de puertos** (puede aparecer como **Virtual Server** o **Port Forwarding**)
3. Haz clic en **Añadir** y rellena:
   - Nombre: `NAS-HTTPS`
   - Protocolo: **TCP**
   - Puerto externo inicio / fin: `443`
   - IP destino: IP de tu NAS Synology (ej. `192.168.1.X`)
   - Puerto interno: `443`
4. Guarda y aplica

> **No** es necesario abrir el puerto 80 del ESP32 al exterior; el NAS lo gestiona internamente.

> ⚠️ Algunos routers proporcionados por operadoras tienen el **NAT de doble capa (CGNAT)** activo, lo que impide el acceso externo aunque configures el reenvío correctamente. Si tras seguir todos los pasos la conexión externa no funciona, llama a tu operadora y pide que te asignen **IP pública directa** (sin CGNAT).

---

## Paso 5 — Reverse Proxy en Synology

1. Ve a **Control Panel → Login Portal → Advanced → Reverse Proxy**
2. Haz clic en **Create**
3. Rellena la pestaña **General**:

   | Campo              | Valor                         |
   |--------------------|-------------------------------|
   | Description        | `ESP32 BMS`                   |
   | Source — Protocol  | `HTTPS`                       |
   | Source — Hostname  | `mi-bms.synology.me`          |
   | Source — Port      | `443`                         |
   | Destination — Protocol | `HTTP`                    |
   | Destination — Hostname | `192.168.1.157`           |
   | Destination — Port | `80`                          |

4. Pestaña **Custom Header** → haz clic en **Create → WebSocket**
   > Esto añade los headers `Upgrade` y `Connection` necesarios para que el WebSocket `/ws` funcione a través del proxy.

5. Haz clic en **Save**

---

## Paso 6 — Asignar el certificado al proxy

1. Ve a **Control Panel → Security → Certificate**
2. Haz clic en **Settings**
3. Busca la entrada `mi-bms.synology.me` (reverse proxy) y asígnale el certificado Let's Encrypt creado en el Paso 3
4. Haz clic en **OK**

---

## Paso 7 — Verificación

Desde un dispositivo **fuera de tu red** (datos móviles, por ejemplo):

```
https://mi-bms.synology.me
```

Deberías ver la página del BMS con candado verde.

Para probar el WebSocket desde la consola del navegador:
```javascript
var ws = new WebSocket("wss://mi-bms.synology.me/ws");
ws.onopen = () => console.log("WS OK");
ws.onmessage = e => console.log(e.data);
```

---

## Diagrama de flujo

```
Navegador externo
       │
       │ HTTPS :443
       ▼
mi-bms.synology.me  ←── Synology DDNS (actualiza IP dinámica)
       │
       │ Port Forward :443 → NAS
       ▼
Synology DS218+  (Reverse Proxy + SSL termination)
       │
       │ HTTP :80 (red local)
       ▼
ESP32-S3  192.168.1.157
```

---

## Solución de problemas

| Síntoma | Causa probable | Solución |
|---------|---------------|----------|
| `ERR_CONNECTION_REFUSED` | Puerto 443 no redirigido | Revisar Port Forwarding en el router |
| Advertencia de certificado | Certificado no asignado al proxy | Repetir Paso 6 |
| Página carga pero WebSocket falla | Falta header WebSocket | Repetir Paso 5, pestaña Custom Header |
| DDNS no resuelve | IP pública no actualizada | DSM → External Access → DDNS → Update Now |

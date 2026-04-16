<?php
/**
 * update.php — Recibe el JSON del ESP32-S3 y lo guarda en disco.
 * Subir a: https://luaresearch.es/bms/update.php
 */

define('API_KEY',   'c864645245ca1dcd5d6a0d6ca3754512c26a7a07ff722c853d1587b18bf1916f');   // igual que en main.cpp
define('DATA_FILE', __DIR__ . '/bms_data.json');
define('MAX_BODY',  8192);  // bytes máximos aceptados

/* ---- Sólo POST ---- */
if ($_SERVER['REQUEST_METHOD'] !== 'POST') {
    http_response_code(405);
    exit;
}

/* ---- Validar API key ---- */
$headers = array_change_key_case(getallheaders(), CASE_LOWER);
$key     = $headers['x-api-key'] ?? '';
if ($key !== API_KEY) {
    http_response_code(403);
    exit('Forbidden');
}

/* ---- Leer body con límite ---- */
$body = file_get_contents('php://input', false, null, 0, MAX_BODY);
if (!$body) {
    http_response_code(400);
    exit('Empty body');
}

$data = json_decode($body, true);
if (!is_array($data)) {
    http_response_code(400);
    exit('Bad JSON');
}

/* ---- Añadir timestamp del servidor ---- */
$data['server_ts'] = time();

/* ---- Escritura atómica (tmp + rename) ---- */
$tmp = DATA_FILE . '.tmp';
if (file_put_contents($tmp, json_encode($data, JSON_UNESCAPED_UNICODE), LOCK_EX) !== false) {
    rename($tmp, DATA_FILE);
    http_response_code(200);
    echo 'OK';
} else {
    http_response_code(500);
    echo 'Write error';
}

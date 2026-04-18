<?php
/**
 * data.php — Sirve el último JSON al browser.
 * No requiere autenticación (sólo lectura de datos públicos del BMS).
 * Subir a: https://luaresearch.es/bms/data.php
 */

define('DATA_FILE', __DIR__ . '/bms_data.json');
define('STALE_SEC', 30);   // considerar datos obsoletos si son > 30 s

header('Content-Type: application/json; charset=utf-8');
header('Cache-Control: no-store');

if (!file_exists(DATA_FILE)) {
    echo '{"error":"no_data"}';
    exit;
}

$age  = time() - filemtime(DATA_FILE);
$json = file_get_contents(DATA_FILE);
$data = json_decode($json, true);

if (!is_array($data)) {
    http_response_code(500);
    echo '{"error":"corrupt"}';
    exit;
}

$data['stale'] = ($age > STALE_SEC);
$data['age_s'] = $age;

echo json_encode($data, JSON_UNESCAPED_UNICODE);

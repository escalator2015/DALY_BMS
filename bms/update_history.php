<?php
/**
 * update_history.php — Recibe una muestra histórica del ESP32-S3 (cada 5 min)
 * y la añade al archivo JSONL del año correspondiente.
 * Subir a: https://luaresearch.es/bms/update_history.php
 */

define('API_KEY',  'c864645245ca1dcd5d6a0d6ca3754512c26a7a07ff722c853d1587b18bf1916f');
define('DATA_DIR', __DIR__ . '/');
define('MAX_BODY', 256);

/* ---- Sólo POST ---- */
if ($_SERVER['REQUEST_METHOD'] !== 'POST') {
    http_response_code(405);
    exit;
}

/* ---- Validar API key ---- */
$headers = array_change_key_case(getallheaders(), CASE_LOWER);
if (($headers['x-api-key'] ?? '') !== API_KEY) {
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
if (!is_array($data) || !isset($data['ts'], $data['v'], $data['i'])) {
    http_response_code(400);
    exit('Bad JSON');
}

$ts = (int)$data['ts'];
$v  = (float)$data['v'];
$i  = (float)$data['i'];

/* ---- Validar rangos razonables ---- */
if ($ts < 1577836800 || $v < 0 || $v > 1000 || $i < -1000 || $i > 1000) {
    http_response_code(400);
    exit('Out of range');
}

/* ---- Determinar año y archivo destino ---- */
$year = (int)date('Y', $ts);
$file = DATA_DIR . 'hist_' . $year . '.jsonl';

/* ---- Append atómico con lock ---- */
$line = json_encode(['ts' => $ts, 'v' => $v, 'i' => $i]) . "\n";
$fp = fopen($file, 'a');
if ($fp) {
    flock($fp, LOCK_EX);
    fwrite($fp, $line);
    flock($fp, LOCK_UN);
    fclose($fp);
    http_response_code(200);
    echo 'OK';
} else {
    http_response_code(500);
    echo 'Write error';
}

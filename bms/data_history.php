<?php
/**
 * data_history.php — Sirve datos históricos agregados por vista temporal.
 * Parámetros GET:
 *   view  = day | week | month | year  (defecto: day)
 *   ts    = Unix timestamp inicio del período  (defecto: inicio del día UTC)
 *
 * Responde JSON idéntico al endpoint /history del ESP32:
 * { "view": "...", "ts_start": N, "bucket_s": N, "buckets": [{ts,v,i,p}, ...] }
 */

define('DATA_DIR', __DIR__ . '/');

$view     = $_GET['view'] ?? 'day';
$ts_start = (int)($_GET['ts'] ?? 0);

if (!in_array($view, ['day', 'week', 'month', 'year'], true)) {
    $view = 'day';
}

/* ---- Configuración de buckets según vista ---- */
switch ($view) {
    case 'week':
        $bucket_s  = 3600;
        $n_buckets = 168;
        $period_s  = 7 * 86400;
        break;
    case 'month':
        $bucket_s  = 86400;
        $n_buckets = 31;
        $period_s  = 31 * 86400;
        break;
    case 'year':
        $bucket_s  = 2635200;
        $n_buckets = 12;
        $period_s  = 366 * 86400;
        break;
    default: /* day */
        $bucket_s  = 300;
        $n_buckets = 288;
        $period_s  = 86400;
        break;
}

if ($ts_start === 0) {
    /* Por defecto: inicio del día actual UTC */
    $ts_start = mktime(0, 0, 0, (int)gmdate('n'), (int)gmdate('j'), (int)gmdate('Y'));
}

$ts_end = $ts_start + $period_s;

/* ---- Inicializar buckets ---- */
$buckets = array_fill(0, $n_buckets, ['v_sum' => 0.0, 'i_sum' => 0.0, 'count' => 0]);

/* ---- Leer archivos de los años que cubre el rango ---- */
$year1 = (int)gmdate('Y', $ts_start);
$year2 = (int)gmdate('Y', $ts_end - 1);

for ($year = $year1; $year <= $year2; $year++) {
    $file = DATA_DIR . 'hist_' . $year . '.jsonl';
    if (!file_exists($file)) {
        continue;
    }
    $fp = fopen($file, 'r');
    if (!$fp) {
        continue;
    }
    while (($line = fgets($fp)) !== false) {
        $rec = json_decode($line, true);
        if (!$rec || !isset($rec['ts'])) {
            continue;
        }
        $rts = (int)$rec['ts'];
        if ($rts < $ts_start || $rts >= $ts_end) {
            continue;
        }
        $bidx = (int)(($rts - $ts_start) / $bucket_s);
        if ($bidx < 0 || $bidx >= $n_buckets) {
            continue;
        }
        $buckets[$bidx]['v_sum'] += (float)$rec['v'];
        $buckets[$bidx]['i_sum'] += (float)$rec['i'];
        $buckets[$bidx]['count']++;
    }
    fclose($fp);
}

/* ---- Construir respuesta ---- */
$out_buckets = [];
for ($b = 0; $b < $n_buckets; $b++) {
    $cnt = $buckets[$b]['count'];
    if ($cnt === 0) {
        continue;
    }
    $avg_v = $buckets[$b]['v_sum'] / $cnt;
    $avg_i = $buckets[$b]['i_sum'] / $cnt;
    $out_buckets[] = [
        'ts' => $ts_start + $b * $bucket_s,
        'v'  => round($avg_v, 2),
        'i'  => round($avg_i, 2),
        'p'  => round($avg_v * $avg_i, 1),
    ];
}

$result = [
    'view'     => $view,
    'ts_start' => $ts_start,
    'bucket_s' => $bucket_s,
    'buckets'  => $out_buckets,
];

header('Content-Type: application/json');
header('Cache-Control: no-cache, no-store');
echo json_encode($result, JSON_UNESCAPED_UNICODE);

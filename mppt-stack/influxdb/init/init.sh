#!/bin/sh
HOST="http://influxdb:8086"
ORG="mppt"
TOK="$INFLUX_TOKEN"

echo "[init] Creating bucket solar_history (ignoring if already exists)..."
influx bucket create \
  --name solar_history \
  --org "$ORG" \
  --retention 0 \
  --token "$TOK" \
  --host "$HOST" 2>&1 | grep -v "already exists" || true

echo "[init] Done."

# MPPT Solar Monitor - Cloud MING Stack

Containerised monitoring solution for Phocos MPPT solar charge controllers, based on **Mosquitto - InfluxDB - Node-RED - Grafana**.

## Services

| Service | Port | Purpose |
|---|---|---|
| Mosquitto | 1883 | MQTT broker - ingests data from `mppt_live` |
| InfluxDB | 8086 | Time-series storage |
| Node-RED | 1880 | Routes MQTT messages into InfluxDB |
| Grafana | 3000 | Dashboard visualisation |
| MQTT Explorer | 4000 | Browser-based MQTT debug tool |

## Quick Start

**1. Create a `.env` file** in this directory:

```bash
INFLUX_ADMIN_PASSWORD=your_strong_password
INFLUX_TOKEN=your_token_for_node_red
GRAFANA_ADMIN_PASSWORD=your_grafana_password
SMTP_SERVER=smtp.example.com
SMTP_PORT=587
SMTP_USER=user@example.com
SMTP_PASS=password
ALERT_EMAIL=alerts@example.com
```

**2. Start all services:**

```bash
docker-compose up -d
```

**3. Access the UIs:**

- Grafana: http://localhost:3000 (user: `admin`)
- Node-RED: http://localhost:1880
- InfluxDB: http://localhost:8086
- MQTT Explorer: http://localhost:4000

## Data Flow

`mppt_live` publishes to MQTT -> Node-RED writes to InfluxDB -> Grafana queries InfluxDB.

| MQTT topic | InfluxDB bucket | Retention |
|---|---|---|
| `mppt/{zone}/{gateway}/{serial}/state` | `solar` | infinite |
| `mppt/{zone}/{gateway}/{serial}/datalog` | `solar_history` | infinite |

Topic segments map to InfluxDB tags for filtering by site, gateway, and device serial.

## Provisioning

- **Grafana** dashboards are pre-provisioned from `grafana/provisioning/dashboards/`.
- **InfluxDB** buckets and access policies are initialised by `influxdb/init/init.sh` on first start.
- **Node-RED** flow data is persisted in `nodered/data/`.

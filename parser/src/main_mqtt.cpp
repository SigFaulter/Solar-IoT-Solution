/**
 * Log file parser with MQTT publish
 *
 * Usage:
 *   ./build/mppt_mqtt <logfile.txt>
 */

#include "space_parser.h"
#include "eeprom_parser.h"
#include "printer.h"
#include "json_builder.h"
#include "mqtt_client.h"
#include "utils.h"
#include "lookups.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>


static void printUsage(const char* prog) {
    fprintf(stderr, "Usage: %s <logfile.txt> [--zone <z>] [--broker <h>] [--mqtt-port <n>]\n", prog);
}


int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return EXIT_FAILURE;
    }

    const char* log_path    = argv[1];
    std::string zone        = "default";
    std::string broker_host = "localhost";
    int         broker_port = 1883;
    std::string gateway     = hostname();

    for (int i = 2; i < argc; ++i) {
        if      (strcmp(argv[i], "--zone")      == 0 && i + 1 < argc) zone        = argv[++i];
        else if (strcmp(argv[i], "--broker")    == 0 && i + 1 < argc) broker_host = argv[++i];
        else if (strcmp(argv[i], "--mqtt-port") == 0 && i + 1 < argc) broker_port = atoi(argv[++i]);
        else { fprintf(stderr, "Unknown option '%s'\n", argv[i]); printUsage(argv[0]); return EXIT_FAILURE; }
    }

    std::ifstream file(log_path);
    if (!file.is_open()) {
        fprintf(stderr, "Error: cannot open '%s'\n", log_path);
        return EXIT_FAILURE;
    }

    PhocosTelemetry         tele{};
    EepromConfig            cfg{};
    DataloggerSummary       summary{};
    std::vector<DailyLog>   daily_logs;
    std::vector<MonthlyLog> monthly_logs;

    bool have_tele   = false;
    bool have_eeprom = false;

    int lines_total   = 0;
    int records_ok    = 0;
    int records_fail  = 0;

    std::string line;
    while (std::getline(file, line)) {
        ++lines_total;
        if (!line.empty() && line.back() == '\r') line.pop_back();

        if (isEepromLine(line)) {
            std::string dump = line.substr(1);
            if (parseEepromDump(dump.c_str(), dump.size(),
                                cfg, summary, daily_logs, monthly_logs)) {
                have_eeprom = true;
            }
            continue;
        }
        if (!isSpaceLine(line)) { continue; }

        tele = PhocosTelemetry{};
        if (parsePhocosLine(line.c_str(), line.size(), tele)) {
            ++records_ok;
            have_tele = true;
        } else {
            ++records_fail;
            fprintf(stderr, "PARSE FAILED (line %d, len=%zu): %.100s...\n",
                    lines_total, line.size(), line.c_str());
        }
    }

    if (!have_tele && !have_eeprom) {
        fprintf(stderr, "No parseable data found in '%s'\n", log_path);
        return EXIT_FAILURE;
    }

    cfg.hw_version = resolveHwVersion(have_eeprom, cfg.hw_version, have_tele, tele.hw_version);

    char ts[32];
    currentTimestamp(ts, sizeof(ts));

    if (have_tele)   { printSystemState(tele, cfg, ts); }
    if (have_eeprom) { printEepromConfig(cfg); }
    if (have_eeprom) { printDataLogger(summary, daily_logs, monthly_logs); }

    const std::string serial =
        !cfg.serial_number.empty() ? cfg.serial_number : "unknown";

    //   mppt/{zone}/{gateway}/{serial}/online   LWT retained
    //   mppt/{zone}/{gateway}/{serial}/state    telemetry  QoS 0 (fire and forget)
    //   mppt/{zone}/{gateway}/{serial}/datalog  EEPROM log QoS 1 + retained (once/day)
    const std::string base_topic = "mppt/" + zone + "/" + gateway + "/" + serial;

    MqttConfig mqtt_cfg;
    mqtt_cfg.host        = broker_host;
    mqtt_cfg.port        = broker_port;
    mqtt_cfg.client_id   = "mppt_" + gateway + "_" + serial;
    mqtt_cfg.lwt_topic   = base_topic + "/online";
    mqtt_cfg.lwt_payload = "0";
    mqtt_cfg.lwt_retain  = true;

    fprintf(stderr, "\n[mqtt] broker   = %s:%d\n",   broker_host.c_str(), broker_port);
    fprintf(stderr, "[mqtt] zone     = %s\n",         zone.c_str());
    fprintf(stderr, "[mqtt] serial   = %s\n",         serial.c_str());
    fprintf(stderr, "[mqtt] base     = %s\n",         base_topic.c_str());
    fprintf(stderr, "[mqtt] lwt      = %s/online -> \"0\"\n", base_topic.c_str());

    MqttClient client(mqtt_cfg);
    bool connected = client.connect();

    if (connected) {
        // Mark device online (retained, broker auto-publishes to new subscribers)
        client.publish(base_topic + "/online", "1", /*retain=*/true);

        //  1: Telemetry (always, QoS 0)
        if (have_tele) {
            const nlohmann::json tele_json = buildTelemetryJSON(tele, cfg, ts);
            const std::string tele_payload = tele_json.dump();

            fprintf(stderr, "[mqtt] telemetry  %zu bytes -> %s/state\n", tele_payload.size(), base_topic.c_str());

            if (client.publish(base_topic + "/state", tele_payload, /*retain=*/false, /*qos=*/0)) {
                fprintf(stderr, "[mqtt] state OK\n");
            } else {
                fprintf(stderr, "[mqtt] state FAILED\n");
            }
        }

        // 2: Datalogger (only when EEPROM available, QoS 1 + retained)
        if (have_eeprom) {
            const nlohmann::json dl_json = buildDataloggerJSON(cfg, summary, daily_logs, monthly_logs, ts);
            const std::string dl_payload = dl_json.dump();

            fprintf(stderr, "[mqtt] datalogger %zu bytes -> %s/datalog\n", dl_payload.size(), base_topic.c_str());

            if (client.publish(base_topic + "/datalog", dl_payload, /*retain=*/true, /*qos=*/1)) {
                fprintf(stderr, "[mqtt] datalog OK\n");
            } else {
                fprintf(stderr, "[mqtt] datalog FAILED\n");
            }
        }

        client.disconnect();
    } else {
        fprintf(stderr, "[mqtt] connection failed - skipping publish\n");
    }

    printf("\n--------------------------------------------\n");
    printf("  Lines read   : %d\n", lines_total);
    printf("  Records OK   : %d\n", records_ok);
    printf("  Records FAIL : %d\n", records_fail);
    printf("--------------------------------------------\n");

    return (records_fail == 0 && connected) ? EXIT_SUCCESS : EXIT_FAILURE;
}

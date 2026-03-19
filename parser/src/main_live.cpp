/**
 * mppt_live - Live RS485 MPPT poller + settings read/write
 *
 * Usage:
 *   ./mppt_live <port> [options]
 *
 * Poll mode (default - runs once, use --loop for continuous):
 *   ./mppt_live /dev/ttyUSB0 --loop --zone site1 --broker 192.168.1.10
 *
 * Settings read:
 *   ./mppt_live /dev/ttyUSB0 --settings
 *
 * Settings write:
 *   ./mppt_live /dev/ttyUSB0 --settings --set night_mode=1 --set evening_min=120
 *
 * Options:
 *   --loop              poll every 30 s continuously (poll mode only)
 *   --zone <z>          MQTT zone segment          (default: "default")
 *   --broker <host>     MQTT broker hostname        (default: "localhost")
 *   --mqtt-port <n>     MQTT broker port            (default: 1883)
 *   --settings          enter settings mode instead of poll mode
 *   --set key=value     patch a setting (requires --settings)
 *
 * Setting keys:
 *   battery_type     0-2   (V2: 0=AGM 1=Liquid 2=LFP; V3: LFP variants)
 *   capacity_ah      1-500
 *   lvd_mv           5000-15000
 *   night_thresh_mv  4000-14000
 *   night_mode       0-3   (0=Off 1=D2D 2=DD 3=MN)
 *   evening_min      0-600
 *   morning_min      0-600
 *   dim_mode         0-3
 *   dim_evening_min  0-600
 *   dim_morning_min  0-600
 *   dimming_pct      0-100
 *   base_dimming_pct 0-100
 *   dali             0|1
 *   alc              0|1
 */

#include "serial.h"
#include "space_parser.h"
#include "eeprom_parser.h"
#include "settings_parser.h"
#include "printer.h"
#include "json_builder.h"
#include "mqtt_client.h"
#include "utils.h"
#include "lookups.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <unistd.h>


static constexpr int TIMEOUT_SPACE_MS    =  500;
static constexpr int TIMEOUT_EEPROM_MS   = 2000;
static constexpr int POLL_INTERVAL_S     =   30;


struct PollResult {
    PhocosTelemetry         tele{};
    EepromConfig            cfg{};
    DataloggerSummary       summary{};
    std::vector<DailyLog>   daily_logs;
    std::vector<MonthlyLog> monthly_logs;
    bool have_tele   = false;
    bool have_eeprom = false;
};

static PollResult pollDevice(int fd) {
    PollResult r;
    std::string line;

    // Drain then flush - handles both USB driver buffering and any partial
    // response left from the previous poll cycle.
    drainUntilQuiet(fd, 80);
    tcflush(fd, TCIFLUSH);

    // Space command - controller sends two '\n' terminated lines
    if (!sendCommand(fd, ' ')) {
        fprintf(stderr, "[poll] failed to send Space command\n");
        return r;
    }

    if (!readLine(fd, line, TIMEOUT_SPACE_MS)) {
        fprintf(stderr, "[poll] no response to Space command (timeout %d ms)\n", TIMEOUT_SPACE_MS);
        return r;
    }

    if (isSpaceLine(line)) {
        if (parsePhocosLine(line.c_str(), line.size(), r.tele)) {
            r.have_tele = true;
            fprintf(stderr, "[poll] Space OK  (%zu bytes, V%d)\n", line.size(), r.tele.hw_version);
        } else {
            fprintf(stderr, "[poll] Space PARSE FAILED: %.80s\n", line.c_str());
        }
    } else {
        fprintf(stderr, "[poll] unexpected Space response (first byte 0x%02X): %.40s\n", (unsigned char)line[0], line.c_str());
    }

    { std::string _unused; readLine(fd, _unused, 200); }

    // EEPROM command
    if (!sendCommand(fd, '!')) {
        fprintf(stderr, "[poll] failed to send EEPROM command\n");
        return r;
    }

    if (!readLine(fd, line, TIMEOUT_EEPROM_MS)) {
        fprintf(stderr, "[poll] no response to EEPROM command (timeout %d ms)\n", TIMEOUT_EEPROM_MS);
        return r;
    }

    if (isEepromLine(line)) {
        std::string dump = line.substr(1);
        if (parseEepromDump(dump.c_str(), dump.size(), r.cfg, r.summary, r.daily_logs, r.monthly_logs)) {
            r.have_eeprom = true;
            fprintf(stderr, "[poll] EEPROM OK (%zu bytes)\n", line.size());
        } else {
            fprintf(stderr, "[poll] EEPROM PARSE FAILED\n");
        }
    } else {
        fprintf(stderr, "[poll] unexpected EEPROM response (first byte 0x%02X)\n", (unsigned char)line[0]);
    }

    return r;
}

static void printAndPublishPoll(const PollResult& r, MqttClient& mqtt,
                                const std::string& base_topic, const char* ts) {
    if (r.have_tele)   { printSystemState(r.tele, r.cfg, ts); }
    if (r.have_eeprom) { printEepromConfig(r.cfg); }
    if (r.have_eeprom) { printDataLogger(r.summary, r.daily_logs, r.monthly_logs); }

    if (r.have_tele) {
        const std::string payload = buildTelemetryJSON(r.tele, r.cfg, ts).dump();
        printf("%s\n", payload.c_str());
        if (mqtt.connected()) {
            mqtt.publish(base_topic + "/state", payload);
            fprintf(stderr, "[mqtt] -> state (%zu B)\n", payload.size());
        }
    }
    if (r.have_eeprom) {
        const std::string payload = buildDataloggerJSON(r.cfg, r.summary, r.daily_logs, r.monthly_logs, ts).dump();
        printf("%s\n", payload.c_str());
        if (mqtt.connected()) {
            mqtt.publish(base_topic + "/datalog", payload, /*retain=*/true, /*qos=*/1);
            fprintf(stderr, "[mqtt] -> datalog (%zu B)\n", payload.size());
        }
    }
}

static int runPollMode(int fd, bool loop_mode,
                       const std::string& zone, const std::string& broker_host,
                       int broker_port) {
    const std::string gateway = hostname();

    // First poll discovers the serial number needed for the MQTT LWT topic.
    PollResult first = pollDevice(fd);
    if (!first.have_tele && !first.have_eeprom) {
        fprintf(stderr, "[poll] no data on first poll - aborting\n");
        return EXIT_FAILURE;
    }
    first.cfg.hw_version = resolveHwVersion(first.have_eeprom, first.cfg.hw_version, first.have_tele, first.tele.hw_version);

    const std::string serial     = !first.cfg.serial_number.empty() ? first.cfg.serial_number : "unknown";
    const std::string base_topic = "mppt/" + zone + "/" + gateway + "/" + serial;

    MqttConfig mqtt_cfg;
    mqtt_cfg.host        = broker_host;
    mqtt_cfg.port        = broker_port;
    mqtt_cfg.client_id   = "mppt_live_" + gateway + "_" + serial;
    mqtt_cfg.lwt_topic   = base_topic + "/online";
    mqtt_cfg.lwt_payload = "0";
    mqtt_cfg.lwt_retain  = true;

    MqttClient mqtt(mqtt_cfg);
    if (mqtt.connect()) {
        mqtt.publish(base_topic + "/online", "1", /*retain=*/true);
        fprintf(stderr, "[mqtt] connected - %s\n", base_topic.c_str());
    } else {
        fprintf(stderr, "[mqtt] broker unreachable - continuing without MQTT\n");
    }

    char ts[32];
    currentTimestamp(ts, sizeof(ts));
    printAndPublishPoll(first, mqtt, base_topic, ts);

    if (!loop_mode) {
        mqtt.disconnect();
        return EXIT_SUCCESS;
    }

    while (true) {
        sleep(POLL_INTERVAL_S);
        fprintf(stderr, "\n[poll] -- polling -----------------------\n");
        PollResult r = pollDevice(fd);
        if (!r.have_tele && !r.have_eeprom) {
            fprintf(stderr, "[poll] no usable data - skipping\n");
            continue;
        }
        r.cfg.hw_version = resolveHwVersion(r.have_eeprom, r.cfg.hw_version, r.have_tele, r.tele.hw_version);
        currentTimestamp(ts, sizeof(ts));
        printAndPublishPoll(r, mqtt, base_topic, ts);
    }

    mqtt.disconnect();
    return EXIT_SUCCESS;
}


struct SettingsContext {
    std::vector<uint8_t> settings_bytes;
    EepromConfig         cfg;
    PhocosTelemetry      tele;
    int                  hw_version = 3;
    int                  load_state = -1;
    bool                 have_tele   = false;
    bool                 have_eeprom = false;
    bool                 ok          = false;
};

static SettingsContext readSettingsFromDevice(int fd) {
    SettingsContext ctx;
    std::string line;

    // 1. Space - hw_version + load_state
    if (!sendCommand(fd, ' ')) {
        fprintf(stderr, "[settings] failed to send Space command\n");
        return ctx;
    }
    if (!readLine(fd, line, TIMEOUT_SPACE_MS)) {
        fprintf(stderr, "[settings] no response to Space command\n");
        return ctx;
    }
    if (!isSpaceLine(line)) {
        fprintf(stderr, "[settings] unexpected Space response (0x%02X): %.40s\n",
                (unsigned char)line[0], line.c_str());
        return ctx;
    }
    if (!parsePhocosLine(line.c_str(), line.size(), ctx.tele)) {
        fprintf(stderr, "[settings] Space PARSE FAILED: %.80s\n", line.c_str());
        return ctx;
    }
    ctx.have_tele  = true;
    ctx.hw_version = ctx.tele.hw_version;
    ctx.load_state = ctx.tele.load_state_raw;
    fprintf(stderr, "[settings] Space OK (V%d, load_state=0x%02X)\n",
            ctx.hw_version, ctx.load_state);

    { std::string _unused; readLine(fd, _unused, 200); }

    // 2. EEPROM dump - provides both config and the raw bytes for settings writes
    if (!sendCommand(fd, '!')) {
        fprintf(stderr, "[settings] failed to send EEPROM command\n");
        return ctx;
    }
    if (!readLine(fd, line, TIMEOUT_EEPROM_MS)) {
        fprintf(stderr, "[settings] no response to EEPROM command\n");
        return ctx;
    }
    if (!isEepromLine(line)) {
        fprintf(stderr, "[settings] unexpected EEPROM response (0x%02X): %.40s\n",
                (unsigned char)line[0], line.c_str());
        return ctx;
    }

    {
        std::string dump = line.substr(1);
        DataloggerSummary sum;
        std::vector<DailyLog>   dl;
        std::vector<MonthlyLog> ml;
        if (!parseEepromDumpRaw(dump.c_str(), dump.size(),
                                ctx.cfg, sum, dl, ml, ctx.settings_bytes)) {
            fprintf(stderr, "[settings] EEPROM PARSE FAILED\n");
            return ctx;
        }
        ctx.have_eeprom = true;
        if (ctx.cfg.hw_version != 0)
            ctx.hw_version = ctx.cfg.hw_version;
        fprintf(stderr, "[settings] EEPROM OK (serial: %s, %zu bytes)\n",
                ctx.cfg.serial_number.c_str(), ctx.settings_bytes.size());
    }

    if ((int)ctx.settings_bytes.size() < 99) {
        fprintf(stderr, "[settings] EEPROM dump too short for settings window: %zu bytes\n", ctx.settings_bytes.size());
        return ctx;
    }

    ctx.ok = true;
    return ctx;
}

static void hexByte(char* out, uint8_t v) {
    static const char hex[] = "0123456789ABCDEF";
    out[0] = hex[v >> 4];
    out[1] = hex[v & 0xF];
}

static bool writeSettingsToDevice(int fd,
                                  const std::vector<uint8_t>& old_bytes,
                                  const std::vector<uint8_t>& new_bytes) {
    auto eepromAddr = [](int idx) -> uint16_t {
        return static_cast<uint16_t>(0x1000 + idx);
    };

    struct Write1 { int idx; uint8_t val; };
    struct Write2 { int idx; uint16_t val; };

    static const int m_fields[][2] = {
        {W(SR_EVENING_MIN),     W(SR_EVENING_MIN)     + 1},
        {W(SR_MORNING_MIN),     W(SR_MORNING_MIN)     + 1},
        {W(SR_LVD_CURRENT_MV),  W(SR_LVD_CURRENT_MV)  + 1},
        {W(SR_LVD_VOLTAGE_MV),  W(SR_LVD_VOLTAGE_MV)  + 1},
        {W(SR_DIM_EVENING_MIN), W(SR_DIM_EVENING_MIN) + 1},
        {W(SR_DIM_MORNING_MIN), W(SR_DIM_MORNING_MIN) + 1},
        {W(SR_NIGHT_THRESH_MV), W(SR_NIGHT_THRESH_MV) + 1},
        {W(SR_CAPACITY_AH),     W(SR_CAPACITY_AH)     + 1},
    };
    static const int h_fields[] = {
        W(SR_NIGHT_MODE_IDX), W(SR_DIM_MODE_IDX), W(SR_LVD_MODE_VOLTAGE),
        W(SR_BATTERY_TYPE_IDX), W(SR_DIMMING_PCT), W(SR_BASE_DIMMING_PCT),
        W(SR_DALI_ENABLE), W(SR_ALC_DIMMING),
    };

    std::vector<Write1> writes1;
    std::vector<Write2> writes2;

    for (auto& mf : m_fields) {
        int msb = mf[0], lsb = mf[1];
        if (new_bytes[msb] != old_bytes[msb] || new_bytes[lsb] != old_bytes[lsb])
            writes2.push_back({msb, (uint16_t)(((uint16_t)new_bytes[msb] << 8) | new_bytes[lsb])});
    }
    for (int idx : h_fields) {
        if (new_bytes[idx] != old_bytes[idx])
            writes1.push_back({idx, new_bytes[idx]});
    }

    if (writes1.empty() && writes2.empty()) {
        fprintf(stderr, "[settings] no changes - nothing to write\n");
        return true;
    }
    fprintf(stderr, "[settings] %zu byte-fields, %zu word-fields changed\n",
            writes1.size(), writes2.size());

    drainUntilQuiet(fd, 80);

    if (!sendAmpersandCommand(fd, "&GAA0900")) {
        fprintf(stderr, "[settings] &G command failed\n");
        return false;
    }
    usleep(50'000);

    char cmd[11];
    for (const auto& w : writes1) {
        uint16_t addr = eepromAddr(w.idx);
        cmd[0] = '&'; cmd[1] = 'H';
        hexByte(cmd + 2, (addr >> 8) & 0xFF);
        hexByte(cmd + 4,  addr       & 0xFF);
        hexByte(cmd + 6, w.val);
        cmd[8] = '\0';
        fprintf(stderr, "[settings] &H EEPROM[0x%04X] = 0x%02X\n", addr, w.val);
        if (!sendAmpersandCommand(fd, cmd)) {
            fprintf(stderr, "[settings] &H failed at offset %d\n", w.idx);
            return false;
        }
        usleep(10'000);
    }
    for (const auto& w : writes2) {
        uint16_t addr = eepromAddr(w.idx);
        cmd[0] = '&'; cmd[1] = 'M';
        hexByte(cmd + 2, (addr >> 8) & 0xFF);
        hexByte(cmd + 4,  addr       & 0xFF);
        hexByte(cmd + 6, (w.val >> 8)  & 0xFF);   // MSB at lower address
        hexByte(cmd + 8,  w.val        & 0xFF);   // LSB at higher address
        cmd[10] = '\0';
        fprintf(stderr, "[settings] &M EEPROM[0x%04X] = 0x%04X\n", addr, w.val);
        if (!sendAmpersandCommand(fd, cmd)) {
            fprintf(stderr, "[settings] &M failed at offset %d\n", w.idx);
            return false;
        }
        usleep(10'000);
    }

    fprintf(stderr, "[settings] write complete\n");
    return true;
}

static int runSettingsMode(int fd, const std::vector<std::string>& patches,
                           const std::string& zone, const std::string& broker_host,
                           int broker_port) {
    SettingsContext ctx = readSettingsFromDevice(fd);
    if (!ctx.ok) return EXIT_FAILURE;

    DeviceSettings current = deviceSettingsFromEeprom(ctx.cfg, ctx.load_state);

    printf("[CURRENT SETTINGS]\n");
    printSettings(current);

    const std::string serial = !ctx.cfg.serial_number.empty() ? ctx.cfg.serial_number : "unknown";
    const std::string gateway    = hostname();
    const std::string base_topic = "mppt/" + zone + "/" + gateway + "/" + serial;

    MqttConfig mqtt_cfg;
    mqtt_cfg.host      = broker_host;
    mqtt_cfg.port      = broker_port;
    mqtt_cfg.client_id = "mppt_settings_" + gateway + "_" + serial;

    MqttClient mqtt(mqtt_cfg);
    if (mqtt.connect()) {
        fprintf(stderr, "[mqtt] connected - %s\n", base_topic.c_str());
        char ts[32]; currentTimestamp(ts, sizeof(ts));
        std::string p = buildSettingsJSON(current, ts).dump();
        mqtt.publish(base_topic + "/settings", p, /*retain=*/true, /*qos=*/1);
        fprintf(stderr, "[mqtt] -> settings (%zu B)\n", p.size());
    } else {
        fprintf(stderr, "[mqtt] broker unreachable - continuing without MQTT\n");
    }

    if (patches.empty()) {
        mqtt.disconnect();
        return EXIT_SUCCESS;
    }

    DeviceSettings updated = current;
    for (const auto& kv : patches)
        if (!applySettingsPatch(kv, updated)) {
            mqtt.disconnect();
            return EXIT_FAILURE;
        }

    printf("[UPDATED SETTINGS]\n");
    printSettings(updated);

    std::vector<uint8_t> new_bytes;
    patchSettingsBytes(ctx.settings_bytes, updated, new_bytes);

    usleep(50'000);
    bool written = writeSettingsToDevice(fd, ctx.settings_bytes, new_bytes);

    if (written && mqtt.connected()) {
        // Re-poll telemetry so state topic reflects new config
        usleep(200'000);
        drainUntilQuiet(fd, 80);
        std::string line;
        if (sendCommand(fd, ' ') && readLine(fd, line, TIMEOUT_SPACE_MS)
                && isSpaceLine(line)) {
            PhocosTelemetry tele{};
            if (parsePhocosLine(line.c_str(), line.size(), tele)) {
                char ts[32]; currentTimestamp(ts, sizeof(ts));
                std::string p = buildTelemetryJSON(tele, ctx.cfg, ts).dump();
                mqtt.publish(base_topic + "/state", p);
                fprintf(stderr, "[mqtt] -> state (%zu B)\n", p.size());
            }
        }
        { std::string _unused; readLine(fd, _unused, 200); }

        char ts[32]; currentTimestamp(ts, sizeof(ts));
        std::string p = buildSettingsJSON(updated, ts).dump();
        mqtt.publish(base_topic + "/settings", p, /*retain=*/true, /*qos=*/1);
        fprintf(stderr, "[mqtt] -> settings updated (%zu B)\n", p.size());
    }

    mqtt.disconnect();
    return written ? EXIT_SUCCESS : EXIT_FAILURE;
}



static void usage(const char* prog) {
    fprintf(stderr,
        "Usage: %s <port> [options]\n"
        "\n"
        "Poll mode (default):\n"
        "  %s <port> [--loop] [--zone <z>] [--broker <h>] [--mqtt-port <n>]\n"
        "\n"
        "Settings mode:\n"
        "  %s <port> --settings [--set key=val ...] [--zone <z>] [--broker <h>]\n"
        "\n"
        "Setting keys: battery_type, capacity_ah, lvd_mv, night_thresh_mv,\n"
        "  night_mode, evening_min, morning_min, dim_mode, dim_evening_min,\n"
        "  dim_morning_min, dimming_pct, base_dimming_pct, dali, alc\n",
        prog, prog, prog);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    const char* port      = argv[1];
    bool        loop_mode     = false;
    bool        settings_mode = false;
    std::string zone        = "default";
    std::string broker_host = "localhost";
    int         broker_port = 1883;
    std::vector<std::string> patches;

    for (int i = 2; i < argc; ++i) {
        if      (strcmp(argv[i], "--loop")      == 0) loop_mode     = true;
        else if (strcmp(argv[i], "--settings")  == 0) settings_mode = true;
        else if (strcmp(argv[i], "--set")       == 0 && i + 1 < argc) patches.push_back(argv[++i]);
        else if (strcmp(argv[i], "--zone")      == 0 && i + 1 < argc) zone        = argv[++i];
        else if (strcmp(argv[i], "--broker")    == 0 && i + 1 < argc) broker_host = argv[++i];
        else if (strcmp(argv[i], "--mqtt-port") == 0 && i + 1 < argc) broker_port = atoi(argv[++i]);
        else { fprintf(stderr, "[main] unknown option '%s'\n", argv[i]); usage(argv[0]); return EXIT_FAILURE; }
    }

    int fd = openSerial(port);
    if (fd < 0) return EXIT_FAILURE;
    fprintf(stderr, "[main] opened %s %s\n", port,
            settings_mode ? " (settings mode)" : loop_mode ? " (loop mode)" : "");

    // Drain bytes buffered by the USB driver from any previous process.
    drainUntilQuiet(fd, 150);

    int rc;
    if (settings_mode)
        rc = runSettingsMode(fd, patches, zone, broker_host, broker_port);
    else
        rc = runPollMode(fd, loop_mode, zone, broker_host, broker_port);

    close(fd);
    return rc;
}

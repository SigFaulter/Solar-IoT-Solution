#include "space_parser.h"
#include "eeprom_parser.h"
#include "printer.h"
#include "json_builder.h"
#include "utils.h"
#include "lookups.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>


int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <logfile.txt> [--json]\n", argv[0]);
        return EXIT_FAILURE;
    }

    bool print_json = false;
    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "--json") == 0) print_json = true;
        else { fprintf(stderr, "Unknown option '%s'\n", argv[i]); return EXIT_FAILURE; }
    }

    std::ifstream file(argv[1]);
    if (!file.is_open()) {
        fprintf(stderr, "Error: cannot open '%s'\n", argv[1]);
        return EXIT_FAILURE;
    }

    PhocosTelemetry         tele{};
    EepromConfig            cfg{};
    DataloggerSummary       summary{};
    std::vector<DailyLog>   daily_logs;
    std::vector<MonthlyLog> monthly_logs;

    bool have_tele     = false;
    bool have_eeprom   = false;

    int lines_total   = 0;
    int records_ok    = 0;
    int records_fail  = 0;
    std::string line;

    while (std::getline(file, line)) {
        ++lines_total;

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        // EEPROM dump line (starts with '!')
        if (isEepromLine(line)) {
            std::string dump = line.substr(1);
            if (parseEepromDump(dump.c_str(), dump.size(), cfg, summary, daily_logs, monthly_logs)) {
                have_eeprom = true;
            }
            continue;
        }

        // Skip anything that doesn't look like a Space line
        if (!isSpaceLine(line)) {
            continue;
        }

        // Space Command response
        tele = PhocosTelemetry{};
        if (parsePhocosLine(line.c_str(), line.size(), tele)) {
            ++records_ok;
            have_tele = true;
        } else {
            ++records_fail;
            fprintf(stderr, "PARSE FAILED (line %d, len=%zu): %.100s...\n", lines_total, line.size(), line.c_str());
        }
    }

    if (!have_tele && !have_eeprom) {
        fprintf(stderr, "No parseable data found in '%s'\n", argv[1]);
        return EXIT_FAILURE;
    }

    char ts[32]; currentTimestamp(ts, sizeof(ts));

    cfg.hw_version = resolveHwVersion(have_eeprom, cfg.hw_version, have_tele, tele.hw_version);

    if (have_tele)     { printSystemState(tele, cfg, ts); }
    if (have_eeprom)   { printEepromConfig(cfg); }
    if (have_eeprom)   { printDataLogger(summary, daily_logs, monthly_logs); }

    if (print_json) {
        if (have_tele)
            printf("%s\n", buildTelemetryJSON(tele, cfg, ts).dump(2).c_str());
        if (have_eeprom)
            printf("%s\n", buildDataloggerJSON(cfg, summary, daily_logs, monthly_logs, ts).dump(2).c_str());
    }

    printf("\n--------------------------------------------\n");
    printf("  Lines read   : %d\n", lines_total);
    printf("  Records OK   : %d\n", records_ok);
    printf("  Records FAIL : %d\n", records_fail);
    printf("--------------------------------------------\n");

    return (records_fail == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

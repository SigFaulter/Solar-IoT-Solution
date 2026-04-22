#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "eeprom_parser.h"
#include "json_builder.h"
#include "lookups.h"
#include "printer.h"
#include "space_parser.h"
#include "utils.h"

namespace fs = std::filesystem;

struct ParseStats {
    int lines_total  = 0;
    int records_ok   = 0;
    int records_fail = 0;
};

ParseStats parse_file(const std::string &logfile_path,
                      bool               print_json,
                      bool               aggregate,
                      JsonScalingFormat  scaling_format) {
    ParseStats    stats{};
    std::ifstream file(logfile_path);

    if (!file.is_open()) {
        std::cerr << "Error: cannot open '" << logfile_path << "'\n";
        return stats;
    }

    PhocosTelemetry   tele{};
    EepromSettings    cfg{};
    DataloggerSummary summary{};
    DailyLogBuffer    daily_logs;
    MonthlyLogBuffer  monthly_logs;

    bool have_tele   = false;
    bool have_eeprom = false;

    std::string line;
    while (std::getline(file, line)) {
        ++stats.lines_total;

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (is_eeprom_line(line)) {
            if (parse_eeprom_dump(line.substr(1), cfg, summary, daily_logs, monthly_logs)) {
                have_eeprom = true;
            }
            continue;
        }

        if (!is_space_line(line)) {
            continue;
        }

        PhocosTelemetry current_tele{};
        if (parse_phocos_line(line, current_tele)) {
            tele = current_tele;
            ++stats.records_ok;
            have_tele = true;
        } else {
            ++stats.records_fail;
            if (!aggregate) {
                std::cerr << "PARSE FAILED (line " << stats.lines_total << ", len=" << line.size()
                          << "): " << (line.size() > 100 ? line.substr(0, 100) : line) << "...\n";
            }
        }
    }

    if (!have_tele && !have_eeprom) {
        if (!aggregate)
            std::cerr << "No parseable data found in '" << logfile_path << "'\n";
        return stats;
    }

    std::time_t ts = current_timestamp();
    cfg.hw_version = resolve_hw_version(have_eeprom, cfg.hw_version, have_tele, tele.hw_version);

    if (aggregate) {
        if (have_tele) {
            std::cout << build_telemetry_json(tele, cfg, ts, scaling_format).dump(2);
            if (have_eeprom)
                std::cout << "," << std::endl;
        }
        if (have_eeprom) {
            std::cout << build_datalogger_json(
                             cfg, summary, daily_logs, monthly_logs, ts, scaling_format)
                             .dump(2);
        }
    } else {
        if (have_tele) {
            print_system_state(tele, cfg, ts);
        }
        if (have_eeprom) {
            print_eeprom_config(cfg);
            print_data_logger(summary, daily_logs, monthly_logs);
        }

        if (print_json) {
            if (have_tele) {
                std::cout << build_telemetry_json(tele, cfg, ts, scaling_format).dump(2) << "\n";
            }
            if (have_eeprom) {
                std::cout << build_datalogger_json(
                                 cfg, summary, daily_logs, monthly_logs, ts, scaling_format)
                                 .dump(2)
                          << "\n";
            }
        }
    }
    return stats;
}

auto main(int argc, char *argv[]) -> int {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " (<logfile.txt> | --log-dir <dir>) [--json [--raw | --scaled]]\n";
        return EXIT_FAILURE;
    }

    bool              print_json     = false;
    JsonScalingFormat scaling_format = JsonScalingFormat::SCALED;
    std::string       logfile_path;
    std::string       log_dir_path;

    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }

    for (size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--json") {
            print_json = true;
        } else if (args[i] == "--raw") {
            scaling_format = JsonScalingFormat::RAW;
        } else if (args[i] == "--scaled") {
            scaling_format = JsonScalingFormat::SCALED;
        } else if (args[i] == "--log-dir") {
            if (i + 1 < args.size()) {
                log_dir_path = args[++i];
            } else {
                std::cerr << "Error: --log-dir requires a path\n";
                return EXIT_FAILURE;
            }
        } else if (logfile_path.empty()) {
            logfile_path = args[i];
        } else {
            std::cerr << "Unknown option or extra argument '" << args[i] << "'\n";
            return EXIT_FAILURE;
        }
    }

    if (!log_dir_path.empty()) {
        if (!fs::exists(log_dir_path) || !fs::is_directory(log_dir_path)) {
            std::cerr << "Error: Invalid directory '" << log_dir_path << "'\n";
            return EXIT_FAILURE;
        }
        std::cout << "[" << '\n';
        bool       first = true;
        ParseStats total_stats{};
        for (const auto &entry : fs::recursive_directory_iterator(log_dir_path)) {
            if (entry.path().extension() == ".log") {
                if (!first) {
                    std::cout << "," << '\n';
                }
                ParseStats s = parse_file(entry.path().string(), print_json, true, scaling_format);
                total_stats.lines_total += s.lines_total;
                total_stats.records_ok += s.records_ok;
                total_stats.records_fail += s.records_fail;
                first = false;
            }
        }
        std::cout << "]" << '\n';
        std::cout << "\n--------------------------------------------\n"
                  << "  Lines read   : " << total_stats.lines_total << "\n"
                  << "  Records OK   : " << total_stats.records_ok << "\n"
                  << "  Records FAIL : " << total_stats.records_fail << "\n"
                  << "--------------------------------------------\n";
    } else if (!logfile_path.empty()) {
        ParseStats s = parse_file(logfile_path, print_json, false, scaling_format);
        std::cout << "\n--------------------------------------------\n"
                  << "  Lines read   : " << s.lines_total << "\n"
                  << "  Records OK   : " << s.records_ok << "\n"
                  << "  Records FAIL : " << s.records_fail << "\n"
                  << "--------------------------------------------\n";
    } else {
        std::cerr << "Error: No logfile or directory provided\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

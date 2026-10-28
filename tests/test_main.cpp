#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <utility>

#include "test.hpp"

std::vector<TestCase>& testRegistry() {
    static std::vector<TestCase> tests;
    return tests;
}

static long long g_checks = 0;
static std::vector<std::pair<std::string, std::string>> g_metrics;

void countCheck() { ++g_checks; }
void metric(const std::string& key, const std::string& value) { g_metrics.emplace_back(key, value); }
std::string validationDir() { return "validation"; }

int main() {
    std::filesystem::create_directories("validation");
    int passed = 0, failed = 0;
    for (const auto& t : testRegistry()) {
        const auto start = std::chrono::steady_clock::now();
        try {
            t.fn();
            ++passed;
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            std::printf("[PASS] %-44s %8.1f ms\n", t.name, ms);
        } catch (const std::exception& e) {
            ++failed;
            std::printf("[FAIL] %s\n       %s\n", t.name, e.what());
        }
    }

    std::printf("\n%d/%d tests passed, %lld checks\n", passed, passed + failed, g_checks);
    std::ofstream out(validationDir() + "/metrics.txt");
    out << "tests_passed = " << passed << "/" << (passed + failed) << "\nchecks = " << g_checks << "\n";
    std::printf("\nMetrics\n");
    for (const auto& [k, v] : g_metrics) {
        std::printf("  %-48s %s\n", k.c_str(), v.c_str());
        out << k << " = " << v << "\n";
    }
    return failed == 0 ? 0 : 1;
}

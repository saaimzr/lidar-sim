// Minimal test harness (no third-party framework).
//
//   TEST(name) { ... }            registers a test
//   CHECK(cond)                   fails the test if cond is false
//   CHECK_NEAR(a, b, tol)         fails if |a - b| > tol, printing both values
//   metric("key", value)          records a measured result; all metrics are printed at the end
//                                 and written to validation/metrics.txt
#pragma once

#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct TestCase {
    const char* name;
    void (*fn)();
};
std::vector<TestCase>& testRegistry();
struct TestRegistrar {
    TestRegistrar(const char* name, void (*fn)()) { testRegistry().push_back({name, fn}); }
};

struct CheckFailure : std::runtime_error {
    using std::runtime_error::runtime_error;
};
void countCheck();

#define TEST(name)                                          \
    static void name();                                     \
    static TestRegistrar name##_registrar(#name, name);     \
    static void name()

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        countCheck();                                                                                \
        if (!(cond))                                                                                 \
            throw CheckFailure(std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #cond); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                      \
    do {                                                                                           \
        countCheck();                                                                              \
        const double a_ = (a), b_ = (b);                                                           \
        if (!(std::fabs(a_ - b_) <= (tol))) {                                                      \
            std::ostringstream os_;                                                                \
            os_.precision(10);                                                                     \
            os_ << __FILE__ << ":" << __LINE__ << ": |" #a " - " #b "| = |" << a_ << " - " << b_ \
                << "| > " << (tol);                                                                \
            throw CheckFailure(os_.str());                                                         \
        }                                                                                          \
    } while (0)

void metric(const std::string& key, const std::string& value);
inline void metric(const std::string& key, double value) {
    std::ostringstream os;
    os.precision(6);
    os << value;
    metric(key, os.str());
}
inline void metric(const std::string& key, long long value) { metric(key, std::to_string(value)); }

// Directory for CSV outputs used by tools/plot_validation.py.
std::string validationDir();

// Stable 64-bit FNV-1a hash of raw bytes (used to compare scans across thread counts).
inline uint64_t fnv1a(const void* data, size_t n, uint64_t h = 1469598103934665603ull) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 1099511628211ull;
    return h;
}

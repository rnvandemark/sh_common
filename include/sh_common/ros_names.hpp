#pragma once

#include <vector>
#include <string>
#include <iterator>
#include <numeric>

#define DECL(name) const static std::string name

namespace sh { namespace names {
    class params {
    public:
        DECL(HEARTBEAT_PERIOD_MS);
    };
}}

#undef DECL

namespace {
    std::string acc(const std::vector<std::string>& in) {
        if (in.empty()) {
            return "";
        }
        return std::accumulate(
            std::next(in.begin()),
            in.end(),
            in[0],
            [](const std::string& a, const std::string& b) {
                return std::move(a) + '/' + b;
            }
        );
    }
}

#define DEFI(name, components) const std::string sh::names::name = acc(components)

DEFI(params::HEARTBEAT_PERIOD_MS, {"heartbeat_period_ms"});

#undef DEFI

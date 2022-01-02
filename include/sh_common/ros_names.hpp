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

    class topics {
    public:
        // Heartbeat
        DECL(HEARTBEAT_PREFIX);
        // Mode change
        DECL(REQUESTED_MODE_CHANGES);
        DECL(CONFIRMED_MODE_CHANGES);
        // Color peak calculation
        DECL(SCC_CAMERA_IMAGE);
        DECL(LEFT_COLOR_PEAK);
        DECL(RIGHT_COLOR_PEAK);
        DECL(COLOR_PEAKS_TELEM);
        // Sound file playback
        DECL(REQUESTED_PLAYBACK_FILES);
        DECL(PLAYBACK_COMMANDS);
        DECL(PLAYBACK_UPDATES);
        DECL(PLAYBACK_FREQUENCIES);
        // Individual control mode
        DECL(INTENSITY_CHANGE_UPDATES);
        // Countdown mode
        DECL(COUNTDOWN_STATE_UPDATES);
        // Wave mode
        DECL(START_WAVE_MODE);
        DECL(WAVE_PARTICIPANT_LOCATION);
        DECL(WAVE_UPDATES);
    };

    class services {
    public:
        // Screen color coordinator (scc)
        DECL(REQUEST_SCREEN_COLOR_CALIBRTION);
        DECL(SET_SCREEN_COLOR_HOMOG_POINTS);
    };

    class actions {
    public:
        // Sound file playback (sfp)
        DECL(DOWNLOAD_AUDIO);
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

#define DEFI(name, value) const std::string sh::names::name = value

// Parameters
DEFI(params::HEARTBEAT_PERIOD_MS, acc({"heartbeat_period_ms"}));

// Topics
DEFI(topics::HEARTBEAT_PREFIX, acc({"heartbeat"}));
DEFI(topics::REQUESTED_MODE_CHANGES, acc({"mode_change", "requested"}));
DEFI(topics::CONFIRMED_MODE_CHANGES, acc({"mode_change", "confirmed"}));
DEFI(topics::SCC_CAMERA_IMAGE, acc({"scc_image"}));
DEFI(topics::LEFT_COLOR_PEAK, acc({"color_peaks", "left"}));
DEFI(topics::RIGHT_COLOR_PEAK, acc({"color_peaks", "right"}));
DEFI(topics::COLOR_PEAKS_TELEM, acc({"color_peaks", "telem"}));
DEFI(topics::REQUESTED_PLAYBACK_FILES, acc({"sound_file_playback", "requested"}));
DEFI(topics::PLAYBACK_COMMANDS, acc({"sound_file_playback", "commands"}));
DEFI(topics::PLAYBACK_UPDATES, acc({"sound_file_playback", "telem"}));
DEFI(topics::PLAYBACK_FREQUENCIES, acc({"sound_file_playback", "frequencies"}));
DEFI(topics::INTENSITY_CHANGE_UPDATES, acc({"device_intensities"}));
DEFI(topics::COUNTDOWN_STATE_UPDATES, acc({"countdown_state"}));
DEFI(topics::START_WAVE_MODE, acc({"wave", "requests"}));
DEFI(topics::WAVE_PARTICIPANT_LOCATION, acc({"wave", "participant_locations"}));
DEFI(topics::WAVE_UPDATES, acc({"wave", "position"}));

// Services
DEFI(services::REQUEST_SCREEN_COLOR_CALIBRTION, acc({"screen_color_coordination", "request_calibration"}));
DEFI(services::SET_SCREEN_COLOR_HOMOG_POINTS, acc({"screen_color_coordination", "set_homog_points"}));

// Actions
DEFI(actions::DOWNLOAD_AUDIO, acc({"sound_file_playback", "download_audio"}));

#undef DEFI

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
        DECL(PLAYBACK_BEGIN);
        DECL(PLAYBACK_UPDATES_VERBOSE);
        DECL(PLAYBACK_STATUS);
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
        // Sound file playback (sfp)
        DECL(PLAYBACK_COMMANDS);
    };

    class actions {
    public:
        // Sound file playback (sfp)
        DECL(DOWNLOAD_AUDIO);
        DECL(REQUEST_PLAY_SOUND_FILE);
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
DEFI(topics::PLAYBACK_BEGIN, acc({"sound_file_playback", "begin"}));
DEFI(topics::PLAYBACK_UPDATES_VERBOSE, acc({"sound_file_playback", "verbose_updates"}));
DEFI(topics::INTENSITY_CHANGE_UPDATES, acc({"device_intensities"}));
DEFI(topics::COUNTDOWN_STATE_UPDATES, acc({"countdown_state"}));
DEFI(topics::START_WAVE_MODE, acc({"wave", "requests"}));
DEFI(topics::WAVE_PARTICIPANT_LOCATION, acc({"wave", "participant_locations"}));
DEFI(topics::WAVE_UPDATES, acc({"wave", "position"}));

// Services
DEFI(services::PLAYBACK_COMMANDS, acc({"sound_file_playback", "request_command"}));

// Actions
DEFI(actions::DOWNLOAD_AUDIO, acc({"sound_file_playback", "download_audio"}));
DEFI(actions::REQUEST_PLAY_SOUND_FILE, acc({"sound_file_playback", "request_play_sound_file"}));

// Special topics
DEFI(topics::PLAYBACK_STATUS, acc({sh::names::actions::REQUEST_PLAY_SOUND_FILE, "_action", "status"}));

#undef DEFI

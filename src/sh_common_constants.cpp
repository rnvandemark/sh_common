#include "sh_common/ros_names.hpp"

#include <boost/python.hpp>

namespace bp = boost::python;
namespace sn = sh::names;

BOOST_PYTHON_MODULE(sh_common_constants) {
    // Add parameters
    {
        bp::scope sc = bp::class_<sn::params>("params", bp::no_init);
        sc.attr("HEARTBEAT_PERIOD_MS") = sn::params::HEARTBEAT_PERIOD_MS;
        sc.attr("FIND_HTTP_VERSION") = sn::params::FIND_HTTP_VERSION;
        sc.attr("FIND_SERVER_HOST") = sn::params::FIND_SERVER_HOST;
        sc.attr("FIND_SERVER_PORT") = sn::params::FIND_SERVER_PORT;
        sc.attr("FIND_SERVER_NODE_NAME") = sn::params::FIND_SERVER_NODE_NAME;
    }

    // Add topics
    {
        bp::scope sc = bp::class_<sn::topics>("topics", bp::no_init);
        sc.attr("HEARTBEAT_PREFIX") = sn::topics::HEARTBEAT_PREFIX;
        sc.attr("REQUESTED_MODE_CHANGES") = sn::topics::REQUESTED_MODE_CHANGES;
        sc.attr("CONFIRMED_MODE_CHANGES") = sn::topics::CONFIRMED_MODE_CHANGES;
        sc.attr("SCC_CAMERA_IMAGE") = sn::topics::SCC_CAMERA_IMAGE;
        sc.attr("LEFT_COLOR_PEAK") = sn::topics::LEFT_COLOR_PEAK;
        sc.attr("RIGHT_COLOR_PEAK") = sn::topics::RIGHT_COLOR_PEAK;
        sc.attr("COLOR_PEAKS_TELEM") = sn::topics::COLOR_PEAKS_TELEM;
        sc.attr("PLAYBACK_UPDATES_VERBOSE") = sn::topics::PLAYBACK_UPDATES_VERBOSE;
        sc.attr("PLAYBACK_AUDIO_CHARACTERISTICS") = sn::topics::PLAYBACK_AUDIO_CHARACTERISTICS;
        sc.attr("PLAYBACK_STATUS") = sn::topics::PLAYBACK_STATUS;
        sc.attr("INTENSITY_CHANGE_UPDATES") = sn::topics::INTENSITY_CHANGE_UPDATES;
        sc.attr("COUNTDOWN_STATE_UPDATES") = sn::topics::COUNTDOWN_STATE_UPDATES;
        sc.attr("START_WAVE_MODE") = sn::topics::START_WAVE_MODE;
        sc.attr("WAVE_PARTICIPANT_LOCATION") = sn::topics::WAVE_PARTICIPANT_LOCATION;
        sc.attr("WAVE_UPDATES") = sn::topics::WAVE_UPDATES;
    }

    // Add services
    {
        bp::scope sc = bp::class_<sn::services>("services", bp::no_init);
        sc.attr("PLAYBACK_COMMANDS") = sn::services::PLAYBACK_COMMANDS;
    }

    // Add actions
    {
        bp::scope sc = bp::class_<sn::actions>("actions", bp::no_init);
        sc.attr("DOWNLOAD_AUDIO") = sn::actions::DOWNLOAD_AUDIO;
        sc.attr("REQUEST_PLAY_SOUND_FILE") = sn::actions::REQUEST_PLAY_SOUND_FILE;
        sc.attr("ANALYZE_SOUND_FILE") = sn::actions::ANALYZE_SOUND_FILE;
    }
}

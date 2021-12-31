#include "sh_common/ros_names.hpp"

#include <boost/python.hpp>

namespace bp = boost::python;
namespace sn = sh::names;

BOOST_PYTHON_MODULE(sh_common_py) {
    // Add parameters
    {
        bp::scope sc = bp::class_<sn::params>("params", bp::no_init);
        sc.attr("HEARTBEAT_PERIOD_MS") = sn::params::HEARTBEAT_PERIOD_MS;
    }

    // Add topics
    {
        bp::scope sc = bp::class_<sn::topics>("topics", bp::no_init);
        sc.attr("HEARTBEAT_SUFFIX") = sn::topics::HEARTBEAT_SUFFIX;
        sc.attr("REQUESTED_MODE_CHANGES") = sn::topics::REQUESTED_MODE_CHANGES;
        sc.attr("CONFIRMED_MODE_CHANGES") = sn::topics::CONFIRMED_MODE_CHANGES;
        sc.attr("LEFT_COLOR_PEAK") = sn::topics::LEFT_COLOR_PEAK;
        sc.attr("RIGHT_COLOR_PEAK") = sn::topics::RIGHT_COLOR_PEAK;
        sc.attr("COLOR_PEAKS_TELEM") = sn::topics::COLOR_PEAKS_TELEM;
        sc.attr("REQUESTED_PLAYBACK_FILES") = sn::topics::REQUESTED_PLAYBACK_FILES;
        sc.attr("PLAYBACK_COMMANDS") = sn::topics::PLAYBACK_COMMANDS;
        sc.attr("PLAYBACK_UPDATES") = sn::topics::PLAYBACK_UPDATES;
        sc.attr("PLAYBACK_FREQUENCIES") = sn::topics::PLAYBACK_FREQUENCIES;
        sc.attr("INTENSITY_CHANGE_UPDATES") = sn::topics::INTENSITY_CHANGE_UPDATES;
        sc.attr("COUNTDOWN_STATE_UPDATES") = sn::topics::COUNTDOWN_STATE_UPDATES;
        sc.attr("START_WAVE_MODE") = sn::topics::START_WAVE_MODE;
        sc.attr("WAVE_PARTICIPANT_LOCATION") = sn::topics::WAVE_PARTICIPANT_LOCATION;
        sc.attr("WAVE_UPDATES") = sn::topics::WAVE_UPDATES;
    }

    // Add services
    {
        bp::scope sc = bp::class_<sn::services>("services", bp::no_init);
        sc.attr("REQUEST_SCREEN_COLOR_CALIBRTION") = sn::services::REQUEST_SCREEN_COLOR_CALIBRTION;
        sc.attr("SET_SCREEN_COLOR_HOMOG_POINTS") = sn::services::SET_SCREEN_COLOR_HOMOG_POINTS;
    }
}

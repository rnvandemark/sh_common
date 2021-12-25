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
}

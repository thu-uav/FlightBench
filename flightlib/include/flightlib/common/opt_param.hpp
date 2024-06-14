#ifndef OPT_PARAM_HPP
#define OPT_PARAM_HPP
#include <string>
namespace flightlib {
    enum OptParam : int{
        mass = 0,
        arm_l = 1,
        motor_tau = 2,
        thrust_map = 3, // 3-dim
        kappa = 6,
        J = 7, // 6dim
        P_gain = 13, // 3dim
        G_equal = 16,
        N_Param = 17
    };
}
#endif

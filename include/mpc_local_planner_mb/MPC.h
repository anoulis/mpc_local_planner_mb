#ifndef MCP_LOCAL_PLANNER_MB_MPC_H
#define MCP_LOCAL_PLANNER_MB_MPC_H

#include <vector>
#include <map>
#include <Eigen/Core>

namespace mpc_local_planner_mb
{
using namespace std;

class MPC
{
    public:
        MPC();

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.
        vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
        vector<double> mpc_x;
        vector<double> mpc_y;

        void LoadParams(const std::map<string, double> &params);

    private:
        // Parameters for mpc solver
        double _max_steering, _max_throttle, _bound_value;
        int _mpc_steps, _x_start, _y_start, _psi_start, _v_start, _cte_start, _epsi_start, _delta_start, _a_start;
        std::map<string, double> _params;

};
}

#endif /* MCP_LOCAL_PLANNER_MB_MPC_H */

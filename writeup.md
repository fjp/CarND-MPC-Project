# Model Predictive Control

Model predictive control reframes the task of following a vehicle into an optimization problem. The solution to this optimization is a time ordered set of optimal actuator inpus for steering delta and acceleration a, which can be positive or negative. When these inputs are integrated using an initial state x and the underlying vehicle model f(x), the result is an optimal trajectory. To find this optimal trajectory, a cost function is utilized, which also accounts for the vehicle constraints.

The result of the implemented MPC can be seen in the video "CarND-MPC-video.mpg".


## The Model

As mentioned the model predictive control is based on a vehicle model which is used to predict vehicle states given actuator inputs.

### State

A trajectory is a set of time ordered states. In this project, one state vector consists of the following elements:

- px the x position of the vehicle
- py the y position of the vehicle
- psi the orientation of the vehicle
- v the vehicle velocity in longitudinal direction
- cte the cross-track error measured from the vehicle position (px, py) to a reference f(x).
- epsi the orientation error of the vehicle with respect to the reference orientation f'(x).

Note that the position and the orientation of the vehicle can be represented in global map coordinates or local vehicle coordinates.



### Actuators

During each time step the two actuator inpus steering and acceleration are optimized,
whereas only the first optimal inputs are applied to the steer the vehicle.
The rest of the optimized inputs is ignored because of the changing environment and the approximate vehicle model.

### Update Equations

To yield an optimal vehicle trajectory, a dynamic vehicle model serves as a constraint for the cost function.
These state update equations are defined as follows:

'''
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
'''

where the state elemnts are defined above and 'dt' is the elapsed time between timesteps, which was set to 100 ms.

## Timestep Length and Elapsed Duration (N & dt)

The number of optimization steps determins the size of the control input vector '[delta1,a1,delta2,a2,…,deltaN−1,aN−1]'. Integrating this vector using the vehicle model results in an optimal trajectory, found by minimizing a cost function. Increasing the horizon results in higher computational costs.

For a computers continuous trajectories need to be discretized. The step size of these discretizations is 'dt'. Large values of 'dt' result in a high discretization errors. To accurately represent a continous trajectory 'dt' should be small. On the other hand, low values increase the computational cost and are not always necessary. A lower limit depends on the rate new measurements are received.

To find parameters for 'N' and 'dt' is to define a resonable time horizon 'T = N*dt' and think about the consequences of the two parameters.

The number of time steps is set to 'N = 12' and 'dt = 0.1's. These values achived a stable behavior.
Lowering 'dt' or 'N' results in a shorter time horizon 'T' and lead to instability.

## MPC Preprocessing and Latency Handling

To account for the (simulated) actuator latency of 100 ms, the received vehicle states from the simulator are predicted 100 ms into the future (main.cpp lines 106-110).

'''
// Put latency into initial state values
// predict state in 100ms to account for the actuator (simulated) latency
double latency = 0.1;
px = px + v*cos(psi)*latency;
py = py + v*sin(psi)*latency;
psi = psi + v*delta/mpc.Lf*latency;
v = v + acceleration*latency;
'''

These predictions are transformed into local vehicle coordinates, which makes it easier to calculate the cross-track and orientation errors (main.cpp lines 114-127).

'''
// TODO: fit a polynomial to the above x and y coordinates
double* pptsx = &ptsx[0];
double* pptsy = &ptsy[0];

Eigen::Map<Eigen::VectorXd> ptx_map(pptsx,6);
Eigen::Map<Eigen::VectorXd> pty_map(pptsy,6);

// Transform waypoints from map to vehicle coordinates.
auto ptsx_vehicle = Eigen::VectorXd(ptsx.size());
auto ptsy_vehicle = Eigen::VectorXd(ptsy.size());
for (auto i = 0; i < ptsx.size(); ++i){
      ptsx_vehicle(i) = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
      ptsy_vehicle(i) = -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi);
}
'''


A polynomial is fitted to the transformed waypoints in local vehicle coordinates.

// Fit a polynomial to upcoming waypoints
Eigen::VectorXd coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);

These polynomial coefficients are used to calculate the cross-track error 'cte' (main.cpp line 135) and the orientation error 'epsi' (main.cpp line 142).

'''
// The cross track error is calculated by evaluating a polynomial at x, f(x)
// and subtracting y, which is zero in vehicle coordinates.
double cte = polyeval(coeffs, 0) - 0;

// Due to the sign starting at 0, the orientation error is -f'(x).
// epsi is the difference between desired heading and actual at px = 0
// px and psi are zero in vehicle coordinates
double epsi = 0 - atan(coeffs[1]);
'''

Finally the predicted state in vehicle coordinates is fed to the MPC solver (main.cpp line 148).

'''
// px, py and psi are zero in vehicle coordinates
Eigen::VectorXd state(6);
state << 0, 0, 0, v, cte, epsi;
'''

The transformed waypoints can be displayed with the simulator in the following way (main.cpp lines 190-201):

'''
//Display the waypoints/reference line
vector<double> next_x_vals;
vector<double> next_y_vals;

//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
// the points in the simulator are connected by a Yellow line
for (auto i = 0; i < ptsx.size() ; ++i){
    next_x_vals.push_back(ptsx_vehicle(i));
    next_y_vals.push_back(ptsy_vehicle(i));
}

msgJson["next_x"] = next_x_vals;
msgJson["next_y"] = next_y_vals;
'''

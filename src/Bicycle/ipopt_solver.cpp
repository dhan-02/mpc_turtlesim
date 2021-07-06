# include <cppad/ipopt/solve.hpp>
# include <cppad/cppad.hpp>
# include <ros/ros.h>
# include <geometry_msgs/Twist.h>
# include <geometry_msgs/Pose.h>
# include <math.h>
# include <turtlesim/Pose.h>

using namespace std;
int N;
float dt;
int L;
float goal_x,goal_y;
float v_limit,sa_limit;
float msg_linear,msg_angular;
float obs_x,obs_y;
float x_now,y_now,theta_now,vel_now,steer_angle_now;
namespace
{
     using CppAD::AD;

     class FG_eval
     {
     public:
          typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
          void operator()(ADvector &fg, const ADvector x)
          {
               
                assert(fg.size() == 2*(N-1)+7); 
                assert(x.size() == 2*N);
                
                 
                // Fortran style indexing               
                AD<double> vel[N]; 
                AD<double> steer_angle[N]; 
                for (int i = 0; i < N; i++) 
                {
                    vel[i] = x[2*i];
                    steer_angle[i] = x[2*i+1];
                }
                AD<double> x_last_step = x_now;
                AD<double> y_last_step = y_now; 
                AD<double> theta_last_step = theta_now;
                AD<double> X[N+1], Y[N+1]; 
                
                for (int i = 0; i < N; i++)
                {
                    x_last_step += vel[i]*CppAD::cos(steer_angle[i]+theta_last_step)*dt;
                    y_last_step += vel[i]*CppAD::sin(steer_angle[i]+theta_last_step)*dt;
                    theta_last_step += CppAD::sin(steer_angle[i])*vel[i]*dt/L;
                }
                
                
                fg[0] = (goal_x-x_last_step)*(goal_x-x_last_step)+(goal_y-y_last_step)*(goal_y-y_last_step);
                fg[1] = (x_last_step-obs_x)*(x_last_step- obs_x) + (y_last_step- obs_y)*(y_last_step- obs_y); 
                for (int i = 0; i < N-1; i++)
                {
                    fg[2*i+2] = abs(vel[i+1] - vel[i]);
                    fg[2*i+3] = abs(steer_angle[i+1] - steer_angle[i]);
                }
                fg[2*(N+1)] = x_last_step; 
                fg[2*(N+1)+1] = y_last_step;
                fg[2*(N+1)+2] = theta_last_step;
                return;
                
          }
     };
}

bool get_started(void)
{
     bool ok = true;
     size_t i;
     typedef CPPAD_TESTVECTOR(double) Dvector;

     // number of independent variables (domain dimension for f and g)
     size_t nx = 2*N;
     // number of constraints (range dimension for g)
     size_t ng = 2*N + 4;
     // initial value of the independent variables
     Dvector xi(nx);
     Dvector xl(nx), xu(nx);
     
     for (int i = 0; i < 2*N; i++)
     {
        if (i%2==0)
        {
             xl[i] = -v_limit;
             xu[i] = v_limit; 
             xi[i] = 0;   
          
        }
        else
        {
             xl[i] = -sa_limit;
             xu[i] = sa_limit;
             xi[i] = 0;
        }
     
     }

     // lower and upper limits for g
     Dvector gl(ng), gu(ng);
     gl[0] = 4;
     gu[0] = 1e19;
     for (i = 1; i < 2*N+1; i++)
     {
          gl[i] = 0;
          gu[i] = 0.5;
     }
     gl[2*N+1] = gl[2*N+2] = 0;
     gu[2*N+1] = gu[2*N+2] = 12;
     gl[2*N+3] = -3.14;
     gu[2*N+3] = 3.14;
     
     
     // object that computes objective and constraints
     FG_eval fg_eval;

     // options
     std::string options;
     // Uncomment this if you'd like more print information
     options += "Integer print_level  0\n";
     // Disables printing IPOPT creator banner
     options += "String  sb          yes\n";
     // NOTE: Setting sparse to true allows the solver to take advantage
     // of sparse routines, this makes the computation MUCH FASTER. If you
     // can uncomment 1 of these and see if it makes a difference or not but
     // if you uncomment both the computation time should go up in orders of
     // magnitude.
     options += "Sparse  true        forward\n";
     // options += "Sparse  true        reverse\n";

     // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
     // Change this as you see fit.
     options += "Numeric max_cpu_time          0.5\n";

     // place to return solution
     CppAD::ipopt::solve_result<Dvector> solution;

     // solve the problem
     CppAD::ipopt::solve<Dvector, FG_eval>(
     options, xi, xl, xu, gl, gu, fg_eval, solution);

     //cout<<"Solution x: "<<solution.x<<endl ;
     //cout<<"f(x): "<<solution.obj_value<<endl ;
     // Check some of the solution values
     //
     ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
     
     
     msg_linear = solution.x[0];
     msg_angular = CppAD::sin(solution.x[1])*solution.x[0]/L;
     cout<<abs(msg_linear)<<" ";
     
     
     cout<<abs(msg_angular)<<" ";
     return ok;
}

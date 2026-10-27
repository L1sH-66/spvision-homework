// C interface to the production cores for tests/offline_plant.py (ctypes).
#include <cstring>
#include <string>
#include "sp_controller_server/tracking_core.hpp"
#include "sp_global_planner/astar_core.hpp"
static sp_tracking::Tracker tracker;
static sp_tracking::YawPredictor yaw_predictor;
extern "C" {
int make_plan(const signed char* map,int w,int h,double sx,double sy,double gx,double gy,
              int lethal,double weight,double spacing_cells,int max_cost,int curvature_iterations,double curvature_weight,
              double* out,int capacity){
 sp_astar::SmoothOptions opt;opt.spacing=spacing_cells;opt.max_cost=max_cost;
 opt.curvature_iterations=curvature_iterations;opt.curvature_weight=curvature_weight;
 auto p=sp_astar::plan(std::vector<int8_t>(map,map+w*h),w,h,{sx,sy},{gx,gy},lethal,weight,opt);
 if(int(p.size())>capacity)return -1;
 for(size_t i=0;i<p.size();++i){out[2*i]=p[i].x;out[2*i+1]=p[i].y;}return p.size();
}
// Returns 0 if the key is unknown.
int set_param(const char* key,double v){
 auto& c=tracker.cfg;const std::string k(key);
 struct Entry{const char* name;double* field;};
 const Entry table[]={{"max_speed",&c.max_speed},{"lateral_accel",&c.lateral_accel},{"brake_decel",&c.brake_decel},
  {"min_corner_speed",&c.min_corner_speed},{"curvature_window",&c.curvature_window},{"dead_time",&c.dead_time},
  {"lag_gain",&c.lag_gain},{"max_acceleration",&c.max_acceleration},{"mpc_step",&c.mpc_step},
  {"mpc_q_cross",&c.mpc_q_cross},{"mpc_q_along",&c.mpc_q_along},{"mpc_q_vel",&c.mpc_q_vel},
  {"mpc_r_delta",&c.mpc_r_delta},{"mpc_r_ref",&c.mpc_r_ref},{"lookahead",&c.lookahead},
  {"lookahead_time",&c.lookahead_time},{"position_gain",&c.position_gain},{"lateral_damping",&c.lateral_damping},
  {"velocity_kp",&c.velocity_kp},{"velocity_ki",&c.velocity_ki},{"velocity_integral_limit",&c.velocity_integral_limit},
  {"goal_radius",&c.goal_radius},{"goal_gain",&c.goal_gain},{"stop_tolerance",&c.stop_tolerance},
  {"stop_speed",&c.stop_speed},{"command_slew",&c.command_slew},{"max_command",&c.max_command},
  {"plan_keep_distance",&c.plan_keep_distance},{"yaw_latency",&yaw_predictor.latency}};
 for(const auto& e:table)if(k==e.name){*e.field=v;return 1;}
 if(k=="mpc_horizon"){c.mpc_horizon=int(v);return 1;}
 if(k=="use_mpc"){c.use_mpc=v>0.5;return 1;}
 return 0;
}
void validate(){tracker.cfg.validate();}
void clear(){tracker.clearPlan();yaw_predictor=sp_tracking::YawPredictor{};}
void set_plan(const double* p,int n){std::vector<sp_tracking::Vec> v;for(int i=0;i<n;++i)v.push_back({p[2*i],p[2*i+1]});tracker.setPlan(v);}
// Same as the ROS plugin: map-frame command rotated into base_link with the predicted yaw.
void compute(double t,double stamp,double x,double y,double vx,double vy,double yaw,double* out){
 yaw_predictor.update(t,stamp,yaw);
 auto v=tracker.compute(t,{x,y},{vx,vy});
 const double a=yaw_predictor.predicted();
 out[0]=std::cos(a)*v.x+std::sin(a)*v.y;out[1]=-std::sin(a)*v.x+std::cos(a)*v.y;
}
}

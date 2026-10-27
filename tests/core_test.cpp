#include "sp_controller_server/tracking_core.hpp"
#include "sp_global_planner/astar_core.hpp"
#include <cassert>
#include <iostream>
#include <limits>
int main(){
 using namespace sp_tracking;
 Tracker t;
 t.cfg.validate();
 // no plan -> zero
 assert(norm(t.compute(0.0,{0,0},{0,0}))==0);
 // straight plan, robot left of it: moves forward and towards the line, within the command limit
 t.setPlan({{0,0},{0,0},{1,0},{2,0},{3,0}});
 Vec v{};for(int k=0;k<10;++k)v=t.compute(0.02*k,{.5,.1},{0,0});
 assert(v.x>0&&v.y<0&&norm(v)<=t.cfg.max_command+1e-9);
 assert(t.lastCrossTrackError()<0);
 // MPC exposes its prediction (state after the dead time + horizon) and reference for RViz
 assert(t.lastPrediction().size()==static_cast<size_t>(t.cfg.mpc_horizon)+1&&t.lastReference().size()==static_cast<size_t>(t.cfg.mpc_horizon));
 assert(t.lastPrediction().back().x>t.lastPrediction().front().x);
 // non-finite input -> zero
 assert(norm(t.compute(1.0,{NAN,0},{0,0}))==0);
 // speed profile: zero at the goal, never above max_speed, slower in a corner than on a straight
 t.setPlan({{0,0},{3,0},{3,3}});
 assert(t.profileSpeed(t.length())==0);
 assert(t.profileSpeed(0.5)<=t.cfg.max_speed+1e-9);
 assert(t.profileSpeed(3.0)<t.profileSpeed(1.0));
 // single-point plan at the robot -> stop
 t.clearPlan();t.setPlan({{1,1}});assert(norm(t.compute(2.0,{1,1},{0,0}))==0);
 // goal servo pulls back after an overshoot
 t.clearPlan();t.setPlan({{0,0},{1,0}});Vec back{};for(int k=0;k<10;++k)back=t.compute(3.0+0.02*k,{1.2,0},{0,0});
 assert(back.x<0);
 // invalid plan rejected and cleared
 bool rejected=false;try{t.setPlan({{INFINITY,0}});}catch(const std::invalid_argument&){rejected=true;}
 assert(rejected&&!t.hasPlan()&&norm(t.compute(4.0,{0,0},{0,0}))==0);
 // re-plan hysteresis: a nearly identical plan is kept, a different one replaces it
 t.setPlan({{0,0},{5,0}});
 assert(!t.setPlan({{0,0.05},{5,0}}));
 assert(t.setPlan({{0,0},{2,2},{5,0}}));
 // pursuit fallback also respects the limits
 Tracker p;p.cfg.use_mpc=false;p.setPlan({{0,0},{4,0}});
 Vec q{};for(int k=0;k<10;++k)q=p.compute(0.02*k,{1,.2},{.5,0});
 assert(q.x>0&&q.y<0&&norm(q)<=p.cfg.max_command+1e-9);
 // bad configuration is rejected
 Config bad;bad.mpc_horizon=1;rejected=false;try{bad.validate();}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
 // plant table inverse is consistent
 for(double s=0.1;s<1.4;s+=0.1)assert(std::fabs(plant::steadySpeed(plant::commandFor(s))-s)<1e-9);

 // A*: blocked wall, exact endpoints, unknown cells, diagonal corner, smoothing stays below max_cost
 std::vector<int8_t> grid(25,0);for(int y=0;y<5;++y)grid[y*5+2]=100;
 assert(sp_astar::plan(grid,5,5,{.5,.5},{4.5,4.5},100,4).empty());
 grid[2*5+2]=0;auto path=sp_astar::plan(grid,5,5,{.7,.6},{4.2,4.1},100,4);
 assert(!path.empty()&&path.front().x==.7&&path.back().x==4.2);
 for(auto c:path)assert(grid[int(c.y)*5+int(c.x)]<100);
 assert(sp_astar::plan({0,100,100,0},2,2,{.5,.5},{1.5,1.5},100,4).empty());
 assert(sp_astar::plan({0,0,0,-1},2,2,{.5,.5},{1.5,1.5},100,4).empty());   // unknown goal
 assert(!sp_astar::plan({-1,0,0,0},2,2,{.5,.5},{1.5,1.5},100,4).empty());  // start escapes
 // start inside a blocked cell next to free space: escape to the nearest free cell
 std::vector<int8_t> strip(25,0);strip[2*5+2]=100;auto esc=sp_astar::plan(strip,5,5,{2.5,2.5},{4.5,4.5},100,4);
 assert(!esc.empty()&&esc.front().x==2.5&&esc.back().x==4.5);
 std::vector<int8_t> room(40*40,0);
 for(int y=0;y<40;++y)for(int x=0;x<40;++x){if(x>=18&&x<22&&y<30)room[y*40+x]=100;else if(x>=14&&x<26&&y<34)room[y*40+x]=60;}
 auto around=sp_astar::plan(room,40,40,{5.5,5.5},{35.5,5.5},100,4);
 assert(around.size()>10);
 for(size_t i=1;i<around.size();++i)assert(sp_astar::segmentCost(room,40,40,around[i-1],around[i])<100);
 std::cout<<"PASS: tracker (empty/invalid/limits/MPC prediction/profile/goal servo/hysteresis/pursuit/config), plant table, "
            "A* (blocked/unknown/diagonal/exact endpoints/start escape/smoothing safety)\n";
}

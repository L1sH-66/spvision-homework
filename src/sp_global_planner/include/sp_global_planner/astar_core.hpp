#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>
namespace sp_astar {
struct Point {double x,y;};
// Grid coordinates measured in cells, centres at i+0.5. Unknown is blocked.
struct SmoothOptions {
  bool enabled=true;
  double spacing=2.0;        // re-sampling distance in cells (2 cells = 0.10 m at 0.05 m/cell)
  int iterations=300;
  double data_weight=0.01;   // pull back towards the A* cells
  double smooth_weight=0.45; // curvature (second difference) penalty
  int max_cost=40;           // smoothed points/segments may not exceed this cost
  // Second stage: minimum-curvature refinement (biharmonic flow).  The first stage
  // shortens the path (Laplacian) and leaves the tightest bend where the cost
  // limit stops it; this stage spreads that bend over a larger radius inside the
  // same safe corridor, which is what limits the speed in corners.
  int curvature_iterations=200;
  double curvature_weight=0.04;  // step of the biharmonic flow (< 1/16 for stability)
};

inline int costAt(const std::vector<int8_t>& grid,int w,int h,Point q){
  const int x=static_cast<int>(std::floor(q.x)),y=static_cast<int>(std::floor(q.y));
  if(x<0||y<0||x>=w||y>=h)return 100;
  const int c=grid[static_cast<size_t>(y)*w+x];
  return c<0?100:c;
}
inline int segmentCost(const std::vector<int8_t>& grid,int w,int h,Point a,Point b){
  const int n=std::max(1,static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y)*2.0)));  // every half cell
  int worst=0;
  for(int k=0;k<=n;++k){const double t=double(k)/n;worst=std::max(worst,costAt(grid,w,h,{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t}));}
  return worst;
}
inline std::vector<Point> resample(const std::vector<Point>& in,double spacing){
  if(in.size()<2||spacing<=0)return in;
  std::vector<Point> out{in.front()};
  double carry=0.0;
  for(size_t i=1;i<in.size();++i){
    const double seg=std::hypot(in[i].x-in[i-1].x,in[i].y-in[i-1].y);
    if(seg<1e-9)continue;
    double pos=spacing-carry;
    for(;pos<=seg;pos+=spacing){const double t=pos/seg;out.push_back({in[i-1].x+t*(in[i].x-in[i-1].x),in[i-1].y+t*(in[i].y-in[i-1].y)});}
    carry=seg-(pos-spacing);
  }
  if(std::hypot(out.back().x-in.back().x,out.back().y-in.back().y)>1e-6)out.push_back(in.back());
  return out;
}
// Cost-aware gradient smoothing: each interior point moves towards the mid-point of
// its neighbours (curvature penalty) and weakly back towards the A* path.  A move
// is accepted only if the point and both adjacent segments stay below max_cost (or
// below the cost the original path already had there), so smoothing never cuts
// into the inflated obstacles.  Endpoints are fixed (exact start / goal).
inline std::vector<Point> smooth(const std::vector<int8_t>& grid,int w,int h,int lethal,
 const std::vector<Point>& raw,const SmoothOptions& opt){
  const std::vector<Point> original=resample(raw,opt.spacing);
  std::vector<Point> p=original;
  const size_t n=p.size();
  if(n<3)return p;
  std::vector<int> bound(n);
  for(size_t i=0;i<n;++i){
    int worst=costAt(grid,w,h,original[i]);
    if(i>0)worst=std::max(worst,segmentCost(grid,w,h,original[i-1],original[i]));
    if(i+1<n)worst=std::max(worst,segmentCost(grid,w,h,original[i],original[i+1]));
    bound[i]=std::min(std::max(opt.max_cost,worst),lethal-1);
  }
  for(int iter=0;iter<opt.iterations;++iter){
    double moved=0.0;
    for(size_t i=1;i+1<n;++i){
      const Point q{p[i].x+opt.data_weight*(original[i].x-p[i].x)+opt.smooth_weight*(p[i-1].x+p[i+1].x-2*p[i].x),
                    p[i].y+opt.data_weight*(original[i].y-p[i].y)+opt.smooth_weight*(p[i-1].y+p[i+1].y-2*p[i].y)};
      const int lim=std::max({bound[i-1],bound[i],bound[i+1]});
      if(costAt(grid,w,h,q)>lim||segmentCost(grid,w,h,p[i-1],q)>lim||segmentCost(grid,w,h,q,p[i+1])>lim)continue;
      moved=std::max(moved,std::hypot(q.x-p[i].x,q.y-p[i].y));
      p[i]=q;
    }
    if(moved<2e-3)break;  // 0.1 mm at 0.05 m/cell
  }
  if(opt.curvature_iterations<=0||n<5)return p;
  for(int iter=0;iter<opt.curvature_iterations;++iter){
    double moved=0.0;
    for(size_t i=2;i+2<p.size();++i){
      const double gx=p[i-2].x-4*p[i-1].x+6*p[i].x-4*p[i+1].x+p[i+2].x;
      const double gy=p[i-2].y-4*p[i-1].y+6*p[i].y-4*p[i+1].y+p[i+2].y;
      // never raise the local cost above max_cost (or above what is already there)
      const int here=std::max({costAt(grid,w,h,p[i]),segmentCost(grid,w,h,p[i-1],p[i]),segmentCost(grid,w,h,p[i],p[i+1])});
      const int lim=std::min(std::max(opt.max_cost,here),lethal-1);
      // backtracking: a shorter step may still be admissible near the cost limit
      for(double step=opt.curvature_weight;step>opt.curvature_weight/8;step*=0.5){
        const Point q{p[i].x-step*gx,p[i].y-step*gy};
        if(costAt(grid,w,h,q)>lim||segmentCost(grid,w,h,p[i-1],q)>lim||segmentCost(grid,w,h,q,p[i+1])>lim)continue;
        moved=std::max(moved,std::hypot(q.x-p[i].x,q.y-p[i].y));
        p[i]=q;
        break;
      }
    }
    if((iter+1)%50==0){p=resample(p,opt.spacing);}
    if(moved<2e-3)break;
  }
  return resample(p,opt.spacing);
}

inline std::vector<Point> plan(const std::vector<int8_t>& grid,int w,int h,
 Point start,Point goal,int lethal,double weight,const SmoothOptions& opt=SmoothOptions{}) {
 auto blocked=[&](int x,int y){return x<0||y<0||x>=w||y>=h||grid[y*w+x]<0||grid[y*w+x]>=lethal;};
 if(w<=0||h<=0||grid.size()!=static_cast<size_t>(w*h))return {};
 if(!std::isfinite(start.x)||!std::isfinite(start.y)||!std::isfinite(goal.x)||!std::isfinite(goal.y))return {};
 int sx=std::floor(start.x),sy=std::floor(start.y),gx=std::floor(goal.x),gy=std::floor(goal.y);
 if(blocked(gx,gy))return {};
 if(blocked(sx,sy)){
  // The robot has drifted into the inflated zone (or a peer appeared next to it):
  // start the search from the nearest traversable cell within 10 cells instead of
  // failing forever from the same spot.
  int best=-1;double best_d=std::numeric_limits<double>::infinity();
  for(int dy=-10;dy<=10;++dy)for(int dx=-10;dx<=10;++dx){
   const int x=sx+dx,y=sy+dy;const double d=std::hypot(double(dx),double(dy));
   if(d<=10.0&&d<best_d&&!blocked(x,y)){best_d=d;best=y*w+x;}
  }
  if(best<0)return {};
  sx=best%w;sy=best/w;
 }
 int src=sy*w+sx,dst=gy*w+gx;
 struct Item {double f;int i;bool operator<(const Item& b)const{return f>b.f;}};
 std::priority_queue<Item> open;
 std::vector<double> distance(w*h,std::numeric_limits<double>::infinity());
 std::vector<int> parent(w*h,-1);std::vector<bool> closed(w*h,false);
 distance[src]=0;open.push({std::hypot(sx-gx,sy-gy),src});
 const int dx[8]={1,-1,0,0,1,1,-1,-1},dy[8]={0,0,1,-1,1,-1,1,-1};
 while(!open.empty()){
  int i=open.top().i;open.pop();if(closed[i])continue;closed[i]=true;if(i==dst)break;
  int x=i%w,y=i/w;
  for(int k=0;k<8;++k){int nx=x+dx[k],ny=y+dy[k];if(blocked(nx,ny))continue;
   if(k>=4&&(blocked(nx,y)||blocked(x,ny)))continue;
   int j=ny*w+nx;double d=distance[i]+(k<4?1.:std::sqrt(2.))*(1.+weight*grid[j]/100.);
   if(d<distance[j]){distance[j]=d;parent[j]=i;open.push({d+std::hypot(nx-gx,ny-gy),j});}
  }
 }
 if(!closed[dst])return {};
 std::vector<Point> p;for(int i=dst;i!=-1;i=parent[i]){p.push_back({i%w+0.5,i/w+0.5});if(i==src)break;}
 std::reverse(p.begin(),p.end());
 if(p.size()==1){p={start,goal};}else{p.front()=start;p.back()=goal;}
 if(!opt.enabled) return p;
 return smooth(grid,w,h,lethal,p,opt);
}
}

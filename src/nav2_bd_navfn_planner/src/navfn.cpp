// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//
// Navigation function computation
// Modified for Bidirectional A* search
//

#include "nav2_bd_navfn_planner/navfn.hpp"

#include <algorithm>
#include <queue>
#include "rclcpp/rclcpp.hpp"

namespace nav2_bd_navfn_planner
{

// Define constants
#define INVSQRT2 0.707106781

//
// create nav fn buffers
//

NavFn::NavFn(int xs, int ys)
{
  // create cell arrays
  costarr = NULL;
  potarr = NULL;
  pending = NULL;
  gradx = grady = NULL;

  setNavArr(xs, ys);

  // priority buffers
  pb1 = new int[PRIORITYBUFSIZE];
  pb2 = new int[PRIORITYBUFSIZE];
  pb3 = new int[PRIORITYBUFSIZE];

  // for Dijkstra (breadth-first), set to COST_NEUTRAL
  // for A* (best-first), set to COST_NEUTRAL
  priInc = 2 * COST_NEUTRAL;

  // goal and start
  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;

  // path buffers
  npathbuf = npath = 0;
  pathx = pathy = NULL;
  pathStep = 0.5;
}


NavFn::~NavFn()
{
  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }
  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }
  if (pathx) {
    delete[] pathx;
  }
  if (pathy) {
    delete[] pathy;
  }
  if (pb1) {
    delete[] pb1;
  }
  if (pb2) {
    delete[] pb2;
  }
  if (pb3) {
    delete[] pb3;
  }
}


//
// set goal, start positions for the nav fn
//

void
NavFn::setGoal(int * g)
{
  goal[0] = g[0];
  goal[1] = g[1];
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void
NavFn::setStart(int * g)
{
  start[0] = g[0];
  start[1] = g[1];
  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"), "[NavFn] Setting start to %d,%d\n", start[0],
    start[1]);
}

//
// Set/Reset map size
//

void
NavFn::setNavArr(int xs, int ys)
{
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Array is %d x %d\n", xs, ys);

  nx = xs;
  ny = ys;
  ns = nx * ny;

  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }

  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }

  costarr = new COSTTYPE[ns];  // cost array, 2d config space
  memset(costarr, 0, ns * sizeof(COSTTYPE));
  potarr = new float[ns];  // navigation potential array
  pending = new bool[ns];
  memset(pending, 0, ns * sizeof(bool));
  gradx = new float[ns];
  grady = new float[ns];
}


//
// set up cost array, usually from ROS
//

void
NavFn::setCostmap(const COSTTYPE * cmap, bool isROS, bool allow_unknown)
{
  COSTTYPE * cm = costarr;
  if (isROS) {  // ROS-type cost array
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        // This transforms the incoming cost values:
        // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
        // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated obstacle")
        // values in range 0 to 252 -> values from COST_NEUTRAL to COST_OBS_ROS.
        *cm = COST_OBS;
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS && allow_unknown) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  } else {  // not a ROS map, just a PGM
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        *cm = COST_OBS;
        if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
          continue;  // don't do borders
        }
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  }
}

bool NavFn::calcBidirectionalAstar() 
{
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Starting bidirectional A* search");
    setupNavFn(true);
    
    // Calculate maximum number of cycles - but adapt based on map size
    int max_cycles = std::max(nx * ny / 20, nx + ny);
    
    // Initialize temporary arrays for the bidirectional search
    float* potarr_start = new float[ns];
    float* potarr_goal = new float[ns];
    bool* pending_start = new bool[ns];
    bool* pending_goal = new bool[ns];
    
    // Initialize arrays
    for (int i = 0; i < ns; i++) {
        potarr_start[i] = POT_HIGH;
        potarr_goal[i] = POT_HIGH;
    }
    memset(pending_start, 0, ns * sizeof(bool));
    memset(pending_goal, 0, ns * sizeof(bool));
    
    // Priority queues for forward (start to goal) and reverse (goal to start) searches
    std::priority_queue<std::pair<float, int>, 
                       std::vector<std::pair<float, int>>, 
                       std::greater<std::pair<float, int>>> startQueue, goalQueue;
    
    // Start and goal cells
    int startCell = start[1] * nx + start[0];
    int goalCell = goal[1] * nx + goal[0];
    
    // Quick check - if start or goal is in obstacle, return immediately
    if (costarr[startCell] >= COST_OBS || costarr[goalCell] >= COST_OBS) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), 
                    "[NavFn] Start or goal is in obstacle - no path possible");
        
        delete[] potarr_start;
        delete[] potarr_goal;
        delete[] pending_start;
        delete[] pending_goal;
        
        return false;
    }
    
    // Calculate direct distance for heuristic scaling
    int goal_x = goal[0];
    int goal_y = goal[1];
    int start_x = start[0];
    int start_y = start[1];
    float direct_distance = hypot(goal_x - start_x, goal_y - start_y);
    
    // Early termination if start and goal are close
    if (direct_distance < 5.0) {
        // For very short paths, create a direct path potential field
        for (int i = 0; i < ns; i++) {
            potarr[i] = POT_HIGH;
        }
        potarr[goalCell] = 0;
        
        // Create a simple gradient to the goal
        std::queue<int> q;
        q.push(goalCell);
        
        while (!q.empty()) {
            int current = q.front();
            q.pop();
            
            if (current == startCell) break;
            
            int x = current % nx;
            int y = current / nx;
            
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int nx = x + dx;
                    int ny = y + dy;
                    int nbr = ny * this->nx + nx;
                    
                    // Check bounds
                    if (nx < 0 || nx >= this->nx || ny < 0 || ny >= this->ny || nbr >= ns) {
                        continue;
                    }
                    
                    // Skip obstacles
                    if (costarr[nbr] >= COST_OBS) {
                        continue;
                    }
                    
                    // Calculate cost to neighbor
                    float move_cost;
                    if (dx == 0 || dy == 0) {
                        move_cost = costarr[nbr];  // Orthogonal
                    } else {
                        move_cost = INVSQRT2 * costarr[nbr];  // Diagonal
                    }
                    
                    float new_cost = potarr[current] + move_cost;
                    
                    if (new_cost < potarr[nbr]) {
                        potarr[nbr] = new_cost;
                        q.push(nbr);
                    }
                }
            }
        }
        
        last_path_cost_ = potarr[startCell];
        return true;
    }
    
    // Adjust heuristic weight based on map size and obstacle density
    // Higher values make search more greedy/direct but less optimal
    float heuristic_weight = 1.2f;
    
    // Count obstacles for adaptive heuristic
    int obstacle_count = 0;
    for (int i = 0; i < ns; i++) {
        if (costarr[i] >= COST_OBS) {
            obstacle_count++;
        }
    }
    
    // Adjust weight based on obstacle density
    float obstacle_ratio = static_cast<float>(obstacle_count) / ns;
    if (obstacle_ratio < 0.1f) {
        // Sparse map - be more greedy
        heuristic_weight = 1.5f;
    } else if (obstacle_ratio > 0.3f) {
        // Dense map - be more cautious
        heuristic_weight = 1.0f;
    }
    
    // Initialize the bidirectional search
    potarr_start[startCell] = 0;
    startQueue.push({0, startCell});
    
    potarr_goal[goalCell] = 0;
    goalQueue.push({0, goalCell});
    
    // Best meeting point and cost
    int meeting_point = -1;
    float best_cost = POT_HIGH;
    
    // Expand forward and backward frontiers at different rates based on obstacle density
    int forward_expand_rate = 1;
    int backward_expand_rate = 1;
    
    // Main search loop
    int cycle = 0;
    int checkMeetingFrequency = 1; // Check meeting points every N cycles
    
    while ((!startQueue.empty() || !goalQueue.empty()) && cycle < max_cycles) {
        cycle++;
        
        // Forward search from start - with adaptive expansion
        for (int i = 0; i < forward_expand_rate && !startQueue.empty(); i++) {
            auto [cost, current] = startQueue.top();
            startQueue.pop();
            
            if (pending_start[current]) {
                continue;  // Already processed
            }
            
            pending_start[current] = true;
            
            // Check if we've found a meeting point - but only every N cycles to reduce overhead
            if (cycle % checkMeetingFrequency == 0 && pending_goal[current]) {
                float total_cost = potarr_start[current] + potarr_goal[current];
                if (total_cost < best_cost) {
                    best_cost = total_cost;
                    meeting_point = current;
                }
            }
            
            // Process neighbors in 8-connected grid
            int x = current % nx;
            int y = current / nx;
            
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int nx = x + dx;
                    int ny = y + dy;
                    int nbr = ny * this->nx + nx;
                    
                    // Check bounds
                    if (nx < 0 || nx >= this->nx || ny < 0 || ny >= this->ny || nbr >= ns) {
                        continue;
                    }
                    
                    // Skip obstacles
                    if (costarr[nbr] >= COST_OBS) {
                        continue;
                    }
                    
                    // Calculate movement cost
                    float move_cost;
                    if (dx == 0 || dy == 0) {
                        move_cost = costarr[nbr];  // Orthogonal
                    } else {
                        move_cost = INVSQRT2 * costarr[nbr];  // Diagonal
                    }
                    
                    float new_cost = potarr_start[current] + move_cost;
                    
                    if (new_cost < potarr_start[nbr]) {
                        potarr_start[nbr] = new_cost;
                        // Apply weighted heuristic for more directed search
                        float h = heuristic_weight * hypot(goal[0] - nx, goal[1] - ny) * COST_NEUTRAL;
                        startQueue.push({new_cost + h, nbr});
                    }
                }
            }
        }
        
        // Backward search from goal - with adaptive expansion
        for (int i = 0; i < backward_expand_rate && !goalQueue.empty(); i++) {
            auto [cost, current] = goalQueue.top();
            goalQueue.pop();
            
            if (pending_goal[current]) {
                continue;  // Already processed
            }
            
            pending_goal[current] = true;
            
            // Check if we've found a meeting point - but only every N cycles to reduce overhead
            if (cycle % checkMeetingFrequency == 0 && pending_start[current]) {
                float total_cost = potarr_start[current] + potarr_goal[current];
                if (total_cost < best_cost) {
                    best_cost = total_cost;
                    meeting_point = current;
                }
            }
            
            // Process neighbors in 8-connected grid
            int x = current % nx;
            int y = current / nx;
            
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int nx = x + dx;
                    int ny = y + dy;
                    int nbr = ny * this->nx + nx;
                    
                    // Check bounds
                    if (nx < 0 || nx >= this->nx || ny < 0 || ny >= this->ny || nbr >= ns) {
                        continue;
                    }
                    
                    // Skip obstacles
                    if (costarr[nbr] >= COST_OBS) {
                        continue;
                    }
                    
                    // Calculate movement cost
                    float move_cost;
                    if (dx == 0 || dy == 0) {
                        move_cost = costarr[nbr];  // Orthogonal
                    } else {
                        move_cost = INVSQRT2 * costarr[nbr];  // Diagonal
                    }
                    
                    float new_cost = potarr_goal[current] + move_cost;
                    
                    if (new_cost < potarr_goal[nbr]) {
                        potarr_goal[nbr] = new_cost;
                        // Apply weighted heuristic for more directed search
                        float h = heuristic_weight * hypot(start[0] - nx, start[1] - ny) * COST_NEUTRAL;
                        goalQueue.push({new_cost + h, nbr});
                    }
                }
            }
        }
        
        // Adjust expansion rates dynamically based on queue sizes
        if (cycle % 10 == 0) {
            if (startQueue.size() > 2 * goalQueue.size()) {
                forward_expand_rate = 1;
                backward_expand_rate = 2;
            } else if (goalQueue.size() > 2 * startQueue.size()) {
                forward_expand_rate = 2;
                backward_expand_rate = 1;
            } else {
                forward_expand_rate = 1;
                backward_expand_rate = 1;
            }
            
            // Increase meeting point check frequency as we get further in the search
            if (cycle > max_cycles / 4) {
                checkMeetingFrequency = 1;  // Check every cycle in later stages
            }
        }
        
        // Check if we can terminate early
        if (meeting_point != -1 && cycle % 5 == 0) {  // Check more frequently
            float min_start = startQueue.empty() ? POT_HIGH : startQueue.top().first;
            float min_goal = goalQueue.empty() ? POT_HIGH : goalQueue.top().first;
            
            if (best_cost <= min_start + min_goal) {
                break;  // We found the best path already
            }
        }
    }
    
    if (meeting_point == -1) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] No path found in bidirectional A*");
        
        // Clean up
        delete[] potarr_start;
        delete[] potarr_goal;
        delete[] pending_start;
        delete[] pending_goal;
        
        return false;
    }
    
    RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[NavFn] Bidirectional A* found path at meeting point (%d,%d) with cost %f after %d cycles", 
        meeting_point % nx, meeting_point / nx, best_cost, cycle);
    
    // Create a single potential field for path extraction
    // This is critical for Nav2's path extraction
    
    // First, propagate potentials from goal
    for (int i = 0; i < ns; i++) {
        potarr[i] = POT_HIGH;
    }
    
    // Set goal potential to 0 to allow path extraction towards goal
    potarr[goalCell] = 0;
    
    // Create gradient from the meeting point to the goal - using optimized BFS
    std::queue<int> q;
    q.push(goalCell);
    
    while (!q.empty()) {
        int current = q.front();
        q.pop();
        
        // Stop BFS once we reach start cell - no need to fill entire potential field
        if (current == startCell) {
            break;
        }
        
        int x = current % nx;
        int y = current / nx;
        
        // Prioritize orthogonal neighbors for better path quality
        // First process orthogonal neighbors
        const int dx_ortho[4] = {0, 1, 0, -1};
        const int dy_ortho[4] = {-1, 0, 1, 0};
        
        for (int i = 0; i < 4; i++) {
            int nx = x + dx_ortho[i];
            int ny = y + dy_ortho[i];
            int nbr = ny * this->nx + nx;
            
            // Check bounds
            if (nx < 0 || nx >= this->nx || ny < 0 || ny >= this->ny || nbr >= ns) {
                continue;
            }
            
            // Skip obstacles
            if (costarr[nbr] >= COST_OBS) {
                continue;
            }
            
            float new_cost = potarr[current] + costarr[nbr];
            
            if (new_cost < potarr[nbr]) {
                potarr[nbr] = new_cost;
                q.push(nbr);
            }
        }
        
        // Then process diagonal neighbors
        const int dx_diag[4] = {-1, -1, 1, 1};
        const int dy_diag[4] = {-1, 1, -1, 1};
        
        for (int i = 0; i < 4; i++) {
            int nx = x + dx_diag[i];
            int ny = y + dy_diag[i];
            int nbr = ny * this->nx + nx;
            
            // Check bounds
            if (nx < 0 || nx >= this->nx || ny < 0 || ny >= this->ny || nbr >= ns) {
                continue;
            }
            
            // Skip obstacles
            if (costarr[nbr] >= COST_OBS) {
                continue;
            }
            
            float new_cost = potarr[current] + INVSQRT2 * costarr[nbr];
            
            if (new_cost < potarr[nbr]) {
                potarr[nbr] = new_cost;
                q.push(nbr);
            }
        }
    }
    
    // Set path cost for metrics
    last_path_cost_ = potarr[startCell];
    
    // Clean up
    delete[] potarr_start;
    delete[] potarr_goal;
    delete[] pending_start;
    delete[] pending_goal;
    
    return true;
}

// Set up navigation potential arrays for new propagation

void
NavFn::setupNavFn(bool keepit)
{
  // reset values in propagation arrays
  for (int i = 0; i < ns; i++) {
    potarr[i] = POT_HIGH;
    if (!keepit) {
      costarr[i] = COST_NEUTRAL;
    }
    gradx[i] = grady[i] = 0.0;
  }

  // outer bounds of cost array
  COSTTYPE * pc;
  pc = costarr;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }

  // priority buffers
  curT = COST_OBS;
  curP = pb1;
  curPe = 0;
  nextP = pb2;
  nextPe = 0;
  overP = pb3;
  overPe = 0;
  memset(pending, 0, ns * sizeof(bool));

  // set goal
  int k = goal[0] + goal[1] * nx;
  initCost(k, 0);

  // find # of obstacle cells
  pc = costarr;
  int ntot = 0;
  for (int i = 0; i < ns; i++, pc++) {
    if (*pc >= COST_OBS) {
      ntot++;  // number of cells that are obstacles
    }
  }
  nobs = ntot;
}


// inserting onto the priority blocks
#define push_cur(n)  {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && curPe < PRIORITYBUFSIZE) \
    {curP[curPe++] = n; pending[n] = true;}}
#define push_next(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && nextPe < PRIORITYBUFSIZE) \
    {nextP[nextPe++] = n; pending[n] = true;}}
#define push_over(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && overPe < PRIORITYBUFSIZE) \
    {overP[overPe++] = n; pending[n] = true;}}

// initialize a goal-type cost for starting propagation

void
NavFn::initCost(int k, float v)
{
  potarr[k] = v;
  push_cur(k + 1);
  push_cur(k - 1);
  push_cur(k - nx);
  push_cur(k + nx);
}

//
// returning values
//

float * NavFn::getPathX() {return pathx;}
float * NavFn::getPathY() {return pathy;}
int NavFn::getPathLen() {return npath;}

//
// Path construction
// Find gradient at array points, interpolate path
// Use step size of pathStep, usually 0.5 pixel
//
// Some sanity checks:
//  1. Stuck at same index position
//  2. Doesn't get near goal
//  3. Surrounded by high potentials
//

int
NavFn::calcPath(int n, int * st)
{
  // test write
  // savemap("test");

  // check path arrays
  if (npathbuf < n) {
    if (pathx) {delete[] pathx;}
    if (pathy) {delete[] pathy;}
    pathx = new float[n];
    pathy = new float[n];
    npathbuf = n;
  }

  // set up start position at cell
  // st is always upper left corner for 4-point bilinear interpolation
  if (st == NULL) {st = start;}
  int stc = st[1] * nx + st[0];

  // set up offset
  float dx = 0;
  float dy = 0;
  npath = 0;

  // go for <n> cycles at most
  for (int i = 0; i < n; i++) {
    // check if near goal
    int nearest_point = std::max(
      0,
      std::min(
        nx * ny - 1, stc + static_cast<int>(round(dx)) +
        static_cast<int>(nx * round(dy))));
    if (potarr[nearest_point] < COST_NEUTRAL) {
      pathx[npath] = static_cast<float>(goal[0]);
      pathy[npath] = static_cast<float>(goal[1]);
      return ++npath;  // done!
    }

    if (stc < nx || stc > ns - nx) {  // would be out of bounds
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Out of bounds");
      return 0;
    }

    // add to path
    pathx[npath] = stc % nx + dx;
    pathy[npath] = stc / nx + dy;
    npath++;

    bool oscillation_detected = false;
    if (npath > 2 &&
      pathx[npath - 1] == pathx[npath - 3] &&
      pathy[npath - 1] == pathy[npath - 3])
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[PathCalc] oscillation detected, attempting fix.");
      oscillation_detected = true;
    }

    int stcnx = stc + nx;
    int stcpx = stc - nx;

    // check for potentials at eight positions near cell
    if (potarr[stc] >= POT_HIGH ||
      potarr[stc + 1] >= POT_HIGH ||
      potarr[stc - 1] >= POT_HIGH ||
      potarr[stcnx] >= POT_HIGH ||
      potarr[stcnx + 1] >= POT_HIGH ||
      potarr[stcnx - 1] >= POT_HIGH ||
      potarr[stcpx] >= POT_HIGH ||
      potarr[stcpx + 1] >= POT_HIGH ||
      potarr[stcpx - 1] >= POT_HIGH ||
      oscillation_detected)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);

      // check eight neighbors to find the lowest
      int minc = stc;
      int minp = potarr[stc];
      int st = stcpx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc + 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stcnx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      stc = minc;
      dx = 0;
      dy = 0;

      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
        potarr[stc], pathx[npath - 1], pathy[npath - 1]);

      if (potarr[stc] >= POT_HIGH) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, high potential");
        // savemap("navfn_highpot");
        return 0;
      }
    } else {  // have a good gradient here
      // get grad at four positions near cell
      gradCell(stc);
      gradCell(stc + 1);
      gradCell(stcnx);
      gradCell(stcnx + 1);


      // get interpolated gradient
      float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
      float x2 = (1.0 - dx) * gradx[stcnx] + dx * gradx[stcnx + 1];
      float x = (1.0 - dy) * x1 + dy * x2;  // interpolated x
      float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
      float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
      float y = (1.0 - dy) * y1 + dy * y2;  // interpolated y

      // check for zero gradient, failed
      if (x == 0.0 && y == 0.0) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Zero gradient");
        return 0;
      }

      // move in the right direction
      float ss = pathStep / hypot(x, y);
      dx += x * ss;
      dy += y * ss;

      // check for overflow
      if (dx > 1.0) {stc++; dx -= 1.0;}
      if (dx < -1.0) {stc--; dx += 1.0;}
      if (dy > 1.0) {stc += nx; dy -= 1.0;}
      if (dy < -1.0) {stc -= nx; dy += 1.0;}
    }
  }

  //  return npath;  // out of cycles, return failure
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
  // savemap("navfn_pathlong");
  return 0;  // out of cycles, return failure
}


//
// gradient calculations
//

// calculate gradient at a cell
// positive value are to the right and down
float
NavFn::gradCell(int n)
{
  if (gradx[n] + grady[n] > 0.0) {  // check this cell
    return 1.0;
  }

  if (n < nx || n > ns - nx) {  // would be out of bounds
    return 0.0;
  }

  float cv = potarr[n];
  float dx = 0.0;
  float dy = 0.0;

  // check for in an obstacle
  if (cv >= POT_HIGH) {
    if (potarr[n - 1] < POT_HIGH) {
      dx = -COST_OBS;
    } else if (potarr[n + 1] < POT_HIGH) {
      dx = COST_OBS;
    }
    if (potarr[n - nx] < POT_HIGH) {
      dy = -COST_OBS;
    } else if (potarr[n + nx] < POT_HIGH) {
      dy = COST_OBS;
    }
  } else {  // not in an obstacle
    // dx calc, average to sides
    if (potarr[n - 1] < POT_HIGH) {
      dx += potarr[n - 1] - cv;
    }
    if (potarr[n + 1] < POT_HIGH) {
      dx += cv - potarr[n + 1];
    }

    // dy calc, average to sides
    if (potarr[n - nx] < POT_HIGH) {
      dy += potarr[n - nx] - cv;
    }
    if (potarr[n + nx] < POT_HIGH) {
      dy += cv - potarr[n + nx];
    }
  }

  // normalize
  float norm = hypot(dx, dy);
  if (norm > 0) {
    norm = 1.0 / norm;
    gradx[n] = norm * dx;
    grady[n] = norm * dy;
  }
  return norm;
}

float NavFn::getLastPathCost()
{
  return last_path_cost_;
}

}  // namespace nav2_bd_navfn_planner
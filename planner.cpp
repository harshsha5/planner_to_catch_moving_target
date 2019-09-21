/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <climits>
#include <unordered_map>
#include <chrono>

using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 9

//#######################################################################################################################

struct coordinate
{

public:
    pair<int,int> point;
    double gcost;
    double hcost;
    double fcost;
    int time_to_reach;
    friend bool operator== (const coordinate &c1, const coordinate &c2);
    friend bool operator!= (const coordinate &c1, const coordinate &c2);
    friend bool operator< (const coordinate &c1, const coordinate &c2);
//    friend ostream& operator << (ostream &out, const coordinate &c);

    void update_fcost()
    {
        const double epsilon=0; //Dijkastra
        fcost = gcost + (epsilon*hcost);
    }

    void update_hcost()
    {
        /// This is the present heuristic (Euclidian for now)
        //hcost = (double)sqrt(((point.first-goal_position.first)*(point.first-goal_position.first) + (point.second-goal_position.second)*(point.second-goal_position.second)));
        hcost = 0; //Let's do Dijkastra first.
    }

    coordinate(int x,
               int y,
               int time):
               point(make_pair(x,y)),
               gcost(INT_MAX),
               hcost(0){
        update_fcost();
    }
};

bool operator== (const coordinate &c1, const coordinate &c2)
{
    return (c1.point.first== c2.point.first &&
            c1.point.second== c2.point.second);
}

bool operator!= (const coordinate &c1, const coordinate &c2)
{
    return !(c1== c2);
}

bool operator< (const coordinate &c1, const coordinate &c2)
{
    return c1.gcost < c2.gcost;
}

struct Comp{
    bool operator()(const coordinate &a, const coordinate &b){
        return a.gcost>b.gcost;
    }
};

struct custom_coord_compare{
    bool operator()(const coordinate &c1, const coordinate &c2) const{
        return !(c1.point.first== c2.point.first &&
                c1.point.second== c2.point.second);
    }
};

//#######################################################################################################################
//Global Variable declaration

vector<coordinate> best_trajectory;

//#######################################################################################################################

void debug_result(const coordinate &c,const int &flag = 100)
{   //flag=1 then that is debugging expanded state.
    //flag=0 then that is the state we are adding to the open list
    //flag=-1 then that is the state which is our start state
    if(flag==0)
        cout<<"Adding point: "<<c.point.first<<"\t"<<c.point.second<<"\t"<<c.gcost<<"\t"<<c.time_to_reach<<endl;
    else if(flag==-1)
        cout<<"Start Pose: "<<c.point.first<<"\t"<<c.point.second<<"\t"<<c.gcost<<"\t"<<c.time_to_reach<<endl;
    else if(flag==1)
        cout<<"Expanding State: "<<c.point.first<<"\t"<<c.point.second<<"\t"<<c.gcost<<"\t"<<c.time_to_reach<<endl;
    else
        cout<<c.point.first<<"\t"<<c.point.second<<"\t"<<c.gcost<<"\t"<<c.time_to_reach<<endl;
}

//#######################################################################################################################

void print_priority_queue(priority_queue<coordinate, vector<coordinate>, Comp> pq)
{
    while (!pq.empty())
    {
        debug_result(pq.top(),0);
        pq.pop();
    }
}

//#######################################################################################################################

void print_coordinate_set(set<coordinate> seto)
{
    set <coordinate> :: iterator itr;
    for (itr = seto.begin(); itr != seto.end(); ++itr)
    {
        debug_result(*itr);
    }
}

//#######################################################################################################################

int hash_coordinate(const int &x,
                    const int &y,
                    const int &n_col)
{
    return x*n_col + y;
}

//#######################################################################################################################

pair<int,int> unhash_coordinate(const int &hash,
                                const int &n_col)
{
    return make_pair((int)(hash/n_col),(int)(hash%n_col));
}

//#######################################################################################################################

void expand_state(const coordinate &state_to_expand,
                  priority_queue<coordinate, vector<coordinate>, Comp> &open,
                  vector<vector<coordinate>> &cost_map,
                  const int* dX,
                  const int* dY,
                  const int &x_size,
                  const int &y_size,
                  const double*	map,
                  const set<coordinate,custom_coord_compare> &closed,
                  const int &collision_thresh,
                  const set<pair<int,int>> &target_trajectory_points)
{
    const auto current_x = state_to_expand.point.first;
    const auto current_y = state_to_expand.point.second;        //These are both zero indexed

    for(size_t dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = current_x + 1 + dX[dir];
        int newy = current_y + 1 + dY[dir];                     //new_x and new_y are 1 indexed

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                if(closed.count(cost_map[newx-1][newy-1])==0 && (cost_map[newx-1][newy-1].gcost > cost_map[current_x][current_y].gcost + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]))
                {
                    /// Note here we checked with closed list and expand only those which haven't been expanded. See if this is in sync with staying stationary.
                    cost_map[newx-1][newy-1].gcost = cost_map[current_x][current_y].gcost + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                    cost_map[newx-1][newy-1].update_fcost();    //Don't forget this-> because everytime g/h changes f changes
                    cost_map[newx-1][newy-1].time_to_reach = cost_map[current_x][current_y].time_to_reach + 1;
                    open.push(cost_map[newx-1][newy-1]);
                }
            }
        }
    }
}

//#######################################################################################################################

pair<int,int> find_least_cost_path(unordered_map<int,int> point_count,
                                   unordered_map<int,vector<int>> point_time,
                                   const vector<vector<coordinate>> &cost_map,
                                   const int &time_taken_to_plan,
                                   const int &x_size,
                                   const int &y_size,
                                   const double* map)
{   pair<int,int> best_coordinate;
    double least_g_cost = INT_MAX;
    for(auto q:point_count)
    {
        const auto curr_point = unhash_coordinate(q.first,y_size);
        /// The below if condition needs to be altered to take into account all times in point_time.
        if(point_time[q.first].size()<=0)
        {
            continue;
        }
//        cout<<"Time to reach the point by our robot: "<<cost_map[curr_point.first][curr_point.second].time_to_reach<<endl;
//        cout<<"Time taken by the target to reach that point"<<point_time[q.first][point_time[q.first].size()-1]<<endl;
        const auto time_diff = time_taken_to_plan+cost_map[curr_point.first][curr_point.second].time_to_reach - point_time[q.first][point_time[q.first].size()-1];
//        cout<<"time_diff = "<<time_diff<<endl;
//        cout<<"==================================================="<<endl;
        const double cost_to_reach_and_wait = cost_map[curr_point.first][curr_point.second].gcost + (-1*time_diff*(int)map[GETMAPINDEX(curr_point.first+1,curr_point.second+1,x_size,y_size)]);
        if(time_diff<0 && cost_to_reach_and_wait<least_g_cost)
        {
            least_g_cost = cost_to_reach_and_wait;
            best_coordinate = curr_point;
        }

    }
    return best_coordinate;
}

//#######################################################################################################################

vector<coordinate> backtrack(const vector<vector<coordinate>> &cost_map,
                        const int* dX,
                        const int* dY,
                        const int &x_size,
                        const int &y_size,
                        const coordinate &goal_coordinate,
                        const coordinate &start_coordinate)
{
    vector<coordinate> stack;
    stack.push_back(goal_coordinate);
    auto curr_coordinate = goal_coordinate;
    while(curr_coordinate!=start_coordinate)
    {
        double g_min=INT_MAX;
        int bestx = -1;
        int besty = -1;
        for(size_t dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = curr_coordinate.point.first + dX[dir];
            int newy = curr_coordinate.point.second + dY[dir];
            // newx and newy are 0 indexed since goal and start coords are zero indexed in the planner function
            if (newx >= 0 && newx < x_size && newy >= 0 && newy < y_size)
            {
                if(cost_map[newx][newy].gcost<g_min)
                {
                    g_min = cost_map[newx][newy].gcost;
                    bestx = newx;
                    besty = newy;
                }
            }
        }
        curr_coordinate = coordinate(bestx,besty,INT_MAX);
        stack.push_back(curr_coordinate);
    }

    return std::move(stack);
}

//#######################################################################################################################

set<pair<int,int>> get_target_trajectory_and_count(const double* target_traj,
                                               const int &target_steps,
                                               unordered_map<int,int> &point_count,
                                               unordered_map<int,vector<int>> &point_time,
                                               const int &y_size)
{   /// This extracts all the target poses from the given target trajectory
    /// Adds the unique ones in the set
    /// But keeps track of duplication in the unordered_map
    set<pair<int,int>> target_trajectory_set;
    for (int i=1; i<=target_steps;i++)
    {
        //cout<<(int) target_traj[target_steps-1]<<"\t"<<(int) target_traj[target_steps-1+target_steps]<<endl;
        auto target_pose = make_pair((int) target_traj[i-1],(int) target_traj[i-1+target_steps]);
        int curr_x = target_pose.first;
        int curr_y = target_pose.second;
        const auto hashed_coordinate = hash_coordinate(curr_x,curr_y,y_size);

        if(point_count.count(hashed_coordinate)==0)
        {
            point_count[hashed_coordinate] = 1;
            point_time[hashed_coordinate] = vector<int> {i-1};
        }
        else
        {
            point_count[hashed_coordinate]++;
            point_time[hashed_coordinate].push_back(i-1);
        }

        target_trajectory_set.insert(target_pose);
    };
    return std::move(target_trajectory_set);
}

//#######################################################################################################################

bool are_all_trajectory_points_covered(const set<pair<int,int>> &target_trajectory_points,
                                       const set<coordinate,custom_coord_compare> &closed)
{   // This does have a bug in the sense that if the last n steps are repeated. It will skip those n steps in the second+ iterations and terminate the search early
    for(const auto &point:target_trajectory_points)
    {
        const coordinate coordinate_to_check(point.first,point.second,INT_MAX);
        if(closed.count(coordinate_to_check)==0)
            return false;
    }
    return true;
}

//#######################################################################################################################

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    if(curr_time==0)
    {
        auto start = std::chrono::high_resolution_clock::now();

        // 9-connected grid
        int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
        int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

        priority_queue<coordinate, vector<coordinate>, Comp> open;
        set<coordinate,custom_coord_compare> closed;
        const coordinate random_init_coordinate(-1,-1,INT_MAX);
        vector<vector<coordinate>> cost_map(x_size,vector<coordinate> (y_size,random_init_coordinate));
        const coordinate start_coordinate(robotposeX-1,robotposeY-1,curr_time);

        //CREATE COST_MAP
        for(size_t i=0;i<x_size;i++)
        {
            for(size_t j=0;j<y_size;j++)
            {
                cost_map[i][j] = coordinate(i,j,INT_MAX);
            }
        }

        /// At this point we have initialized all g values to inf and update all h_costs and f_costs according to heuristics
        cost_map[robotposeX-1][robotposeY-1].time_to_reach = curr_time;
        cost_map[robotposeX-1][robotposeY-1].gcost = (int)map[GETMAPINDEX(robotposeX,robotposeY,x_size,y_size)]; //See if -1 here also
        cost_map[robotposeX-1][robotposeY-1].update_fcost();
        open.push(cost_map[robotposeX-1][robotposeY-1]);

        /// This stores the point and the various times to reach those points
        unordered_map<int,vector<int>> point_time;      //This is only for target_points

        /// This stores the point and the count of the various times it was encountered in the trajectory
        unordered_map<int,int> point_count;             //This is only for target_points

        const auto target_trajectory_points = get_target_trajectory_and_count(target_traj,target_steps,point_count,point_time,y_size);

        while (!are_all_trajectory_points_covered(target_trajectory_points,closed) && !open.empty())
        {
            const auto state_to_expand = open.top();
//            cout<<"Probable Expansion "<<endl;
//            debug_result(state_to_expand);
            //cout<<"=================Printing closed set========================================"<<endl;
            //print_coordinate_set(closed);
            if(closed.count(state_to_expand)==0)       //Added this new condition to avoid multiple expansion of the same state
            {
                closed.insert(state_to_expand);
                debug_result(state_to_expand,1);
                cout<<"======================================================="<<endl;
                expand_state(state_to_expand,open,cost_map,dX,dY,x_size,y_size,map,closed,collision_thresh,target_trajectory_points);
//                print_priority_queue(open);
//                cout<<"======================================================="<<endl;
            }
            else
            {cout<<"Not doing shit"<<endl;}

            open.pop();
        }
        cout<<"Relevant states expanded"<<endl;
        auto stop = std::chrono::high_resolution_clock::now();
        auto time_taken_to_plan = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        cout<<"Planning time: "<<time_taken_to_plan.count()<<endl;

        const auto goal_point = find_least_cost_path(point_count,std::move(point_time),cost_map,(int)time_taken_to_plan.count()+1,x_size,y_size,map);
        const coordinate goal_coordinate(goal_point.first,goal_point.second,INT_MAX);
        cout<<"Goal coordinate to target"<<endl;
        debug_result(goal_coordinate);
        if(point_count.count(hash_coordinate(goal_coordinate.point.first,goal_coordinate.point.first,y_size))!=0)
            cout<<"Valid goal target"<<endl;
        const auto trajectory_obtained = backtrack(cost_map,dX,dY,x_size,y_size,goal_coordinate,start_coordinate);
        cout<<trajectory_obtained.size()<<" ======================================="<<endl;
        for(const auto x:trajectory_obtained)
        {
            //cout<<x.point.first<<"\t"<<x.point.second<<"\n";
            best_trajectory.push_back(x);
        }
////      best_trajectory = vector<coordinate> (trajectory_obtained.size(),random_init_coordinate)
////      best_trajectory = backtrack(cost_map,dX,dY,x_size,y_size,goal_coordinate,start_coordinate);
    }

    if(!(best_trajectory.size()-2-curr_time<0))
    {
        action_ptr[0] = best_trajectory[best_trajectory.size()-2-curr_time].point.first;
        action_ptr[1] = best_trajectory[best_trajectory.size()-2-curr_time].point.second;
    }
    else
    {
        action_ptr[0] = best_trajectory[0].point.first;
        action_ptr[1] = best_trajectory[0].point.second;
    }
//
//    action_ptr[0] = robotposeX;
//    action_ptr[1] = robotposeY;

    return;
}

// ###################################################################################################################

// Ideas for the planner
// First simply code an A* with Euclidian heuristic and see how it performs.
// You can then try the same with the look-ahead method
// Try saving the last distance from the goal to the start and use it to compute your look-ahead.
// See if implicit graphs can make the planner more efficient
// Now change heuristic and see how code performs

// ###################################################################################################################

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make

//#######################################################################################################################

void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}

// Do a simple Dijkastra from the start pose to all the positions in the target trajectory. Save the cost to reach that point
// and time to reach each of these points. Then run a loop to see which all points we can cover in this time.
// ie. Time for robot to reach that point + current_time + Planning time < Time taken for target to reach that point.
// Once you identify all these points you can see which has the min g-value.
// Then you simply backtrack and execute that trajectory. So no future planning is required.
// Add a const time (say 2s as buffer for search within the array)
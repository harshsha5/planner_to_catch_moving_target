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

class planning_essentials
{
    double big_look_ahead_time;
    double small_look_ahead_time;

public:
    planning_essentials(double big_look_ahead,
                        double small_look_ahead):
                        big_look_ahead_time(big_look_ahead),
                        small_look_ahead_time(small_look_ahead){}
};

//#######################################################################################################################

class planner_return
{
    pair<int,int> next_step;
    int steps_to_goal; /// These steps are for the least cost path and not for the shortest distance

public:
    planner_return(pair<int,int> next_steps,
                    int steps_to_goal_from_start):
                    next_step(next_steps),
                    steps_to_goal(steps_to_goal_from_start){}
};

//#######################################################################################################################

struct coordinate
{

public:
    pair<int,int> point;
    double gcost;
    double hcost;
    double fcost;
    friend bool operator== (const coordinate &c1, const coordinate &c2);
    friend bool operator!= (const coordinate &c1, const coordinate &c2);
    friend bool operator< (const coordinate &c1, const coordinate &c2);
//    friend ostream& operator << (ostream &out, const coordinate &c);

    void update_fcost()
    {
        fcost = gcost + hcost;
    }

    void update_hcost(pair<int,int> goal_position)
    {
        /// This is the present heuristic (Euclidian for now)
        hcost = (double)sqrt(((point.first-goal_position.first)*(point.first-goal_position.first) + (point.second-goal_position.second)*(point.second-goal_position.second)));
    }

    coordinate(int x,
               int y,
               pair<int,int> target_pos):
               point(make_pair(x,y)),
               gcost(INT_MAX){
        update_hcost(target_pos);
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
    return c1.fcost < c2.fcost;
}

//ostream& operator << (ostream &out, const coordinate &c)
//{
//    out << c.point.first;
//    out <<",";
//    out << c.point.second;
//    out << endl;
//    out << gcost <<"\t"<<hcost<<"\t"<<fcost<<endl;
//    return out;
//}

struct Comp{
    bool operator()(const coordinate &a, const coordinate &b){
        return a.fcost<b.fcost;
    }
};

//#######################################################################################################################
//Global Variable declaration

planning_essentials p{3.0,1.0}; /// This is hard-coded as of now. But think over this and alter it according to mean planning time.

//#######################################################################################################################

void expand_state(const coordinate &state_to_expand,
                  priority_queue<coordinate, vector<coordinate>, Comp> &open,
                  vector<vector<coordinate>> &cost_map,
                  const int* dX,
                  const int* dY,
                  const int &x_size,
                  const int &y_size,
                  const double*	map,
                  const set<coordinate> &closed,
                  const int &collision_thresh)
{
    const auto current_x = state_to_expand.point.first;
    const auto current_y = state_to_expand.point.second;        //These are both zero indexed

    for(size_t dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = current_x + 1 + dX[dir];
        int newy = current_y + 1 + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                if(closed.count(cost_map[newx-1][newy-1])==0 && cost_map[newx-1][newy-1].gcost > cost_map[current_x-1][current_y-1].gcost + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)])
                {
                    cost_map[newx-1][newy-1].gcost = cost_map[current_x-1][current_y-1].gcost + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                    cost_map[newx-1][newy-1].update_fcost();    //Don't forget this-> because everytime g/h changes f changes
                    open.push(cost_map[newx-1][newy-1]);
                }
            }
        }
    }
}

//#######################################################################################################################

pair<int,int> backtrack(vector<vector<coordinate>> &cost_map,
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
        curr_coordinate = coordinate(bestx,besty,goal_coordinate.point);
        stack.push_back(curr_coordinate);
    }

    return stack[stack.size()-2].point; //Last point is start, I need to return the point before that

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
    // 9-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

    priority_queue<coordinate, vector<coordinate>, Comp> open;
    set<coordinate> closed;
    vector<vector<coordinate>> cost_map;
    const coordinate goal_coordinate(targetposeX-1,targetposeY-1,make_pair(targetposeX-1,targetposeY-1));
    const coordinate start_coordinate(robotposeX-1,robotposeY-1,make_pair(targetposeX-1,targetposeY-1));

    /// As of now we are planning for the goal position. Add look-ahead and see if performance improves.
    /// Also see how to keep a variables scope till lifetime. ie. it doesn't looses it's value. Once the planner outputs its value

    //CREATE COST_MAP
    for(size_t i=0;i<x_size;i++)
    {
        for(size_t j=0;j<y_size;j++)
        {
            cost_map[i][j] = coordinate(i,j,make_pair(targetposeX-1,targetposeY-1));
        }
    }
    /// At this point we have initialized all g values to inf and update all h_costs and f_costs according to heuristics
    cost_map[robotposeX-1][robotposeY-1].gcost = (int)map[GETMAPINDEX(robotposeX,robotposeY,x_size,y_size)]; //See if -1 here also
    cost_map[robotposeX-1][robotposeY-1].update_fcost();

    open.push(cost_map[robotposeX-1][robotposeY-1]);

    cout<<"Start_pose = "<<open.top().point.first<<","<<open.top().point.second<<endl;
    cout<<endl;
    cout<<"Goal pose = "<<goal_coordinate.point.first<<","<<goal_coordinate.point.second<<endl;
    cout<<endl<<"======================================================================="<<endl;

    while (!open.empty() || closed.count(goal_coordinate)==0)
    {
        const auto state_to_expand = open.top();
        open.pop();
        closed.insert(state_to_expand);
        expand_state(state_to_expand,open,cost_map,dX,dY,x_size,y_size,map,closed,collision_thresh);
//        cout<<"Expanded state was: "<<state_to_expand<<endl;
//        cout<<"======================================================="<<endl;
    }

    const auto best_next_coordinate = backtrack(cost_map,dX,dY,x_size,y_size,goal_coordinate,start_coordinate);
    robotposeX = best_next_coordinate.first;
    robotposeY = best_next_coordinate.second;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;

    return;
}

// ###################################################################################################################

// Ideas for the planner
// First simply code an A* with Euclidian heuristic and see how it performs.
// You can then try the same with the look-ahead method
// Try saving the last distance from the goal to the start and use it to compute your look-ahead.
// See if implicit graphs can make the planner more efficient
// Now change heuristic and see how code performs

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
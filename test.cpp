//
// Created by Harsh Sharma on 14/09/19.
//

#pragma once

#include "planner.cpp"

int main()
{
    int collision_thresh = 10;
    double* map = [8,8,8,
                   8,10,8,
                   8,10,8,
                   8,8,10];
    int x_size = 4;
    int y_size = 3;
    int robotposeX = 3;
    int robotposeY = 2;
    int target_steps = 10;
    double* target_traj = [1,1,1,1,1,1,1,1,1,1,
                           3,3,3,3,3,3,3,3,3,3];
    int targetposeX = 1;
    int targetposeY = 3;
    int curr_time = 0;
    double* action_ptr = 0;
    planner(map,collision_thresh,x_size,y_size,robotposeX,
            robotposeY,target_steps,target_traj,targetposeX,targetposeY,curr_time,)
    return 0;
}
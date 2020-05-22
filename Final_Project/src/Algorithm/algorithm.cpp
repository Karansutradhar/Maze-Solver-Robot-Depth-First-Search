/**
* @file algorithm.cpp
* @authors Group 4
*
* Karan Sutradhar (117037272)
* Sudharsan Balasubramani (116298636)
* Sai Bhamidipati (117023640)
* Ashwin Prabhakaran (117030402)
*
* @version 1.0
*
* @section LICENSE
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* @section DESCRIPTION
*
*  This is implementation of Maze solver and Depth first search algorithm.
*/

#include "algorithm.h"
#include "../API/api.h"
#include "../Maze/maze.h"

/**
 * @brief clear the stack
 * @param none
 * @return none
 */
void fp::Algorithm::ClearStack() {
    while(!this->stack_.empty()) this->stack_.pop();
}

/**
 * @brief takes a node as input and verify whether the node has been explored
 * @param std::array<int, 2> cur_node
 * @return Returns true if the node is already explored or else false
 */
bool fp::Algorithm ::IsExplored(std::array<int, 2> cur_node){
    return this->explored_node_.at(cur_node.at(0)).at(cur_node.at(1));
}

/**
 * @brief takes a node as input and verify whether the node has been visited by  robot
 * @param std::array<int, 2> cur_node
 * @return Returns true if the node is already explored or else false
 */
bool fp::Algorithm ::IsVisited(std::array<int, 2> cur_node){
    return this->visited_node_.at(cur_node.at(0)).at(cur_node.at(1));
}

/**
 * @brief Add the neighbour to the stack
 * @param std::array<int, 2> cur_node
 * @param std::array<int, 2> neighbour
 * @return Returns boolean
 */
bool fp::Algorithm::AddNeighbour(std::array<int, 2> cur_node, std::array<int, 2> neighbour) {
    if(!IsExplored(neighbour)) {
        this->stack_.push(neighbour);
        this->node_info[neighbour[0]][neighbour[1]].parent_node_ = cur_node;
        if (neighbour == this->goal1_ || neighbour == this->goal2_ ||
            neighbour == this->goal3_||neighbour == this->goal4_) {
            this->temp_goal_ = true;
            return true;
        }
    }
    return false;
}
/**
 * @brief finds the neighbouring nodes of a given input node and add the found neighbours to the stack
 * @param std::array<int, 2> cur_node
 * @return none
 */
void fp::Algorithm::FindNeighbours(std::array<int, 2> cur_node) {

    char robotDirection{this->robot_->GetDirection()};
    bool N{this->maze_info.North_[cur_node[0]][cur_node[1]]},
    S{this->maze_info.South_[cur_node[0]][cur_node[1]]},
    E{this->maze_info.East_[cur_node[0]][cur_node[1]]},
    W{this->maze_info.West_[cur_node[0]][cur_node[1]]};

    //---> Compute the neighbouring possible  nodes <---//
    std::array<int, 2> node_N{cur_node[0] - 1, cur_node[1]},
    node_W{cur_node[0], cur_node[1] - 1},
    node_S{cur_node[0] + 1, cur_node[1]},
    node_E{cur_node[0], cur_node[1] + 1};

    //---> Adds neighbouring nodes to the stack based on the robot direction in the maze <---//
    if(robotDirection == 'N'){
        if (!this->temp_goal_ && node_W[1] >= 0 && !W) AddNeighbour(cur_node, node_W);
        if (!this->temp_goal_ && node_N[0] >= 0 && !N) AddNeighbour(cur_node, node_N);
        if (!this->temp_goal_ && node_E[1] <= 15 && !E) AddNeighbour(cur_node, node_E);
        if (!this->temp_goal_ && node_S[0] <= 15 && !S) AddNeighbour(cur_node, node_S);

    }
    else if(robotDirection == 'S') {
        if (!this->temp_goal_ && node_E[1] <= 15 && !E) AddNeighbour(cur_node, node_E);
        if (!this->temp_goal_ && node_S[0] <= 15 && !S) AddNeighbour(cur_node, node_S);
        if (!this->temp_goal_ && node_W[1] >= 0 && !W) AddNeighbour(cur_node, node_W);
        if (!this->temp_goal_ && node_N[0] >= 0 && !N) AddNeighbour(cur_node, node_N);
    }
    else if(robotDirection == 'E') {
        if (!this->temp_goal_ && node_N[0] >= 0 && !N) AddNeighbour(cur_node, node_N);
        if (!this->temp_goal_ && node_E[1] <= 15 && !E) AddNeighbour(cur_node, node_E);
        if (!this->temp_goal_ && node_S[0] <= 15 && !S) AddNeighbour(cur_node, node_S);
        if (!this->temp_goal_ && node_W[1] >= 0 && !W) AddNeighbour(cur_node, node_W);
    }
    else if(!this->temp_goal_ && robotDirection == 'W') {
        if (!this->temp_goal_ && node_S[0] <= 15 && !S) AddNeighbour(cur_node, node_S);
        if (!this->temp_goal_ && node_W[1] >= 0 && !W) AddNeighbour(cur_node, node_W);
        if (!this->temp_goal_ && node_N[0] >= 0 && !N) AddNeighbour(cur_node, node_N);
        if (!this->temp_goal_ && node_E[1] <= 15 && !E) AddNeighbour(cur_node, node_E);
    }
}

/**
 * @brief takes start and implements Depth First Search algorithm
 * @param std::array<int, 2> start
 * @return Returns true if there exist a path to goal
 */
bool fp::Algorithm::DFSAlgorithm(std::array<int, 2> start) {
    std::array<int,2> curr_node{};
    this->temp_goal_=false;
    //---> Reset Stack and exploration History <---//
    this->ClearStack();
    this->explored_node_={};
    /* ---> Step 01: Add start to node <--- */
    this->stack_.push(start);
    /* ---> Step 02: Initialize and assign parent node to start node <--- */
    this->node_info = std::array<std::array<Node, 16>, 16>();
    this->node_info[start[0]][start[1]].parent_node_ = start;
    /* ---> Step 03: loop until stack_ exhausts <--- */
    while(!this->stack_.empty()) {
        /* ---> Step 04: Set Current Node <--- */
        curr_node = stack_.top();                                                                   //--> Set the top last-in node as current node
        this->stack_.pop();                                                                         //--> Pop/remove the last element from the stack
        /* ---> Step 05: check IsGoal <--- */
        if (curr_node == this->goal1_ || curr_node == this->goal2_ ||
            curr_node == this->goal3_ || curr_node == this->goal4_) {
            this->end_goal_ = curr_node;
            return true;
        }
        /* ---> Step 06: Check Visited <--- */
        else if (!IsExplored(curr_node)) FindNeighbours(curr_node);
        /* ---> Step 07: Mark visited <--- */
        this->explored_node_[curr_node[0]][curr_node[1]] = true;
    }
    return false;
}

/**
 * @brief Back track to the start node from a given node
 * @param std::array<int, 2> start
 * @return Returns std::stack<std::array<int, 2>>
 */
std::stack<std::array<int, 2>> fp::Algorithm::BackTrack(std::array<int, 2> current_node, std::array<std::array<Node, 16>, 16>& node) {
    std::array<int, 2> parent_node  = node[current_node[0]][current_node[1]].parent_node_;
    this->path_stack_.push(current_node);
    while (current_node != parent_node) {
        fp::Maze::ColorPath(current_node);
        current_node = parent_node;
        parent_node = node[current_node[0]][current_node[1]].parent_node_;
        this->path_stack_.push(current_node);
    }
    return this->path_stack_;
}

/**
 * @brief Maze solver algorithm using given search algorithm
 * @param std::array<int, 2> start
 * @return Returns std::stack<std::array<int, 2>>
 */
void fp::Algorithm::Solve(const std::shared_ptr<fp::LandBasedRobot>& robot) {
    bool path {};
    this->robot_= robot;
    char curr_direction{};
    std::array<int,2> curr_node{};
    std::stack<std::array<int, 2>> local_path{};
    curr_direction = robot->GetDirection();
    curr_node = {robot->get_x_(), robot->get_y_()};
    this->node_info[curr_node[0]][curr_node[1]].parent_node_ = curr_node;
    this->node_master_[curr_node[0]][curr_node[1]].parent_node_ = this->parent_node_;

    //---> Step 01: Clear all tile color <---//
    SetDefaults();
    while(true){
        //---> Step 02: Mark Current Node Visited <---//
        this->visited_node_[curr_node[0]][curr_node[1]] = true;
        //---> Step 03: Read walls around the robot <---//
        this->maze_info.ReadMaze(curr_node, curr_direction);
        if (this->path_blocked) {
            //---> Step 04: Generate Path using DFS Algorithm <---//
            path = this->DFSAlgorithm(curr_node);
            SetDefaults();
            //---> Step 05: BackTrack the current path <---//
            if(path) {
                local_path = this->BackTrack(this->end_goal_, this->node_info);
                this->path_blocked = false;
            }
            else break;
        }
        //---> Step 06: Navigate to next node <---//
        this->Navigate(local_path);
        //---> Step 07: Update the parent node <---//
        this->parent_node_ = curr_node;
        //---> Step 08: Update current node<---//
        curr_direction = robot->GetDirection();
        curr_node = {robot->get_x_(), robot->get_y_()};
        //---> Step 09: Update the parent node <---//
        if (!IsVisited(curr_node)) this->node_master_[curr_node[0]][curr_node[1]].parent_node_ = this->parent_node_;
        //---> Step 10: Check for goal <---//
        if (curr_node == this->goal1_ || curr_node == this->goal2_ ||
            curr_node == this->goal3_ || curr_node == this->goal4_) {
            SetDefaults();
            this->end_goal_ = curr_node;
            fp::API::setColor(curr_node[1],15-curr_node[0], 'r');
            return;
        }
    }
    std::cerr<<"\nNo path found!\n"<<std::endl;
}

/**
 * @brief Maze solver algorithm using given search algorithm
 * @param std::array<int, 2> start
 * @return Returns std::stack<std::array<int, 2>>
 */
void fp::Algorithm::Navigate(std::stack<std::array<int, 2>>& local_path) {
    int x{},y{};
    char curr_direction{};
    char direction_togo{};
    std::array<int, 2> node_curr{}, node_next{};

    //---> Step 01: Extract the current Node and Next Node <---//
    node_curr = local_path.top();
    local_path.pop();
    node_next = local_path.top();

    //---> Step 02: Compute the togo direction <---//
    x = node_next[0] - node_curr[0];
    y = node_next[1] - node_curr[1];
    curr_direction = this->robot_->GetDirection();

    if(x==-1 && y == 0) direction_togo = 'N';
    else if(x == 1 && y == 0) direction_togo = 'S';
    else if(x == 0 && y ==-1) direction_togo = 'W';
    else if(x == 0 && y == 1) direction_togo = 'E';

    //---> Step 03: Navigate the Robot and update new location and direction info <---//
    if(curr_direction =='N'){
        if(direction_togo == 'N') {
            if(!fp::API::wallFront()) this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            else this->path_blocked = true;
        }
        else if(direction_togo == 'S') {
            this->robot_->TurnLeft();
            this->robot_->TurnLeft();
            this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
        }
        else if(direction_togo == 'E') {
            if(!fp::API::wallRight()) {
                this->robot_->TurnRight();
                this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            }else this->path_blocked = true;
        }
        else if(direction_togo == 'W') {
            if(!fp::API::wallLeft()) {
                this->robot_->TurnLeft();
                this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            }else this->path_blocked = true;
        }
    }
    else if(curr_direction =='S') {
        if(direction_togo == 'N') {
            this->robot_->TurnLeft();
            this->robot_->TurnLeft();
            this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
        }
        else if(direction_togo == 'S') {
            if(!fp::API::wallFront()) this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            else this->path_blocked = true;
        }
        else if(direction_togo == 'E') {
            if(!fp::API::wallLeft()) {
                this->robot_->TurnLeft();
                this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            }else this->path_blocked = true;
        }
        else if(direction_togo == 'W') {
            if(!fp::API::wallRight()){
                this->robot_->TurnRight();
                this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            }else this->path_blocked = true;
        }
    }
    else if(curr_direction =='E') {
        if(direction_togo == 'N') {
            if(!fp::API::wallLeft()) {
                this->robot_->TurnLeft();
                this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            }else this->path_blocked = true;
        }
        else if(direction_togo == 'S') {
            if(!fp::API::wallRight()) {
                this->robot_->TurnRight();
                this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            }else this->path_blocked = true;
        }
        else if(direction_togo == 'E') {
            if(!fp::API::wallFront()) this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            else this->path_blocked = true;
        }
        else if(direction_togo == 'W') {
            this->robot_->TurnLeft();
            this->robot_->TurnLeft();
            this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
        }
    }
    else if(curr_direction =='W') {
        if(direction_togo == 'N') {
            if(!fp::API::wallRight()) {
                this->robot_->TurnRight();
                this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            }else this->path_blocked = true;
        }
        else if(direction_togo == 'S') {
            if(!fp::API::wallLeft()) {
                this->robot_->TurnLeft();
                this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            }else this->path_blocked = true;
        }
        else if(direction_togo == 'E') {
            this->robot_->TurnLeft();
            this->robot_->TurnLeft();
            this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
        }
        else if(direction_togo == 'W') {
            if(!fp::API::wallFront()) this->robot_->MoveForward(node_next[0], node_next[1], direction_togo);
            else this->path_blocked = true;
        }
    }
}

/**
 * @brief set Default Values
 * @param none
 * @return none
 */
void fp::Algorithm::SetDefaults(){
    this->temp_goal_=false;
    fp::API::clearAllColor();
    fp::API::setColor(0,0,'g');
    fp::API::setColor(this->goal1_[1], 15-this->goal1_[0],'w');
    fp::API::setColor(this->goal2_[1], 15-this->goal2_[0],'w');
    fp::API::setColor(this->goal3_[1], 15-this->goal3_[0],'w');
    fp::API::setColor(this->goal4_[1], 15-this->goal4_[0],'w');
}


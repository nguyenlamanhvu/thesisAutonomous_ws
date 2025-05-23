/*
 * Copyright 2024 Fahim Shahriar
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "../include/robot_navigation/hybrid_local_planner/Edge.h"

namespace hybrid_local_planner {
    Edge::Edge(int p_node_index, double p_cost){
        node_index = p_node_index;
        cost = p_cost;
    }

    int Edge::get_node_index(){
        return node_index;
    }
    
    double Edge::get_cost(){
        return cost;
    }

    bool Edge::operator<(const hybrid_local_planner::Edge &e)const{
        return cost > e.cost;
    }
};
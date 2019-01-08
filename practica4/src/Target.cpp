/*
 * Copyright 2018 <copyright holder> <email>
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

#include "target.h"

/**
 * \brief Default constructor
 **/
Target::Target()
{
    coordenadasRobot.resize(2);
}

/**
* \brief Default destructor
*/
Target::~Target()
{

}

QVec Target::extract(){
     
    std::lock_guard<std::mutex> lock(hiloBloq);
    return coordenadasRobot;
}

void Target::insert(float x, float z)
{
    hiloBloq.lock();
    
    coordenadasRobot[0] = x;
    coordenadasRobot[1] = z;
    pendiente = true;
    hiloBloq.unlock();
}

void Target::setPendiente(bool pend){
    
    std::lock_guard<std::mutex> lock(hiloBloq);
    pendiente = pend;
    
}

bool Target::getPendiente(){
     std::lock_guard<std::mutex> lock(hiloBloq);
    return pendiente;
}
    

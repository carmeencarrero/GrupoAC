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

}

/**
* \brief Default destructor
*/
Target::~Target()
{

}

bool Target::isEmpty(){
    bool vacio;
    hiloBloq.lock();
    
    if (heLlegado)
        vacio = true;
    else
        vacio = false;
    hiloBloq.unlock();
    
  return vacio;
}

pair<float,float> Target::extract(){
    
    hiloBloq.lock();

    hiloBloq.unlock();
     
    return coordenadas;
}

void Target::insert(float x, float z)
{
    hiloBloq.lock();
    
    coordenadas.fi
    
    hiloBloq.unlock();
}

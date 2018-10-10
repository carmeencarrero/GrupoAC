/*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);


	return true;
}

/**
* @brief ...
* 
*/
void SpecificWorker::compute()
{
    const float threshold = 200; // millimeters
    static int cont = 0;
    static int giro = 0;
    
    try
    {
    	// read laser data 
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        
        //para dividir el vector en particiones
        int l = ldata.size()/10;
 
        //ordenamos las posiciones del vector que están dentro del rango por distancia
        std::sort(ldata.begin() + l * 4, ldata.begin() + l * 6 , [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; });
        
        //velocidadAvance = 1000 (velocidadMaxima) * y (distMin/5000-mm que mide la habitacion-)
        float velocidadAdv = 4000 * (ldata[l*4].dist / 5000);
        if (velocidadAdv > 1000) //para asegurarnos que el robot va a 1000 como velocidad maxima
            velocidadAdv = 1000;
        
        float velocidadGiro = 10 / (ldata[l*4].dist / 5000);
         if (velocidadGiro > 2) //para asegurarnos que el robot va a 2 como velocidad maxima
            velocidadGiro = 2;
         
       //ordenamos ahora el vector entero de menor a mayor distancia usando una funcion lambda
        std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; });  
        
	if(ldata.front().dist < threshold)
	{
         if(ldata.front().angle < 0 && giro == 0){
            differentialrobot_proxy->setSpeedBase(0, velocidadGiro); //girar derecha (objeto izquierda)
         }
          else{
          differentialrobot_proxy->setSpeedBase(0, -velocidadGiro); //girar izquierda (objeto derecha)
          giro = 1;
          }
        cont = 0;
    
    usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
	}
	else
	{
        if(cont == 25){
            cont = 0;
        
         int num = 0;
         float r = rand() % 3; //aleatorio entre 0 y 2
         
         while(num != 30){ //para que se gire mas de una vez
         if(ldata.front().angle < 0)
           differentialrobot_proxy->setSpeedBase(0, r);
        else
           differentialrobot_proxy->setSpeedBase(0, -r); 
        num++;
         }
        } else{
            
        cont ++;
        differentialrobot_proxy->setSpeedBase(velocidadAdv, 0); //moverse recto
        giro = 0;
        }
         }
  	}
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::setPick(const Pick &myPick)
{
//subscribesToCODE

}

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
    try
    {

        //Obtener estado de la base
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);

        //Si tiene coordenadas pendientes
        if (coord.getPendiente()) {

            Rot2D baseAngle (bState.alpha); //Matriz en dos dimensiones que son las coordenadas,
            // obtengo las coordenadas del robot con respecto al mundo real

            QVec T = QVec::vec2(bState.x, bState.z);
            QVec Y = coord.extract();
            QVec posicR = baseAngle.invert() * (Y - T);

            float rot = atan2(posicR[0], posicR[1]);
            float f = exp(-rot*rot);
            float dist = posicR.norm2();
            float g = (1./1000.) * dist;
            if (dist > 1000)
                g = 1;
            
            //Calculamos la velocidad
            float veloc = 600 * f * g;
            
            differentialrobot_proxy->setSpeedBase(veloc, rot);

            //Si la distancia es menor que 50 suponemos que se ha llegado al objetivo.
            if (dist < 50) {
                coord.setPendiente(false);
                differentialrobot_proxy->setSpeedBase(0, 0);
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
    coord.insert(myPick.x, myPick.z);
}



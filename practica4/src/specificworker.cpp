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
{}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

    try
    {
        RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
        innerModel = std::make_shared<InnerModel>(par.value);
    }

    catch(std::exception e) {
        std::cout << e.what() << std::endl;
        qFatal("Error reading config params");
    }

    timer.start(Period);

    return true;
}

//Metodo que inserta las coordenadas del mundo
void SpecificWorker::setPick(const Pick &myPick)
{
    coord.insert(myPick.x, myPick.z);
    calcularRecta();
}

/**
* @brief ...
*
*/
void SpecificWorker::compute()
{
  
    try {
    
         //Obtener estado de la base
        differentialrobot_proxy->getBaseState(bState);
        
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        
        ldata = laser_proxy->getLaserData();
        
        switch(state) {

        case State::IDLE:
            
            if (coord.getPendiente())
                state = State::GOTO;
            break;

        case State::GOTO:
            
            gotoTarget();
            break;

        case State::BUG:

            bug();
            break;

        case State::ROTATE:

           rotar();
           break;

        }

    }
    catch(const Ice::Exception &ex) {
        std::cout << ex << std::endl;
    }
}

//Calcula A, B y C
void SpecificWorker::calcularRecta()
{

    QVec final = coord.extract();
    
    QVec inicio;
    inicio.resize(2);
    
    inicio[0] = bState.x;
    inicio[1] = bState.z;
    
    QVec vectorDirector = final - inicio;
    
    A = vectorDirector[1];
    B = - vectorDirector[0];
    C = (vectorDirector[0] * bState.z) - (vectorDirector[1] * bState.x);
}

//Metodo que comprueba si corta la recta o no
bool SpecificWorker::cortaRecta(){    
    
    //Obtener estado de la base
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    
    QVec inicio;
    inicio.resize(2);
    
    inicio[0] = bState.x;
    inicio[1] = bState.z;
    
    float recta = (A * bState.x) + (B * bState.z) + C;
    
        if (fabs(recta) <= 0.5)
            return true;
        else
            return false;
    
}


//Sigue el camino
void SpecificWorker::gotoTarget()
{

    if(obstacle())   // If ther is an obstacle ahead, then transit to ROTATE
      {
        state = State::ROTATE;
     differentialrobot_proxy->setSpeedBase(0, 0);
          return;
    }

    Rot2D baseAngle (bState.alpha); //Matriz en dos dimensiones que son las coordenadas,
    // obtengo las coordenadas del robot con respecto al mundo real

    QVec T = QVec::vec2(bState.x, bState.z);
    QVec Y = coord.extract();
    QVec posicR = baseAngle.invert() * (Y - T);

    float angulo = atan2(posicR[0], posicR[2]);
    float dist = posicR.norm2();

    differentialrobot_proxy->setSpeedBase(600, angulo);
   
    //Si la distancia es menor que 250 suponemos que se ha llegado al objetivo y cambiamos el estado
    if (dist < 250) {
        state = State::IDLE;
        differentialrobot_proxy->setSpeedBase(0, 0);
        coord.setPendiente(false);
        return;
    }
}

//Gira sobre sí mismo hasta que deja de ver el obstaculo
void SpecificWorker::rotar() {

    //para dividir el vector en particiones
    int l = ldata.size()/10;

    //ordenamos las posiciones del vector que están dentro del rango por distancia (ordenamos la parte derecha)
    std::sort(ldata.begin() + l * 2, ldata.begin() + l * 5, [](RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return a.dist < b.dist;
    });
   
    if (ldata[l*5].dist < 450){
        differentialrobot_proxy->setSpeedBase(0, -1); //giramos a izquierda
    }
    else if (ldata[l*2].dist >= 450){
         state = State::BUG; //deja de ver el obstaculo
         differentialrobot_proxy->setSpeedBase(0, 0);
         return;
    }
}

//Bordea el objeto
void SpecificWorker::bug() {

    if (targetAtSight()) {
        state = State::GOTO;
        return;
    }

    if (cortaRecta()){
        state = State::GOTO;
        return;
    }
    
    //ordenamos las posiciones del vector que están dentro del rango por distancia (ordenamos la parte derecha)
    std::sort(ldata.begin(), ldata.begin() + 10, [](RoboCompLaser::TData a, RoboCompLaser::TData b) 
    {
        return a.dist < b.dist;
    });
    
   float dist = ldata.begin()->dist;
   
   if (dist < 300){
       differentialrobot_proxy->setSpeedBase(0, -0.2);
   }
   if (dist >= 300 && dist <= 350){
     differentialrobot_proxy->setSpeedBase(600, 0);
    }
    if (dist > 350){
     differentialrobot_proxy->setSpeedBase(0, 0.2);   
    }
    
        Rot2D baseAngle (bState.alpha); //Matriz en dos dimensiones que son las coordenadas,
    // obtengo las coordenadas del robot con respecto al mundo real

    QVec T = QVec::vec2(bState.x, bState.z);
    QVec Y = coord.extract();
    QVec posicR = baseAngle.invert() * (Y - T);

    float distObj = posicR.norm2();

   if (distObj < 50) {
        state = State::IDLE;
        coord.setPendiente(false);
        differentialrobot_proxy->setSpeedBase(0, 0);
        return;
    }
}

//Hay obstaculo
bool SpecificWorker::obstacle() {

    //para dividir el vector en particiones
    int l = ldata.size()/10;
    auto inicio = ldata.begin() + l * 4;

    //ordenamos las posiciones del vector que están dentro del rango por distancia
    std::sort(inicio, ldata.begin() + l * 6 , [](RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return a.dist < b.dist;
    });

    if(inicio->dist <= 250)
        return true;
    else
        return false;
}

//Ve el objetivo
bool SpecificWorker::targetAtSight() {
    
    TLaserData lasercopy = laser_proxy->getLaserData();
    QPolygonF polygon;
    
    for (auto l:lasercopy){
        QVec lr = innerModel->laserTo("world", "laser", l.dist, l.angle);
                  polygon << QPointF(lr.x(), lr.z());
    }
    
    QVec t = coord.extract();
    return  polygon.containsPoint( QPointF(t[0], t[1] ), Qt::WindingFill);
}

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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "Target.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void setPick(const Pick &myPick);
    void bug();
    bool obstacle();
    bool targetAtSight();
    void gotoTarget();
    void rotar();
    bool cortaRecta();
    void calcularRecta();
    
public slots:
	void compute();

private:
    //puntero inteligente
	std::shared_ptr<InnerModel> innerModel;
    Target coord;
    enum State {BUG, IDLE, GOTO, ROTATE};
    enum State state = IDLE;
    float A, B, C;
    TBaseState bState;
    TLaserData ldata;

};

#endif

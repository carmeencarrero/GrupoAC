/*
 *    Copyright (C)2019 by YOUR NAME HERE
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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>

#include <ui_mainUI.h>

#include <CommonBehavior.h>

#include <Laser.h>
#include <GenericBase.h>
#include <DifferentialRobot.h>
#include <GenericBase.h>
#include <RCISMousePicker.h>
#include <GotoPoint.h>
#include <AprilTags.h>
#include <GenericBase.h>
#include <JointMotor.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompDifferentialRobot;
using namespace RoboCompGenericBase;
using namespace RoboCompGotoPoint;
using namespace RoboCompJointMotor;
using namespace RoboCompLaser;
using namespace RoboCompRCISMousePicker;
using namespace RoboCompAprilTags;




class GenericWorker :
#ifdef USE_QTGUI
public QWidget, public Ui_guiDlg
#else
public QObject
#endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	DifferentialRobotPrx differentialrobot_proxy;
	LaserPrx laser_proxy;

	virtual void go(const string &nodo, const float x, const float y, const float alpha) = 0;
	virtual void turn(const float speed) = 0;
	virtual bool atTarget() = 0;
	virtual void stop() = 0;
	virtual void setPick(const Pick &myPick) = 0;
	virtual void newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState) = 0;
	virtual void newAprilTag(const tagsList &tags) = 0;

protected:
	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif

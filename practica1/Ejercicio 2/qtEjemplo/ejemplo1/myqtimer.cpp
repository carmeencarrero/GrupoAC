#include "myqtimer.h"

MyQtimer::MyQtimer()
{
}

void MyQtimer::start(int num)
{
    stopp=false;
    this->periodo = num;
    QThread::start();
}

void MyQtimer::stop()
{
  stopp = true;
}

bool MyQtimer::isActive()
{
  if(stopp==true){
    return false;
  }
  else
    return true;  
}

void MyQtimer::run()
{
  int periodo_local;
  bool stopp_local;
  while(true)
{
    mutex.lock();
    periodo_local=periodo;
    stopp_local=stopp;
    mutex.unlock();
    if(!stopp_local)
    {
      emit timeout();
    }
    msleep(periodo_local);
}

}
#ifndef MYQTIMER_H
#define MYQTIMER_H

#include "QMutex"
#include <qt4/QtCore/qthread.h>

class MyQtimer : public QThread
{
Q_OBJECT
  public:
    MyQtimer();
  
    void start(int num);
    void stop();
    bool isActive();
    virtual void run();
    
  signals:
  void timeout();
  
  private:
  int periodo;
  bool stopp;
  QMutex mutex;
};

#endif // MYQTIMER_H

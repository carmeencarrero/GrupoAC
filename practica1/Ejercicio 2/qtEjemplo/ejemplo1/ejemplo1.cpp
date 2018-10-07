#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
    connect(&timer, SIGNAL(timeout()), this, SLOT(mostrar()));
    timer.start();
	connect(button, SIGNAL(clicked()), this, SLOT(parar()));
}

void ejemplo1::parar()
{
//static bool primerClick = false;

    
 //   if(!primerClick){
    
    if(timer.isActive()){
        
    timer.stop();
  //  primerClick = true;
    
    }
    else {
  //      primerClick = false;
        timer.start();
        mostrar();
    }
    
}

void ejemplo1::mostrar()
{
    
    lcdNumber->display(lcdNumber->intValue() + 1);
}

ejemplo1::~ejemplo1()
{}




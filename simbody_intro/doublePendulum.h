/*pendulum.h to create our double pendulum object*/

#include <iostream>
#include <stdio.h>
#include "Simbody.h"
using namespace SimTK;

class doublePendulum{

	public:
		doublePuendulum(); //just the default constructor
		
	private:
		MobilizedBody::Pin pendulum1; //upper pendulum
                MobilizedBody::Pin pendulum2;

		

}

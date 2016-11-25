#ifndef LIGHTING_H
#define LIGHTING_H

#include "bRenderer.h"

class Lighting {

public:

	Lighting(ObjectManagerPtr objectManager);

	void setup();


private:

	ObjectManagerPtr _objectManager;

};


typedef std::shared_ptr< Lighting >  LightingPtr;


#endif
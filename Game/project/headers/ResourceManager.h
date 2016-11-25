#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include "bRenderer.h"

class ResourceManager {

public:

	ResourceManager(Renderer* bRenderer);

	void loadResources();


private:

	Renderer* _bRenderer;

};


typedef std::shared_ptr< ResourceManager >  ResourceManagerPtr;


#endif

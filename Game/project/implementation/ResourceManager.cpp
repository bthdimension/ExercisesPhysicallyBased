#include "ResourceManager.h"


ResourceManager::ResourceManager(Renderer* bRenderer) {
	_bRenderer = bRenderer;
}


void ResourceManager::loadResources() {

	// Load Block
	_bRenderer->getObjects()->loadObjModel("block.obj", false, false, false, 2, true, true);

	// Load Sphere
	_bRenderer->getObjects()->loadObjModel("sphere.obj", false, true, false, 2, true, true);

	// Load Base
	_bRenderer->getObjects()->loadObjModel("base.obj", false, false, false, 2, true, true);

	// Load Debug
	_bRenderer->getObjects()->loadObjModel("debug.obj", false, true, false, 2, true, true);
}
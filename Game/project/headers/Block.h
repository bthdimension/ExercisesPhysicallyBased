#ifndef BLOCK_H
#define BLOCK_H

#include "bRenderer.h"
#include "ARigidBodyOctree.h"


class Block : public ARigidBodyOctree {

public:

	Block(ModelPtr model) : ARigidBodyOctree(model) {}

	Block(ModelPtr model, vmml::Vector3f position) : ARigidBodyOctree(model, position) {}

	void update(const double &deltaTime);
	void draw(ModelRendererPtr modelRenderer, int id);


private:

};


#endif

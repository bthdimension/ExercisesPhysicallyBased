#ifndef BLOCK_H
#define BLOCK_H

#include "bRenderer.h"
#include "ARigidBodyOctree.h"


class BlockRigidBody : public ARigidBodyOctree {

public:

	BlockRigidBody(ModelPtr model, vmml::Vector3f position, vmml::Vector3f axesRotation) : ARigidBodyOctree(model, position, axesRotation) {}

	virtual Type getType();

	void update(const double &deltaTime) override;
	void draw(ModelRendererPtr modelRenderer, int id) override;

	bool isFixed();


private:

};


#endif

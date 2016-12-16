#ifndef FLOORRIGIDBODY_H
#define FLOORRIGIDBODY_H

#include "bRenderer.h"
#include "ARigidBodyOctree.h"


class FloorRigidBody : public ARigidBodyOctree {

public:

	FloorRigidBody(ModelPtr model, vmml::Vector3f position, vmml::Vector3f axesRotation) : ARigidBodyOctree(model, position, axesRotation) {}

	void update(const double &deltaTime) override;
	void draw(ModelRendererPtr modelRenderer, int id);

	bool isFixed();

private:

};


#endif

#ifndef SPHERE_H
#define SPHERE_H

#include "bRenderer.h"
#include "ARigidBodyOctree.h"


class SphereRigidBody : public ARigidBodyOctree {

public:
	
	SphereRigidBody(ModelPtr model, vmml::Vector3f position, vmml::Vector3f axesRotation);

	virtual Type getType();

	void update(const double &deltaTime) override;
	void draw(ModelRendererPtr modelRenderer, int id) override;

	bool isFixed();

	float getRadius();

private:

};


#endif

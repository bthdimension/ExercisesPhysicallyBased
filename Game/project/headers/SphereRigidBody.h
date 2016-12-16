#ifndef SPHERE_H
#define SPHERE_H

#include "bRenderer.h"
#include "ARigidBodyOctree.h"


class SphereRigidBody : public ARigidBodyOctree {

public:

	SphereRigidBody(ModelPtr model);
	
	SphereRigidBody(ModelPtr model, vmml::Vector3f position);

	void update(const double &deltaTime) override;
	bool handleCollision(ARigidBodyOctree * collider, CollisionInformation *collisionInformation) override;
	void draw(ModelRendererPtr modelRenderer, int id) override;

private:

};


#endif

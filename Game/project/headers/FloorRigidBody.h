#ifndef FLOORRIGIDBODY_H
#define FLOORRIGIDBODY_H

#include "ARigidBodyOctree.h"


class FloorRigidBody : public ARigidBodyOctree {

public:

	FloorRigidBody(ModelPtr model) : ARigidBodyOctree(model) {}

	FloorRigidBody(ModelPtr model, vmml::Vector3f position) : ARigidBodyOctree(model, position) {}

	void FloorRigidBody::update(const double &deltaTime) override {}

	void FloorRigidBody::handleCollision(ARigidBodyOctree *collider /*, other stuff*/) override	{}

	void draw(ModelRendererPtr modelRenderer, int id) {
		modelRenderer->queueModelInstance(
			"base",
			"base_" + std::to_string(id),
			"camera",
			getWorldMatrix(),
			std::vector<std::string>({ "sun", "moon" }),
			true,
			true
		);
	}

private:

};


#endif

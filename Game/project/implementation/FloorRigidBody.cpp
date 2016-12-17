#include "FloorRigidBody.h"


ARigidBodyOctree::Type FloorRigidBody::getType() {
	return ARigidBodyOctree::Type::VERTICES;
}


void FloorRigidBody::update(const double &deltaTime) {
	updateMatrices();
}


void FloorRigidBody::draw(ModelRendererPtr modelRenderer, int id) {
	modelRenderer->queueModelInstance(
		"base",
		"base" + std::to_string(id),
		"camera",
		getWorldMatrix(),
		std::vector<std::string>({ "sun", "moon" }),
		true,
		true
	);
}


bool FloorRigidBody::isFixed() {
	return true;
}
#include "FloorRigidBody.h"


void FloorRigidBody::update(const double &deltaTime) {
	updateMatrices();
	registerInOctTree();
}


void FloorRigidBody::draw(ModelRendererPtr modelRenderer, int id) {
	modelRenderer->queueModelInstance(
		"sphere", //"base",
		"sphere_" + std::to_string(id),
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
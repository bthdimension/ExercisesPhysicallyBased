#include "BlockRigidBody.h"


ARigidBodyOctree::Type BlockRigidBody::getType() {
	return ARigidBodyOctree::Type::VERTICES;
}


void BlockRigidBody::update(const double &deltaTime) {
	updateMatrices();
	if (getVelocity() * deltaTime > 0) {
		registerInOctTree();
	}
	resetDebugPoints();
}


void BlockRigidBody::draw(ModelRendererPtr modelRenderer, int id) {
	modelRenderer->queueModelInstance(
		"block",
		"block" + std::to_string(id),
		"camera", 
		getWorldMatrix(),
		std::vector<std::string>({ "sun", "moon" }),
		true,
		true
	);
}


bool BlockRigidBody::isFixed() {
	return false;
}

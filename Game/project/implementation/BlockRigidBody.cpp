#include "BlockRigidBody.h"



void BlockRigidBody::update(const double &deltaTime) {
	updateMatrices();
	registerInOctTree();
}


void BlockRigidBody::draw(ModelRendererPtr modelRenderer, int id) {
	modelRenderer->queueModelInstance(
		"sphere", // "block",
		"sphere" + std::to_string(id),
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

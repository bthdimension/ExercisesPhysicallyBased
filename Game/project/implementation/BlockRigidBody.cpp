#include "BlockRigidBody.h"



void BlockRigidBody::update(const double &deltaTime) {
	// TODO do physics of block here

	setPosition(getPosition() + getVelocity() * deltaTime);

	updateMatrices();

	registerInOctTree();
}

void BlockRigidBody::handleCollision(ARigidBodyOctree *collider /*, other stuff*/)
{
	
}


void BlockRigidBody::draw(ModelRendererPtr modelRenderer, int id) {
	modelRenderer->queueModelInstance(
		"block",
		"block_" + std::to_string(id),
		"camera", 
		getWorldMatrix(),
		std::vector<std::string>({ "sun", "moon" }),
		true,
		true
	);
}
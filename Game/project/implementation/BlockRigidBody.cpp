#include "BlockRigidBody.h"



void BlockRigidBody::update(const double &deltaTime) {
	// TODO do physics of block here

	//
    vmml::Vector3f currPos = getPosition();
    
    vmml::Vector3f f(0.f,-9.81f, 0.f);
    
    
    //    v2 = ((m - dt * dforcedv) * v2 + dt * (springForce2 + dampingForce2 - m * g)) / (m - dt * dforcedv - dt * dt * dforcedx );
    
    
    setVelocity(getVelocity() + f * deltaTime);
    
    setPosition(getPosition() + getVelocity() * deltaTime);

	vmml::Vector3f currRot = getAxesRotation();

	currRot.y() = currRot.y() + deltaTime * 0.25 * 3.1412;

	if (getPosition().y() < 1.0) {
		currPos = getPosition();
		currPos.y() = 1.0;
		setPosition(currPos);
	}
    
	updateMatrices();

	registerInOctTree();
}

bool BlockRigidBody::handleCollision(ARigidBodyOctree *collider, CollisionInformation *collisionInformation){
    
    //setVelocity(-getVelocity());
	return false;
	
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

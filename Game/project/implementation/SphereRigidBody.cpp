#include "SphereRigidBody.h"


SphereRigidBody::SphereRigidBody(ModelPtr model, vmml::Vector3f position, vmml::Vector3f axesRotation) : ARigidBodyOctree(model, position, axesRotation) {
	getMeshCollider()->setIsPerfectSphere(true);
}


void SphereRigidBody::update(const double &deltaTime) {


	// TODO do physics of block here

	// just as example:
	setPosition(getPosition() + getVelocity() * deltaTime);

	updateMatrices();

	// only do this if there was a change in position/orientation
	if(getVelocity() * deltaTime > 0)
		registerInOctTree();
}


void SphereRigidBody::draw(ModelRendererPtr modelRenderer, int id) {
	modelRenderer->queueModelInstance(
		"sphere",
		"sphere_" + std::to_string(id),
		"camera", 
		getWorldMatrix(),
		std::vector<std::string>({ "sun", "moon" }),
		true,
		true
	);
}


bool SphereRigidBody::isFixed() {
	return false;
}

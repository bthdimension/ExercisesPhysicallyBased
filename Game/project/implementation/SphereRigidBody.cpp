#include "SphereRigidBody.h"


ARigidBodyOctree::Type SphereRigidBody::getType() {
	return ARigidBodyOctree::Type::SPHERE;
}


SphereRigidBody::SphereRigidBody(ModelPtr model, vmml::Vector3f position, vmml::Vector3f axesRotation) : ARigidBodyOctree(model, position, axesRotation) {
	vmml::Matrix3f momentsOfInertiaObject = vmml::Matrix3f();
	float r = getRadius();
	for (int i = 0; i < 2; i++) {
		momentsOfInertiaObject(i, i) = 0.4 * getMass() * r * r;
	}
	setMomentsOfIntertia(momentsOfInertiaObject);
	getMeshCollider()->setIsPerfectSphere(true);
}


void SphereRigidBody::update(const double &deltaTime) {
	updateMatrices();
	if (getVelocity() * deltaTime > 0) {
		registerInOctTree();
	}
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


float SphereRigidBody::getRadius() {
	return 1.0;
};

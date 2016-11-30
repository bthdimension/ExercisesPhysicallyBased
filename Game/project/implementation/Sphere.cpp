#include "Sphere.h"


Sphere::Sphere(ModelPtr model) : ARigidBodyOctree(model) {
	getMeshCollider()->setIsPerfectSphere(true);
}

Sphere::Sphere(ModelPtr model, vmml::Vector3f position) : ARigidBodyOctree(model, position) {
	getMeshCollider()->setIsPerfectSphere(true);
}


void Sphere::update(const double &deltaTime) {


	// TODO do physics of block here

	// just as example:
	setPosition(getPosition() + getVelocity() * deltaTime);

	updateMatrices();

	// only do this if there was a change in position/orientation
	//if(getVelocity() * deltaTime > 0)
	registerInOctTree();
}


void Sphere::draw(ModelRendererPtr modelRenderer, int id) {

	vmml::Matrix4f modelMatrix = getWorldMatrix();

	modelRenderer->queueModelInstance(
		"sphere",
		"sphere_" + std::to_string(id),
		"camera", 
		modelMatrix,
		std::vector<std::string>({ "sun", "moon" }),
		true,
		true
	);
}

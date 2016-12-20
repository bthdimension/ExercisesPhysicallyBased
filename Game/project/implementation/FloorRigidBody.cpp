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

	std::vector<vmml::Vector3f> de = getDebugPoints();
	for (int i = 0; i < de.size(); i++) {
		modelRenderer->queueModelInstance(
			"debug",
			"debug_" + std::to_string(id),
			"camera",
			vmml::create_translation(de[i]) * vmml::create_scaling(vmml::Vector3f(0.2, 0.2, 0.2)),
			std::vector<std::string>({ "sun", "moon" }),
			true,
			true
			);
	}
}


bool FloorRigidBody::isFixed() {
	return true;
}
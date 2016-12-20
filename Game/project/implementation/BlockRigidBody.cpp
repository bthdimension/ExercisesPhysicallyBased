#include "BlockRigidBody.h"


ARigidBodyOctree::Type BlockRigidBody::getType() {
	return ARigidBodyOctree::Type::VERTICES;
}


void BlockRigidBody::update(const double &deltaTime) {
	updateMatrices();
	if (getVelocity() * deltaTime > 0) {
		registerInOctTree();
	}
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


bool BlockRigidBody::isFixed() {
	return false;
}

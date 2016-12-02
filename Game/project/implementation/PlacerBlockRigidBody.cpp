#include "PlacerBlockRigidBody.h"



void PlacerBlockRigidBody::update(const double &deltaTime) 
{
	vmml::Vector3f pos = getPosition();

	//setPosition(vmml::Vector3f(pos.x(), 0.f, pos.z()));

	BlockRigidBody::update(deltaTime);
}

void PlacerBlockRigidBody::handleCollision(ARigidBodyOctree *collider /*, other stuff*/)
{
	if (!collider->getMeshCollider()->isPerfectSphere()) {
		vmml::AABBf aabbThis = getMeshCollider()->getBoundingVolumeWorldSpace();
		vmml::AABBf aabbCol = collider->getMeshCollider()->getBoundingVolumeWorldSpace();
		float height = aabbCol.getMax().y() - aabbThis.getMin().y();
		setPosition(getPosition() + vmml::Vector3f(0.f, height, 0.f));
	}

	updateMatrices();
}
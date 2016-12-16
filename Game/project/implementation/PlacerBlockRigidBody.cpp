#include "PlacerBlockRigidBody.h"



void PlacerBlockRigidBody::update(const double &deltaTime) 
{
	vmml::Vector3f pos = getPosition();
	vmml::AABBf aabbThis = getMeshCollider()->getBoundingVolumeWorldSpace();
	float height = aabbThis.getMax().y() - aabbThis.getMin().y();
	setPosition(vmml::Vector3f(pos.x(), height, pos.z()));

	/*BlockRigidBody::update(deltaTime);*/
	updateMatrices();

	registerInOctTree();
}


/*bool PlacerBlockRigidBody::handleCollision(ARigidBodyOctree *collider, CollisionInformation *collisionInformation)
{
	if (!collider->getMeshCollider()->isPerfectSphere()) {
		//vmml::AABBf aabbThis = getMeshCollider()->getBoundingVolumeWorldSpace();
		//vmml::AABBf aabbCol = collider->getMeshCollider()->getBoundingVolumeWorldSpace();
		//float height = aabbCol.getMax().y() - aabbThis.getMin().y();
		//setPosition(getPosition() + vmml::Vector3f(0.f, height, 0.f));


		setPosition(getPosition() - collisionInformation->colNormal*collisionInformation->penetratonDepth);

		updateMatrices();		
		return true;
	}
}*/


void PlacerBlockRigidBody::draw(ModelRendererPtr modelRenderer, int id) {
	if (_visible)
		BlockRigidBody::draw(modelRenderer, id);
}
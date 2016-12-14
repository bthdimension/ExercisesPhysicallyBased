#ifndef BLOCK_H
#define BLOCK_H

#include "bRenderer.h"
#include "ARigidBodyOctree.h"


class BlockRigidBody : public ARigidBodyOctree {

public:

	BlockRigidBody(ModelPtr model) : ARigidBodyOctree(model) {}

	BlockRigidBody(ModelPtr model, vmml::Vector3f position, vmml::Vector3f rotationAxes) : ARigidBodyOctree(model, position) { setAxesRotation(rotationAxes); }

	void update(const double &deltaTime) override;
	bool handleCollision(ARigidBodyOctree *collider, vmml::Vector3f &minimumTranslationVector) override;
	void draw(ModelRendererPtr modelRenderer, int id) override;


private:

};


#endif

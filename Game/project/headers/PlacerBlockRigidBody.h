#ifndef PLACERBLOCK_H
#define PLACERBLOCK_H

#include "BlockRigidBody.h"


class PlacerBlockRigidBody : public BlockRigidBody {

public:

	PlacerBlockRigidBody(ModelPtr model) : BlockRigidBody(model) {}

	PlacerBlockRigidBody(ModelPtr model, vmml::Vector3f position, vmml::Vector3f rotationAxes) : BlockRigidBody(model, position, rotationAxes) {}

	void update(const double &deltaTime) override;

	bool handleCollision(ARigidBodyOctree *collider, CollisionInformation *collisionInformation) override;

	void draw(ModelRendererPtr modelRenderer, int id) override;

	void setVisible(bool visibility) { _visible = visibility; }

	bool isVisible() { return _visible; }

private:
	bool _visible = true;
};


#endif

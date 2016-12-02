#ifndef PLACERBLOCK_H
#define PLACERBLOCK_H

#include "BlockRigidBody.h"


class PlacerBlockRigidBody : public BlockRigidBody {

public:

	PlacerBlockRigidBody(ModelPtr model) : BlockRigidBody(model) {}

	PlacerBlockRigidBody(ModelPtr model, vmml::Vector3f position) : BlockRigidBody(model, position) {}

	void update(const double &deltaTime) override;

	void handleCollision(ARigidBodyOctree *collider /*, other stuff*/) override;

private:

};


#endif

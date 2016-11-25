#ifndef I_COLLISION_HANDLER_H
#define I_COLLISION_HANDLER_H

#include "bRenderer.h"
#include "ARigidBody.h"

class ARigidBody; // forward declaration
class Block; // forward declaration
class Sphere; // forward declaration


class CollisionHandler {

public:

	static void collide(ARigidBody* bodyA, ARigidBody* bodyB);

};


#endif

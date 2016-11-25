#include "CollisionHandler.h"


void CollisionHandler::collide(ARigidBody* bodyA, ARigidBody* bodyB) {

	int typeA = bodyA->getType();
	int typeB = bodyB->getType();

	// Collision between two blocks
	if (typeA == ARigidBody::TYPE_BLOCK && typeB == ARigidBody::TYPE_BLOCK) {
		Block* a = (Block*)bodyA;
		Block* b = (Block*)bodyB;

		// TODO
	}


	// Collision between two spheres
	else if (typeA == ARigidBody::TYPE_SPHERE && typeB == ARigidBody::TYPE_SPHERE) {
		Sphere* a = (Sphere*)bodyA;
		Sphere* b = (Sphere*)bodyB;

		// TODO
	}


	// Collision between block and sphere
	else if (typeA == ARigidBody::TYPE_BLOCK && typeB == ARigidBody::TYPE_SPHERE ||
		typeA == ARigidBody::TYPE_SPHERE && typeB == ARigidBody::TYPE_BLOCK) {
		
		Block* a;
		Sphere* b;
		if (typeA == ARigidBody::TYPE_BLOCK) {
			a = (Block*)bodyA;
			b = (Sphere*)bodyB;
		}
		else {
			a = (Block*)bodyB;
			b = (Sphere*)bodyA;
		}

		// TODO
	}
}
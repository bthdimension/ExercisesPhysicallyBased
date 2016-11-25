#include "ARigidBody.h"


void ARigidBody::setOctTree(OctTreeNodePtr octTree) {
	_octTree = octTree;
	_leavesAtWhichBodyIsRegistered = std::vector<OctTreeNode*>();
}


void ARigidBody::registerInOctTree() {
	for (std::vector<OctTreeNode*>::size_type i = 0; i != _leavesAtWhichBodyIsRegistered.size(); i++) {
		_leavesAtWhichBodyIsRegistered[i]->unregisterRigidBody(this);
	}
	_octTree->registerRigidBody(this);
}
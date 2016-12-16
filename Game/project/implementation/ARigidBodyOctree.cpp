#include "ARigidBodyOctree.h"
#include "OctTreeNode.h" // mir rede nie wieder über das


void ARigidBodyOctree::setOctTree(OctTreeNodePtr octTree) {
	_octTree = octTree;
	_leavesAtWhichBodyIsRegistered = std::vector<OctTreeNode*>();
}


void ARigidBodyOctree::registerInOctTree() {
	for (std::vector<OctTreeNode*>::size_type i = 0; i != _leavesAtWhichBodyIsRegistered.size(); i++) {
		_leavesAtWhichBodyIsRegistered[i]->unregisterRigidBody(this);
	}
	_leavesAtWhichBodyIsRegistered = _octTree->registerRigidBody(this);
}


void ARigidBodyOctree::setIndex(int index) {
	_index = index;
}


int ARigidBodyOctree::getIndex() {
	return _index;
}
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


std::vector<Vector3f> ARigidBodyOctree::getVertices() {
	std::vector<Vector3f> worldVertices = std::vector<Vector3f>();
	std::vector<vmml::Vector3f> bodyVertices = getMeshCollider()->getVertices();
	for (std::vector<vmml::Vector3f>::size_type i = 0; i != bodyVertices.size(); i++) {
		worldVertices.push_back(Utils::vec3fVmmlToEigen(getWorldMatrix() * bodyVertices[i]));
	}
	return worldVertices;
}


std::vector<std::vector<Vector3f>> ARigidBodyOctree::getTriangles() {
	std::vector<std::vector<Vector3f>> worldTriangles = std::vector<std::vector<Vector3f>>();
	std::vector<std::vector<vmml::Vector3f>> bodyTriangles = getMeshCollider()->getTriangles("default");
	for (std::vector<std::vector<vmml::Vector3f>>::size_type i = 0; i != bodyTriangles.size(); i++) {
		std::vector<Vector3f> triangle = std::vector<Vector3f>();
		for (std::vector<vmml::Vector3f>::size_type j = 0; j != bodyTriangles[i].size(); j++) {
			triangle.push_back(Utils::vec3fVmmlToEigen(getWorldMatrix() * bodyTriangles[i][j]));
		}
		worldTriangles.push_back(triangle);
	}
	return worldTriangles;
}


void ARigidBodyOctree::addDebugPoint(vmml::Vector3f point) {
	_debugPoints.push_back(point);
}


void ARigidBodyOctree::resetDebugPoints() {
	_debugPoints.clear();
}
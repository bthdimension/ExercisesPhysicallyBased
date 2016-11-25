#include "OctTreeNode.h"


OctTreeNode::OctTreeNode(OctTreeNode* parent, int depth, double x0, double x1, double y0, double y1, double z0, double z1) {

	_parent = parent;

	_depth = depth;

	_x0 = x0;
	_x1 = x1;
	_y0 = y0;
	_y1 = y1;
	_z0 = z0;
	_z1 = z1;

	_numOfElementsInLeaves = 0;

	// Leaf node
	if (depth <= 1) {
		_rigidBodies = std::vector<ARigidBody*>();
	}

	// Not leaf node
	else {

		double x05 = (x0 + x1) / 2.0;
		double y05 = (y0 + y1) / 2.0;
		double z05 = (z0 + z1) / 2.0;

		_childNodes = new OctTreeNode[8]{
			OctTreeNode(this, depth - 1, x0, x05, y0, y05, z0, z05),
			OctTreeNode(this, depth - 1, x0, x05, y0, y05, z05, z1),
			OctTreeNode(this, depth - 1, x0, x05, y05, y1, z0, z05),
			OctTreeNode(this, depth - 1, x0, x05, y05, y1, z05, z1),
			OctTreeNode(this, depth - 1, x05, x1, y0, y05, z0, z05),
			OctTreeNode(this, depth - 1, x05, x1, y0, y05, z05, z1),
			OctTreeNode(this, depth - 1, x05, x1, y05, y1, z0, z05),
			OctTreeNode(this, depth - 1, x05, x1, y05, y1, z05, z1)
		};
	}
}


OctTreeNode::~OctTreeNode() {
	// if not leaf, delete child nodes
	if (_depth > 1) {
		delete[] _childNodes;
	}
}


std::vector<OctTreeNode*> OctTreeNode::registerRigidBody(ARigidBody* rigidBody) {

	// if ridig body is in node
	if (rigidBody->isInAABB(_x0, _x1, _y0, _y1, _z0, _z1)) {

		// if leaf node, add to rigid bodies and return this node
		if (_depth <= 1) {
			_rigidBodies.push_back(rigidBody);
			_numOfElementsInLeaves++;
			return std::vector<OctTreeNode*>({ this });
		}
	
		// if is not leaf node try to register ridig body with all child nodes
		else {

			std::vector<OctTreeNode*> leaves = std::vector<OctTreeNode*>();
			for (int i = 0; i < 8; i++) {
				std::vector<OctTreeNode*> leavesFromChild = _childNodes[i].registerRigidBody(rigidBody);
				if(leavesFromChild.size() > 0) {
					leaves.insert(leaves.end(), leavesFromChild.begin(), leavesFromChild.end());
					_numOfElementsInLeaves += leavesFromChild.size();
				}
			}

			// TODO for debug
			//std::cout << "depth: " << _depth << " / num of non-empty leaves: " << _numOfElementsInLeaves << "\n";
			return leaves;
		}
	}


	// if rigid body is not in node, return empty list of leaves
	else {
		return std::vector<OctTreeNode*>();
	}
}


void OctTreeNode::unregisterRigidBody(ARigidBody* rigidBody) {

	_numOfElementsInLeaves--;
	
	// if leaf node, remove rigid body
	if (_depth <= 1) {
		std::vector<ARigidBody*>::iterator index;
		index = std::find(_rigidBodies.begin(), _rigidBodies.end(), rigidBody);
		if (index != _rigidBodies.end()) {
			_rigidBodies.erase(index);
		}
	}

	// in not root, make parent update number of elements in leaves
	if (_parent != 0) {
		_parent->unregisterRigidBody(rigidBody);
	}
}


void OctTreeNode::collide() {

	// only continue colliding down the tree if node has leaves with elements
	if (_numOfElementsInLeaves > 0) {

		// if leaf collide all objects
		if (_depth <= 1) {
			for (std::vector<ARigidBody*>::size_type i = 0; i != _rigidBodies.size(); i++) {
				for (std::vector<ARigidBody*>::size_type j = i + 1; j != _rigidBodies.size(); j++) {
					CollisionHandler::collide(_rigidBodies[i], _rigidBodies[j]);
				}
			}
		}

		// if not leaf, get all children to collide
		else {
			for (int i = 0; i < 8; i++) {
				_childNodes[i].collide();
			}
		}
	}
}
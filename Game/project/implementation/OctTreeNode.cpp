#include "OctTreeNode.h"


OctTreeNode::OctTreeNode(Scene* scene, OctTreeNode* parent, int depth, vmml::AABBf aabb) {

	_scene = scene;

	_parent = parent;

	_depth = depth;

	_aabb = aabb;

	_numOfElementsInLeaves = 0;

	// Leaf node
	if (depth <= 1) {
		_rigidBodies = std::vector<ARigidBodyOctree*>();
	}

	// Not leaf node
	else {
		vmml::Vector3f center = _aabb.getCenter();
		vmml::Vector3f min = _aabb.getMin();
		vmml::Vector3f max = _aabb.getMax();

		_childNodes = new OctTreeNode[8]{
			OctTreeNode(_scene, this, (depth - 1), vmml::AABBf({ min.x(),	 min.y(),	 min.z() },	   { center.x(), center.y(), center.z()})),
			OctTreeNode(_scene, this, (depth - 1), vmml::AABBf({ min.x(),	 min.y(),	 center.z() }, { center.x(), center.y(), max.z()})),
			OctTreeNode(_scene, this, (depth - 1), vmml::AABBf({ min.x(),	 center.y(), min.z() },	   { center.x(), max.y(),    center.z()})),
			OctTreeNode(_scene, this, (depth - 1), vmml::AABBf({ min.x(),	 center.y(), center.z() }, { center.x(), max.y(),    max.z()})),
			OctTreeNode(_scene, this, (depth - 1), vmml::AABBf({ center.x(), min.y(),	 min.z() },	   { max.x(),    center.y(), center.z()})),
			OctTreeNode(_scene, this, (depth - 1), vmml::AABBf({ center.x(), min.y(),	 center.z() }, { max.x(),    center.y(), max.z()})),
			OctTreeNode(_scene, this, (depth - 1), vmml::AABBf({ center.x(), center.y(), min.z() },	   { max.x(),    max.y(),    center.z()})),
			OctTreeNode(_scene, this, (depth - 1), vmml::AABBf({ center.x(), center.y(), center.z() }, { max.x(),    max.y(),    max.z()}))
		};																								
	}
}


OctTreeNode::~OctTreeNode() {
	// if not leaf, delete child nodes
	if (_depth > 1) {
		delete[] _childNodes;
	}
}


std::vector<OctTreeNode*> OctTreeNode::registerRigidBody(ARigidBodyOctree* rigidBody) {

	// if ridig body is in node
	if (rigidBody->isInAABB(_aabb)) {

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
			/*if(_depth == 4 && leaves.size() > 0) {
				std::cout << "depth: " << _depth << " / num of non-empty leaves: " << _numOfElementsInLeaves << "\n";
				} */
			return leaves;
		}
	}


	// if rigid body is not in node, return empty list of leaves
	else {
		return std::vector<OctTreeNode*>();
	}
}


void OctTreeNode::unregisterRigidBody(ARigidBodyOctree* rigidBody) {

	_numOfElementsInLeaves--;
	
	// if leaf node, remove rigid body
	if (_depth <= 1) {
		std::vector<ARigidBodyOctree*>::iterator index;
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

			if(_rigidBodies.size() > 1) {
				for (std::vector<ARigidBodyOctree*>::size_type i = 0; i < _rigidBodies.size() - 1; i++) {
					for (std::vector<ARigidBodyOctree*>::size_type j = i + 1; j < _rigidBodies.size(); j++) {

						//std::cout << "TWO IN SAME NODE: ";

						ConstraintInformation info = SupportPointCalculator::getContraintInformation(_rigidBodies[i], _rigidBodies[j]);
						if (info.penetrationDepth > 0.0) {
							_scene->registerSolverConstraint(_rigidBodies[i], _rigidBodies[j], info);
                        } else {
//                            std::cout << "no collision" << std::endl;
                        }
					}
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

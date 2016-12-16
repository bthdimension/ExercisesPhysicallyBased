#ifndef OCT_TREE_NODE_H
#define OCT_TREE_NODE_H

#include "bRenderer.h"
#include "vmmlib/aabb.hpp"
#include "ARigidBodyOctree.h"
#include "Scene.h"

class Scene;
class ARigidBodyOctree;



class OctTreeNode {

public:
    OctTreeNode(){};
	OctTreeNode(Scene* scene, OctTreeNode* parent, int depth, vmml::AABBf aabb);
	~OctTreeNode();

	std::vector<OctTreeNode*> registerRigidBody(ARigidBodyOctree* ridigBody);
	void unregisterRigidBody(ARigidBodyOctree* ridigBody);

	/**	@brief Finds and resolves collisions in (sub)tree, returns false if no collisions are found
	*/
	bool collide();


private:

	Scene* _scene;

	OctTreeNode* _parent;

	int _depth;

	vmml::AABBf _aabb;

	OctTreeNode* _childNodes;
	std::vector<ARigidBodyOctree*> _rigidBodies;

	int _numOfElementsInLeaves;

};

typedef std::shared_ptr< OctTreeNode >  OctTreeNodePtr;


#endif

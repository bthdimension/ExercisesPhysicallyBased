#ifndef OCT_TREE_NODE_H
#define OCT_TREE_NODE_H

#include "bRenderer.h"
#include "CollisionHandler.h"
#include "ARigidBody.h"

class ARigidBody; // forward declaration


class OctTreeNode {

public:

	OctTreeNode(OctTreeNode* parent, int depth, double x0, double x1, double y0, double y1, double z0, double z1);
	~OctTreeNode();

	std::vector<OctTreeNode*> registerRigidBody(ARigidBody* ridigBody);
	void unregisterRigidBody(ARigidBody* ridigBody);

	void collide();


private:

	OctTreeNode* _parent;

	int _depth;

	double _x0;
	double _x1;
	double _y0;
	double _y1;
	double _z0;
	double _z1;

	OctTreeNode* _childNodes;
	std::vector<ARigidBody*> _rigidBodies;

	int _numOfElementsInLeaves;

};

typedef std::shared_ptr< OctTreeNode >  OctTreeNodePtr;


#endif

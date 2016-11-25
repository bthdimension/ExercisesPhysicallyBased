#ifndef A_RIGID_BODY_H
#define A_RIGID_BODY_H

#include "bRenderer.h"
#include "OctTreeNode.h"

class OctTreeNode; // forward declaration
typedef std::shared_ptr< OctTreeNode >  OctTreeNodePtr; // forward declaration



class ARigidBody {

public:

	static const int TYPE_BLOCK = 0;
	static const int TYPE_SPHERE = 1;

	virtual int getType() = 0;

	virtual bool isInAABB(double x0, double x1, double y0, double y1, double z0, double z1) = 0;

	virtual void update(const double &deltaTime) = 0;
	virtual void draw(ModelRendererPtr modelRenderer, int id) = 0;

	void setOctTree(OctTreeNodePtr octTree);
	void registerInOctTree();


private:

	OctTreeNodePtr _octTree;
	std::vector<OctTreeNode*> _leavesAtWhichBodyIsRegistered;

};


#endif

#ifndef A_RIGID_BODY_H
#define A_RIGID_BODY_H

#include "bRenderer.h"
#include "OctTreeNode.h"
#include "headers/IRigidBody.h"

class OctTreeNode; // forward declaration
typedef std::shared_ptr< OctTreeNode >  OctTreeNodePtr; // forward declaration



class ARigidBodyOctree : public IRigidBody{

public:

	ARigidBodyOctree(ModelPtr model) : IRigidBody(model) {}
	ARigidBodyOctree(ModelPtr model, vmml::Vector3f position) : IRigidBody(model) { setPosition(position); }

	virtual bool isInAABB(vmml::AABBf aabb) {
		return getMeshCollider()->intersectBoundingVolumes(aabb, false);
	}

	virtual void draw(ModelRendererPtr modelRenderer, int id) = 0;
	virtual void handleCollision(ARigidBodyOctree *collider /*, other stuff*/) {}

	void setOctTree(OctTreeNodePtr octTree);
	void registerInOctTree();

	


private:

	OctTreeNodePtr _octTree;
	std::vector<OctTreeNode*> _leavesAtWhichBodyIsRegistered;
	std::string _modelName;

};


#endif

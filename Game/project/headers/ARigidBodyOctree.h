#ifndef A_RIGID_BODY_H
#define A_RIGID_BODY_H

#include "bRenderer.h"
#include "headers/IRigidBody.h"

class OctTreeNode; // forward declaratio
typedef std::shared_ptr< OctTreeNode >  OctTreeNodePtr; // forward declaration


class ARigidBodyOctree : public IRigidBody{

public:

	ARigidBodyOctree(ModelPtr model, vmml::Vector3f position, vmml::Vector3f axesRotation) : IRigidBody(model) {
		setPosition(position);
		setAxesRotation(axesRotation);
	}

	virtual bool isInAABB(vmml::AABBf aabb) {
		return getMeshCollider()->intersectBoundingVolumes(aabb, nullptr, false);
	}

	virtual void draw(ModelRendererPtr modelRenderer, int id) = 0;

	void setOctTree(OctTreeNodePtr octTree);
	void registerInOctTree();

	void setIndex(int index);
	int getIndex();

	virtual bool isFixed() = 0;

	


private:

	OctTreeNodePtr _octTree;
	std::vector<OctTreeNode*> _leavesAtWhichBodyIsRegistered;
	std::string _modelName;

	int _index;

};


#endif

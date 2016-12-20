#ifndef A_RIGID_BODY_H
#define A_RIGID_BODY_H

#include "bRenderer.h"
#include "headers/IRigidBody.h"
#include <eigen3/Eigen/Dense>

class OctTreeNode; // forward declaratio
typedef std::shared_ptr< OctTreeNode >  OctTreeNodePtr; // forward declaration

using namespace Eigen;


class ARigidBodyOctree : public IRigidBody{

public:

	enum Type { SPHERE, VERTICES };

	ARigidBodyOctree(ModelPtr model, vmml::Vector3f position, vmml::Vector3f axesRotation) : IRigidBody(model) {
		setPosition(position);
		setAxesRotation(axesRotation);
	}

	virtual bool isInAABB(vmml::AABBf aabb) {
		return getMeshCollider()->intersectBoundingVolumes(aabb, nullptr, false);
	}

	virtual Type getType() = 0;

	virtual void draw(ModelRendererPtr modelRenderer, int id) = 0;

	void setOctTree(OctTreeNodePtr octTree);
	void registerInOctTree();

	void setIndex(int index);
	int getIndex();

	std::vector<Vector3f> getVertices();
    std::vector<Vector3f> getWorldVertices();

	virtual bool isFixed() = 0;

	


private:

	OctTreeNodePtr _octTree;
	std::vector<OctTreeNode*> _leavesAtWhichBodyIsRegistered;
	std::string _modelName;

	int _index;

};


#endif

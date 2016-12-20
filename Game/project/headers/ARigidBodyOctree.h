#ifndef A_RIGID_BODY_H
#define A_RIGID_BODY_H

#include "bRenderer.h"
#include "headers/IRigidBody.h"
#include <Dense>

class OctTreeNode; // forward declaratio
typedef std::shared_ptr< OctTreeNode >  OctTreeNodePtr; // forward declaration

using namespace Eigen;


class ARigidBodyOctree : public IRigidBody{

public:

	enum Type { SPHERE, VERTICES };

	ARigidBodyOctree(ModelPtr model, vmml::Vector3f position, vmml::Vector3f axesRotation) : IRigidBody(model) {
		setPosition(position);
		setAxesRotation(axesRotation);
		_debugPoints = std::vector<vmml::Vector3f>();
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

	std::vector<std::vector<Vector3f>> getTriangles();


	virtual bool isFixed() = 0;

	void addDebugPoint(vmml::Vector3f point);
	void resetDebugPoints();
	std::vector<vmml::Vector3f> getDebugPoints();

private:

	OctTreeNodePtr _octTree;
	std::vector<OctTreeNode*> _leavesAtWhichBodyIsRegistered;
	std::string _modelName;

	std::vector<vmml::Vector3f> _debugPoints;

	int _index;

};


#endif

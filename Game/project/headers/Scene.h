#ifndef SCENE_H
#define SCENE_H

#include "bRenderer.h"
#include "BlockRigidBody.h"
#include "InputController.h"
#include "FreeCamera.h"
#include "SceneEditor.h"
#include "Lighting.h"
#include "SphereRigidBody.h"
#include "FloorRigidBody.h"
#include "OctTreeNode.h"
#include "Solver.h"

class SceneEditor; // forward declaration
typedef std::shared_ptr< SceneEditor >  SceneEditorPtr; // forward declaration


class Scene {

public:

	Scene(Renderer* bRenderer, InputControllerPtr inputController, FreeCameraPtr freeCamera);
	~Scene();

	void addRigidBody(ARigidBodyOctree* solidBody);

	void loop(const double &deltaTime, bool* running);

	OctTreeNodePtr getOctree() { return _octTree; }

	const int NUM_SPHERES = 1;

private:

	void generateSpheres();

	void update(const double &deltaTime);
	void draw();


private:

	Renderer* _bRenderer;
	ModelRendererPtr _modelRenderer;
	InputControllerPtr _inputController;
	FreeCameraPtr _freeCamera;
	
	SceneEditorPtr _sceneEditor;
	
	LightingPtr _lighting;

	std::vector<ARigidBodyOctree*> _rigidBodies;
	OctTreeNodePtr _octTree;

	SolverPtr _solver;

};


typedef std::shared_ptr< Scene >  ScenePtr;


#endif

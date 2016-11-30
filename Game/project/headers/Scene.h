#ifndef SCENE_H
#define SCENE_H

#include "bRenderer.h"
#include "Block.h"
#include "InputController.h"
#include "FreeCamera.h"
#include "SceneEditor.h"
#include "Lighting.h"
#include "Sphere.h"
#include "OctTreeNode.h"

class SceneEditor; // forward declaration
typedef std::shared_ptr< SceneEditor >  SceneEditorPtr; // forward declaration


class Scene {

public:

	Scene(Renderer* bRenderer, InputControllerPtr inputController, FreeCameraPtr freeCamera);
	~Scene();

	void addRigidBody(ARigidBody* solidBody);

	void loop(const double &deltaTime, bool* running);


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

	std::vector<ARigidBody*> _rigidBodies;
	OctTreeNodePtr _octTree;

};


typedef std::shared_ptr< Scene >  ScenePtr;


#endif

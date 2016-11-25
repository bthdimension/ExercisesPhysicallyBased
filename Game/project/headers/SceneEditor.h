#ifndef SCENE_EDITOR_H
#define SCENE_EDITOR_H

#include "bRenderer.h"
#include "Scene.h"
#include "InputController.h"
#include "FreeCamera.h"

class Scene; // forward declaration


class SceneEditor {

public:

	SceneEditor(Scene* scene, ModelRendererPtr modelRenderer, InputControllerPtr inputController, FreeCameraPtr freeCamera);

	void update();


private:

	Scene* _scene;
	ModelRendererPtr _modelRenderer;
	InputControllerPtr _inputController;
	FreeCameraPtr _freeCamera;

	bool _lastLeftMouseButtonState;
};


typedef std::shared_ptr< SceneEditor >  SceneEditorPtr;


#endif
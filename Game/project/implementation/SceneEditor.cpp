#include "SceneEditor.h"


SceneEditor::SceneEditor(Scene* scene, ModelRendererPtr modelRenderer, InputControllerPtr inputController, FreeCameraPtr freeCamera) {
	_scene = scene;
	_modelRenderer = modelRenderer;
	_inputController = inputController;
	_freeCamera = freeCamera;

	_lastLeftMouseButtonState = false;
}


void SceneEditor::update() {

	bool leftMouseDown = _inputController->isLeftMouseButtonDown();
	vmml::Vector2d mousePosition = _inputController->getMousePosition();
	vmml::Vector3f blockPosition = _freeCamera->screenToWorld(mousePosition);

	if (blockPosition.x() < -25.0) {
		blockPosition.x() = -25.0;
	}
	if (blockPosition.x() > 25.0) {
		blockPosition.x() = 25.0;
	}
	if (blockPosition.z() < -25.0) {
		blockPosition.z() = -25.0;
	}
	if (blockPosition.z() > 25.0) {
		blockPosition.z() = 25.0;
	}

	blockPosition.y() = 1.0;

	if (!leftMouseDown && leftMouseDown != _lastLeftMouseButtonState) {
		_scene->addRigidBody((ARigidBodyOctree*) new Block(_modelRenderer->getObjectManager()->getModel("block"), blockPosition));
	}
	if (leftMouseDown) {
		vmml::Matrix4f modelMatrix = vmml::create_translation(blockPosition);
		modelMatrix *= vmml::create_scaling(vmml::Vector3f(1.0f));
		_modelRenderer->queueModelInstance(
			"block",
			"block_temp",
			"camera",
			modelMatrix,
			std::vector<std::string>({ "sun", "moon" }),
			true,
			true
		);
	}
	_lastLeftMouseButtonState = leftMouseDown;
}
#include "SceneEditor.h"


SceneEditor::SceneEditor(Scene* scene, ModelRendererPtr modelRenderer, InputControllerPtr inputController, FreeCameraPtr freeCamera) {
	_scene = scene;
	_modelRenderer = modelRenderer;
	_inputController = inputController;
	_freeCamera = freeCamera;

	placerBlock = new PlacerBlockRigidBody(_modelRenderer->getObjectManager()->getModel("block"));
	placerBlock->setOctTree(_scene->getOctree());
	placerBlock->registerInOctTree();

	_lastLeftMouseButtonState = false;
}


void SceneEditor::update(const double &deltaTime) {

	bool leftMouseDown = _inputController->isLeftMouseButtonDown();


	if (!leftMouseDown && leftMouseDown != _lastLeftMouseButtonState) {		
		_scene->addRigidBody(new BlockRigidBody(_modelRenderer->getObjectManager()->getModel("block"), placerBlock->getPosition()));
	}
	//if (leftMouseDown) {

		vmml::Vector2d mousePosition = _inputController->getMousePosition();
		vmml::Vector3f blockPosition = _freeCamera->screenToWorld(mousePosition);
		blockPosition[1] = placerBlock->getPosition().y();

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

		placerBlock->setPosition(blockPosition);
		placerBlock->update(deltaTime);
		placerBlock->draw(_modelRenderer, -1);
	//}
	_lastLeftMouseButtonState = leftMouseDown;
}
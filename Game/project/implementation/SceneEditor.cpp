#include "SceneEditor.h"


SceneEditor::SceneEditor(Scene* scene, ModelRendererPtr modelRenderer, InputControllerPtr inputController, FreeCameraPtr freeCamera) {
	_scene = scene;
	_modelRenderer = modelRenderer;
	_inputController = inputController;
	_freeCamera = freeCamera;

	/*placerBlock = new PlacerBlockRigidBody(_modelRenderer->getObjectManager()->getModel("block"));

    _scene->addRigidBody(placerBlock);
	placerBlock->setOctTree(_scene->getOctree());
	placerBlock->registerInOctTree();*/

	_lastLeftMouseButtonState = false;
}


void SceneEditor::createDebugScene() {
	for (int i = 1; i < 2; i++) {
		float j = i * 30.f;
		ModelPtr modelptr = _modelRenderer->getObjectManager()->getModel("block");
		vmml::Vector3f pos = vmml::Vector3f(0.5f, j + 1.0f, -0.5f);
		vmml::Vector3f rotAx = vmml::Vector3f(0.f, 0.0f, 0.0f);

		ARigidBodyOctree* rb = new BlockRigidBody(modelptr, pos, rotAx);
		_scene->addRigidBody(rb);
	}
};


void SceneEditor::update(const double &deltaTime) {
    
	/*bool leftMouseDown = _inputController->isLeftMouseButtonDown();
	placerBlock->setVisible(false);

	if (!leftMouseDown && leftMouseDown != _lastLeftMouseButtonState) {	
		_scene->addRigidBody(new BlockRigidBody(_modelRenderer->getObjectManager()->getModel("block"), placerBlock->getPosition(), placerBlock->getAxesRotation()));
	}

	if(_inputController->isPressed(bRenderer::KEY_Q))
		placerBlock->setAxesRotation(placerBlock->getAxesRotation() + vmml::Vector3f(1.f*(float)deltaTime, 0.f, 0.f));
	if (_inputController->isPressed(bRenderer::KEY_E))
		placerBlock->setAxesRotation(placerBlock->getAxesRotation() - vmml::Vector3f(1.f*(float)deltaTime, 0.f, 0.f));

	//if (leftMouseDown) {

		vmml::Vector2d mousePosition = _inputController->getMousePosition();
		vmml::Vector3f blockPosition = _freeCamera->screenToWorld(mousePosition);
		blockPosition.y() = placerBlock->getPosition().y();

		if (blockPosition.x() < -25.0)			blockPosition.x() = -25.0;
		if (blockPosition.x() > 25.0)			blockPosition.x() = 25.0;
		if (blockPosition.z() < -25.0) 			blockPosition.z() = -25.0;
		if (blockPosition.z() > 25.0) 			blockPosition.z() = 25.0;

		placerBlock->setPosition(blockPosition);
		placerBlock->setVisible(true);
	//}
	_lastLeftMouseButtonState = leftMouseDown;*/
}

#include "FreeCamera.h"


FreeCamera::FreeCamera(ViewPtr view, ObjectManagerPtr objectManager, InputControllerPtr inputController) {

	_view = view;

	vmml::Vector3f cameraPosition = vmml::Vector3f(25.0f, -15.0f, 25.0f);
	vmml::Vector3f cameraRotation = vmml::Vector3f(-0.5f, -0.7854f, 0.0f);

	_camera = objectManager->createCamera("camera", cameraPosition, cameraRotation);
	_inputController = inputController;
}


void FreeCamera::update(const double &deltaTime) {

	_camera->setAspectRatio(_view->getAspectRatio());

	move(deltaTime);
	rotate(deltaTime);
}


void FreeCamera::move(const double &deltaTime) {

	vmml::Vector3d cameraMovement = vmml::Vector3d(0.0, 0.0, 0.0);

	bool keyW = _inputController->isPressed(bRenderer::KEY_W);
	bool keyA = _inputController->isPressed(bRenderer::KEY_A);
	bool keyS = _inputController->isPressed(bRenderer::KEY_S);
	bool keyD = _inputController->isPressed(bRenderer::KEY_D);

	bool keyUp = _inputController->isPressed(bRenderer::KEY_UP);
	bool keyLeft = _inputController->isPressed(bRenderer::KEY_LEFT);
	bool keyRight = _inputController->isPressed(bRenderer::KEY_RIGHT);
	bool keyDown = _inputController->isPressed(bRenderer::KEY_DOWN);

	bool keySpace = _inputController->isPressed(bRenderer::KEY_SPACE);
	bool keyShift = _inputController->isPressed(bRenderer::KEY_LEFT_SHIFT);
	bool keyCtrl = _inputController->isPressed(bRenderer::KEY_LEFT_CONTROL);

	// Forward/Backward
	if ((keyW || keyUp) && !keyS && !keyDown) {
		cameraMovement.z() = 1.0f;
	}
	else if ((keyS || keyDown) && !keyW && !keyUp) {
		cameraMovement.z() = -1.0f;
	}

	// Left/Right
	if ((keyA || keyLeft) && !keyD && !keyRight) {
		cameraMovement.x() = -1.0f;
	}
	else if ((keyD || keyRight) && !keyA && !keyLeft) {
		cameraMovement.x() = 1.0f;
	}

	// Up/Down
	if (keySpace && !keyShift) {
		cameraMovement.y() = 1.0f;
	}
	else if (keyShift && !keySpace) {
		cameraMovement.y() = -1.0f;
	}

	// Sprint
	if (keyCtrl) {
		cameraMovement *= 3.0;
	}

	cameraMovement *= CAMERA_MOVEMENT_SPEED * deltaTime;

	_camera->moveCameraForward(cameraMovement.z());
	_camera->moveCameraSideward(cameraMovement.x());
	_camera->moveCameraUpward(cameraMovement.y());
}


void FreeCamera::rotate(const double &deltaTime) {

	vmml::Vector3d cameraRotation = vmml::Vector3d(0.0, 0.0, 0.0);

	if (_inputController->isRightMouseButtonDown() ||
		_inputController->isPressed(bRenderer::KEY_C)) {

		vmml::Vector2d mouseDelta = _inputController->getMouseDelta();
		cameraRotation.x() = mouseDelta.y();
		cameraRotation.y() = mouseDelta.x();
	}

	cameraRotation *= CAMERA_ROTATION_SPEED / _view->getWidth();

	_camera->rotateCamera(cameraRotation.x(), cameraRotation.y(), cameraRotation.z());
}


vmml::Vector3f FreeCamera::screenToWorld(vmml::Vector2f screenPoint) {

	// TODO make this better, i.e. real projection

	double screenWidth = _view->getWidth();
	double screenHeight = _view->getHeight();

	screenPoint.x() = screenPoint.x() - ((screenWidth - screenHeight) / 2.0);

	double x = 50 * ((screenPoint.x() / screenHeight) - 0.5);
	double y = 0.0;
	double z = -50 * ((screenPoint.y() / screenHeight) - 0.5);

	return vmml::Vector3f(x, y, z);
}

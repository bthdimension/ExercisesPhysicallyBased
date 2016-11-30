#include "InputController.h"


InputController::InputController(Renderer* bRenderer, bool* running) {
	_bRenderer = bRenderer;
	_input = bRenderer->getInput();
	_running = running;

	_mousePosition = vmml::Vector3d(0.0, 0.0);
	_mouseDelta = vmml::Vector3d(0.0, 0.0);
	_leftMouseButtonDown = false;
	_rightMouseButtonDown = false;

	_lastStatePauseKey = false;
}


void InputController::update(const double &deltaTime) {

	updateMouse();

	quitGame();
	pauseUnpause();
}


void InputController::updateMouse() {

	double xpos = _input->getCursorPositionX();
	double ypos = _input->getCursorPositionY();

	_mouseDelta.x() = xpos - _mousePosition.x();
	_mouseDelta.y() = ypos - _mousePosition.y();

	_mousePosition.x() = xpos;
	_mousePosition.y() = ypos;

	_leftMouseButtonDown = _input->getMouseButtonState(bRenderer::LEFT_MOUSE_BUTTON) == bRenderer::INPUT_PRESS;
	_rightMouseButtonDown = _input->getMouseButtonState(bRenderer::RIGHT_MOUSE_BUTTON) == bRenderer::INPUT_PRESS;
}


void InputController::quitGame() {
	if (isPressed(bRenderer::KEY_ESCAPE)) {
		_bRenderer->terminateRenderer();
	}
}


void InputController::pauseUnpause() {
	GLint currentStatePauseKey = isPressed(bRenderer::KEY_P);
	if (currentStatePauseKey != _lastStatePauseKey) {
		_lastStatePauseKey = currentStatePauseKey;
		if (!currentStatePauseKey) {
			*_running = !*_running;
		}
	}
}


vmml::Vector2d InputController::getMousePosition() {
	return _mousePosition;
}


vmml::Vector2d InputController::getMouseDelta() {
	return _mouseDelta;
}


bool InputController::isLeftMouseButtonDown() {
	return _leftMouseButtonDown;
}


bool InputController::isRightMouseButtonDown() {
	return _rightMouseButtonDown;
}


bool InputController::isPressed(const GLint& key) {
	return _input->getKeyState(key) == bRenderer::INPUT_PRESS;
}
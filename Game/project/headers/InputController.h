#ifndef INPUT_CONTROLLER_H
#define INPUT_CONTROLLER_H

#include "bRenderer.h"

class InputController {

public:

	InputController(Renderer* bRenderer, bool* running);

	void update(const double &deltaTime);

	vmml::Vector2d getMousePosition();
	vmml::Vector2d getMouseDelta();
	bool isLeftMouseButtonDown();
	bool isRightMouseButtonDown();

	bool isPressed(const GLint& key);


private:
	
	void updateMouse();

	void quitGame();
	void pauseUnpause();


private:

	Renderer* _bRenderer;
	InputPtr _input;
	bool* _running;

	vmml::Vector2d _mousePosition;
	vmml::Vector2d _mouseDelta;
	bool _leftMouseButtonDown;
	bool _rightMouseButtonDown;

	bool _lastStatePauseKey;

};


typedef std::shared_ptr< InputController >  InputControllerPtr;


#endif

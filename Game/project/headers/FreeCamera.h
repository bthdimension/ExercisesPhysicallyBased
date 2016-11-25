#ifndef FREE_CAMERA_H
#define FREE_CAMERA_H

#include "bRenderer.h"
#include "InputController.h"


class FreeCamera {

public:

	FreeCamera(ViewPtr view, ObjectManagerPtr objectManager, InputControllerPtr inputController);

	void update(const double &deltaTime);

	vmml::Vector3f screenToWorld(vmml::Vector2f screenPoint);


private:

	void move(const double &deltaTime);
	void rotate(const double &deltaTime);


private:

	const GLfloat CAMERA_MOVEMENT_SPEED = 20.0f;
	const GLfloat CAMERA_ROTATION_SPEED = 3.0f;

	ViewPtr _view;
	CameraPtr _camera;
	InputControllerPtr _inputController;

};


typedef std::shared_ptr< FreeCamera >  FreeCameraPtr;

#endif
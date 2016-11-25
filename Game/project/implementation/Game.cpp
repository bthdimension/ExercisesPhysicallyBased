#include "Game.h"


void Game::init() {
	_settingsController = SettingsControllerPtr(new SettingsController(&bRenderer()));
	_settingsController->setup();

	bRenderer().runRenderer();
}


void Game::initFunction() {

	_running = true;

	// Setup input controller
	_inputController = InputControllerPtr(new InputController(&bRenderer(), &_running));

	// Setup free camera
	ViewPtr view = bRenderer().getView();
	ObjectManagerPtr objectManager = bRenderer().getObjects();
	_freeCamera = FreeCameraPtr(new FreeCamera(view, objectManager, _inputController));

	// Load Resources
	_resourceManager = ResourceManagerPtr(new ResourceManager(&bRenderer()));
	_resourceManager->loadResources();

	// Create and draw scene
	_scene = ScenePtr(new Scene(&bRenderer(), _inputController, _freeCamera));
	_scene->loop(0.0);
}


void Game::loopFunction(const double &deltaTime, const double &elapsedTime) {

	// Process keyboard and mouse
	_inputController->update(deltaTime);

	// Update camera
	_freeCamera->update(deltaTime);

	// Update and draw scene
	_scene->loop(deltaTime);
}


void Game::terminateFunction() {
	bRenderer::log("Terminated");
}
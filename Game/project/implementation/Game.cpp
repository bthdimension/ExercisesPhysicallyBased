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
	_scene->loop(0.0, &_running);
}


void Game::loopFunction(const double &deltaTime, const double &elapsedTime) {
	showFPS(deltaTime);
	// Process keyboard and mouse
	_inputController->update(deltaTime);
	if (bRenderer().isInitialized()) {
		// Update camera
		_freeCamera->update(deltaTime);

		// Update and draw scene
		_scene->loop(deltaTime, &_running);
	}
}


void Game::terminateFunction() {
	bRenderer::log("Terminated");
}

void Game::showFPS(const double &deltaTime) {
	if (fpsTime >= 1)
	{
		bRenderer::log("FPS: " + std::to_string(framesPerSec));
		framesPerSec = 0;
		fpsTime--;
	}
	framesPerSec++;
	fpsTime += deltaTime;
}
#include "SettingsController.h"


SettingsController::SettingsController(Renderer* bRenderer) {
	_bRenderer = bRenderer;
}


void SettingsController::setup() {

	bRenderer::setStandardFilePath("../../project/data");
	bRenderer::loadConfigFile("config.json");	// load custom configurations replacing the default values in Configuration.cpp

	int windowWidth = 1200;
	int windowHeight = 720;
	bool windowFullscreen = false;
	std::string windowTitle = "PBS Game";

	if (windowFullscreen) {
		_bRenderer->initRenderer(true, windowTitle);
	}
	else {
		_bRenderer->initRenderer(windowWidth, windowHeight, false, windowTitle);
	}

	// get OpenGL and shading language version
	bRenderer::log("OpenGL Version: ", glGetString(GL_VERSION));
	bRenderer::log("Shading Language Version: ", glGetString(GL_SHADING_LANGUAGE_VERSION));

	// set shader versions (optional)
	_bRenderer->getObjects()->setShaderVersionDesktop("#version 120");
	_bRenderer->getObjects()->setShaderVersionES("#version 100");
}

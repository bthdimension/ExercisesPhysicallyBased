#include "Block.h"



void Block::update(const double &deltaTime) {
	// TODO do physics of block here

	updateMatrices();

	// do stuff
}


void Block::draw(ModelRendererPtr modelRenderer, int id) {

	vmml::Matrix4f modelMatrix = getWorldMatrix();

	modelRenderer->queueModelInstance(
		"block",
		"block_" + std::to_string(id),
		"camera", 
		modelMatrix,
		std::vector<std::string>({ "sun", "moon" }),
		true,
		true
	);
}
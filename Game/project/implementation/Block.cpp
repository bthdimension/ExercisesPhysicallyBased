#include "Block.h"


Block::Block(vmml::Vector3f position) {
	_position = position;
}


int Block::getType() {
	return ARigidBody::TYPE_BLOCK;
}


bool Block::isInAABB(double x0, double x1, double y0, double y1, double z0, double z1) {

	// TODO: This is the worst-case AABB of the block, we can improve this
	double bX0 = _position.x() - 2.0;
	double bX1 = _position.x() + 2.0;
	double bY0 = _position.y() - 2.0;
	double bY1 = _position.y() + 2.0;
	double bZ0 = _position.z() - 2.0;
	double bZ1 = _position.z() + 2.0;
	
	return ((x0 > bX0 && x0 < bX1) || (x1 > bX0 && x1 < bX1) || (x0 < bX0 && x1 > bX1)) &&
		((y0 > bY0 && y0 < bY1) || (y1 > bY0 && y1 < bY1) || (y0 < bY0 && y1 > bY1)) &&
		((z0 > bZ0 && z0 < bZ1) || (z1 > bZ0 && z1 < bZ1) || (z0 < bZ0 && z1 > bZ1));
}


void Block::update(const double &deltaTime) {
	// TODO do physics of block here
}


void Block::draw(ModelRendererPtr modelRenderer, int id) {

	vmml::Matrix4f modelMatrix = vmml::create_translation(_position);
	modelMatrix *= vmml::create_scaling(vmml::Vector3f(1.0f));

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
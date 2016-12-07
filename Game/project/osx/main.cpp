#include "bRenderer.h"
#include "Game.h"

int main(void) {

	Game *game = new Game();
	game->init();
	delete game;

	std::cout << "press ENTER to quit" << std::endl;
	std::cin.ignore();

	return 0;
}
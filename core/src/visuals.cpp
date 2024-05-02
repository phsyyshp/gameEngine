#include "visuals.hpp"
sf::RenderWindow Visual::window(sf::VideoMode(800, 600), "My window",
                                sf::Style::Default);
bool Visual::isDebug = true;
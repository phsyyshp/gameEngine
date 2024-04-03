#include "entity.hpp"
#include "forceGeneration.hpp"
#include "world.hpp"
#include <SFML/Graphics.hpp>
#include <iostream>
int main() {
  sf::ContextSettings settings;
  sf::RenderWindow window(sf::VideoMode(800, 600), "My window",
                          sf::Style::Default);
  window.setFramerateLimit(60);
  float deltaTime = 1.f;
  Box plane(150.f, 150.f, 50.f, 10.f);
  plane.setInverseMass(1.f);
  plane.setFillColor(sf::Color(100, 250, 50));
  Gravity gravity({0.f, 0.1f});
  World world;
  world.registerBody(plane);
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    world.startFrame();
    gravity.updateForce(plane, deltaTime);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
      plane.addForceOnBody({0.f, -1.f}, {50.f, 0.f});
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
      plane.addForceOnBody({0.f, -1.f}, {0.f, 0.f});
    }

    world.runPhysics(deltaTime);
    window.clear(sf::Color::Black);
    window.draw(plane);
    window.display();
  }
  return 0;
}
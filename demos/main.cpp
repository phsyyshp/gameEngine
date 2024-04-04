#include "entity.hpp"
#include "forceGeneration.hpp"
#include "world.hpp"
#include <SFML/Graphics.hpp>
#include <chrono>
#include <iostream>
int main() {
  sf::ContextSettings settings;
  sf::RenderWindow window(sf::VideoMode(800, 600), "My window",
                          sf::Style::Default);
  window.setFramerateLimit(60);
  float deltaTime = 0.01f;
  float timeScale = 10;
  Box plane(150.f, 150.f, 15.f, 10.f);
  plane.setInverseMass(0.05f);
  plane.setFillColor(sf::Color(100, 250, 50));
  Gravity gravity({0.f, 9.8f});
  //   World world;
  //   world.registerBody(plane);
  while (window.isOpen()) {
    // record the time in high precision
    std::chrono::high_resolution_clock::time_point begining =
        std::chrono::high_resolution_clock::now();

    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    // world.startFrame();
    gravity.updateForce(plane, deltaTime);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
      plane.addForceOnBody({0.f, -200.f}, {-7.5f, 0.f});
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
      plane.addForceOnBody({0.f, -200.f}, {+7.5f, 0.f});
    }

    // world.runPhysics(deltaTime);
    plane.integrate(deltaTime);
    std::chrono::high_resolution_clock::time_point end =
        std::chrono::high_resolution_clock::now();
    // record delta time in seconds
    deltaTime =
        std::chrono::duration_cast<std::chrono::duration<float>>(end - begining)
            .count();
    deltaTime *= timeScale;
    window.clear(sf::Color::Black);
    window.draw(plane);
    plane.update();
    window.display();
  }
  return 0;
}
#include "collisionHandler.hpp"
#include "forceGeneration.hpp"
#include "shapes.hpp"
#include "types.hpp"
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
  float timeScale = 1;
  float lengthScale = 2000.f;
  lengthScale = 5.f;
  Circle mcirc(400, 300, 100);
  std::vector<Circle> circles;
  mcirc.setInverseMass(0);
  circles.push_back(mcirc);

  CollisionData cd;

  Gravity gravity({0.f, lengthScale * 9.8f});
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
    for (auto &circle : circles) {
      for (auto &other : circles) {
        if (&circle == &other) {
          continue;
        }
        std::cout << Collider::sphereAndSphere(circle, other, cd) << "\n";
      }
    }
    for (auto &contact : cd.contacts) {
      contact.applyImpulse();
    }
    // create a circle at the point  where mouse clicked
    if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
      sf::Vector2i mousePos = sf::Mouse::getPosition(window);
      circles.push_back(Circle(mousePos.x, mousePos.y, 100));
      (*(circles.end() - 1)).setInverseMass(0.05f);
    }

    // world.startFrame();
    for (auto &circle : circles) {
      gravity.updateForce(circle, deltaTime);
      circle.integrate(deltaTime);
    }

    // world.runPhysics(deltaTime);
    std::chrono::high_resolution_clock::time_point end =
        std::chrono::high_resolution_clock::now();
    // record delta time in seconds
    deltaTime =
        std::chrono::duration_cast<std::chrono::duration<float>>(end - begining)
            .count();
    // deltaTime *= timeScale;
    window.clear(sf::Color::Black);
    for (auto &circle : circles) {
      window.draw(circle);
      circle.update();
    }
    window.display();
  }
  return 0;
}
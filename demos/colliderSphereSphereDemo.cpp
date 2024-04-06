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
  float lengthScale = 8001.f;
  //   lengthScale = 0.f;
  //   Circle mcirc(400, 1500, 1001);
  Circle mcirc(100, 300, 100);
  Circle mcirc2(500, 300, 100);

  mcirc.setInverseMass(0.05);
  mcirc2.setInverseMass(0.05);
  mcirc.addVelocity({3000.f, 0.f});
  mcirc2.addVelocity({-3000.f, 0.f});
  std::vector<Circle> circles;
  circles.push_back(mcirc);
  circles.push_back(mcirc2);

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
      if (event.type == sf::Event::MouseButtonPressed) {
        if (event.mouseButton.button == sf::Mouse::Left) {
          sf::Vector2i mousePos = sf::Mouse::getPosition(window);
          circles.push_back(Circle(mousePos.x, mousePos.y, 10));
          (*(circles.end() - 1)).setInverseMass(0.05f);
        }
      }
    }
    // std::cout << "Number of circles: " << circles.size() << "\n";
    for (int i = 0; i < circles.size(); i++) {
      for (int j = i + 1; j < circles.size(); j++) {
        std::cout << Collider::sphereAndSphere(circles[i], circles[j], cd)
                  << "\n";
      }
    }
    for (auto &contact : cd.contacts) {
      contact.applyVelocityChange();
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
    cd.clear();
  }
  return 0;
}
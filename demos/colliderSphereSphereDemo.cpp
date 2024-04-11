#include "collisionHandler.hpp"
#include "contactResolver.hpp"
#include "forceGeneration.hpp"
#include "shapes.hpp"
#include "world.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <chrono>
#include <iostream>

int main() {
  sf::ContextSettings settings;
  sf::RenderWindow window(sf::VideoMode(800, 600), "My window",
                          sf::Style::Default);
  // window.setFramerateLimit(60);
  float deltaTime = 0.001f;
  float frameDuration;
  float timeScale = 1;
  float lengthScale = 10001.f;
  int frameNo = 0;
  //   lengthScale = 0.f;
  Circle mcirc0(400, 1500, 1001);
  Circle mcirc(-850, 100, 1000);
  mcirc.setPointCount(200);
  mcirc0.setPointCount(200);

  Circle mcirc2(1370, 300, 1000);

  mcirc2.setPointCount(200);

  //   mcirc2.setInverseMass(0.05);
  //   mcirc.addVelocity({3000.f, 0.f});
  //   mcirc2.addVelocity({-3000.f, 0.f});
  std::vector<Circle> circles;
  std::vector<Box> boxes;
  Box mbox(400, 600, 800, 100);
  Box mboxLeft(0, 250, 100, 600);
  Box mboxRight(600, 250, 100, 600);
  ContactResolver contactResolver;
  mbox.setInverseInertia(0.F);
  mboxLeft.setInverseInertia(0.F);
  mboxRight.setInverseInertia(0.F);
  boxes.push_back(mbox);
  boxes.push_back(mboxLeft);
  boxes.push_back(mboxRight);
  // circles.push_back(mcirc);
  // circles.push_back(mcirc2);
  // circles.push_back(mcirc0);

  CollisionData cd;

  Gravity gravity({0.f, lengthScale * 9.8f});
  //   World world;
  //   world.registerBody(plane);

  std::chrono::steady_clock::time_point lastFrameTime =
      std::chrono::steady_clock::now();
  while (window.isOpen()) {
    // record the time in high precision

    std::chrono::steady_clock::time_point currentFrameTime =
        std::chrono::steady_clock::now();
    frameDuration = std::chrono::duration_cast<std::chrono::duration<float>>(
                        currentFrameTime - lastFrameTime)
                        .count();
    lastFrameTime = currentFrameTime;
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
      if (event.type == sf::Event::MouseButtonPressed) {
        if (event.mouseButton.button == sf::Mouse::Left) {
          sf::Vector2i mousePos = sf::Mouse::getPosition(window);
          circles.push_back(Circle(mousePos.x, mousePos.y, 30));
          (*(circles.end() - 1)).setInverseMass(0.05f);
        }
        if (event.mouseButton.button == sf::Mouse::Right) {
          sf::Vector2i mousePos = sf::Mouse::getPosition(window);
          boxes.push_back(Box(mousePos.x, mousePos.y, 200, 25));
          (*(boxes.end() - 1)).setInverseMass(0.05f);
        }
      }
    }
    // std::cout << "Number of circles: " << circles.size() << "\n";
    int subStep = 10;

    sf::CircleShape cpx(5.f);
    for (int a = 0; a < subStep; a++) {

      for (int i = 0; i < circles.size(); i++) {
        for (int j = i + 1; j < circles.size(); j++) {
          Collider::sphereAndSphere(circles[i], circles[j], cd);
        }
        for (auto &box : boxes) {
          Collider::sphereAndRectangle(circles[i], box, cd);
        }
      }

      for (int i = 0; i < boxes.size(); i++) {
        for (int j = i + 1; j < boxes.size(); j++) {
          Collider::rectangleAndRectangle(boxes[i], boxes[j], cd);
        }
      }

      contactResolver.resolveContacts(cd, deltaTime / subStep);
      cd.clear();
      // world.startFrame();
      for (auto &circle : circles) {
        gravity.updateForce(circle, deltaTime / subStep);
        circle.integrate(deltaTime / subStep);
      }
      for (auto &box : boxes) {
        gravity.updateForce(box, deltaTime / subStep);
        box.integrate(deltaTime / subStep);
      }
    }

    // world.runPhysics(deltaTime);
    // std::chrono::high_resolution_clock::time_point end =
    //     std::chrono::high_resolution_clock::now();
    // record delta time in seconds
    // deltaTime *= timeScale;
    for (auto &circle : circles) {
      window.draw(circle);
      circle.update();
      circle.setOutlineThickness(1.f);
      circle.setOutlineColor(sf::Color::Red);
    }
    for (auto &box : boxes) {
      window.draw(box);
      box.update();
      box.setOutlineThickness(1.f);
      box.setOutlineColor(sf::Color::Red);
    }

    frameNo++;
    if (frameNo % 100 == 0) {
      std::cout << "Fps:" << 1 / frameDuration << "\n";
    }
    // std::cout << cd.contacts.size() << "\n";
    if (!boxes.empty()) {

      // std::cout << boxes[0].getAngularVelocity() << "\n";
    }

    window.draw(cpx);
    window.display();
    window.clear(sf::Color::Black);
  }
  return 0;
}
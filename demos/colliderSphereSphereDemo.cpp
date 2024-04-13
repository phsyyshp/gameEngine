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

int main(int argc, char *argv[]) {
  sf::ContextSettings settings;
  sf::RenderWindow window(sf::VideoMode(800, 600), "My window",
                          sf::Style::Default);
  // window.setFramerateLimit(60);
  float deltaTime = 0.001f;
  float frameDuration;
  bool isDebug = false;
  if (argc > 1 && std::string(argv[1]) == "-d") {
    isDebug = true;
  }
  World world(200, isDebug);

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
  Box mbox(400, 605, 800, 100);
  Box mboxLeft(0, 250, 100, 600);
  Box mboxRight(600, 250, 100, 600);
  mbox.setInverseInertia(0.F);
  mboxLeft.setInverseInertia(0.F);
  mboxRight.setInverseInertia(0.F);
  Gravity gravity({0.f, lengthScale * 9.8f});
  world.registerBody(mbox);
  world.registerBody(mboxLeft);
  world.registerBody(mboxRight);
  world.registerGravity(gravity);

  world.setWindow(window);
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
          Circle circle(mousePos.x, mousePos.y, 10);
          float density = 1.0f;
          float mass = density * 3.14159f * 10 * 10;

          circle.setInverseMass(1 / mass);

          world.registerBody(circle);
        }
        if (event.mouseButton.button == sf::Mouse::Right) {
          sf::Vector2i mousePos = sf::Mouse::getPosition(window);
          Box box(mousePos.x, mousePos.y, 200, 25);
          float density = 0.5f;
          float mass = density * 200 * 25;
          box.setInverseMass(1 / mass);
          world.registerBody(box);
        }
      }
    }
    // std::cout << "Number of circles: " << circles.size() << "\n";
    int subStep = 10;
    world.runPhysics(deltaTime, subStep);
    for (auto &circle : world.getCircles()) {
      window.draw(circle);
      circle.update();
      circle.setOutlineThickness(1.f);
      circle.setOutlineColor(sf::Color::Red);
      if (isDebug) {
        circle.setFillColor(sf::Color::Transparent);
      }
      // if (circle.isAwake()) {
      //   circle.setFillColor(sf::Color::Green);
      // } else {
      //   circle.setFillColor(sf::Color::Red);
      // }
    }
    for (auto &box : world.getBoxes()) {
      window.draw(box);
      box.update();
      box.setOutlineThickness(1.f);
      if (isDebug) {

        box.setOutlineColor(sf::Color::Blue);
        box.setFillColor(sf::Color::Transparent);
      }

      // if (box.isAwake()) {
      //   box.setFillColor(sf::Color::Green);
      // } else {
      //   box.setFillColor(sf::Color::Red);
      // }
    }

    frameNo++;
    if (frameNo % 100 == 0) {
      std::cout << frameDuration * 1000 << " ms " << world.getBodySize()
                << "\n";
    }
    // std::cout << cd.contacts.size() << "\n";

    window.display();
    window.clear(sf::Color::Black);
  }
  return 0;
}
#include "forceGeneration.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include "visuals.hpp"
#include "world.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <memory>

int main(int argc, char *argv[]) {
  sf::ContextSettings settings;
  auto &window = Visual::window;
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
  // lengthScale = 0.F;
  std::unique_ptr<Box> mbox = std::make_unique<Box>(400, 605, 800, 100);
  std::unique_ptr<Box> mboxLeft = std::make_unique<Box>(0, 250, 100, 600);
  std::unique_ptr<Box> mboxRight = std::make_unique<Box>(600, 250, 100, 600);
  mbox->setInverseInertia(0.F);
  mboxLeft->setInverseInertia(0.F);
  mboxRight->setInverseInertia(0.F);
  Gravity gravity({0.f, lengthScale * 9.8f});
  world.registerBody(std::move(mbox));
  world.registerBody(std::move(mboxLeft));
  world.registerBody(std::move(mboxRight));
  world.registerGravity(gravity);

  world.setWindow(window);

  while (window.isOpen()) {
    sf::Event event;

    // Take inputs;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
      if (event.type == sf::Event::MouseButtonPressed) {
        if (event.mouseButton.button == sf::Mouse::Left) {
          sf::Vector2i mousePos = sf::Mouse::getPosition(window);
          std::unique_ptr<Circle> circle =
              std::make_unique<Circle>(mousePos.x, mousePos.y, 30);
          float density = 1.0f;
          float mass = density * 3.14159f * 10 * 10;
          circle->setInverseMass(1 / mass);
          world.registerBody(std::move(circle));
        }
        if (event.mouseButton.button == sf::Mouse::Right) {
          sf::Vector2i mousePos = sf::Mouse::getPosition(window);
          std::unique_ptr<Box> box =
              std::make_unique<Box>(mousePos.x, mousePos.y, 200, 25);
          float density = 0.5f;
          float mass = density * 200 * 25;
          box->setInverseMass(1 / mass);
          world.registerBody(std::move(box));
        }
      }
    }

    // run physics
    int subStep = 1;
    world.runPhysics(deltaTime, subStep);
    for (auto &body : world.getBodies()) {
      if (body->type() == RigidBody2DType::CIRCLE) {

        Circle *bodyc = static_cast<Circle *>(body.get());
        bodyc->update();
        bodyc->setOutlineThickness(1.f);
        bodyc->setOutlineColor(sf::Color::Red);
        if (isDebug) {
          bodyc->setFillColor(sf::Color::Transparent);
        }
        window.draw(*bodyc);
      }
      if (body->type() == RigidBody2DType::BOX) {

        Box *bodyb = static_cast<Box *>(body.get());
        bodyb->update();
        bodyb->setOutlineThickness(1.f);
        bodyb->setOutlineColor(sf::Color::Red);
        if (isDebug) {
          bodyb->setFillColor(sf::Color::Transparent);
        }
        window.draw(*bodyb);
      }
    }

    window.display();
    window.clear(sf::Color::Black);
  }
  return 0;
}
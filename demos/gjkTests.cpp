
#include "forceGeneration.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include "world.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/System/Vector2.hpp>
#include <memory>

int main(int argc, char *argv[]) {
  // sf::View view;
  // view.zoom(2.F);
  auto &window = Visual::window;
  sf::View view(sf::Vector2f(window.getSize().x / 2, window.getSize().y / 2),
                sf::Vector2f(window.getSize().x, window.getSize().y));

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
  std::unique_ptr<Box> box1 = std::make_unique<Box>(200, 100, 100, 50);
  std::unique_ptr<Circle> circle = std::make_unique<Circle>(400, 300, 30);
  mbox->setInverseInertia(0.F);
  mboxLeft->setInverseInertia(0.F);
  mboxRight->setInverseInertia(0.F);
  box1->setInverseMass(0.5F);
  circle->setInverseMass(0.5F);
  Gravity gravity({0.f, lengthScale * 9.8f});
  world.registerBody(std::move(mbox));
  world.registerBody(std::move(mboxLeft));
  world.registerBody(std::move(mboxRight));
  world.registerBody(std::move(circle));
  world.registerBody(std::move(box1));

  // world.registerGravity(gravity);

  world.setWindow(window);

  while (window.isOpen()) {
    float userAddedOrientation = 0;
    sf::Vector2f userAdedVelocity = {0.f, 0.f};
    sf::Event event;

    // Take inputs;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
      if (event.type == sf::Event::KeyPressed) {

        if (event.key.code == sf::Keyboard::Left) {
          userAdedVelocity += {-1000.f, 0.f};
        }
        if (event.key.code == sf::Keyboard::Right) {
          userAdedVelocity += {1000.f, 0.f};
        }
        if (event.key.code == sf::Keyboard::Up) {
          userAdedVelocity = {0.f, -1000.f};
        }
        if (event.key.code == sf::Keyboard::Down) {
          userAdedVelocity = {0.f, 1000.f};
        }
        if (event.key.code == sf::Keyboard::A) {
          view.move(-10.f, 0.f);
        }
        if (event.key.code == sf::Keyboard::D) {
          view.move(10.f, 0.f);
        }
        if (event.key.code == sf::Keyboard::W) {
          view.move(0.f, -10.f);
        }
        if (event.key.code == sf::Keyboard::S) {
          view.move(0.f, 10.f);
        }

        if (event.key.code == sf::Keyboard::Q) {
          userAddedOrientation = M_PI / 10;
        }
        if (event.key.code == sf::Keyboard::E) {
          userAddedOrientation = -M_PI / 10;
        }
      }
    }

    // run physics
    int subStep = 1;

    world.getBodies().back()->addVelocity(userAdedVelocity);
    world.runPhysics(deltaTime, subStep);
    world.getBodies().back()->addVelocity(-userAdedVelocity);
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

    window.setView(view);
    window.display();
    window.clear(sf::Color::Black);
  }
  return 0;
}
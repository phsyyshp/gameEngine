#include "forceGeneration.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include "visuals.hpp"
#include "world.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/Vertex.hpp>
#include <SFML/System/Sleep.hpp>
#include <memory>

int main(int argc, char *argv[]) {
  sf::ContextSettings settings;
  Visual vs;
  auto &window = vs.window;
  // window.setFramerateLimit(60);
  float deltaTime = 0.001f;
  float frameDuration;
  int frameCounter = 0;
  if (argc > 1 && std::string(argv[1]) == "-d") {
    vs.isDebug = true;
  }
  World world(200,false);

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

  //   for (int i = 0; i < 5; i++) {
  //     sf::Vector2i mousePos = {100, 200};
  //     std::unique_ptr<Circle> circle =
  //         std::make_unique<Circle>(mousePos.x + 10 * i, mousePos.y + 60 * i,
  //         30);
  //     float density = 0.1f;
  //     float mass = density * 3.14159f * 10 * 10;
  //     circle->setInverseMass(1 / mass);
  //     // circle->addAngularVelocity(50.5F);
  //     world.registerBody(std::move(circle));
  //   }
  for (int i = 0; i < 10; i++) {
    sf::Vector2i mousePos = {200, 200};
    std::unique_ptr<Box> box = std::make_unique<Box>(
        mousePos.x + 0 * 70 * i, mousePos.y + 25 * i, 70, 25);
    float density = 0.001f;
    float mass = density * 200 * 25;
    box->setInverseMass(1 / mass);
    world.registerBody(std::move(box));
  }
  while (window.isOpen()) {
    sf::Event event;
    // Take inputs;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    world.runPhysics(deltaTime);

    std::vector<RigidBody2D *> rawPtrVec;

    // Transform unique_ptr vector to raw pointer vector
    std::transform(
        world.getBodies().begin(), world.getBodies().end(),
        std::back_inserter(rawPtrVec),
        [](const std::unique_ptr<RigidBody2D> &ptr) { return ptr.get(); });
    vs.render(rawPtrVec);
    window.display();
    window.clear(sf::Color::Black);
    frameCounter++;
    if (frameCounter > 2000) {
      window.close();
    }
    // sf::sleep(sf::seconds(20));
  }
  return 0;
}
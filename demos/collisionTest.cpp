
#include "contactResolver.hpp"
#include "forceGeneration.hpp"
#include "shapes.hpp"
#include "world.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <chrono>
#include <iostream>

void showVertices(sf::RenderWindow &window,
                  const std::array<sf::Vector2f, 4> &vertices,
                  sf::Color color = sf::Color::Red) {
  for (int k = 0; k < 4; k++) {
    sf::CircleShape ax(5.f);
    ax.setFillColor(color);
    ax.setPosition(vertices[k]);
    ax.setOrigin(5, 5);
    window.draw(ax);
  }
}
std::array<sf::Vertex, 2> showConnectionNormal(sf::RenderWindow &window,
                                               const sf::Vector2f &pos,
                                               const sf::Vector2f &normal) {
  std::array<sf::Vertex, 2> line = {
      sf::Vertex(pos, sf::Color::Green),
      sf::Vertex(pos + normal * 100.f, sf::Color::Green),
  };
  return line;
}
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
  float userAdedRotation = 0.f;
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
  Box box1(400, 300, 100, 50);
  Box box2(400, 100, 100, 50);
  box1.setInverseMass(0.05f);
  box2.setInverseMass(0.05f);
  box2.setOrientation(M_PI / 3);
  boxes.push_back(box1);
  boxes.push_back(box2);
  box1.setFillColor(sf::Color::Red);

  CollisionData cd;

  Gravity gravity({0.f, 0 * lengthScale * 9.8f});
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
    sf::Vector2f userAdedVelocity = {0.f, 0.f};
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
          userAdedVelocity += {0.f, -1000.f};
        }
        if (event.key.code == sf::Keyboard::Down) {
          userAdedVelocity += {0.f, 1000.f};
        }
        if (event.key.code == sf::Keyboard::Space) {
          userAdedRotation += M_PI / 10;
        }
      }
    }
    // std::cout << "Number of circles: " << circles.size() << "\n";
    int subStep = 10;

    sf::CircleShape cpx(5.f);
    std::array<sf::Vertex, 2> line;
    // for (int a = 0; a < subStep; a++) {

    for (int i = 0; i < boxes.size(); i++) {
      for (int j = i + 1; j < boxes.size(); j++) {

        if (Collider::rectangleAndRectangle(boxes[i], boxes[j], cd)) {
          boxes[i].setFillColor(sf::Color::Red);
          boxes[j].setFillColor(sf::Color::Red);
          // std::cout << con.getPosition().x << "||" << con.getPosition().y
          //           << "\n";
          Box boxA = boxes[i];
          Box boxB = boxes[j];
          std::array<sf::Vector2f, 4> verticesB = boxB.getVertices();
          // check if each vertex of B is inside A.
          for (auto vertex : verticesB) {
            if (boxA.isPointIn(vertex)) {
              cpx.setFillColor(sf::Color::Green);
              cpx.setOrigin(5, 5);
              cpx.setPosition(vertex);
            }
          }
          showVertices(window, verticesB);
          showVertices(window, boxA.getVertices(), sf::Color::Blue);
          // line = showConnectionNormal(
          //     window, (cd.contacts.end() - 1)->getContactPoint(),
          //     (cd.contacts.end() - 1)->getContactNormal());
          // }
          //   Collider::rectangleAndRectangle(boxes[j], boxes[i], cd);
        }
      }
      // draw all contacts
      for (auto &mans : cd.getContactManifolds()) {
        for (auto cons : mans.getContacts()) {
          sf::CircleShape ax(5.f);
          ax.setFillColor(sf::Color::Green);
          ax.setPosition(cons.getContactPoint());
          ax.setOrigin(5, 5);
          window.draw(ax);
          std::array<sf::Vertex, 2> line = {
              sf::Vertex(cons.getContactPoint(), sf::Color::Green),
              sf::Vertex(cons.getContactPoint() +
                             cons.getContactNormal() *
                                 std::abs(cons.getPenetrationDepth()),
                         sf::Color::Green),
          };
          window.draw(line.data(), 2, sf::Lines);
        }
      }

      // contactResolver.resolveContacts(cd, deltaTime / subStep);
      cd.clear();
      boxes[3].addVelocity(userAdedVelocity);
      for (auto &box : boxes) {

        box.integrate(deltaTime / subStep);
      }
      boxes[3].addVelocity(-userAdedVelocity);
    }
    boxes[3].setOrientation(userAdedRotation);

    for (auto &box : boxes) {
      box.setFillColor(sf::Color::Transparent);
      box.setOutlineColor(sf::Color::Red);
      box.setOutlineThickness(2.F);
      window.draw(box);
      box.update();
    }

    window.draw(cpx);
    // window.draw(line.data(), line.size(), sf::Lines);

    window.display();
    window.clear(sf::Color::Black);
  }
  return 0;
}
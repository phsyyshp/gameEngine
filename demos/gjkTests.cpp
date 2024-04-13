

#include "collisionHandler.hpp"
#include "contactResolver.hpp"
#include "forceGeneration.hpp"
#include "shapes.hpp"
#include "world.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/System/Vector2.hpp>
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

  ContactResolver contactResolver;

  Box box1(400, 100, 100, 50);
  Box box2(300, 200, 100, 50);

  box1.setInverseMass(0.05f);
  Circle circle(200, 200, 50.f);

  Box orvect(400, 300, 20, 5);
  CollisionData cd;

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
    float userAddedOrientation = 0;
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
        if (event.key.code == sf::Keyboard::W) {
          userAddedOrientation = 0.1;
        }
      }
    }

    box1.addVelocity(userAdedVelocity);
    box1.integrate(deltaTime);
    box1.addVelocity(-userAdedVelocity);
    orvect.setOrientation(userAddedOrientation + orvect.getOrientation());
    sf::Vector2f orvec = rotate(sf::Vector2f{1, 0}, orvect.getOrientation());
    std::vector<sf::Vector2f> vertices;
    std::vector<sf::Vector2f> vertices2;

    for (auto vert : box1.getVertices()) {
      vertices.push_back(vert);
    }
    for (auto vert : box2.getVertices()) {
      vertices2.push_back(vert);
    }

    sf::Vector2f spRectangle = Collider::getSupportP(vertices, orvec);
    sf::Vector2f spCircle = Collider::getSupportS(circle, orvec);

    std::cout << spRectangle.x << " " << spRectangle.y << "\n";
    Circle sp(spRectangle.x, spRectangle.y, 5.f);
    Circle spc(spCircle.x, spCircle.y, 5.f);
    bool isCollide = Collider::GJKintersectionPP(box1, box2);
    if (isCollide) {
      box2.setFillColor(sf::Color::Red);
    } else {
      box2.setFillColor(sf::Color::White);
    }
    orvect.update();
    box1.update();
    box2.update();
    window.draw(sp);
    window.draw(spc);
    window.draw(box2);

    window.draw(box1);
    window.draw(circle);
    orvect.update();
    window.draw(orvect);

    window.display();
    window.clear(sf::Color::Black);
  }
  return 0;
}
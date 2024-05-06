
#include "visuals.hpp"
#include "linearalg.hpp"
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/System/Vector2.hpp>
#include <memory>
// sf::RenderWindow Visual::window(sf::VideoMode(800, 600), "My window",
// sf::Style::Default);
void Visual::showPoints(sf::RenderWindow &window,
                        const std::vector<la::Vector> &p, sf::Color color) {
  for (auto point : p) {
    sf::CircleShape ax(2.f);
    ax.setFillColor(color);
    ax.setPosition({point.x, point.y});
    ax.setOrigin(2, 2);
    window.draw(ax);
  }
}

la::Vector Visual::plotLine(const std::vector<la::Vector> &points,
                            sf::RenderWindow &window, sf::Color color) {
  std::array<sf::Vertex, 2> line = {
      sf::Vertex({points[0].x, points[0].y}, color),
      sf::Vertex({points[1].x, points[1].y}, color)};
  window.draw(line.data(), 2, sf::Lines);
}

la::Vector Visual::plotLine(const std ::array<la::Vector, 2> &points,
                            sf::RenderWindow &window, sf::Color color) {
  std::array<sf::Vertex, 2> line = {
      sf::Vertex({points[0].x, points[0].y}, color),
      sf::Vertex({points[1].x, points[1].y}, color)};
  window.draw(line.data(), 2, sf::Lines);
}
void Visual::render(const std::vector<RigidBody2D *> &bodies) {

  for (auto &body : bodies) {
    if (body->type() == RigidBody2DType::CIRCLE) {
      Circle *bodyc = static_cast<Circle *>(body);
      sf::CircleShape circle(bodyc->getRadius());
      circle.setOrigin(bodyc->getRadius(), bodyc->getRadius());
      circle.setPosition(bodyc->getPosition().x, bodyc->getPosition().y);
      circle.setRotation(bodyc->getOrientation() / M_PIf * 180.f);
      circle.setOutlineThickness(1.f);
      circle.setOutlineColor(sf::Color::Red);
      if (isDebug) {
        if (bodyc->isAwake()) {
          circle.setOutlineColor(sf::Color::Green);
        }
        circle.setFillColor(sf::Color::Transparent);
      }
      window.draw(circle);
    }
    if (body->type() == RigidBody2DType::BOX) {

      Box *bodyb = static_cast<Box *>(body);
      sf::RectangleShape rectangle(
          sf::Vector2f{2 * bodyb->getHalfSize().x, 2 * bodyb->getHalfSize().y});

      rectangle.setPosition(bodyb->getPosition().x, bodyb->getPosition().y);
      rectangle.setOrigin(bodyb->getHalfSize().x, bodyb->getHalfSize().y);
      rectangle.setRotation(bodyb->getOrientation() / M_PIf * 180.f);
      rectangle.setOutlineThickness(1.f);
      rectangle.setOutlineColor(sf::Color::Red);
      if (isDebug) {
        if (bodyb->isAwake()) {
          rectangle.setOutlineColor(sf::Color::Green);
        }
        rectangle.setFillColor(sf::Color::Transparent);
      }
      window.draw(rectangle);
    }
  }
}
void Visual::showContacts(std::map<ManifoldKey, Manifold> &manifolds) {
  for (auto &[key, manifold] : manifolds) {

    for (const Contact &contact : manifold.getContacts()) {
      float radius = 2.f;

      sf::CircleShape ax(radius);
      sf::Vector2f contactPoint = {contact.position.x, contact.position.y};
      ax.setFillColor(sf::Color::Green);
      ax.setPosition(contactPoint);
      ax.setOrigin(radius, radius);
      window.draw(ax);
      std::array<sf::Vertex, 2> line = {
          sf::Vertex(contactPoint, sf::Color::Green),
          sf::Vertex(contactPoint +
                         sf::Vector2f{contact.normal.x, contact.normal.y} *
                             contact.penetrationDepth,
                     sf::Color::Green),
      };
      window.draw(line.data(), 2, sf::Lines);
    }
  }
}
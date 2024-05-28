#pragma once
#include "collisionHandler.hpp"
#include "linearalg.hpp"
#include "manifold.hpp"
#include "rigidBody2D.hpp"
#include "shapes.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics/Vertex.hpp>
#include <SFML/System/Vector2.hpp>
#include <memory>

class Visual {
public:
  Visual() : window(sf::VideoMode(800, 600), "My window", sf::Style::Default){};
  sf::RenderWindow window;

  static sf::ContextSettings settings;
  bool isDebug = false;
  la::Vector plotLine(const std::vector<la::Vector> &points,
                      sf::RenderWindow &window,
                      sf::Color color = sf::Color::Yellow);

  la::Vector plotLine(const std ::array<la::Vector, 2> &points,
                      sf::RenderWindow &window,
                      sf::Color color = sf::Color::Yellow);
  void showPoints(sf::RenderWindow &window, const std::vector<la::Vector> &p,
                  sf::Color color = sf::Color::Yellow);
  void render(const std::vector<RigidBody2D *> &bodies);
  void showContacts(std::map<ManifoldKey, Manifold> &manifolds);
  void showSettings();
};
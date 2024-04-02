#include "entity.hpp"
#include <SFML/Graphics.hpp>
int main() {
  sf::ContextSettings settings;
  sf::RenderWindow window(sf::VideoMode(800, 600), "My window",
                          sf::Style::Default);
  window.setFramerateLimit(60);
  Entity shape(0.f, 0.f, 10.f, 50.f);
  shape.setFillColor(sf::Color(100, 250, 50));
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    float deltaTime = 1.f;
    shape.update(deltaTime);

    window.clear(sf::Color::Black);
    window.draw(shape);
    window.display();
  }
  return 0;
}
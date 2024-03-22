#include <SFML/Graphics.hpp>
int main() {
  sf::ContextSettings settings;
  // settings.antialiasingLevel = 13;
  sf::RenderWindow window(sf::VideoMode(800, 600), "My window",
                          sf::Style::Default);
  window.setFramerateLimit(60);
  sf::CircleShape shape(50.f);
  shape.setFillColor(sf::Color(100, 250, 50));
  float xCoord = 10.f;
  float yCoord = 50.f;

  shape.setPosition(xCoord, yCoord);
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
      shape.move(5.f, 0.f);
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
      shape.move(-5.f, 0.f);
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
      shape.move(0.f, -5.f);
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
      shape.move(0.f, 5.f);
    }

    window.clear(sf::Color::Black);
    window.draw(shape);
    window.display();
  }
  return 0;
}
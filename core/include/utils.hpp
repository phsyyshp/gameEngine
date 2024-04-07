#include <SFML/Graphics.hpp>
#include <SFML/System/Vector2.hpp>
#include <cmath>
#include <iostream>
float cross(const sf::Vector2f &, const sf::Vector2f &);
float dot(const sf::Vector2f &, const sf::Vector2f &);
float magnitude(const sf::Vector2f &);
sf::Vector2f normalise(sf::Vector2f);
sf::Vector2f perpendicular(const sf::Vector2f &);
sf::Vector2f transformToCordinateSystem(const sf::Vector2f &,
                                        const sf::Vector2f &, float);

sf::Vector2f inverseTransformToCordinateSystem(const sf::Vector2f &,
                                               const sf::Vector2f &, float);
sf::Vector2f rotate(const sf::Vector2f &, float);
#pragma once
#include <SFML/Graphics.hpp>
#include <SFML/System/Vector2.hpp>
#include <cmath>
#include <iostream>
#include <numbers>
float cross(const sf::Vector2f &, const sf::Vector2f &);
float dot(const sf::Vector2f &, const sf::Vector2f &);
float absDot(const sf::Vector2f &, const sf::Vector2f &);

float magnitude(const sf::Vector2f &);
sf::Vector2f normalise(sf::Vector2f);
sf::Vector2f perpendicular(const sf::Vector2f &);
sf::Vector2f transformToCordinateSystem(const sf::Vector2f &vectorToTransformed,
                                        const sf::Vector2f &positionOfOrigin,
                                        float orientationOfOrigin);

sf::Vector2f inverseTransformToCordinateSystem(const sf::Vector2f &,
                                               const sf::Vector2f &, float);
sf::Vector2f rotate(const sf::Vector2f &, float);
std::array<sf::Vector2f, 2> getBaseCoordinateSystem(float);
sf::Vector2f elementViseMultipication(const sf::Vector2f &,
                                      const sf::Vector2f &);
sf::Vector2f tripleproduct(const sf::Vector2f &, const sf::Vector2f &,
                           const sf::Vector2f &);
void showPoints(sf::RenderWindow &window, const std::vector<sf::Vector2f> &p,
                sf::Color color = sf::Color::Yellow);
sf::Vector2f computeIntersection(const std::array<sf::Vector2f, 2> &,
                                 const std::array<sf::Vector2f, 2> &);
sf::Vector2f computeIntersection(const std::vector<sf::Vector2f> &,
                                 const std::array<sf::Vector2f, 2> &);
sf::Vector2f plotLine(const std::vector<sf::Vector2f> &points,
                      sf::RenderWindow &window,
                      sf::Color color = sf::Color::Yellow);

sf::Vector2f plotLine(const std ::array<sf::Vector2f, 2> &points,
                      sf::RenderWindow &window,
                      sf::Color color = sf::Color::Yellow);
#include <SFML/Graphics.hpp>
struct Contact {
  sf::Vector2f contactPosition;
  sf::Vector2f contactNormal;
  float penetrationDepth;
};
class CollisionData {
public:
  std::array<Contact, 256> contacts;
  void push_back(const Contact &contact) {
    contacts[lastContactIdx] = contact;
    lastContactIdx++;
  }
  size_t size() const { return lastContactIdx; }

private:
  size_t lastContactIdx = 0;
};
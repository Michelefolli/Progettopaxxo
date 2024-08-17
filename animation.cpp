#include "Boids.hpp"
const float Pi = 3.14159265358979323846264f;
/*class Animation {
 public:
  Animation(int width, int height) {
    texture.loadFromFile("Boids_flight_cycle.png");
    for (int i = 0; i <= nFrames; ++i) {
      frames[i] = {i * (width), 0, (i + 1) * width, height};
    }
  }
  void ApplyToSprite(sf::Sprite& s) const {
    s.setTexture(texture);
    s.setTextureRect(frames[initial_frame]);
  }
  void Update (float dt){time += dt;}

 private:
  static const int nFrames = 8;
  static const float holdTime = 0.1f;
  sf::Texture texture;
  sf::IntRect frames[nFrames];
  sf::Sprite sprite;
  int initial_frame = 0;
  float initial_timme = 0.f;
};*/
class Animation {
 private:
  sf::Texture texture;
  sf::IntRect frame{0, 0, 160, 160};

 public:
  void Animation_setup() {
    texture.loadFromFile("Boids_flight_cycle.png");
    sprite.setOrigin(80, 80);
  };
  sf::Sprite sprite{texture, frame};
  void AdvanceFrame(sf::Clock clock) {
    if (clock.getElapsedTime().asSeconds() > 0.5f) {
      if (frame.left == 1120)
        frame.left = 0;
      else
        frame.left += 160;
      sprite.setTextureRect(frame);
      clock.restart();
    }
  }
};

void Boid::draw_animation(sf::RenderWindow& window, sf::Sprite& sprite) const {
  sprite.setPosition((position.x), (position.y));
  float angle = (std::atan2(velocity.y, velocity.x) * 180 / (Pi)) + 90;
  sprite.setRotation(angle);
  window.draw(sprite);
};

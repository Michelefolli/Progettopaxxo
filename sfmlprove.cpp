#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include "Boids_ver_2.hpp"

void draw(sf::RenderWindow& window, Boid boid) {
  sf::CircleShape shape(5);
  shape.setPosition(boid.position.x, boid.position.y);
  shape.setFillColor(sf::Color::White);
  window.draw(shape);
}

int main() {
  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids Simulation");
  Sim sim;
  for (int i = 0; i < 100; ++i) {
    Boid boid(
        {static_cast<double>(std::rand() % 800), static_cast<double>(std::rand() % 600)},
        {static_cast<double>((std::rand() % 100 - 50) / 50.0f), static_cast<double>((std::rand() % 100 - 50) / 50.0f)});

    sim.add(boid);
  }

  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    window.clear();
    for (auto boid : sim.stormo_) {
      draw(window, boid);
    }
    window.display();
    sim.alignment_and_cohesion();
    sim.separation();
  }
}
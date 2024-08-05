#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/System.hpp>
#include <memory>

#include "Boids_ver_2.hpp"

int width = 1920;
int height = 1080;

void draw(sf::RenderWindow& window, const Boid& boid) {
  sf::CircleShape shape(4);  // dimensioni del cerchio
  shape.setPosition(
      static_cast<float>(boid.position.x),
      static_cast<float>(
          boid.position.y));  // uso i float perché sfml funziona con i float
  shape.setFillColor(sf::Color::White);
  window.draw(shape);
}  // disegna il boid come un cerchio

void out_of_border(Boid& boid) {
  if (boid.position.x < 0) {
    boid.position.x = 10;
    boid.velocity.x = boid.velocity.x *-1;
  }
  if (boid.position.y < 0) {
    boid.position.y = 10;
    boid.velocity.y = boid.velocity.y *-1;
  }
  if (boid.position.x > width) {
    boid.position.x = width-10;
    boid.velocity.x = boid.velocity.x *-1;
  }
  if (boid.position.y > height) {
    boid.position.y = height-10;
    boid.velocity.y = boid.velocity.y *-1;
  }
}  // questa funzione dovrebbe in teoria essere responsabile dell'effetto
   // pacman. Vedendo i risultati questa scelta non so se è delle migliori
   // perché sminchia un po' le regole forse

int main() {
  sf::RenderWindow window(sf::VideoMode(width, height),
                          "Boids Simulation");  // crea la finestra
  window.setFramerateLimit(1);                  // numero di fps
  std::srand(std::time(
      nullptr));  // setta il seed della generazione casuale attraverso il tempo
  Sim sim;        // inizializzo la simulazione
  sim.GetParams(10, 0.3, 0.2, 20,
                12);  // parametri totalmente a caso, sono quasi sicuramente
                      // responsabili dello strano comportamento
  for (int i = 0; i < 200; ++i) {
    Boid boid({static_cast<double>(std::rand() % width),
               static_cast<double>(std::rand() % height)},
              {static_cast<double>(((std::rand() % 4) - 2) * (0.0000000005)),
               static_cast<double>(((std::rand() % 4) - 2) * (0.0000000005))});

    sim.add(boid);
  }  // genera i boid casualmente e li aggiunge allo stormo

  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }  // permette di chiudere la finestra cliccado sulla x
    window.clear();  // pulisce la finestra ogni frame
    for (auto boid : sim.stormo_) {
      out_of_border(boid);
      draw(window, boid);
    }  // disegna tutti i boid, ma non li fa vedere ancora, quello è display
    window.display();
    sim.alignment_and_cohesion();
    sim.separation();
    sim.travel();
      // fa vedere i boid disegnati
  }
  return 0;  // perché è un int main deve ritornare quando finisce
}
#include <random>

#include "Boids.hpp"

const int width = sf::VideoMode::getDesktopMode().width;
const int height = sf::VideoMode::getDesktopMode().height;
//sono ridefinite che è una cosa molto stupida

const float max_speed = 8;


/*void out_of_border(Boid& boid) {
  if (boid.position.x < 0) {
    boid.position.x = 0;
    boid.velocity.x = boid.velocity.x * -1;
  }
  if (boid.position.y < 0) {
    boid.position.y = 0;
    boid.velocity.y = boid.velocity.y * -1;
  }
  if (boid.position.x > width) {
    boid.position.x = width;
    boid.velocity.x = boid.velocity.x * -1;
  }
  if (boid.position.y > height) {
    boid.position.y = height;
    boid.velocity.y = boid.velocity.y * -1;
  }
}  // questa funzione dovrebbe in teoria essere responsabile dell'effetto
   // pacman. Vedendo i risultati questa scelta non so se è delle migliori
   // perché sminchia un po' le regole forse
*/
int main() {
  sf::RenderWindow window(sf::VideoMode(width, height),
                          "Boids Simulation");  // crea la finestra
  // window.setFramerateLimit(1000);                // numero di fps
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<float> velocity_distribution(-4.0f, 4.0f);

  Params params{0.3, 0.8, 0.3, 45,
                20};  // parametri totalmente a caso, sono quasi sicuramente
                      // responsabili dello strano comportamento
  std::vector<Boid> flock;
  for (int i = 0; i < 600; ++i) {
    Boid boid(
        {static_cast<float>(std::rand() % width), static_cast<float>(std::rand() % height)},
        {velocity_distribution(generator), velocity_distribution(generator)});

    flock.push_back(boid);
  }  // genera i boid casualmente e li aggiunge allo stormo

  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }  // permette di chiudere la finestra cliccado sulla x
    window.clear();  // pulisce la finestra ogni frame
    for (auto& boid : flock) {
      boid.update(params, flock,
                  max_speed);  // funzione che limita la velocità

      boid.draw_on(window);

    }  // disegna tutti i boid, ma non li fa vedere ancora, quello è display
    window.display();
  }
  return 0;  // perché è un int main deve ritornare quando finisce
}
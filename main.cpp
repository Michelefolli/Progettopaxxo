#include <random>

#include "Boids.hpp"

const int width = sf::VideoMode::getDesktopMode().width;
const int height = sf::VideoMode::getDesktopMode().height;
// sono ridefinite che è una cosa molto stupida

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

  int elapsed = 500;  // millisecondi tra un'acquisizione e l'altra, ho messo 1
                       // per non eccedere il tempo massimo di godbolt
  int flock_size = 600;
  std::vector<Boid> flock;
  for (int i = 0; i < flock_size; ++i) {
    Boid boid(
        {static_cast<float>(std::rand() % width),
         static_cast<float>(std::rand() % height)},
        {velocity_distribution(generator), velocity_distribution(generator)});

    flock.push_back(boid);
  }  // genera i boid casualmente e li aggiunge allo stormo

  std::vector<Stats> timestamped_stats;

  std::thread first(update_Stats, std::cref(flock), std::ref(timestamped_stats),
                    std::ref(flock_size), std::ref(elapsed), std::ref(window));
  simulation(window, flock, params, max_speed);

  first.join();

  int conditional = getUserInput(); //chiediamo all'utente se vuole stampare
  exportStatsIfNeeded(conditional, timestamped_stats); //stampiamo se rechiesto

  std::cout << "The simulation is complete! \n ";

  return 0;
}
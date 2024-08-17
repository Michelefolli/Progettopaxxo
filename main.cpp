#include <mutex>
#include <random>
#include <thread>

#include "Boids.hpp"

const float max_speed = 8;

int main() {
  const unsigned int width = sf::VideoMode::getDesktopMode().width;
  const unsigned int height = sf::VideoMode::getDesktopMode().height;
  sf::RenderWindow window(sf::VideoMode(width, height),
                          "Boids Simulation");  // crea la finestra
  window.setPosition({0, 0});
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<float> velocity_distribution(-4.0f, 4.0f);

  Params params{0.3f, 0.8f, 0.3f, 45.f,
                20.f};  // parametri totalmente a caso, sono quasi sicuramente
                        // responsabili dello strano comportamento

  int frequency = 1000;  // millisecondi tra un'acquisizione e l'altra
  int flock_size = 600;
  std::vector<Boid> flock;
  for (int i = 0; i < flock_size; ++i) {
    Boid boid(
        {static_cast<float>(std::rand() % static_cast<int>(width)),
         static_cast<float>(std::rand() % static_cast<int>(height))},

        {velocity_distribution(generator), velocity_distribution(generator)});

    flock.push_back(boid);
  }  // genera i boid casualmente e li aggiunge allo stormo

  std::vector<Stats> timestamped_stats;

  std::thread first(update_Stats, std::cref(flock), std::ref(timestamped_stats),
                    std::ref(frequency), std::ref(window));

  simulation(window, flock, params, max_speed);

  first.join();  // chiediamo all'utente se vuole stampare
  exportStats(timestamped_stats);
  exportPlot(timestamped_stats);
  std::cout << "The simulation is complete! \n";

  return 0;
}
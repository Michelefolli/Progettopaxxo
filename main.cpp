#include <mutex>
#include <random>
#include <thread>

#include "Boids.hpp"

int main() {
  const float max_speed = 10;
  const unsigned int screen_width = sf::VideoMode::getDesktopMode().width;
  const unsigned int screen_height = sf::VideoMode::getDesktopMode().height;

  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<float> velocity_distribution(
      -(max_speed / std::sqrt(2.f)), (max_speed / std::sqrt(2.f)));

  Params simulation_params{};  // parametri totalmente a caso, sono quasi
                               // sicuramente responsabili dello strano
                               // comportamento
  int acquisiton_period{};     // millisecondi tra un'acquisizione e l'altra
  int flock_size{};

  inputData(flock_size, acquisiton_period, simulation_params);

  std::vector<Boid> flock;
  for (int i = 0; i < flock_size; ++i) {
    Boid boid(
        {static_cast<float>(std::rand() % static_cast<int>(screen_width)),
         static_cast<float>(std::rand() % static_cast<int>(screen_height))},

        {velocity_distribution(generator), velocity_distribution(generator)});

    flock.push_back(boid);
  }  // genera i boid casualmente e li aggiunge allo stormo

  auto flock_view = flock;
  std::vector<Stats> timestamped_stats;
  std::mutex synchro_tool;

  sf::RenderWindow window(sf::VideoMode(screen_width, screen_height),
                          "Boids Simulation");  // crea la finestra
  window.setPosition({0, 0});

  std::thread first(update_Stats, std::cref(flock_view),
                    std::ref(timestamped_stats), std::ref(acquisiton_period),
                    std::ref(window), std::ref(synchro_tool));

  runSimulation(window, flock, simulation_params, max_speed, flock_view,
                synchro_tool);

  first.join();  // chiediamo all'utente se vuole stampare
  exportStats(timestamped_stats);
  exportPlot(timestamped_stats);

  std::cout << "The simulation is complete! \n";

  return 0;
}
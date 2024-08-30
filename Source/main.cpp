#include <iostream>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include "Boids.hpp"
#include "IO_handling.hpp"

int main() {
  const float max_speed = 10;  // Arbitrarily chosen
  const unsigned int screen_width = sf::VideoMode::getDesktopMode().width;
  const unsigned int screen_height = sf::VideoMode::getDesktopMode().height;

  // Initializes key objects
  Params simulation_params{};
  int acquisition_period{};
  int flock_size{};

  inputData(flock_size, acquisition_period, simulation_params);

  // Sets up the rng infrastructure
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<float> velocity_distribution(
      -(max_speed / std::sqrt(2.f)), (max_speed / std::sqrt(2.f)));

  // Randomly generates the flock
  std::vector<Boid> flock;
  for (int i = 0; i < flock_size; ++i) {
    Boid boid_i(
        {static_cast<float>(std::rand() % static_cast<int>(screen_width)),
         static_cast<float>(std::rand() % static_cast<int>(screen_height))},

        {velocity_distribution(generator), velocity_distribution(generator)});

    flock.push_back(boid_i);
  }

  // Creates a copy of flock that can be read to minimize lag due to mutex lock
  auto flock_view = flock;
  std::vector<Stats> timestamped_stats;

  // Mutex used to ensure consistency in the multithreaded environment
  std::mutex synchro_tool;

  // Ensures the background is correctly loaded
  sf::Texture backgroundTexture;

  if (!backgroundTexture.loadFromFile("assets/sky_background.jpg")) {
    std::cout << "Failed to load background. Simulation aborted\n";
    return 1;
  }

  // Initialization of the SFML window
  sf::RenderWindow window(sf::VideoMode(screen_width, screen_height),
                          "Boids Simulation");  
  window.setPosition({0, 0});

  // Sets up parallel thread for stats handling
  std::thread parallel(
      updateStats, std::cref(flock_view), std::ref(timestamped_stats),
      std::ref(acquisition_period), std::ref(window), std::ref(synchro_tool));

  // Starts simulation
  runSimulation(window, backgroundTexture, flock, simulation_params, max_speed,
                flock_view, synchro_tool);

  // As soon as the window is closed the 2 functions stop looping and the thread
  // are re-joined
  parallel.join();

  // Calls for output handling
  exportStats(timestamped_stats);
  exportPlot(timestamped_stats);

  std::cout << "\033[96mThe simulation is complete!\033[0m \n";

  return 0;
}
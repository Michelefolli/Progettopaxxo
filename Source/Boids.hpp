#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <SFML/Graphics.hpp>
#include <chrono>
#include <cmath>
#include <mutex>
#include <vector>

// Minimal implementation of a 2 dimensional vector with basic algebraic
// operations
struct Vec_2d {
  Vec_2d(float x_val, float y_val)  // Constructor used to ensure the proper
      : x{x_val}, y{y_val} {}       // initialization of every Vec_2d object
  float x;
  float y;
  // algebraic operations
  Vec_2d operator+(const Vec_2d& v) const;
  Vec_2d operator-(const Vec_2d& v) const;
  Vec_2d operator*(const float c) const;
  Vec_2d operator/(const float c) const;
  Vec_2d& operator+=(const Vec_2d& v);

  float norm() const;
};

// Structure to hold all parameters for the flock simulation model
struct Params {
  float sep{};       // Separation coefficient
  float alig{};      // Alignment coefficient
  float cohes{};     // Cohesion coefficient
  float dist{};      // Maximum interaction distance
  float dist_sep{};  // Distance threshold for separation law
};

/*
 * Class that governs the behavior of each boid in the simulated flock.
 * Private member functions implement the flight laws for the boids,
 * applied through the public `update` method.
 * Public methods provide access to boid properties for external functions.
 */

class Boid {
 private:
  Vec_2d position;
  Vec_2d velocity;
  void limit(const float max_speed);
  const Vec_2d separation(const std::vector<Boid>& flock,
                          const Params& simulation_params) const;

  const Vec_2d alignment_and_cohesion(const std::vector<Boid>& flock,
                                      const Params& simulation_params) const;
  void avoidEdges(const float edges_width, const float edges_height);

 public:
  float abs_distance_from(const Boid& boid_j) const;
  Boid(Vec_2d position_val,  // Constructor to ensure the correct initialization
       Vec_2d velocity_val)  // of every BOID object
      : position{position_val}, velocity{velocity_val} {}
  const Vec_2d& getPosition() const;
  const Vec_2d& getVelocity() const;
  void update(const Params& simulation_params, const std::vector<Boid>& flock,
              const float max_speed, const float edges_width,
              const float edges_height);
  void drawOn(sf::RenderWindow& window) const;
};

// Aggregate of the statistical values calculated during the simulation along
// with a time coordinate
struct Stats {
  float v_mean{};   // Mean velocity
  float d_mean{};   // Mean distance
  float sigma_v{};  // Velocity standard deviation
  float sigma_d{};  // Distance standard deviation
  float time{};
};

// Functions responsible for running the simulation
auto scaleBackground(const sf::RenderWindow& window,
                     const sf::Texture& backgroundTexture);
void runSimulation(sf::RenderWindow& window,
                   const sf::Texture& backgroundTexture,
                   std::vector<Boid>& flock, const Params& simulation_params,
                   const float max_speed, std::vector<Boid>& flock_view,
                   std::mutex& synchro_tool);

// Functions responsible for calculating and gathering the stats during the
// simuation
Stats calculateStatistics(
    const std::vector<Boid>& flock_view,
    const std::chrono::time_point<std::chrono::steady_clock>& start_time);
void fillStatsVector(
    const std::vector<Boid>& flock_view, std::vector<Stats>& timestamped_stats,
    const std::chrono::time_point<std::chrono::steady_clock>& start_time);
void updateStats(const std::vector<Boid>& flock_view,
                 std::vector<Stats>& timestamped_stats, int acquisiton_period,
                 sf::RenderWindow& window, std::mutex& synchro_tool);

#endif
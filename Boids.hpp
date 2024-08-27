#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <SFML/Graphics.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

// Minimal implementation of a 2 dimensional vector with basic algebraic
// operations
struct Vec_2d {
  Vec_2d(float x_val, float y_val)  // Constructor used to ensure the proper
      : x(x_val), y(y_val) {}       // inizialization of every Vec_2d object
  float x;
  float y;
  // algebracic operations
  Vec_2d operator+(const Vec_2d& v) const { return Vec_2d(x + v.x, y + v.y); }
  Vec_2d operator-(const Vec_2d& v) const { return Vec_2d(x - v.x, y - v.y); }
  Vec_2d operator*(const float c) const { return Vec_2d(x * c, y * c); }
  Vec_2d operator/(const float c) const { return Vec_2d(x / c, y / c); }
  Vec_2d& operator+=(const Vec_2d& v) {
    x += v.x;
    y += v.y;
    return *this;
  }

  float norm() const { return std::sqrt(x * x + y * y); }
};

// The struct provides a way to gather all of the model's parameters in a single
// object
struct Params {
  float sep{};       // Separation coefficient
  float alig{};      // Alignment coefficient
  float cohes{};     // Cohesion coefficient
  float dist{};      // Maximum interaction distance
  float dist_sep{};  // Distance threshold for separation law
};

/*
 * Class governing the behaviour of the constituents of the simulated flock
 * Private member functions are the various flight laws imposed on the BOIDs
 * and are applied through the public member function update.
 * The other public member functions offer access to the BOIDs' specifics for
 * external functions
 */
class Boid {
 private:
  Vec_2d position;
  Vec_2d velocity;
  void limit(const float max_speed);
  const Vec_2d separation(const std::vector<Boid>& flock,   //
                          const Params& simulation_params)  //
      const;

  const Vec_2d alignment_and_cohesion(const std::vector<Boid>& flock,
                                      const Params& simulation_params) const;
  void avoid_edges(const float edges_width, const float edges_height);

 public:
  float abs_distance_from(const Boid& boid_j) const;
  Boid(Vec_2d position_val,  // Constructor to ensure the correct inizialization
       Vec_2d velocity_val)  // of every BOID object
      : position(position_val), velocity(velocity_val) {}
  const Vec_2d& getPosition() const;
  const Vec_2d& getVelocity() const;
  void update(const Params& simulation_params, const std::vector<Boid>& flock,
              const float max_speed, const float edges_width,
              const float edges_height);
  void draw_on(sf::RenderWindow& window) const;
};

// Aggregate of the statistical values calculated during the simulation along
// with a time coordinate
struct Stats {
  float v_mean{};   // Mean velocity
  float d_mean{};   // Mean distance
  float sigma_v{};  // Velocity standard deviation
  float sigma_d{};  // Distance standard deviation
  float time{};
  /*
    Stats& operator+=(const Stats& s) {
      v_mean += s.v_mean;
      d_mean += s.d_mean;
      sigma_v += s.sigma_v;
      sigma_d += s.sigma_d;
      time += 0;
      return *this;
    }*/
};

// Input handling functions
bool checkParametersValidity(int flock_size, int acquisiton_period,
                             Params& simulation_params);
void inputData(int& flock_size, int& acquisiton_period,
               Params& simulation_params);

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
    const std::chrono::time_point<std::chrono::steady_clock>&
        start_time);  // calcola le statistiche dello stormo in un
                      // determinato momento
void fillStatsVector(
    const std::vector<Boid>& flock_view, std::vector<Stats>& timestamped_stats,
    const std::chrono::time_point<std::chrono::steady_clock>& start_time);
void update_Stats(const std::vector<Boid>& flock_view,
                  std::vector<Stats>& timestamped_stats, int acquisiton_period,
                  sf::RenderWindow& window, std::mutex& synchro_tool);

// Output handling functions
std::string namingFile();
bool askForTxt();
void instantiateStatsFile(std::ostringstream& output_str);
void exportStats(const std::vector<Stats>& timestamped_stats);
bool askForPng();
void plotStats(const std::vector<Stats>& timestamped_stats, bool png_option,
               const std::string& png_file_name);
void exportPlot(const std::vector<Stats>& timestamped_stats);
#endif
#include "Boids.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <random>
#include <sstream>
#include <vector>

/*
 * Algebraic operations for 2d vectors
 */
Vec_2d Vec_2d::operator+(const Vec_2d& v) const {
  return Vec_2d(x + v.x, y + v.y);
}
Vec_2d Vec_2d::operator-(const Vec_2d& v) const {
  return Vec_2d(x - v.x, y - v.y);
}
Vec_2d Vec_2d::operator*(const float c) const { return Vec_2d(x * c, y * c); }
Vec_2d Vec_2d::operator/(const float c) const { return Vec_2d(x / c, y / c); }
Vec_2d& Vec_2d::operator+=(const Vec_2d& v) {
  x += v.x;
  y += v.y;
  return *this;
}
float Vec_2d::norm() const { return std::sqrt(x * x + y * y); }

// The repulsion from the borders being inversely proportional to the distance
// from the edge minimizes the interference, ensuring minimal changes to the
// original BOID model, while making the simulation more fluid. The exponent
// 0.25 and the repulsive range 100. are arbitrary.
void Boid::avoid_edges(const float edges_width, const float edges_height) {
  float repulsive_range = 100.f;
  auto distance_from_border = velocity.norm();
  auto epsilon = std::numeric_limits<
      float>::epsilon();  // division and multiplication by zero can lead to
                          // NaN errors hence the need for small epsilons

  if (position.x < repulsive_range) {
    velocity.x += (distance_from_border + epsilon) *
                  (1 / (std::pow((std::abs(position.x)), 0.25f) + epsilon));
  }
  if (position.y < repulsive_range) {
    velocity.y += (distance_from_border + epsilon) *
                  (1 / (std::pow((std::abs(position.y)), 0.25f) + epsilon));
  }
  // the "-" sign is necessary produce the negative velocity components for
  // the correct implementation of the repulsion model, hence the magic number
  // "-1"
  if (position.x > (edges_width - repulsive_range)) {
    velocity.x +=
        (distance_from_border + epsilon) * (-1) *
        (1 / (std::pow((std::abs(edges_width - position.x)), 0.25f) + epsilon));
  }
  if (position.y > (edges_height - repulsive_range)) {
    velocity.y += (distance_from_border + epsilon) * (-1) *
                  (1 / (std::pow((std::abs(edges_height - position.y)), 0.25f) +
                        epsilon));
  };
}

// Limiting the velocity increases the fidelity to real life flocks ensuring a
// more significant simulation
void Boid::limit(const float max_speed) {
  velocity = velocity * (max_speed / velocity.norm());
}

float Boid::abs_distance_from(const Boid& boid_j) const {
  return (position - boid_j.position).norm();
};

// Implementation of the separation law for a single Boid object
const Vec_2d Boid::separation(const std::vector<Boid>& flock,
                              const Params& simulation_params) const {
  Vec_2d v_sep =
      std::accumulate(
          flock.begin(), flock.end(), Vec_2d(0., 0.),
          [this, &simulation_params](Vec_2d sum, const Boid& other_boid) {
            if (abs_distance_from(other_boid) <= simulation_params.dist_sep) {
              Vec_2d diff = other_boid.getPosition() - position;
              sum += diff;
            }
            return sum;
          }) *
      (-simulation_params.sep);

  return v_sep;
};

// By combining the alignment and cohesion processes the calculations are
// optimized by reducing the number of times the flock vector has to be read
const Vec_2d Boid::alignment_and_cohesion(
    const std::vector<Boid>& flock, const Params& simulation_params) const {
  int count = 0;

  std::pair<Vec_2d, Vec_2d> sums = std::accumulate(
      flock.begin(), flock.end(),
      std::make_pair(Vec_2d(0.f, 0.f), Vec_2d(0.f, 0.f)),
      [this, &simulation_params, &count](std::pair<Vec_2d, Vec_2d> acc,
                                         const Boid& otherBoid) {
        if (this != &otherBoid &&
            this->abs_distance_from(otherBoid) <= simulation_params.dist) {
          acc.first += otherBoid.velocity;   // summing velocity vectors
          acc.second += otherBoid.position;  // summing position vectors
          ++count;
        }
        return acc;
      });

  if (count > 0) {
    Vec_2d avg_velocity = sums.first / static_cast<float>(count);
    Vec_2d center_of_mass = sums.second / static_cast<float>(count);

    Vec_2d v_alig = (avg_velocity - velocity) * simulation_params.alig;
    Vec_2d v_cohes = (center_of_mass - position) * simulation_params.cohes;

    return v_alig + v_cohes;
  } else
    return Vec_2d(0, 0);
};

const Vec_2d& Boid::getPosition() const { return position; }
const Vec_2d& Boid::getVelocity() const { return velocity; }

// Streamlines the process of updating the Boids' coordinates by applyng all
// of the laws consecutively.
void Boid::update(const Params& simulation_params,
                  const std::vector<Boid>& flock, const float max_speed,
                  const float edges_width, const float edges_height) {
  velocity += alignment_and_cohesion(flock, simulation_params) +
              separation(flock, simulation_params);

  avoid_edges(edges_width, edges_height);
  if (velocity.norm() > max_speed) {
    limit(max_speed);
  }
  position +=
      velocity;  // the time difference is assumed to be 1s per iteration
}

void Boid::draw_on(sf::RenderWindow& window) const {
  sf::CircleShape shape(6, 3);  // sets traingle shape
  shape.setPosition((position.x), (position.y));
  shape.setFillColor(sf::Color::Black);
  window.draw(shape);
}

auto scaleBackground(const sf::RenderWindow& window,
                     const sf::Texture& backgroundTexture) {
  sf::Vector2f windowSize(window.getSize());
  sf::Vector2f textureSize(backgroundTexture.getSize());
  sf::Vector2f scale(windowSize.x / textureSize.x,
                     windowSize.y / textureSize.y);
  return scale;
}

// Encapsulates the whole simulation process in a single function
void runSimulation(sf::RenderWindow& window,
                   const sf::Texture& backgroundTexture,
                   std::vector<Boid>& flock, const Params& simulation_params,
                   const float max_speed, std::vector<Boid>& flock_view,
                   std::mutex& synchro_tool) {
  const float edges_width =
      static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float edges_height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  // Ensures that the simulation keeps running while the window stays open
  sf::Event event;
  sf::Sprite backgroundSprite(backgroundTexture);
  backgroundSprite.setScale(scaleBackground(window, backgroundTexture));

  while (window.isOpen()) {
    // Enables the ability to close the window
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    window.clear();  // clears the window every frame

    window.draw(backgroundSprite);
    // Loops over the flock vector to update and draw every boid entity
    for (auto& boid : flock) {
      boid.update(simulation_params, flock, max_speed, edges_width,
                  edges_height);

      boid.draw_on(window);
    }

    // The possible lag caused by the stats calculation is reduced by
    // shortening the locked mutex window thanks to the use of the flock_view
    // vector
    synchro_tool.lock();
    flock_view = flock;
    synchro_tool.unlock();

    // Displays the boid on the window
    window.display();
  }
}

Stats calculateStatistics(
    const std::vector<Boid>& flock_view,
    const std::chrono::time_point<std::chrono::steady_clock>& start_time) {
  Stats stats{};  // Proper initialization of the Stats instance
  std::chrono::time_point current_time =
      std::chrono::steady_clock::now();  // gets current time
  auto time_span = std::chrono::duration_cast<std::chrono::duration<float>>(
      current_time - start_time);
  stats.time =
      time_span.count();  // The seconds passed since the start of the
                          // simulation are assigned to the time variable

  float n = static_cast<float>(flock_view.size());  // size of the flock

  // Mean velocity vector
  Vec_2d v_mean_vec = std::accumulate(
      flock_view.begin(), flock_view.end(), Vec_2d(0.f, 0.f),
      [](Vec_2d sum, const Boid& boid) { return sum += (boid.getVelocity()); });
  v_mean_vec = v_mean_vec / n;
  stats.v_mean = v_mean_vec.norm();  // Mean velocity norm

  auto sigma_v_val = std::accumulate(
      flock_view.begin(), flock_view.end(), 0.f,
      [&v_mean_vec](float sum, const Boid& boid) {
        return sum += std::pow((boid.getVelocity() - v_mean_vec).norm(), 2.f);
      });

  stats.sigma_v = std::sqrt(sigma_v_val / (n - 1));

  std::vector<float> pairwise_distances{};

  // Nested loop to gather all the pairwise distances between boids
  for (int i{}; i < static_cast<int>(n); ++i) {
    for (int j = i + 1; j < static_cast<int>(n); ++j) {
      pairwise_distances.push_back(
          flock_view[static_cast<unsigned int>(i)].abs_distance_from(
              flock_view[static_cast<unsigned int>(j)]));
    }
  }

  // Get the total number of pairs
  float total_pairs = static_cast<float>(pairwise_distances.size());

  // Sum all of the pairwise distances
  float dist_sum = std::accumulate(pairwise_distances.begin(),
                                   pairwise_distances.end(), 0.f);

  // Get mean distance
  stats.d_mean = dist_sum / total_pairs;

  // Find the sum of all the squared deviation
  float sqrd_deviation = std::accumulate(
      pairwise_distances.begin(), pairwise_distances.end(), 0.f,
      [&stats](float total_dev, const float& dist_ij) {
        return total_dev += std::pow((dist_ij - stats.d_mean), 2.f);
      });

  // Get distance standard deviation
  stats.sigma_d = std::sqrt(sqrd_deviation / (total_pairs - 1));
  
  return stats;
}

void fillStatsVector(
    const std::vector<Boid>& flock_view, std::vector<Stats>& timestamped_stats,
    const std::chrono::time_point<std::chrono::steady_clock>& start_time) {
  timestamped_stats.push_back(calculateStatistics(flock_view, start_time));
}

// Encapsulates the stats handling process in a single function
void updateStats(const std::vector<Boid>& flock_view,
                 std::vector<Stats>& timestamped_stats, int acquisiton_period,
                 sf::RenderWindow& window, std::mutex& synchro_tool) {
  auto start_time = std::chrono::steady_clock::now();
  auto last_update_time = start_time;
  auto current_time = start_time;
  // Ensures that the stats keep getting periodically acquired as long as the
  // simulation is running
  while (window.isOpen()) {
    current_time = std::chrono::steady_clock::now();  // updates current time
    // Checks that enough time has passed since the last acquisition
    if (current_time - last_update_time >=
        std::chrono::milliseconds(acquisiton_period)) {
      last_update_time =
          current_time;  // miantains the if statement proper inner working
      synchro_tool.lock();
      fillStatsVector(flock_view, timestamped_stats, start_time);
      synchro_tool.unlock();
    }
  }
}

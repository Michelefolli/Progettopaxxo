#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <SFML/Graphics.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include<mutex>

struct Vec_2d {
  Vec_2d(float x_val, float y_val)
      : x(x_val), y(y_val) {}  // do we really need it?
  float x;
  float y;
  Vec_2d operator+(const Vec_2d& v) const { return Vec_2d(x + v.x, y + v.y); }
  Vec_2d operator-(const Vec_2d& v) const { return Vec_2d(x - v.x, y - v.y); }
  Vec_2d operator*(const float c) const { return Vec_2d(x * c, y * c); }
  Vec_2d operator/(const float c) const { return Vec_2d(x / c, y / c); }
  Vec_2d& operator+=(const Vec_2d& v) {
    x += v.x;
    y += v.y;
    return *this;
  }

  float norm() const { return std::sqrt(x * x + y * y); }  // norma del vettore

};  // vettore a due dimensioni. DUBBIO: operatori e norma da definire nel cpp?

struct Params {
  float sep{};       // fattore di separazione
  float alig{};      // fattore di allineamento
  float cohes{};     // fattore di coesione
  float dist{};      // raggio visivo dei boids
  float dist_sep{};  // distanza minima
};  // parametri dello stormo

class Boid {
 private:
  Vec_2d position;
  Vec_2d velocity;
  void limit(const float& max_speed);  // limite di velocità
  const Vec_2d separation(const std::vector<Boid>& flock, const float& sep,
                          const float& dist_sep)
      const;  // calcola il vettore velocità di separazione

  const Vec_2d alignment_and_cohesion(const std::vector<Boid>& flock,
                                      const float& alig, const float& cohes,
                                      const float& dist)
      const;  // calcola il vettore velocità di separazione e coesione
  void avoid_edges(const int width, const int height);

 public:
  float abs_distance_from(
      const Boid& boid_j) const;  // valore assoluto della distanza tra due boid
  Boid(Vec_2d position_val, Vec_2d velocity_val)
      : position(position_val), velocity(velocity_val) {}
  const Vec_2d& getPosition() const;
  const Vec_2d& getVelocity() const;
  void setPosition(const Vec_2d& pos);
  void setVelocity(const Vec_2d& vel);
  void update(const Params& params, const std::vector<Boid>& flock,
              const float& max_speed, const int width,
              const int height);  // aggiunge i modificatori di velocità, limita
                                  // la velocità e poi sposta il boid
  void draw_on(
      sf::RenderWindow& window) const;  // disegna il boid sulla finestra sfml
};

struct Stats {
  float v_mean{};   // velocità media
  float d_mean{};   // distanza media
  float sigma_v{};  // deviazione stardard velocità
  float sigma_d{};  // deviazione standard distanza
  float time{};

  Stats& operator+=(const Stats& s) {
    v_mean += s.v_mean;
    d_mean += s.d_mean;
    sigma_v += s.sigma_v;
    sigma_d += s.sigma_d;
    time += 0;
    return *this;
  }

};  // struttura delle statistiche

Stats statistics(const std::vector<Boid>& flock,
                 const std::chrono::time_point<std::chrono::steady_clock>&
                     start_time);  // calcola le statistiche dello stormo in un
                                   // determinato momento

void simulation(sf::RenderWindow& window, std::vector<Boid>& flock,
                Params& params, const float& max_speed, std::vector<Boid>& read,
                std::mutex& synchro);
void pause_thread(int& time_leap);
void fillStatsVector(
    const std::vector<Boid>& flock, std::vector<Stats>& vec,
    const std::chrono::time_point<std::chrono::steady_clock>& start_time);
void update_Stats(const std::vector<Boid>& flock,
                  std::vector<Stats>& timestamped_stats, int& elapsed,
                  sf::RenderWindow& window, std::mutex& synchro);
void printStats(const std::vector<Stats>& vec);
int askTxt();
void exportStats(const std::vector<Stats>& vec);
void exportPlot(const std::vector<Stats>& vec);
int askPng();
void plotStats(const std::vector<Stats>& stats, int conditional,
               const std::string& name);
#endif
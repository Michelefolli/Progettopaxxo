#ifndef BOIDS_VER_2_HPP
#define BOIDS_VER_2_HPP
#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>

struct Vec_2d {
  Vec_2d(float x_val, float y_val) : x(x_val), y(y_val) {}
  float x;
  float y;
  Vec_2d operator+(const Vec_2d& v) const { return Vec_2d(x + v.x, y + v.y); }
  Vec_2d operator-(const Vec_2d& v) const { return Vec_2d(x - v.x, y - v.y); }
  Vec_2d operator*(const float c) const { return Vec_2d(x * c, y * c); }
  Vec_2d operator+=(const Vec_2d& v) {
    x += v.x;
    y += v.y;
    return *this;
  }
  float norm() const { return std::sqrt(x * x + y * y); }
};  // vettore a due dimensioni, utilissimo

struct Params {
  float sep{};       // fattore di separazione
  float alig{};      // fattore di allineamento
  float cohes{};     // fattore di coesione
  float dist{};      // raggio visivo dei boids
  float dist_sep{};  // distanza minima
};

class Boid {
 private:
  Vec_2d position;
  Vec_2d velocity;
  void limit(float max_speed);  // limite di velocità
  float abs_distance_from(const Boid& boid_j);
  Vec_2d separation(const std::vector<Boid>& stormo, const float& sep,
                    const float& dist_sep);
  Vec_2d alignment_and_cohesion(const std::vector<Boid>& stormo,
                                const float& alig, const float& cohes,
                                const float& dist);

 public:
  Boid(Vec_2d position_val, Vec_2d velocity_val)
      : position(position_val), velocity(velocity_val) {}
  Vec_2d getPosition();
  Vec_2d getVelocity();
  void update(Params params, const std::vector<Boid>& stormo,
              const float& max_speed);
  void draw_on(sf::RenderWindow& window);
};

struct Stats {
  Vec_2d v_media{0., 0.};  // velocità media
  Vec_2d d_media{0., 0.};  // distanza media
  float sigma_v{};         // deviazione stardard velocità
  float sigma_d{};         // deviazione standard distanza

  Stats operator+=(const Stats& s) {
    v_media += s.v_media;
    d_media += s.d_media;
    sigma_v += s.sigma_v;
    sigma_d += s.sigma_d;
    return *this;
  }
};  // struttura delle statistiche

/*class Sim {
 private:
  //"_" alla fine è perché il giacomins vuole
  // l'underscore nelle variabili private tipo
  float s_{};    // fattore di separazione
  float a_{};    // fattore di allineamento
  float c_{};    // fattore di coesione
  float d_{};    // raggio visivo dei boids
  float d_s_{};  // distanza minima
                  // i parametri n e T vanno poi nel main
 public:
  std::vector<Boid> stormo_;   // insieme dei boid;
  void add(const Boid& boid);  // per aggiungere un boid al vettore stormo_.
  // distanza tra boids
  void separation();
  void alignment_and_cohesion();
  void travel();       // aggiornamento delle posizioni dei boid
  Stats statistics();  // dichiarazione della funzione statistics
  float abs_distance(const Boid& boid_i, const Boid& boid_j);
  void GetParams(
      float s1, float a1, float c1, float d1,
      float ds1);  // DIchiarazione funzione che gestisce i parametri
};*/

#endif
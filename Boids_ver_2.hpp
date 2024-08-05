#ifndef BOIDS_VER_2_HPP
#define BOIDS_VER_2_HPP
#include <cmath>
#include <vector>
struct Vec_2d {
  Vec_2d(double x_val, double y_val)
      : x(x_val),
        y(y_val) {}
  double x;
  double y;
  Vec_2d operator+(const Vec_2d& v) const { return Vec_2d(x + v.x, y + v.y); }
  Vec_2d operator-(const Vec_2d& v) const { return Vec_2d(x - v.x, y - v.y); }
  Vec_2d operator*(const double c) const { return Vec_2d(x * c, y * c); }
  Vec_2d operator+=(const Vec_2d& v) {
    x += v.x;
    y += v.y;
    return *this;
  }
  double norm() const { return std::sqrt(x * x + y * y); }
};  // vettore a due dimensioni, utilissimo

struct Boid {
  Boid(Vec_2d position_val, Vec_2d velocity_val)
      : position(position_val), velocity(velocity_val) {}
  Vec_2d position;
  Vec_2d velocity;
};

struct Stats {
  Vec_2d v_media{0., 0.};  // velocità media
  Vec_2d d_media{0., 0.};  // distanza media
  double sigma_v{};        // deviazione stardard velocità
  double sigma_d{};        // deviazione standard distanza

  Stats operator+=(const Stats& s) {
    v_media += s.v_media;
    d_media += s.d_media;
    sigma_v += s.sigma_v;
    sigma_d += s.sigma_d;
    return *this;
  }
};  // struttura delle statistiche

class Sim {
 private:
  //"_" alla fine è perché il giacomins vuole
  // l'underscore nelle variabili private tipo
  double s_{};    // fattore di separazione
  double a_{};    // fattore di allineamento
  double c_{};    // fattore di coesione
  double d_{};    // raggio visivo dei boids
  double d_s_{};  // distanza minima
                  // i parametri n e T vanno poi nel main
 public:
  std::vector<Boid> stormo_;   // insieme dei boid;
  void add(const Boid& boid);  // per aggiungere un boid al vettore stormo_.
  // distanza tra boids
  void separation();
  void alignment_and_cohesion();
  void travel();       // aggiornamento delle posizioni dei boid
  Stats statistics();  // dichiarazione della funzione statistics
  double abs_distance(const Boid& boid_i,
                      const Boid& boid_j);
  void GetParams(
      double s1, double a1, double c1, double d1,
      double ds1);  // DIchiarazione funzione che gestisce i parametri
};
  
#endif
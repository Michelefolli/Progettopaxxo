#ifndef MBOIDS_HPP
#define MBOIDS_HPP
#include <vector>
struct Boid {
  double x;
  double y;
  double v_x;
  double v_y;
};  // struttura del singolo boid

struct Stats {
  double v_media{};  // velocità media
  double d_media{};  // distanza media
  double sigma_v{};  // deviazione stardard velocità
  double sigma_d{};  // deviazione standard distanza
};  // struttura delle statistiche

double norm_v(Boid const& B);  // modulo della velocità di un Boid
double abs_distance(Boid const& A, Boid const& B);  // distanza tra due boids

class Sim {
 private:
  std::vector<Boid>
      stormo_{};  // insieme dei boid; "_" alla fine è perché il giacomins vuole
                  // l'underscore nelle variabili private tipo
  double s_{};    // fattore di separazione
  double a_{};    // fattore di allineamento
  double c_{};    // fattore di coesione
  double d_{};    // raggio visivo dei boids
  double d_s_{};  // distanza minima
                  // i parametri n e T vanno poi nel main
 public:
  void add(Boid B);  // per aggiungere un boid al vettore. Non c'è dentro
                     // generazione casuale

  Stats statistics();  // dichiarazione della funzione statistics
};
#endif
#include "Boids_ver_2.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

void Sim::add(const Boid& boid) {
  stormo_.push_back(boid);
}  // per aggiungere boid allo stormo

void Sim::GetParams(double s, double a, double c, double d, double ds) {
  s_ = s;
  a_ = a;
  c_ = c;
  d_ = d;
  d_s_ = ds;
}

double Sim::abs_distance(const Boid& boid_i, const Boid& boid_j) {
  return (boid_i.position + boid_j.position).norm();
};  // modulo della distanza tra due boid

void Sim::separation() {
  for (Boid& boid : stormo_) {
    std::vector<Boid> subvector;
    std::copy_if(
        stormo_.begin(), stormo_.end(), std::back_inserter(subvector),
        [&boid, this](const Boid& other_boid) {
          return abs_distance(boid, other_boid) < d_s_;
        });  // ora il vettore subvector contiene tutti i boid a distanza d_s_
    Vec_2d v_sep =
        std::accumulate(subvector.begin(), subvector.end(), Vec_2d(0., 0.),
                        [&boid](Vec_2d sum, const Boid& other_boid) {
                          return sum += boid.position - other_boid.position;
                        }) *
        (-s_);
    boid.velocity += v_sep;
  };
}  // calcolo della velocità di separazione

void Sim::alignment_and_cohesion() {
  double N = stormo_.size();  // forse da cambiare, perché sono due tipi diversi

  for (Boid& boid : stormo_) {
    std::vector<Boid> subvector;
    std::copy_if(
        stormo_.begin(), stormo_.end(), std::back_inserter(subvector),
        [&boid, this](const Boid& other_boid) {
          return abs_distance(boid, other_boid) < d_ &&
                 abs_distance(boid, other_boid) !=
                     0;  // non è corretto formalmente ma dovrebbe funzionare.
                         // Sarebbe per scartare il boid che è preso in
                         // considerazione nel ciclo for, ma potrebbe scartare
                         // dei boid che occupano la stessa posizione, anche se
                         // questo non dovrebbe succedere.
        });  // ora il vettore subvector contiene tutti i boid a distanza d_s_
    Vec_2d v_alignment = std::accumulate(
        subvector.begin(), subvector.end(), Vec_2d(0., 0.),
        [&boid, this, &N](Vec_2d sum, const Boid& other_boid) {
          return sum += (boid.velocity - other_boid.velocity) * (a_ / (N - 1));
        });  // calcolo della velocità di allineamento
    Vec_2d center_of_mass =
        std::accumulate(subvector.begin(), subvector.end(), Vec_2d(0., 0.),
                        [&N](Vec_2d sum, const Boid& boid_j) {
                          return sum += boid_j.position * (1 / (N - 1));
                        });  // calcolo del centro di massa del subvector
    Vec_2d v_cohesion = (center_of_mass - boid.position) * c_;
    boid.velocity += v_alignment + v_cohesion;
  }
}

void Sim::travel() {
  for (Boid& boid : stormo_) {
    boid.position += boid.velocity;
  };
}  // aggiornamento della posizione dei boid nello stormo

Stats Sim::statistics() {
  double N = stormo_.size();  // poi sarà il parametro N

  Vec_2d v_media = std::accumulate(
      stormo_.begin(), stormo_.end(), Vec_2d(0., 0.),
      [&N](Vec_2d res, const Boid& boid) {
        return res += boid.velocity * (1 / N);
      });  // calcolo della velocità media a parte perché serve come valore
  Vec_2d d_media = std::accumulate(
      stormo_.begin(), stormo_.end(), Vec_2d(0., 0.),
      [&N](Vec_2d res, const Boid& boid) {
        return res += boid.position * (1 / N);
      });  // calcolo della distanza media a parte perché serve come valore

  Stats stats = std::accumulate(
      stormo_.begin(), stormo_.end(), Stats(),
      [&N, &v_media, &d_media](Stats res, const Boid& boid) {
        return res +=
               {v_media, d_media, (boid.velocity.norm() - v_media.norm()) / N,
                (boid.position.norm() - d_media.norm()) / N};
      });  // forse si può fare tutto in un unico accumulate ma non so fares
  stats.sigma_d = std::sqrt(stats.sigma_d);
  stats.sigma_v = std::sqrt(stats.sigma_v);
  return stats;
}
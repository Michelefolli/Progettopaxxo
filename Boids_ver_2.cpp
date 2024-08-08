#include "Boids_ver_2.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

const int width = 1920;
const int height = 1080;
void repulsive_border(Boid& boid) {
  if (boid.position.x < 80) {  // && boid.velocity.x <= 0) {
    boid.velocity.x +=
        (boid.velocity.norm()) * (1) *
        (1 / std::pow(boid.position.x,
                      0.3));  // il fattore additivo è per evitare
                              // che i boid si fermino in teoria
  }
  if (boid.position.y < 80) {  //&& boid.velocity.y <= 0) {
    boid.velocity.y +=
        (boid.velocity.norm()) * (1) * (1 / std::pow(boid.position.y, 0.3));
  }
  if (boid.position.x > (width - 80)) {  //&& boid.velocity.x >= 0) {
    boid.velocity.x += (boid.velocity.norm()) * (-1) *
                       (1 / std::pow((width - boid.position.x), 0.3));
  }
  if (boid.position.y > (height - 80)) {  //&& boid.velocity.y >= 0) {
    boid.velocity.y += (boid.velocity.norm()) * (-1) *
                       (1 / std::pow((height - boid.position.y), 0.3));
  }
}

void Boid::limit(float max_speed) {
  if (this->velocity.norm() > max_speed) {
    float norm = this->velocity.norm();

    this->velocity = {this->velocity.x * max_speed * (1 / norm),
                      this->velocity.y * max_speed * (1 / norm)};
  }
}  // questo limita la velocità, il trucchetto algebrico per mantenere la
   // direzione è dividere per la norma del vettore e poi moltiplicarlo per la
   // velocità massima

/*void Sim::add(const Boid& boid) {
  stormo_.push_back(boid);
}  // per aggiungere boid allo stormo

void Sim::GetParams(float s, float a, float c, float d, float ds) {
  s_ = s;
  a_ = a;
  c_ = c;
  d_ = d;
  d_s_ = ds;
}*/

float Boid::abs_distance_from(const Boid& boid_j) {
  return (this->position - boid_j.position).norm();
};  // modulo della distanza tra due boid

Vec_2d Boid::separation(const std::vector<Boid>& stormo, const float& sep,
                        const float& dist_sep) {
  std::vector<Boid> subvector;
  std::copy_if(
      stormo.begin(), stormo.end(), std::back_inserter(subvector),
      [&dist_sep, this](const Boid& other_boid) {
        return abs_distance_from(other_boid) < dist_sep;
      });  // ora il vettore subvector contiene tutti i boid a distanza d_s_
  Vec_2d v_sep =
      std::accumulate(subvector.begin(), subvector.end(), Vec_2d(0., 0.),
                      [this](Vec_2d sum, const Boid& other_boid) {
                        return sum += (other_boid.position - this->position);
                      }) *
      (-sep);
  return v_sep;
};  // calcolo della velocità di separazione. Questa è testata e funziona

Vec_2d Boid::alignment_and_cohesion(const std::vector<Boid>& stormo,
                                    const float& alig, const float& cohes,
                                    const float& dist) {
  std::vector<Boid> subvector;
  std::copy_if(
      stormo.begin(), stormo.end(), std::back_inserter(subvector),
      [this, &dist](const Boid& other_boid) {
        return abs_distance_from(other_boid) < dist;
      });  // ora il vettore subvector contiene tutti i boid a distanza d_s_
  float n = static_cast<float>(subvector.size());
  if (n > 1) {  // evita la divisione per 0
    Vec_2d subboids_velocity_sum =
        std::accumulate(subvector.begin(), subvector.end(), Vec_2d(0., 0.),
                        [](Vec_2d sum, const Boid& other_boid) {
                          return sum += other_boid.velocity;
                        }) -
        this->velocity;
    Vec_2d v_alignment =
        (subboids_velocity_sum * (1 / (n - 1)) - this->velocity) * alig;
    // calcolo della velocità di allineamento
    Vec_2d center_of_mass =
        std::accumulate(subvector.begin(), subvector.end(), Vec_2d(0., 0.),
                        [&n](Vec_2d sum, const Boid& boid_j) {
                          return sum += (boid_j.position * (1 / (n - 1)));
                        }) -
        this->position *
            (1 / (n - 1));  // calcolo del centro di massa del subvector. Il
                            // subvector contiene anche il boid di riferimento,
                            // quindi dopo la sommatoria glielo togliamo
    Vec_2d v_cohesion = (center_of_mass - this->position) * cohes;
    return (v_alignment + v_cohesion);
  } else
    return {0, 0};
};

void Boid::update(Params params, const std::vector<Boid>& stormo,
                  const float& max_speed) {
  this->velocity +=
      alignment_and_cohesion(stormo, params.alig, params.cohes, params.dist) +
      separation(stormo, params.sep, params.dist_sep);
  repulsive_border(*this);
  this->limit(max_speed);
  this->position += this->velocity;
}

/*void Sim::travel() {
  for (Boid& boid : stormo_) {
    boid.position += boid.velocity;
  };
}  // aggiornamento della posizione dei boid nello stormo

Stats Sim::statistics() {
  float N = stormo_.size();  // poi sarà il parametro N

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
               {{0., 0.},
                {0., 0.},
                std::pow((boid.velocity.norm() - v_media.norm()), 2) / N,
                std::pow((boid.position.norm() - d_media.norm()), 2) / N};
      });  // forse si può fare tutto in un unico accumulate ma non so fares
  stats.sigma_v = std::sqrt(stats.sigma_v);
  stats.sigma_d = std::sqrt(stats.sigma_d);
  stats.v_media = v_media;
  stats.d_media = d_media;
  return stats;
*/

Stats statistics(const std::vector<Boid>& stormo) {
  float N = stormo.size();

  Boid med_vals =
      std::accumulate(stormo.begin(), stormo.end(), Boid({0., 0.}, {0., 0.}),
                      [&N](Boid res, const Boid& boid) {
                        res.position += boid.position * (1 / N);
                        res.velocity += boid.velocity * (1 / N);
                        return res;
                      });

  Stats stats = std::accumulate(
      stormo.begin(), stormo.end(), Stats(),
      [&N, &med_vals](Stats res, const Boid& boid) {
        return res +=
               {{0., 0.},
                {0., 0.},
                static_cast<float>(std::pow((boid.velocity.norm() - med_vals.velocity.norm()), 2)) /
                    N,
                static_cast<float>(std::pow((boid.position.norm() - med_vals.position.norm()), 2)) /
                    N};
      });  // abbiamo dovuto convertire in float perché nella dichiarazione di Stats le sigma sono float
      // forse si può fare tutto in un unico accumulate ma non so fares

  stats.sigma_v = std::sqrt(stats.sigma_v);
  stats.sigma_d = std::sqrt(stats.sigma_d);
  stats.v_media = med_vals.velocity;
  stats.d_media = med_vals.position;
  return stats;
}
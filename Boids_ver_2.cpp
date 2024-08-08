#include "Boids_ver_2.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

const int width = 1920;
const int height = 1080;
void repulsive_border(Boid& boid) {
  auto position = boid.getPosition();
  auto velocity = boid.getVelocity();

  if (position.x < 80) {
    velocity.x += (velocity.norm()) * (1) * (1 / std::pow(position.x, 0.3));
  }
  if (position.y < 80) {
    velocity.y += (velocity.norm()) * (1) * (1 / std::pow(position.y, 0.3));
  }
  if (position.x > (width - 80)) {
    velocity.x +=
        (velocity.norm()) * (-1) * (1 / std::pow((width - position.x), 0.3));
  }
  if (position.y > (height - 80)) {
    velocity.y +=
        (velocity.norm()) * (-1) * (1 / std::pow((height - position.y), 0.3));
  }
}

void Boid::limit(float max_speed) {
  if (velocity.norm() > max_speed) {
    float norm = velocity.norm();

    this->velocity = {velocity.x * max_speed * (1 / norm),
                      velocity.y * max_speed * (1 / norm)};
  }
}  // questo limita la velocità, il trucchetto algebrico per mantenere la
   // direzione è dividere per la norma del vettore e poi moltiplicarlo per la
   // velocità massima

float Boid::abs_distance_from(const Boid& boid_j) {
  return (position - boid_j.position).norm();
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
                        return sum += (other_boid.position - position);
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
        velocity;
    Vec_2d v_alignment =
        (subboids_velocity_sum * (1 / (n - 1)) - velocity) * alig;
    // calcolo della velocità di allineamento
    Vec_2d center_of_mass =
        std::accumulate(subvector.begin(), subvector.end(), Vec_2d(0., 0.),
                        [&n](Vec_2d sum, const Boid& boid_j) {
                          return sum += (boid_j.position * (1 / (n - 1)));
                        }) -
        position *
            (1 / (n - 1));  // calcolo del centro di massa del subvector. Il
                            // subvector contiene anche il boid di riferimento,
                            // quindi dopo la sommatoria glielo togliamo
    Vec_2d v_cohesion = (center_of_mass - position) * cohes;
    return (v_alignment + v_cohesion);
  } else
    return {0, 0};
};

const Vec_2d& Boid::getPosition() const { return position; }
const Vec_2d& Boid::getVelocity() const { return velocity; }
void Boid::setPosition(const Vec_2d& position_values) {
  position = position_values;
}
void Boid::setVelocity(const Vec_2d& velocity_values) {
  velocity = velocity_values;
}
Boid Boid::operator+=(const Boid& boid) {
  position += boid.getPosition();
  velocity += boid.getVelocity();
  return *this;
}

void Boid::update(Params params, const std::vector<Boid>& stormo,
                  const float& max_speed) {
  velocity +=
      alignment_and_cohesion(stormo, params.alig, params.cohes, params.dist) +
      separation(stormo, params.sep, params.dist_sep);
  repulsive_border(*this);
  limit(max_speed);
  position += velocity;
}

void Boid::draw_on(sf::RenderWindow& window) {
  sf::CircleShape shape(4, 3);  // definisce la forma da disegnare
  shape.setPosition((position.x), (position.y));
  shape.setFillColor(sf::Color::White);
  float angle = (std::atan2(velocity.y, velocity.x) * 180 / (M_PI));
  shape.setRotation(angle);
  window.draw(shape);
}  // disegna il boid come un triangolo

Stats statistics(const std::vector<Boid>& stormo) {
  float N = stormo.size();

  Boid sum_vals =
      std::accumulate(stormo.begin(), stormo.end(), Boid({0., 0.}, {0., 0.}),
                      [&N](Boid res, const Boid& boid) {
                        res += boid;
                        return res;
                      });
  Boid med_vals(sum_vals.getPosition() * (1.0f / N),
                sum_vals.getVelocity() * (1.0f / N));

  Stats stats = std::accumulate(stormo.begin(), stormo.end(), Stats(),
                                [&N, &med_vals](Stats res, const Boid& boid) {
                                  float delta_v = boid.getVelocity().norm() -
                                                  med_vals.getVelocity().norm();
                                  float delta_d = boid.getPosition().norm() -
                                                  med_vals.getPosition().norm();
                                  res.sigma_v += delta_v * delta_v / N;
                                  res.sigma_d += delta_d * delta_d / N;
                                  return res;
                                });
  stats.sigma_v = std::sqrt(stats.sigma_v);
  stats.sigma_d = std::sqrt(stats.sigma_d);
  stats.v_media = med_vals.getVelocity();
  stats.d_media = med_vals.getPosition();
  return stats;
}
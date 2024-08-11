#include "Boids.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

// non so dove vanno definite le variabili
const float Pi = 3.14159265358979323846264f;

// funzione che fa in modo che i boid evitino i bordi della finestra sfml
void Boid::avoid_edges(const int width, const int height) {
  if (position.x < 80) {
    velocity.x += (velocity.norm()) * (1) * (1 / std::pow(position.x, 0.25f));
  }
  if (position.y < 80) {
    velocity.y += (velocity.norm()) * (1) * (1 / std::pow(position.y, 0.25f));
  }
  if (position.x > (width - 80)) {
    velocity.x +=
        (velocity.norm()) * (-1) * (1 / std::pow((width - position.x), 0.25f));
  }
  if (position.y > (height - 80)) {
    velocity.y +=
        (velocity.norm()) * (-1) * (1 / std::pow((height - position.y), 0.25f));
  };

}  // la potenza deve essere di ordine pari in modo da recuperare i boid che per
   // sbagli0 finiscono fuori schermo

// funzione che limita la velocità:
void Boid::limit(const float& max_speed) {
  float norm = velocity.norm();
  if (norm > max_speed) {
    velocity = velocity * (max_speed / norm);
  }
}

float Boid::abs_distance_from(const Boid& boid_j) const {
  return (position - boid_j.position).norm();
};  // modulo della distanza tra due boid

const Vec_2d Boid::separation(const std::vector<Boid>& flock, const float& sep,
                              const float& dist_sep) const {
  Vec_2d v_sep =
      std::accumulate(flock.begin(), flock.end(), Vec_2d(0., 0.),
                      [this, &dist_sep](Vec_2d sum, const Boid& other_boid) {
                        if (abs_distance_from(other_boid) <= dist_sep) {
                          Vec_2d diff = other_boid.getPosition() - position;
                          sum += diff;
                        }
                        return sum;
                      }) *
      (-sep);
  return v_sep;
};  // calcolo della velocità di separazione. Questa è testata e funziona

const Vec_2d Boid::alignment_and_cohesion(const std::vector<Boid>& flock,
                                          const float& alig, const float& cohes,
                                          const float& dist) const {
  int count = 0;

  std::pair<Vec_2d, Vec_2d> sums =
      std::accumulate(  // sommatoria per posizioni e velocità
          flock.begin(), flock.end(),
          std::make_pair(Vec_2d(0.f, 0.f), Vec_2d(0.f, 0.f)),
          [this, dist, &count](std::pair<Vec_2d, Vec_2d> acc,
                               const Boid& otherBoid) {
            if (this != &otherBoid &&
                this->abs_distance_from(otherBoid) <= dist) {
              acc.first += otherBoid.velocity;   // somma delle velocità
              acc.second += otherBoid.position;  // Somma delle posizioni
              ++count;
            }
            return acc;
          });

  if (count > 0) {
    Vec_2d avg_velocity = sums.first / static_cast<float>(count);
    Vec_2d center_of_mass = sums.second / static_cast<float>(count);

    Vec_2d v_alig = (avg_velocity - velocity) * alig;
    Vec_2d v_cohes = (center_of_mass - position) * cohes;

    return v_alig + v_cohes;
  } else
    return Vec_2d(0, 0);
};

// getters and setters:
const Vec_2d& Boid::getPosition() const { return position; }
const Vec_2d& Boid::getVelocity() const { return velocity; }
void Boid::setPosition(const Vec_2d& pos) { position = pos; }
void Boid::setVelocity(const Vec_2d& vel) { velocity = vel; }

void Boid::update(const Params& params, const std::vector<Boid>& flock,
                  const float& max_speed, const sf::RenderWindow& window) {
  velocity +=
      alignment_and_cohesion(flock, params.alig, params.cohes, params.dist) +
      separation(flock, params.sep, params.dist_sep);
  const int width = window.getSize().x;
  const int height = window.getSize().y;

  avoid_edges(width, height);
  limit(max_speed);
  position += velocity;
}  // questa funzione aggiorna la velocità del boid e poi lo sposta

void Boid::draw_on(sf::RenderWindow& window) const {
  sf::CircleShape shape(4, 3);  // definisce la forma da disegnare
  shape.setPosition((position.x), (position.y));
  shape.setFillColor(sf::Color::White);
  float angle = (std::atan2(velocity.y, velocity.x) * 180 / (Pi));
  shape.setRotation(angle);
  window.draw(shape);
}  // disegna il boid come un triangolo orientato nella direzione di volo

Stats statistics(std::vector<Boid>& flock) {
  float n = static_cast<float>(flock.size());  // dimensioni dello stormo

  Vec_2d v_mean = std::accumulate(flock.begin(), flock.end(), Vec_2d(0., 0.),
                                  [&n](Vec_2d sum, const Boid& boid) {
                                    return sum += (boid.getVelocity() / n);
                                  });  // calcolo della velocità media

  Vec_2d d_mean = std::accumulate(
      flock.begin(), flock.end(), Vec_2d(0., 0.),
      [&flock, &n](Vec_2d sum, const Boid& boid_i) {
        float d_mean_i = std::accumulate(
            flock.begin(), flock.end(), 0.f,
            [&boid_i, &n](float sum_d, const Boid& boid_j) {
              return sum_d += (boid_i.abs_distance_from(boid_j) / (n - 1));
            });
        return sum += Vec_2d(d_mean_i, d_mean_i) / n;
      });  // calcolo della distanza media tra due boid, utilizza un nested
           // algorythm non so se è una cosa positiva.

  Stats stats{};  // chat gpt dice che è per evitare l'inizializzazione di
                  // varabili inutili dentro gli accumulate
  stats.v_mean = v_mean.norm();
  stats.d_mean = d_mean.norm();
  stats.sigma_v = std::accumulate(
      flock.begin(), flock.end(), 0.f,
      [&v_mean, &n](float sum, const Boid& boid) {
        return sum +=
               std::pow(boid.getVelocity().norm() - v_mean.norm(), 2.f) / n;
      });

  stats.sigma_d = std::accumulate(
      flock.begin(), flock.end(), 0.f,
      [&flock, &d_mean, &n](float sum, const Boid& boid_i) {
        float d_sigma_i = std::accumulate(
            flock.begin(), flock.end(), 0.f,
            [&d_mean, &boid_i, &n](float sum_d, const Boid& boid_j) {
              return sum_d +=
                     std::pow(boid_i.abs_distance_from(boid_j) - d_mean.norm(),
                              2.f) /
                     (n - 1);
            });
        return sum += d_sigma_i / n;
      });
  return stats;
}

void simulation(sf::RenderWindow& window, std::vector<Boid>& flock,
                Params& params, const float& max_speed, sf::Clock& clock,
                float& timePassed, std::vector<Stats>& Statistics_vector) {
  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }  // permette di chiudere la finestra cliccado sulla x
    window.clear();  // pulisce la finestra ogni frame
    for (auto& boid : flock) {
      boid.update(params, flock, max_speed,
                  window);  // update

      boid.draw_on(window);

    }  // disegna tutti i boid, ma non li fa vedere ancora, quello è display
    window.display();
    if (clock.getElapsedTime().asSeconds() >= 1.0) {
      Stats stat_i = statistics(flock); // calcola le statistiche
      stat_i.time = timePassed; //assegna il tempo
      ++timePassed;
      Statistics_vector.push_back(stat_i); 
      clock.restart();
    }
  }
}

void printStats(const std::vector<Stats>& vec) {
  // Use std::for_each to iterate over each element in the vector
  std::for_each(vec.begin(), vec.end(), [](const Stats& stat) {
    std::cout << stat.d_mean << "  " << stat.sigma_d << "  " << stat.v_mean
              << "  " << stat.sigma_v << "  " << stat.time << "\n";
  });

}  // placeholder per la funzione responsabile per la lettura del vettore
   // contenente statistiche e timestamp,
// il disegno dei grafici e la creazione del file di testo con le statistiche

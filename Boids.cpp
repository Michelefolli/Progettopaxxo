#include "Boids.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

// non so dove vanno definite le variabili
const float Pi = 3.14159265358979323846264f;
const int width = 1920;
const int height = 1080;

// funzione che fa in modo che i boid evitino i bordi della finestra sfml
void repulsive_border(Boid& boid) {
  auto position = boid.getPosition();
  auto velocity = boid.getVelocity();

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
  boid.setVelocity(velocity);
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
  std::vector<Boid> subvector;
  std::copy_if(
      flock.begin(), flock.end(), std::back_inserter(subvector),
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

const Vec_2d Boid::alignment_and_cohesion(const std::vector<Boid>& flock,
                                          const float& alig, const float& cohes,
                                          const float& dist) const {
  std::vector<Boid> subvector;
  std::copy_if(
      flock.begin(), flock.end(), std::back_inserter(subvector),
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
    Vec_2d v_alignment = (subboids_velocity_sum / (n - 1) - velocity) * alig;
    // calcolo della velocità di allineamento
    Vec_2d center_of_mass =
        std::accumulate(subvector.begin(), subvector.end(), Vec_2d(0., 0.),
                        [&n](Vec_2d sum, const Boid& boid_j) {
                          return sum += (boid_j.position / (n - 1));
                        }) -
        position / (n - 1);  // calcolo del centro di massa del subvector. Il
                             // subvector contiene anche il boid di riferimento,
                             // quindi dopo la sommatoria glielo togliamo
    Vec_2d v_cohesion = (center_of_mass - position) * cohes;
    return (v_alignment + v_cohesion);
  } else
    return {0, 0};
};

// getters and setters:
const Vec_2d& Boid::getPosition() const { return position; }
const Vec_2d& Boid::getVelocity() const { return velocity; }
void Boid::setPosition(const Vec_2d& pos) { position = pos; }
void Boid::setVelocity(const Vec_2d& vel) { velocity = vel; }

void Boid::update(const Params& params, const std::vector<Boid>& flock,
                  const float& max_speed) {
  velocity +=
      alignment_and_cohesion(flock, params.alig, params.cohes, params.dist) +
      separation(flock, params.sep, params.dist_sep);
  repulsive_border(*this);
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

void simulation(sf::RenderWindow& window, std::vector<Boid>& flock,
                Params& params, const float& max_speed) {
  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }  // permette di chiudere la finestra cliccado sulla x
    window.clear();  // pulisce la finestra ogni frame
    for (auto& boid : flock) {
      boid.update(params, flock,
                  max_speed);  // funzione che limita la velocità

      boid.draw_on(window);

    }  // disegna tutti i boid, ma non li fa vedere ancora, quello è display
    window.display();
  }
}

Stats statistics(
    const std::vector<Boid>& flock,
    std::chrono::time_point<std::chrono::steady_clock> start_time) {
  Stats stats{};  // chat gpt dice che è per evitare l'inizializzazione di
                  // varabili inutili dentro gli accumulate
  std::chrono::time_point current_time =
      std::chrono::steady_clock::now();  // prendo il tempo attuale
  auto time_span = std::chrono::duration_cast<std::chrono::duration<float>>(
      current_time - start_time);  // calcolo il tempo trascorso tra l'inzio del
                                   // ciclo (in Update_stats)
  stats.time = time_span.count();  // assegno il valore del tempo di calcolo
                                   // all'elemento stats

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
}  // funzione che restituisce le statistiches

void pause_thread(int& time_leap) {  // funzione che fa dormire fino
                                     // all'acquisizione successiva
  std::this_thread::sleep_for(std::chrono::milliseconds(time_leap));
}

void fillStatsVector(const std::vector<Boid>& flock, std::vector<Stats>& vec,
                     int& n,
                     std::chrono::time_point<std::chrono::steady_clock>&
                         start_time) {  // funzione che
                                        // riempie il vettore
  for (int i = 0; i < n; ++i) {         // delle statistiche
    vec.push_back(statistics(flock, start_time));
  }
}

void update_Stats(
    const std::vector<Boid>& flock, std::vector<Stats>& timestamped_stats,
    int& n, int& elapsed,
    sf::RenderWindow& window) {  // funzione che riassume il processo
                                 // di acquisizione periodica delle statistiche
  auto start_time = std::chrono::steady_clock::now();

  while (window.isOpen()) {  // questo if era per controllare che compilasse,
                             // diventerà un while(window.isOpen)

    fillStatsVector(flock, timestamped_stats, n, start_time);
    pause_thread(elapsed);
  }
}

void printStats(const std::vector<Stats>& vec) {
  int vec_size = vec.size();
  for (int i{}; i < vec_size; i++) {
    std::cout << vec[i].d_mean << "  " << vec[i].sigma_d << "  "
              << vec[i].v_mean << vec[i].sigma_d << '\n' ;
  };

}  // placeholder per la funzione responsabile per la lettura del vettore
   // contenente statistiche e timestamp,
// il disegno dei grafici e la creazione del file di testo con le statistiche

int getUserInput() {
  int conditional{};
  while (true) {
    std::cout << "Input 1 if you want to export the stats as a .txt file, "
                 "otherwise input 0: ";
    std::cin >> conditional;

    if (conditional == 0 || conditional == 1) {
      break;
    } else {
      std::cout << "Invalid input. Please enter 0 or 1.\n";
    }
  }
  return conditional;
}

void exportStatsIfNeeded(int conditional,
                         const std::vector<Stats>& timestamped_stats) {
  if (conditional == 1) {
    printStats(timestamped_stats);
    std::cout << "The stats were successfully exported.\n";
  } else {
    std::cout << "The stats were not exported.\n";
  }
}
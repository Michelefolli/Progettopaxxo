#include "Boids.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>
// non so dove vanno definite le variabili
const float Pi = 3.14159265358979323846264f;

// funzione che fa in modo che i boid evitino i bordi della finestra sfml
void Boid::avoid_edges(const int width, const int height) {
  float repulsive_range = 100.f;
  auto epsilon = std::numeric_limits<
      float>::epsilon();  // division and moltiplication by zero can lead to NaN
                          // errors hence the need for small epsilons

  if (position.x < repulsive_range) {
    velocity.x += (velocity.norm() + epsilon) * (1) *
                  (1 / (std::pow((position.x), 0.25f) + epsilon));
  }
  if (position.y < repulsive_range) {
    velocity.y += (velocity.norm() + epsilon) * (1) *
                  (1 / (std::pow((position.y), 0.25f) + epsilon));
  }
  if (position.x > (width - repulsive_range)) {
    velocity.x += (velocity.norm() + epsilon) * (-1) *
                  (1 / (std::pow((width - position.x), 0.25f) + epsilon));
  }
  if (position.y > (height - repulsive_range)) {
    velocity.y += (velocity.norm() + epsilon) * (-1) *
                  (1 / (std::pow((height - position.y), 0.25f) + epsilon));
  };

  if (std::isnan(velocity.x) || std::isnan(velocity.y)) {
    std::cout << "Velocity is nan \n";
  };
  if (std::isnan(position.x) || std::isnan(position.y)) {
    std::cout << "Position is NaN \n ";
  };

  if (std::isfinite(velocity.x) || std::isfinite(velocity.y)) {
    ;
  } else {
    std::cout << "velocity is infinite \n";
  };
  if (std::isfinite(position.x) || std::isfinite(position.y)) {
    ;
  } else {
    std::cout << "position is infinite \n";
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
                  const float& max_speed, const int width, const int height) {
  velocity +=
      alignment_and_cohesion(flock, params.alig, params.cohes, params.dist) +
      separation(flock, params.sep, params.dist_sep);
  // const int width = window.getSize().x;
  // const int height = window.getSize().y;

  avoid_edges(width, height);
  limit(max_speed);
  position += velocity;
}  // questa funzione aggiorna la velocità del boid e poi lo sposta

void Boid::draw_on(sf::RenderWindow& window) const {
  sf::CircleShape shape(5, 3);  // definisce la forma da disegnare
  shape.setPosition((position.x), (position.y));
  shape.setFillColor(sf::Color::White);
  float angle = (std::atan2(velocity.y, velocity.x) * 180 / (Pi));
  shape.setRotation(angle);
  window.draw(shape);
}  // disegna il boid come un triangolo orientato nella direzione di volo

void simulation(sf::RenderWindow& window, std::vector<Boid>& flock,
                Params& params, const float& max_speed) {
  const int width = static_cast<int>(sf::VideoMode::getDesktopMode().width);
  const int height = static_cast<int>(sf::VideoMode::getDesktopMode().height);
  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }  // permette di chiudere la finestra cliccado sulla x
    window.clear();  // pulisce la finestra ogni frame

    for (auto& boid : flock) {
      boid.update(params, flock, max_speed, width,
                  height);  // funzione che limita la velocità

      boid.draw_on(window);

    }  // disegna tutti i boid, ma non li fa vedere ancora, quello è display

    window.display();
  }
}

Stats statistics(
    const std::vector<Boid>& flock,
    std::chrono::time_point<std::chrono::steady_clock>& start_time) {
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

  // Vec_2d sum = {0.f , 0.f} ;
  Vec_2d v_mean_val =
      std::accumulate(flock.begin(), flock.end(), Vec_2d(0.f, 0.f),
                      [&n](Vec_2d sum, const Boid& boid) {
                        return sum += (boid.getVelocity() / n);
                      });  // calcolo della velocità media

  // Vec_2d sum_d = {0.f , 0.f} ;
  float d_mean_val = std::accumulate(
      flock.begin(), flock.end(), 0.f,
      [&flock, &n](float sum, const Boid& boid_i) {
        float d_mean_i = std::accumulate(
            flock.begin(), flock.end(), 0.f,
            [&boid_i, &n](float sum_d, const Boid& boid_j) {
              return sum_d += (boid_i.abs_distance_from(boid_j) / (n - 1));
            });
        return sum += (d_mean_i / n);
      });  // calcolo della distanza media tra i boid dello stormo, utilizza un
           // nested algorithm non so se è una cosa positiva.

  stats.v_mean = v_mean_val.norm();
  stats.d_mean = d_mean_val;
  stats.sigma_v = std::accumulate(
      flock.begin(), flock.end(), 0.f,
      [&v_mean_val, &n](float sum, const Boid& boid) {
        return sum +=
               std::pow(boid.getVelocity().norm() - v_mean_val.norm(), 2.f) / n;
      });

  stats.sigma_d = std::accumulate(
      flock.begin(), flock.end(), 0.f,
      [&flock, &d_mean_val, &n](float sum, const Boid& boid_i) {
        float d_sigma_i = std::accumulate(
            flock.begin(), flock.end(), 0.f,
            [&d_mean_val, &boid_i, &n](float sum_d, const Boid& boid_j) {
              return sum_d +=
                     std::pow(boid_i.abs_distance_from(boid_j) - d_mean_val,
                              2.f) /
                     (n - 1);
            });
        return sum += d_sigma_i / n;
      });

  return stats;
}  // funzione che restituisce le statistiches

void fillStatsVector(std::vector<Boid> flock, std::vector<Stats>& vec,
                     std::chrono::time_point<std::chrono::steady_clock>&
                         start_time) {  // funzione che
                                        // riempie il vettore
                                        // delle statistiche

  vec.push_back(statistics(flock, start_time));
}

void update_Stats(
    const std::vector<Boid>& flock, std::vector<Stats>& timestamped_stats,
    int& elapsed,
    sf::RenderWindow& window) {  // funzione che riassume il processo
                                 // di acquisizione periodica delle statistiche
  auto start_time = std::chrono::steady_clock::now();
  auto last_update_time = start_time;
  while (window.isOpen()) {  // questo if era per controllare che compilasse,
                             // diventerà un while(window.isOpen)

    auto current_time = std::chrono::steady_clock::now();
    if (current_time - last_update_time >= std::chrono::milliseconds(elapsed)) {
      fillStatsVector(flock, timestamped_stats, start_time);
      last_update_time = current_time;
    }
  }
}

void printStats(const std::vector<Stats>& vec) {
  std::ostringstream output_str;

  // Use std::for_each to iterate over each element in the vector
  std::for_each(vec.begin(), vec.end(), [&output_str](const Stats& stat) {
    output_str << stat.d_mean << "  " << stat.sigma_d << "  " << stat.v_mean
               << "  " << stat.sigma_v << "  " << stat.time << "\n";
  });

  // Print the accumulated string
  std::cout << output_str.str();
}

void instantiateStatsFile(std::string& file, const std::vector<Stats> vec) {
  std::ofstream file_output(file);
  if (!file_output) {
    std::cout << "There was an error in the creation of the file \n";
  };
  // file_output << "Time\tMean Distance\tDistance std\tMean Speed\tSpeed
  // std\n";

  for (size_t i; i < vec.size(); i++) {
    file_output << vec[i].time << "\t";
    file_output << vec[i].d_mean << "\t";
    file_output << vec[i].sigma_d << "\t";
    file_output << vec[i].v_mean << "\t";
    file_output << vec[i].sigma_v << "\t";
    file_output << '\n';
  }
  file_output.close();
}

std::string namingFile() {  // ha lo scopo di ricevere il nome inserito e
                            // restituirlo alla funzione caller
  std::string file_name{};
  std::cout << "Insert the name of the  file: ";
  std::cin >> file_name;
  return file_name;
}

int askTxt() {  // chiede all'utilizzatore se vuole salvare le statistiche come
                // file .txt
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

void exportStats(const std::vector<Stats>& vec) {  // esporta le statistiche
  int conditional = askTxt();
  if (conditional == 1) {
    std::string name = namingFile() + ".txt";
    instantiateStatsFile(name, vec);
    std::cout << "The stats were successfully exported in " << name << ".\n";
  }
  printStats(vec);
}

int askPng() {  // chiede allo user se vuole esportare i plots come file .png
  int conditional{};
  while (true) {
    std::cout << "Input 1 if you want to export the plots as a .png file, "
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

void exportPlot(const std::vector<Stats>& vec) {
  int option = askPng();
  std::string name{};
  if (option == 1) {
    name = namingFile() + ".png";
  }
  plotStats(vec, option, name);
}

void plotStats(const std::vector<Stats>& stats, int conditional,
               const std::string& name) {
  FILE* gnuplotPipe = popen("gnuplot -persistent", "w");

  if (gnuplotPipe) {
    // Output to a PNG file
    if (conditional == 1) {
      fprintf(
          gnuplotPipe,
          "set terminal pngcairo size 800,600 enhanced font 'Verdana,10'\n");
      fprintf(gnuplotPipe, "set output '%s'\n", name.c_str());
    }

    // Set up the 2x2 grid of plots
    fprintf(gnuplotPipe,
            "set multiplot layout 2,2 title 'Statistics Over Time'\n");

    // Plot v_mean vs time
    fprintf(gnuplotPipe, "set title 'v\\_mean vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'v\\_mean'\n");
    for (const auto& stat : stats) {
      fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.v_mean);
    }
    fprintf(gnuplotPipe, "e\n");

    // Plot d_mean vs time
    fprintf(gnuplotPipe, "set title 'd\\_mean vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'd\\_mean'\n");
    for (const auto& stat : stats) {
      fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.d_mean);
    }
    fprintf(gnuplotPipe, "e\n");

    // Plot sigma_v vs time
    fprintf(gnuplotPipe, "set title 'sigma\\_v vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'sigma\\_v'\n");
    for (const auto& stat : stats) {
      fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.sigma_v);
    }
    fprintf(gnuplotPipe, "e\n");

    // Plot sigma_d vs time
    fprintf(gnuplotPipe, "set title 'sigma\\_d vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'sigma\\_d'\n");
    for (const auto& stat : stats) {
      fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.sigma_d);
    }
    fprintf(gnuplotPipe, "e\n");

    // End multiplot
    fprintf(gnuplotPipe, "unset multiplot\n");

    // Close the pipe to Gnuplot
    pclose(gnuplotPipe);

    if (conditional) {
      std::cout << "Plot saved to " << name << std::endl;
    }
  } else {
    std::cerr << "Error: Could not open pipe to Gnuplot. \n" << std::endl;
  }
}

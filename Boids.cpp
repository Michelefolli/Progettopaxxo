#include "Boids.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <random>
#include <sstream>
#include <vector>

// non so dove vanno definite le variabili
// const float Pi = 3.14159265358979323846264f;

bool checkParametersValidity(int flock_size, int acquisiton_period,
                             Params& simulation_params) {
  if (flock_size > 1 && acquisiton_period > 10 && simulation_params.sep > 0 &&
      0 < simulation_params.alig && simulation_params.alig < 1 &&
      simulation_params.cohes > 0 && 0 < simulation_params.dist_sep &&
      simulation_params.dist_sep < simulation_params.dist) {
    return true;
  } else {
    std::cout << "Invalid input";
    return false;
  }
}

void inputData(int& flock_size, int& acquisiton_period,
               Params& simulation_params) {
  while (!checkParametersValidity(flock_size, acquisiton_period,
                                  simulation_params)) {
    std::cout << "Input the required data \n";
    std::cout << "Number of Boids: ";
    std::cin >> flock_size;
    std::cout << "Stats' period of acquisition: ";
    std::cin >> acquisiton_period;
    std::cout << "Simulation parameters:\n" << "Separation coefficient: ";
    std::cin >> simulation_params.sep;
    std::cout << "Alignment coefficient: ";
    std::cin >> simulation_params.alig;
    std::cout << "Cohesion coefficient: ";
    std::cin >> simulation_params.cohes;
    std::cout << "Interaction distance: ";
    std::cin >> simulation_params.dist;
    std::cout << "Separation distance: ";
    std::cin >> simulation_params.dist_sep;
  }
}

// funzione che fa in modo che i boid evitino i bordi della finestra sfml
void Boid::avoid_edges(const float edges_width,
                       const float edges_height) {  // debug mode
  float repulsive_range = 100.f;
  auto distance_from_border = velocity.norm();
  auto epsilon = std::numeric_limits<
      float>::epsilon();  // division and moltiplication by zero can lead to NaN
                          // errors hence the need for small epsilons

  if (position.x < repulsive_range) {
    velocity.x += (distance_from_border + epsilon) * (1) *
                  (1 / (std::pow((std::abs(position.x)), 0.25f) + epsilon));
  }
  if (position.y < repulsive_range) {
    velocity.y += (distance_from_border + epsilon) * (1) *
                  (1 / (std::pow((std::abs(position.y)), 0.25f) + epsilon));
  }
  if (position.x > (edges_width - repulsive_range)) {
    velocity.x +=
        (distance_from_border + epsilon) * (-1) *
        (1 / (std::pow((std::abs(edges_width - position.x)), 0.25f) + epsilon));
  }
  if (position.y > (edges_height - repulsive_range)) {
    velocity.y += (distance_from_border + epsilon) * (-1) *
                  (1 / (std::pow((std::abs(edges_height - position.y)), 0.25f) +
                        epsilon));
  };
}

// funzione che limita la velocità:
void Boid::limit(const float max_speed) {
  velocity = velocity * (max_speed / velocity.norm());
}

float Boid::abs_distance_from(const Boid& boid_j) const {
  return (position - boid_j.position).norm();
};  // modulo della distanza tra due boid

const Vec_2d Boid::separation(const std::vector<Boid>& flock,
                              const Params& simulation_params) const {
  Vec_2d v_sep =
      std::accumulate(
          flock.begin(), flock.end(), Vec_2d(0., 0.),
          [this, &simulation_params](Vec_2d sum, const Boid& other_boid) {
            if (abs_distance_from(other_boid) <= simulation_params.dist_sep) {
              Vec_2d diff = other_boid.getPosition() - position;
              sum += diff;
            }
            return sum;
          }) *
      (-simulation_params.sep);

  return v_sep;
};  // calcolo della velocità di separazione. Questa è testata e funziona

const Vec_2d Boid::alignment_and_cohesion(
    const std::vector<Boid>& flock, const Params& simulation_params) const {
  int count = 0;

  std::pair<Vec_2d, Vec_2d> sums =
      std::accumulate(  // sommatoria per posizioni e velocità
          flock.begin(), flock.end(),
          std::make_pair(Vec_2d(0.f, 0.f), Vec_2d(0.f, 0.f)),
          [this, &simulation_params, &count](std::pair<Vec_2d, Vec_2d> acc,
                                             const Boid& otherBoid) {
            if (this != &otherBoid &&
                this->abs_distance_from(otherBoid) <= simulation_params.dist) {
              acc.first += otherBoid.velocity;   // somma delle velocità
              acc.second += otherBoid.position;  // Somma delle posizioni
              ++count;
            }
            return acc;
          });

  if (count > 0) {
    Vec_2d avg_velocity = sums.first / static_cast<float>(count);
    Vec_2d center_of_mass = sums.second / static_cast<float>(count);

    Vec_2d v_alig = (avg_velocity - velocity) * simulation_params.alig;
    Vec_2d v_cohes = (center_of_mass - position) * simulation_params.cohes;

    return v_alig + v_cohes;
  } else
    return Vec_2d(0, 0);
};

// getters and setters:
const Vec_2d& Boid::getPosition() const { return position; }
const Vec_2d& Boid::getVelocity() const { return velocity; }
void Boid::setPosition(const Vec_2d& pos) { position = pos; }
void Boid::setVelocity(const Vec_2d& vel) { velocity = vel; }

void Boid::update(const Params& simulation_params,
                  const std::vector<Boid>& flock, const float max_speed,
                  const float edges_width, const float edges_height) {
  velocity += alignment_and_cohesion(flock, simulation_params) +
              separation(flock, simulation_params);

  avoid_edges(edges_width, edges_height);
  if (velocity.norm() > max_speed) {
    limit(max_speed);
  }
  position += velocity;
}  // questa funzione aggiorna la velocità del boid e poi lo sposta

void Boid::draw_on(sf::RenderWindow& window) const {
  sf::CircleShape shape(5, 3);  // definisce la forma da disegnare
  shape.setPosition((position.x), (position.y));
  shape.setFillColor(sf::Color::White);
  // float angle = (std::atan2(velocity.y, velocity.x) * 180 / (Pi));
  // shape.setRotation(angle);
  window.draw(shape);
}  // disegna il boid come un triangolo orientato nella direzione di volo

void runSimulation(sf::RenderWindow& window, std::vector<Boid>& flock,
                   const Params& simulation_params, const float max_speed,
                   std::vector<Boid>& flock_view, std::mutex& synchro_tool) {
  const float edges_width =
      static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float edges_height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  while (window.isOpen()) {
    sf::Event event;

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }  // permette di chiudere la finestra cliccado sulla x
    window.clear();  // pulisce la finestra ogni frame

    for (auto& boid : flock) {
      boid.update(simulation_params, flock, max_speed, edges_width,
                  edges_height);  // funzione che limita la velocità

      boid.draw_on(window);
    }

    // disegna tutti i boid, ma non li fa vedere ancora, quello è display
    synchro_tool.lock();
    flock_view = flock;
    synchro_tool.unlock();
    window.display();
  }
}

Stats calculateStatistics(
    const std::vector<Boid>& flock_view,
    const std::chrono::time_point<std::chrono::steady_clock>& start_time) {
  Stats stats{};  // chat gpt dice che è per evitare l'inizializzazione di
                  // varabili inutili dentro gli accumulate
  std::chrono::time_point current_time =
      std::chrono::steady_clock::now();  // prendo il tempo attuale
  auto time_span = std::chrono::duration_cast<std::chrono::duration<float>>(
      current_time - start_time);  // calcolo il tempo trascorso tra l'inzio del
                                   // ciclo (in Update_stats)
  stats.time = time_span.count();  // assegno il valore del tempo di calcolo
                                   // all'elemento stats

  float n = static_cast<float>(flock_view.size());  // dimensioni dello stormo

  // Vec_2d sum = {0.f , 0.f} ;
  Vec_2d v_mean_val =
      std::accumulate(flock_view.begin(), flock_view.end(), Vec_2d(0.f, 0.f),
                      [n](Vec_2d sum, const Boid& boid) {
                        return sum += (boid.getVelocity() / n);
                      });  // calcolo della velocità media

  // Vec_2d sum_d = {0.f , 0.f} ;
  float d_mean_val = std::accumulate(
      flock_view.begin(), flock_view.end(), 0.f,
      [&flock_view, n](float sum, const Boid& boid_i) {
        float d_mean_i = std::accumulate(
            flock_view.begin(), flock_view.end(), 0.f,
            [&boid_i, n](float sum_d, const Boid& boid_j) {
              return sum_d += (boid_i.abs_distance_from(boid_j) / (n - 1));
            });
        return sum += (d_mean_i / n);
      });  // calcolo della distanza media tra i boid dello stormo, utilizza un
           // nested algorithm non so se è una cosa positiva.

  stats.v_mean = v_mean_val.norm();
  stats.d_mean = d_mean_val;
  auto sigma_v_val = std::accumulate(
      flock_view.begin(), flock_view.end(), 0.f,
      [&v_mean_val, n](float sum, const Boid& boid) {
        return sum +=
               std::pow(boid.getVelocity().norm() - v_mean_val.norm(), 2.f) /
               (n - 1);
      });

  stats.sigma_v = std::sqrt(sigma_v_val);

  auto sigma_d_val = std::accumulate(
      flock_view.begin(), flock_view.end(), 0.f,
      [&flock_view, &d_mean_val, n](float sum, const Boid& boid_i) {
        float d_sigma_i = std::accumulate(
            flock_view.begin(), flock_view.end(), 0.f,
            [&d_mean_val, &boid_i, n](float sum_d, const Boid& boid_j) {
              return sum_d +=
                     std::pow(boid_i.abs_distance_from(boid_j) - d_mean_val,
                              2.f) /
                     (n - 2);
            });
        return sum += d_sigma_i / n;
      });
  stats.sigma_d =
      std::sqrt(sigma_d_val - (std::pow(d_mean_val, 2.f) / (n - 2)));

  return stats;
}
// funzione che restituisce le statistiches

void fillStatsVector(const std::vector<Boid>& flock_view,
                     std::vector<Stats>& timestamped_stats,
                     const std::chrono::time_point<std::chrono::steady_clock>&
                         start_time) {  // funzione che
                                        // riempie il vettore
                                        // delle statistiche

  timestamped_stats.push_back(calculateStatistics(flock_view, start_time));
}

void update_Stats(
    const std::vector<Boid>& flock_view, std::vector<Stats>& timestamped_stats,
    int acquisiton_period, sf::RenderWindow& window,
    std::mutex& synchro_tool) {  // funzione che riassume il processo
  // di acquisizione periodica delle statistiche
  auto start_time = std::chrono::steady_clock::now();
  auto last_update_time = start_time;
  auto current_time = start_time;
  while (window.isOpen()) {  // questo if era per controllare che compilasse,
                             // diventerà un while(window.isOpen)

    current_time = std::chrono::steady_clock::now();
    if (current_time - last_update_time >=
        std::chrono::milliseconds(acquisiton_period)) {
      last_update_time = current_time;
      synchro_tool.lock();
      fillStatsVector(flock_view, timestamped_stats, start_time);
      synchro_tool.unlock();
    }
  }
}

std::string namingFile() {  // ha lo scopo di ricevere il nome inserito e
                            // restituirlo alla funzione caller
  std::string file_name{};
  std::cout << "Insert the name of the  file: ";
  std::cin >> file_name;
  return file_name;
}

bool askForTxt() {  // chiede all'utilizzatore se vuole salvare le statistiche
                    // come
                    // file .txt
  int txt_option{};
  while (true) {
    std::cout << "Input 1 if you want to export the stats as a .txt file, "
                 "otherwise input 0: ";
    std::cin >> txt_option;

    if (txt_option == 0 || txt_option == 1) {
      break;
    } else {
      std::cout << "Invalid input. Please enter 0 or 1.\n";
    }
  }
  return txt_option;
}

void instantiateStatsFile(std::ostringstream& output_str) {
  std::string txt_file_name = namingFile() + ".txt";
  std::ofstream file_output(txt_file_name);
  if (!file_output) {
    std::cout << "There was an error in the creation of the file \n";
  } else
    file_output << output_str.str();
  file_output.close();
}

void exportStats(const std::vector<Stats>& timestamped_statistics) {
  std::ostringstream output_str;

  std::for_each(timestamped_statistics.begin(), timestamped_statistics.end(),
                [&output_str](const Stats& stat) {
                  output_str << std::fixed << std::setprecision(1)
                             << stat.d_mean << "  " << stat.sigma_d << "  "
                             << std::setprecision(3) << stat.v_mean << "  "
                             << stat.sigma_v << "  " << std::setprecision(2)
                             << stat.time << "\n";
                });

  if (askForTxt()) {
    instantiateStatsFile(output_str);
  }

  std::cout << output_str.str();
}

bool askForPng() {  // chiede allo user se vuole esportare i plots come file
                    // .png
  int png_option{};
  while (true) {
    std::cout << "Input 1 if you want to export the plots as a .png file, "
                 "otherwise input 0: ";
    std::cin >> png_option;

    if (png_option == 0 || png_option == 1) {
      break;
    } else {
      std::cout << "Invalid input. Please enter 0 or 1.\n";
    }
  }
  return png_option;
}

void plotStats(const std::vector<Stats>& timestamped_stats, bool png_option,
               const std::string& png_file_name) {
  FILE* gnuplotPipe = popen("gnuplot -persistent", "w");

  if (gnuplotPipe) {
    // Output to a PNG file
    if (png_option) {
      fprintf(
          gnuplotPipe,
          "set terminal pngcairo size 800,600 enhanced font 'Verdana,10'\n");
      fprintf(gnuplotPipe, "set output '%s'\n", png_file_name.c_str());
    }

    // Set up the 2x2 grid of plots
    fprintf(gnuplotPipe,
            "set multiplot layout 2,2 title 'Statistics Over Time'\n");

    // Plot v_mean vs time
    fprintf(gnuplotPipe, "set title 'v\\_mean vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'v\\_mean'\n");
    std::for_each(timestamped_stats.begin(), timestamped_stats.end(),
                  [&gnuplotPipe](const Stats& stat) {
                    fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.v_mean);
                  });
    fprintf(gnuplotPipe, "e\n");

    // Plot sigma_v vs time
    fprintf(gnuplotPipe, "set title 'sigma\\_v vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'sigma\\_v'\n");
    std::for_each(timestamped_stats.begin(), timestamped_stats.end(),
                  [&gnuplotPipe](const Stats& stat) {
                    fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.sigma_v);
                  });
    fprintf(gnuplotPipe, "e\n");

    // Plot d_mean vs time
    fprintf(gnuplotPipe, "set title 'd\\_mean vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'd\\_mean'\n");
    std::for_each(timestamped_stats.begin(), timestamped_stats.end(),
                  [&gnuplotPipe](const Stats& stat) {
                    fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.d_mean);
                  });
    fprintf(gnuplotPipe, "e\n");

    // Plot sigma_d vs time
    fprintf(gnuplotPipe, "set title 'sigma\\_d vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'sigma\\_d'\n");
    std::for_each(timestamped_stats.begin(), timestamped_stats.end(),
                  [&gnuplotPipe](const Stats& stat) {
                    fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.sigma_d);
                  });
    fprintf(gnuplotPipe, "e\n");

    // End multiplot
    fprintf(gnuplotPipe, "unset multiplot\n");

    // Close the pipe to Gnuplot
    pclose(gnuplotPipe);

  } else {
    std::cerr << "Error: Could not open pipe to Gnuplot. \n" << std::endl;
  }
}

void exportPlot(const std::vector<Stats>& timestamped_stats) {
  bool png_option = askForPng();
  std::string png_file_name{};
  if (png_option == 1) {
    png_file_name = namingFile() + ".png";
  }
  plotStats(timestamped_stats, png_option, png_file_name);
}

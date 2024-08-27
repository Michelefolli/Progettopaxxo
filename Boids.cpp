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

// The significance of the simulation and it's statistics is upheld by ensuring
// the correctness of the inputs
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

// Flexibility is provided by enabling the user to choose many of the
// simulation's key parameters
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

// The repulsion from the borders being inversely proportional to the distance
// from the edge minimizes the interference, ensuring minimal changes to the
// original BOID model, while making the simulation more fluid. The exponent
// 0.25 and the repulsive range 100. are arbitrary.
void Boid::avoid_edges(const float edges_width, const float edges_height) {
  float repulsive_range = 100.f;
  auto distance_from_border = velocity.norm();
  auto epsilon = std::numeric_limits<
      float>::epsilon();  // division and moltiplication by zero can lead to NaN
                          // errors hence the need for small epsilons

  if (position.x < repulsive_range) {
    velocity.x += (distance_from_border + epsilon) *
                  (1 / (std::pow((std::abs(position.x)), 0.25f) + epsilon));
  }
  if (position.y < repulsive_range) {
    velocity.y += (distance_from_border + epsilon) *
                  (1 / (std::pow((std::abs(position.y)), 0.25f) + epsilon));
  }
  // the "-" sign is necessary produce the negative velocity components for the
  // correct implementation of the repulsion model, hence the magic number "-1"
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

// Limitig the velocity increases the fidelity to real life flocks ensuring a
// more significant simulation
void Boid::limit(const float max_speed) {
  velocity = velocity * (max_speed / velocity.norm());
}

float Boid::abs_distance_from(const Boid& boid_j) const {
  return (position - boid_j.position).norm();
};

// Implementation of the separation law for a single Boid object
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
};

// By combining the alignment and cohesion processes the calculations are
// optimized by reducing the number of times the flock vector has to be read
const Vec_2d Boid::alignment_and_cohesion(
    const std::vector<Boid>& flock, const Params& simulation_params) const {
  int count = 0;

  std::pair<Vec_2d, Vec_2d> sums = std::accumulate(
      flock.begin(), flock.end(),
      std::make_pair(Vec_2d(0.f, 0.f), Vec_2d(0.f, 0.f)),
      [this, &simulation_params, &count](std::pair<Vec_2d, Vec_2d> acc,
                                         const Boid& otherBoid) {
        if (this != &otherBoid &&
            this->abs_distance_from(otherBoid) <= simulation_params.dist) {
          acc.first += otherBoid.velocity;   // summing velocity vectors
          acc.second += otherBoid.position;  // summing position vectors
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

const Vec_2d& Boid::getPosition() const { return position; }
const Vec_2d& Boid::getVelocity() const { return velocity; }

// Streamlines the process of updating the Boids' coordinates by applyng all of
// the laws consecutively.
void Boid::update(const Params& simulation_params,
                  const std::vector<Boid>& flock, const float max_speed,
                  const float edges_width, const float edges_height) {
  velocity += alignment_and_cohesion(flock, simulation_params) +
              separation(flock, simulation_params);

  avoid_edges(edges_width, edges_height);
  if (velocity.norm() > max_speed) {
    limit(max_speed);
  }
  position +=
      velocity;  // the time difference is assumed to be 1s per iteration
}

void Boid::draw_on(sf::RenderWindow& window) const {
  sf::CircleShape shape(6, 3);  // sets traingle shape
  shape.setPosition((position.x), (position.y));
  shape.setFillColor(sf::Color::Black);
  window.draw(shape);
}

auto scaleBackground(const sf::RenderWindow& window,
                     const sf::Texture& backgroundTexture) {
  sf::Vector2f windowSize(window.getSize());
  sf::Vector2f textureSize(backgroundTexture.getSize());
  sf::Vector2f scale(windowSize.x / textureSize.x,
                     windowSize.y / textureSize.y);
  return scale;
}

// Encapsulates the whole simulation process in a single function
void runSimulation(sf::RenderWindow& window,
                   const sf::Texture& backgroundTexture,
                   std::vector<Boid>& flock, const Params& simulation_params,
                   const float max_speed, std::vector<Boid>& flock_view,
                   std::mutex& synchro_tool) {
  const float edges_width =
      static_cast<float>(sf::VideoMode::getDesktopMode().width);
  const float edges_height =
      static_cast<float>(sf::VideoMode::getDesktopMode().height);
  // Ensures that the simulation keeps running while the window stays open
  sf::Event event;
  sf::Sprite backgroundSprite(backgroundTexture);
  backgroundSprite.setScale(scaleBackground(window, backgroundTexture));
  
  while (window.isOpen()) {
    // Enables the ability to close thw window
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    window.clear();  // clears the window every frame

    window.draw(backgroundSprite);
    // Loops over the flock vector to update and draw every boid entity
    for (auto& boid : flock) {
      boid.update(simulation_params, flock, max_speed, edges_width,
                  edges_height);

      boid.draw_on(window);
    }

    // The possible lag caused by the stats calculation is reduced by shortening
    // the locked mutex window thanks to the use of the flock_view vector
    synchro_tool.lock();
    flock_view = flock;
    synchro_tool.unlock();

    // Displays the boid on the window
    window.display();
  }
}

Stats calculateStatistics(
    const std::vector<Boid>& flock_view,
    const std::chrono::time_point<std::chrono::steady_clock>& start_time) {
  Stats stats{};  // Proper inizialization of the Stats instance
  std::chrono::time_point current_time =
      std::chrono::steady_clock::now();  // gets current time
  auto time_span = std::chrono::duration_cast<std::chrono::duration<float>>(
      current_time - start_time);
  stats.time =
      time_span.count();  // The seconds passed since the start of the
                          // simulation are assigned to the time variable

  float n = static_cast<float>(flock_view.size());  // size of the flock

  // Mean velocity vector
  Vec_2d v_mean_vec = std::accumulate(
      flock_view.begin(), flock_view.end(), Vec_2d(0.f, 0.f),
      [](Vec_2d sum, const Boid& boid) { return sum += (boid.getVelocity()); });
  v_mean_vec = v_mean_vec / n;
  stats.v_mean = v_mean_vec.norm();  // Mean velocity norm

  
  auto sigma_v_val = std::accumulate(
      flock_view.begin(), flock_view.end(), 0.f,
      [&v_mean_vec](float sum, const Boid& boid) {
        return sum += std::pow((boid.getVelocity() - v_mean_vec).norm(), 2.f);
      });

  stats.sigma_v = std::sqrt(sigma_v_val / (n - 1));

  std::vector<float> pairwise_distances{};

  // Nested loop to gather all the pairwise distances between boids
  for (int i{}; i < static_cast<int>(n); ++i) {
    for (int j = i + 1; j < static_cast<int>(n); ++j) {
      pairwise_distances.push_back(
          flock_view[static_cast<unsigned int>(i)].abs_distance_from(
              flock_view[static_cast<unsigned int>(j)]));
    }
  }

  // Get the total number of pairs
  float total_pairs = static_cast<float>(pairwise_distances.size());

  // Sum all of the pairwise distances
  float dist_sum = std::accumulate(pairwise_distances.begin(),
                                   pairwise_distances.end(), 0.f);

  // Get mean distance
  stats.d_mean = dist_sum / total_pairs;

  // Find the sum of all the squared deviation
  float sqrd_deviation = std::accumulate(
      pairwise_distances.begin(), pairwise_distances.end(), 0.f,
      [&stats](float total_dev, const float& dist_ij) {
        return total_dev += std::pow((dist_ij - stats.d_mean), 2.f);
      });

  // Get distance standard deviatiom
  stats.sigma_d = std::sqrt(sqrd_deviation / (total_pairs - 1));
  /*  // The mean of the individual means is calculated
    float d_mean_val = std::accumulate(
        flock_view.begin(), flock_view.end(), 0.f,
        [&flock_view, n](float sum, const Boid& boid_i) {
          float d_mean_i = std::accumulate(
              flock_view.begin(), flock_view.end(), 0.f,
              [&boid_i, n](float sum_d, const Boid& boid_j) {
                return sum_d += (boid_i.abs_distance_from(boid_j) / (n - 1));
              });
          return sum += (d_mean_i / n);
        });


    stats.d_mean = d_mean_val;
    // Standard deviation for velocity

    // The distance standard deviation is obtained through the Root square sum
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
          return sum += d_sigma_i / (n * n);
        });
    // The subratction is a correction for the fact that the nested accumulate
    // accounts for the case where i = j
    stats.sigma_d =
        std::sqrt(sigma_d_val - (std::pow(d_mean_val, 2.f) / (n * (n - 2)))); */

  return stats;
}

void fillStatsVector(const std::vector<Boid>& flock_view,
                     std::vector<Stats>& timestamped_stats,
                     const std::chrono::time_point<std::chrono::steady_clock>&
                         start_time) {  // funzione che
                                        // riempie il vettore
                                        // delle statistiche

  timestamped_stats.push_back(calculateStatistics(flock_view, start_time));
}

// Encapsulates the stats handling process in a single function
void update_Stats(const std::vector<Boid>& flock_view,
                  std::vector<Stats>& timestamped_stats, int acquisiton_period,
                  sf::RenderWindow& window, std::mutex& synchro_tool) {
  auto start_time = std::chrono::steady_clock::now();
  auto last_update_time = start_time;
  auto current_time = start_time;
  // Ensures that the stats keep getting periodically acquired as long as the
  // simulation is running
  while (window.isOpen()) {
    current_time = std::chrono::steady_clock::now();  // updates current time
    // Checks that enough time has passed since the last acquisition
    if (current_time - last_update_time >=
        std::chrono::milliseconds(acquisiton_period)) {
      last_update_time =
          current_time;  // mantains the if statement proper inner working
      synchro_tool.lock();
      fillStatsVector(flock_view, timestamped_stats, start_time);
      synchro_tool.unlock();
    }
  }
}

// Uses the same function for naming both .png and .txt files
std::string namingFile() {
  std::string file_name{};
  std::cout << "Insert the name of the  file: ";
  std::cin >> file_name;
  return file_name;
}

// Ensures that the input is either 1 or 0 as so to avoid missclicks
bool askForTxt() {
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

// Enables the ability to easily save the simulation's stats by .txt file
void instantiateStatsFile(std::ostringstream& output_str) {
  std::string txt_file_name = namingFile() + ".txt";
  std::ofstream file_output(txt_file_name);
  if (!file_output) {  // Ensures the output pipeline was succesfuly opened
    std::cout << "There was an error in the creation of the file \n";
  } else
    file_output << output_str.str();
  file_output.close();
}

// Handles that the stats get exported properly and with an appropriate number
// of significant digits. The setprecision numbers are arbitrary
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

  if (askForTxt()) {  // Ask for permission to save the stats before calling the
                      // function
    instantiateStatsFile(output_str);
  }

  std::cout << output_str.str();  // Prints the stats on the shell
}

// Ensures that the input is either 1 or 0 as so to avoid missclicks
bool askForPng() {
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
  // Open pipeline
  FILE* gnuplotPipe = popen("gnuplot -persistent", "w");

  if (gnuplotPipe) {  // Ensure the pipeline was correctly opened

    if (png_option) {  // Creates .png file if asked to
      fprintf(
          gnuplotPipe,
          "set terminal pngcairo size 1200,800 enhanced font 'Verdana,10'\n");
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
    std::cout << "Error: Could not open pipe to Gnuplot. \n" << std::endl;
  }
}

// Handles the creation of the stats' plots
void exportPlot(const std::vector<Stats>& timestamped_stats) {
  bool png_option = askForPng();  // Asks the user if he wants to export as .png
  std::string png_file_name{};
  if (png_option) {
    png_file_name =
        namingFile() + ".png";  // Eventually asks for the .png's name
  }
  // Creates plots and exports if asked
  plotStats(timestamped_stats, png_option, png_file_name);
}

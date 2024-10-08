#include "IOhandling.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

/*
 * INPUT HANDLING
 */

// The significance of the simulation and its statistics is upheld by ensuring
// the correctness of the inputs
bool checkParametersValidity(int flock_size, int acquisition_period,
                             Params& simulation_params) {
  if (flock_size > 1 && acquisition_period >= 10 && simulation_params.sep > 0 &&
      0 < simulation_params.alig && simulation_params.alig < 1 &&
      simulation_params.cohes > 0 && 0 < simulation_params.dist_sep &&
      simulation_params.dist_sep < simulation_params.dist) {
    return true;
  } else {
    return false;
  }
}
// Flexibility is provided by enabling the user to choose many of the
// simulation's key parameters
void inputData(int& flock_size, int& acquisition_period,
               Params& simulation_params) {
  // Using a do while statement to enter the parameters before the while
  // input status is evaluated to avoid early "Invalid input" outputs
  do {
    std::cout << "Input the required data \n";
    std::cout << "Number of Boids: ";
    std::cin >> flock_size;
    std::cout << "Stats' period of acquisition: ";
    std::cin >> acquisition_period;
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

    // Checks for invalid inputs and clears buffer
    if (std::cin.fail()) {
      std::cout << '\n';
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "\033[31mInvalid input, please input numbers!\033[0m\n";
      continue;
    }

    // Checks input status to send error message
    if (!checkParametersValidity(flock_size, acquisition_period,
                                 simulation_params)) {
      std::cout << "\033[31mInvalid input, try again\033[0m\n";
    }

  } while (!checkParametersValidity(flock_size, acquisition_period,
                                    simulation_params));
}

/*
 * OUTPUT HANDLING
 */

// Uses the same function for naming both .png and .txt files
std::string namingFile() {
  std::string file_name{};
  std::cout << "Insert the name of the  file: ";
  std::cin >> file_name;
  return file_name;
}

// Ensures that the input is either 1 or 0 as so to avoid misclicks
bool askForTxt() {
  int txt_option{};
  while (true) {
    std::cout << "Input 1 if you want to export the stats as a .txt file, "
                 "otherwise input 0: ";
    std::cin >> txt_option;

    if (txt_option == 0 || txt_option == 1) {
      break;
    } else {
      std::cout << "\033[31mInvalid input. Please enter 0 or 1.\033[0m\n";
    }
  }
  return txt_option;
}

// Enables the ability to easily save the simulation's stats by .txt file
void instantiateStatsFile(std::ostringstream& output_str) {
  std::string txt_file_name = namingFile() + ".txt";
  std::ofstream file_output(txt_file_name);
  if (!file_output) {  // Ensures the output pipeline was successfully opened
    std::cout
        << "\033[31mThere was an error in the creation of the file\033[0m \n";
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

  if (askForTxt()) {  // Asks for permission to save the stats before calling
                      // the function
    instantiateStatsFile(output_str);
  }

  std::cout << output_str.str();  // Prints the stats on the shell
}

// Ensures that the input is either 1 or 0 as so to avoid misclicks
bool askForPng() {
  int png_option{};
  while (true) {
    std::cout << "Input 1 if you want to export the plots as a .png file, "
                 "otherwise input 0: ";
    std::cin >> png_option;

    if (png_option == 0 || png_option == 1) {
      break;
    } else {
      std::cout << "\033[31mInvalid input. Please enter 0 or 1.\033[0m\n";
    }
  }
  return png_option;
}

void plotStats(const std::vector<Stats>& timestamped_stats, bool png_option,
               const std::string& png_file_name) {
  // Opens pipeline
  FILE* gnuplotPipe = popen("gnuplot -persistent", "w");

  if (gnuplotPipe) {  // Ensures the pipeline was correctly opened

    if (png_option) {  // Creates .png file if asked to
      fprintf(
          gnuplotPipe,
          "set terminal pngcairo size 1200,800 enhanced font 'Verdana,10'\n");
      fprintf(gnuplotPipe, "set output '%s'\n", png_file_name.c_str());
    }

    // Sets up the 2x2 grid of plots
    fprintf(gnuplotPipe,
            "set multiplot layout 2,2 title 'Statistics Over Time'\n");

    // Plots v_mean vs time
    fprintf(gnuplotPipe, "set title 'v\\_mean vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'v\\_mean'\n");
    std::for_each(timestamped_stats.begin(), timestamped_stats.end(),
                  [&gnuplotPipe](const Stats& stat) {
                    fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.v_mean);
                  });
    fprintf(gnuplotPipe, "e\n");

    // Plots sigma_v vs time
    fprintf(gnuplotPipe, "set title 'sigma\\_v vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'sigma\\_v'\n");
    std::for_each(timestamped_stats.begin(), timestamped_stats.end(),
                  [&gnuplotPipe](const Stats& stat) {
                    fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.sigma_v);
                  });
    fprintf(gnuplotPipe, "e\n");

    // Plots d_mean vs time
    fprintf(gnuplotPipe, "set title 'd\\_mean vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'd\\_mean'\n");
    std::for_each(timestamped_stats.begin(), timestamped_stats.end(),
                  [&gnuplotPipe](const Stats& stat) {
                    fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.d_mean);
                  });
    fprintf(gnuplotPipe, "e\n");

    // Plots sigma_d vs time
    fprintf(gnuplotPipe, "set title 'sigma\\_d vs time'\n");
    fprintf(gnuplotPipe, "plot '-' with lines title 'sigma\\_d'\n");
    std::for_each(timestamped_stats.begin(), timestamped_stats.end(),
                  [&gnuplotPipe](const Stats& stat) {
                    fprintf(gnuplotPipe, "%f %f\n", stat.time, stat.sigma_d);
                  });
    fprintf(gnuplotPipe, "e\n");

    // Ends multiplot
    fprintf(gnuplotPipe, "unset multiplot\n");

    // Closes the pipe to Gnuplot
    pclose(gnuplotPipe);

  } else {
    std::cout << "\033[31mError: Could not open pipe to Gnuplot.\033[0m \n"
              << std::endl;
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

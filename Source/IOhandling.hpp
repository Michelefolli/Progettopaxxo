#ifndef IO_HANDLING_HPP
#define IO_HANDLING_HPP

#include "Boids.hpp"

// Input handling functions
bool checkParametersValidity(int flock_size, int acquisiton_period,
                             Params& simulation_params);
void inputData(int& flock_size, int& acquisiton_period,
               Params& simulation_params);

// Output handling functions
std::string namingFile();
bool askForTxt();
void instantiateStatsFile(std::ostringstream& output_str);
void exportStats(const std::vector<Stats>& timestamped_stats);
bool askForPng();
void plotStats(const std::vector<Stats>& timestamped_stats, bool png_option,
               const std::string& png_file_name);
void exportPlot(const std::vector<Stats>& timestamped_stats);

#endif

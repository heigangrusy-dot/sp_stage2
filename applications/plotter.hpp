#ifndef APPLICATIONS_PLOTTER_HPP
#define APPLICATIONS_PLOTTER_HPP

#include <array>

using PlotData = std::array<float, 18>;
void update_plot_data(const PlotData & data);

#endif  // APPLICATIONS_PLOTTER_HPP

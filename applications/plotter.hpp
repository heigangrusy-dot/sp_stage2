#ifndef APPLICATIONS_PLOTTER_HPP
#define APPLICATIONS_PLOTTER_HPP

#include <array>

// 15路数据沿用原通道顺序；更新快照不发送串口数据。
using PlotData = std::array<float, 15>;
void update_plot_data(const PlotData & data);

#endif  // APPLICATIONS_PLOTTER_HPP

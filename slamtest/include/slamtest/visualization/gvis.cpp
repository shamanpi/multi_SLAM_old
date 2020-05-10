#include <visualization/gvis.h>

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

GVisualizer::GVisualizer() {
  
}

size_t GVisualizer::AddFigure(std::string figure_title, bool force_square) {
  plots.emplace_back("lines");
  plots.back().set_title(figure_title);
  if (force_square) {
    plots.back().set_xautoscale();
    plots.back().set_yautoscale();
  }
  return plots.size() - 1;
}

void GVisualizer::ShowFigures() {
  for (auto& plot : plots) {
    plot.showonscreen();
  }
}

void GVisualizer::PauseFigures(double pause_time_s) {
  if (pause_time_s < 0) {
    for (auto& plot : plots) {
      if (plot.is_valid()) {
        plot << "pause mouse close";
      }
    }
  } else {
    std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int>(pause_time_s * 1e9)));
  }
}

void GVisualizer::Plot(size_t fig_id, const std::vector<Eigen::VectorXd> &points,
                      std::string color) {
  double x, x_prev = points[0][0], y, y_prev = points[0][1];
  for (size_t i = 0; i < points.size(); ++i) {
    x = points[i][0];
    y = points[i][1];

    plots[fig_id] << "set arrow from " + std::to_string(x_prev) + "," + std::to_string(y_prev) + " to " + std::to_string(x) + "," + std::to_string(y) + " nohead lc rgb \""+color+"\"\r\n";

    x_prev = x;
    y_prev = y;
  }
}

void GVisualizer::SetFigureAxisLimits(size_t fig_id,
                                     const Eigen::Vector2d &x_lim,
                                     const Eigen::Vector2d &y_lim) {
  plots[fig_id].set_xrange(x_lim[0], x_lim[1]);
  plots[fig_id].set_yrange(y_lim[0], y_lim[1]);
}

void GVisualizer::ClearFigure(size_t fig_id) {
  plots[fig_id].reset_plot();
  plots[fig_id].cmd("unset arrow");
}

void GVisualizer::DrawFigure(size_t fig_id) {
  plots[fig_id].cmd("plot -1000 with line lc rgb \"white\" ");
}

GVisualizer::~GVisualizer() {
}
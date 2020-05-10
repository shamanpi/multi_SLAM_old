#pragma once

#include <visualization/gnuplot_i.h>
#include <Eigen/Core>
#include <vector>

class GVisualizer {
public:
  GVisualizer();

  // Adds a new figure, bool controls if the x and y units should be equally
  // spaced (i.e. forces circles to look circular) Note that this means that
  // axis limits will be adjusted after you try to set them to the closest with
  // the correct aspect ratio
  size_t AddFigure(std::string figure_title, bool force_square);

  // All figures start hidden, this will make all figures visible, note that
  // once you show a figure, closing it via the GUI will mark that figure's
  // index as invalid
  void ShowFigures();

  // Pauses for the time (in seconds) specified in pause_time_s, figures stay
  // interactable If negative, pauses until all plots are closed by the user.
  void PauseFigures(double pause_time_s);

  // Plot the points to figure fig_id, as specified by the format string
  // (See https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.pyplot.plot.html)
  void Plot(size_t fig_id, const std::vector<Eigen::VectorXd> &points,
            std::string color);

  // Set the x and y axis limits on the plot.  Note that if the figure was
  // defined with force_square=true, these will not be the exact limits after
  // it's executed
  void SetFigureAxisLimits(size_t fig_id, const Eigen::Vector2d &x_lim,
                           const Eigen::Vector2d &y_lim);

  // Removes anything plotted from the specified figure
  void ClearFigure(size_t fig_id);

  void DrawFigure(size_t fig_id);

  ~GVisualizer();

private:
  // Don't allow copy constructors, it would break the C++->Python interface
  GVisualizer(const GVisualizer &);
  void operator=(GVisualizer const &);

  std::vector<Gnuplot> plots;
};
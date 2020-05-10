#pragma once
#include <visualization/opengl/opengl_utils.h>

#include <Eigen/Core>

#include <thread>
#include <mutex>

enum class Color { RED, BLUE, GREEN, YELLOW, CYAN, MAGENTA, BLACK };

inline Eigen::Vector3f ColorToRGB(Color c) {
  switch (c)
  {
  case Color::RED:
    return Eigen::Vector3f(1.0,0.0,0.0);
  case Color::BLUE:
    return Eigen::Vector3f(0.0,0.0,1.0);
  case Color::GREEN:
    return Eigen::Vector3f(0.0,1.0,0.0);
  case Color::YELLOW:
    return Eigen::Vector3f(1.0,1.0,0.0);
  case Color::CYAN:
    return Eigen::Vector3f(0.0,1.0,1.0);
  case Color::MAGENTA:
    return Eigen::Vector3f(1.0,0.0,1.0);
  case Color::BLACK:
    return Eigen::Vector3f(0.0,0.0,0.0);
  }
  return Eigen::Vector3f(0.0,0.0,0.0);
}

struct Line {
  std::vector<Eigen::VectorXd> segs;
  Color c;
  double width;
};

class GLVisualizer {
public:
  GLVisualizer();
  void AddTempLine(std::vector<Eigen::VectorXd> line, Color c, double pix_width);
  void AddTempEllipse(Eigen::Vector2d x, Eigen::Matrix2d Sig, Color c, double pix_width);
  void ClearTempLines();
  void AddPermLine(std::vector<Eigen::VectorXd> line, Color c, double pix_width);
  void ClearPermLines();
  void UpdateLines();

  void SetCenterAndHeight(Eigen::Vector2d cen, double height);

  void PauseFigure(double pause_time_s);

  ~GLVisualizer();

  // SDL event handling:
  // void handleMouseScroll(SDL_Event& e);
  // void handleMouseButtonDown(SDL_Event& e);
  // void handleMouseButtonUp(SDL_Event& e);
  // void handleMouseMotion(SDL_Event& e);
  // void handleKeyup(SDL_Event& e);

private:
  void setActiveProgram(GLint program);
  // void ConvertMouseXY(int x, int y, float& m_x, float& m_y);

  std::mutex perm_line_lock, temp_line_lock;
  std::vector<Line> perm_lines;
  std::vector<Line> temp_lines;
  bool clear_perm = false;
  bool clear_temp = false;
  std::vector<Line> new_perm_lines;
  std::vector<Line> new_temp_lines;

  void handleEvents();
  
  void Draw();
  void DrawBackground();
  static constexpr float bg_size = 0.2f;
  void DrawLines();
  void DrawLine(Line& l);


  static constexpr int MAX_LINE_SEGMENTS = 20000;
  static constexpr int SEG_SIZE = 2; // x,y
  GLuint vbo[1];
  float* line_buffer;
  GLint shaderProgram;
  glm::mat4 proj,view;

  GLFWwindow* wind;
  int w, h; // window width and height in pixels
  float view_height = 10.0;
  float cen_x = 0.0f, cen_y = 0.0f;

  std::thread drawing_thread;
  void DrawLoop();
  bool running = true;
};
#include <visualization/opengl/glvis.h>

#include <Eigen/Dense>
#include <chrono>

void GLVisualizer::handleEvents() {
  glfwPollEvents();
  // Check if we're supposed to close
  if (glfwWindowShouldClose(wind)) {
    running = false;
    return;
  }
	// SDL_Event windowEvent;
	// while (SDL_PollEvent(&windowEvent)){
	// 	if (windowEvent.type == SDL_QUIT) {
	// 		running = false;
	// 		break;
	// 	}
  // }
}

GLVisualizer::GLVisualizer() {
  wind = GL_GLFW_init("GLVis");
  glfwMakeContextCurrent(nullptr);
  // Fork off the drawing loop
  drawing_thread = std::thread(&GLVisualizer::DrawLoop, this);
}

void GLVisualizer::DrawLoop() {
  glfwMakeContextCurrent(wind);
  glEnable(GL_DEPTH_TEST);
  
  line_buffer = new float[MAX_LINE_SEGMENTS * SEG_SIZE];

  glGenBuffers(1, vbo);  //Create 1 buffer called vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]); //Set the vbo as the active array buffer (Only one buffer can be active at a time)
	glBufferData(GL_ARRAY_BUFFER, MAX_LINE_SEGMENTS*SEG_SIZE*sizeof(float), line_buffer, GL_STREAM_DRAW); //upload vertices to vbo

  std::string fShader = std::string(GL_SHADER_PATH) + std::string("fragShader.glsl");
  std::string vShader = std::string(GL_SHADER_PATH) + std::string("vertShader.glsl");

  shaderProgram = InitShader(vShader.c_str(), fShader.c_str());

	while (running) {
    auto t_iter_start = std::chrono::high_resolution_clock::now();
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    Draw();

    glfwSwapBuffers(wind); //Double buffering
    // Try to run at 100 fps
    std::this_thread::sleep_until(std::chrono::milliseconds(10) + t_iter_start);
  }
}

void GLVisualizer::UpdateLines() {
  handleEvents();
  temp_line_lock.lock();
  if (clear_temp) {
    temp_lines.clear();
  }
  for (Line l : new_temp_lines) {
    temp_lines.push_back(l);
  }
  new_temp_lines.clear();
  temp_line_lock.unlock();
  perm_line_lock.lock();
  if (clear_perm) {
    perm_lines.clear();
  }
  for (Line l : new_perm_lines) {
    perm_lines.push_back(l);
  }
  new_perm_lines.clear();
  perm_line_lock.unlock();
}  

void GLVisualizer::AddTempLine(std::vector<Eigen::VectorXd> line, Color c, double pix_width) {
  Line l;
  l.segs = line;
  l.c = c;
  l.width = pix_width;
  new_temp_lines.push_back(l);
}

void GLVisualizer::AddTempEllipse(Eigen::Vector2d x, Eigen::Matrix2d Sig, Color c, double pix_width) {
  std::vector<Eigen::VectorXd> ell_line(20,Eigen::Vector2d::Zero());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(Sig);

  Eigen::Vector2d e_val = eigensolver.eigenvalues();
  Eigen::Matrix2d e_vec = eigensolver.eigenvectors();

  double ellipse_orientation = std::atan2(e_vec.col(0)[1], e_vec.col(0)[0]);
  Eigen::Matrix2d Rot;
  Rot << std::cos(ellipse_orientation), -std::sin(ellipse_orientation), std::sin(ellipse_orientation), std::cos(ellipse_orientation);

  for (size_t i = 0; i < 20; ++i) {
    // n_pts-1 so that the first and last points are the same (completing the
    // circle)
    double theta = 6.28318530718 * i / (19);

    // Scale by major / minor axis, then rotate and offset
    ell_line[i] = Eigen::Vector2d(2.0 * std::sqrt(5.991 * e_val[0]) * std::cos(theta), 2.0 * std::sqrt(5.991 * e_val[1]) * std::sin(theta));
    ell_line[i] = Rot * ell_line[i] + x;
  }

  AddTempLine(ell_line, c, pix_width);
}

void GLVisualizer::ClearTempLines() {
  clear_temp = true;
  new_temp_lines.clear();
}

void GLVisualizer::AddPermLine(std::vector<Eigen::VectorXd> line, Color c, double pix_width) {
  Line l;
  l.segs = line;
  l.c = c;
  l.width = pix_width;
  new_perm_lines.push_back(l);
}

void GLVisualizer::ClearPermLines() {
  clear_perm = true;
  new_perm_lines.clear();
}

void GLVisualizer::SetCenterAndHeight(Eigen::Vector2d cen, double height) {
  cen_x = cen[0];
  cen_y = cen[1];
  view_height = height;
}

void GLVisualizer::DrawBackground() {
  // glDepthMask(GL_FALSE);
  // float max_y = 0.5f * view_height, min_y = -max_y;
  // float max_x = max_y * static_cast<float>(w) / static_cast<float>(h), min_x = -max_x;
  
  // // TODO: Switch shader programs to a textured one?

  // glDepthMask(GL_TRUE);
}

void GLVisualizer::DrawLine(Line& l) {
  // Lines are the same width, regardless of zoom level
  float lw = l.width / static_cast<float>(h) * view_height;
  if (l.segs.size() < 2) {
    return;
  }
  // Set the line color
  Eigen::Vector3f c_vec = ColorToRGB(l.c);
  glUniform3f(glGetUniformLocation(shaderProgram, "color"), c_vec[0], c_vec[1], c_vec[2]);

  Eigen::Vector2f p1(l.segs[0][0], l.segs[0][1]), p2; //, p_end1 = p1, p_end2 = p2;
  for (size_t i = 1; i < l.segs.size(); ++i) {
    p2 = Eigen::Vector2f(l.segs[i][0], l.segs[i][1]);
    if ((p1 - p2).norm() == 0.0f) {
      // Don't Draw this segment
      continue;
    }
    // First, compute the four endpoints
    Eigen::Vector2f v = (p2 - p1).normalized(), v_perp = Eigen::Vector2f(-v[1], v[0]);
    Eigen::Vector2f tl = p1 + lw * v_perp, bl = p1 - lw * v_perp;
    Eigen::Vector2f tr = p2 + lw * v_perp, br = p2 - lw * v_perp;

    // TODO: Draw a sort of cap?

    // Draw CCW:
    // bl, br, tl, tr
    line_buffer[0] = bl.x();
    line_buffer[1] = bl.y();
    line_buffer[2] = tl.x();
    line_buffer[3] = tl.y();
    line_buffer[4] = br.x();
    line_buffer[5] = br.y();
    line_buffer[6] = tr.x();
    line_buffer[7] = tr.y();

    glBufferSubData(GL_ARRAY_BUFFER, 0, 4 * SEG_SIZE * sizeof(float), line_buffer);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    // p_end1 = tr; p_end2 = br;
    p1 = p2;
  }
}

void GLVisualizer::DrawLines() {
  perm_line_lock.lock();
  for (auto& line : perm_lines) {
    DrawLine(line);
  }
  perm_line_lock.unlock();
  temp_line_lock.lock();
  for (auto& line : temp_lines) {
    DrawLine(line);
  }
  temp_line_lock.unlock();
}

void GLVisualizer::Draw() {
	glfwGetWindowSize(wind, &w, &h);
	glViewport(0,0,w,h);
  float max_y = 0.5f * view_height, min_y = -max_y;
  float max_x = max_y * static_cast<float>(w) / static_cast<float>(h), min_x = -max_x;
  proj = glm::ortho(min_x + cen_x, max_x + cen_x, min_y + cen_y, max_y + cen_y, 1.0f, 4.0f); // near, far
	
  view = glm::lookAt(
    glm::vec3(0.0f,0.0f,2.0f),  //Cam Position
		glm::vec3(0.0f, 0.0f, 0.0f), //Look at point
		glm::vec3(0.0f, 1.0f, 0.0f)); //Up
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setActiveProgram(shaderProgram);

  DrawBackground();
  DrawLines();

	glDisable(GL_BLEND);
}

void GLVisualizer::PauseFigure(double pause_time_s) {
  if (pause_time_s < 0) {
    while (running) {
      handleEvents();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } else {
    std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int>(pause_time_s * 1e9)));
  }
}

GLVisualizer::~GLVisualizer() {
  running = false;
  if (drawing_thread.joinable()) {
    drawing_thread.join();
  }

  // Do cleanup
  delete[] line_buffer;
  glDeleteBuffers(1, vbo);

	glfwDestroyWindow(wind);
	glfwTerminate();
}

void GLVisualizer::setActiveProgram(GLint program) {
	glUseProgram(program);
	
	GLint uniView = glGetUniformLocation(program, "view");
	glUniformMatrix4fv(uniView, 1, GL_FALSE, glm::value_ptr(view));
	
	GLint uniProj = glGetUniformLocation(program, "proj");
	glUniformMatrix4fv(uniProj, 1, GL_FALSE, glm::value_ptr(proj));
	
	//Tell OpenGL how to set shader input 
	GLint posAttrib = glGetAttribLocation(program, "pos");
	glVertexAttribPointer(posAttrib, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(posAttrib);
	// GLint texAttrib = glGetAttribLocation(program, "tex_coord");
	// glVertexAttribPointer(texAttrib, 2, GL_FLOAT, GL_FALSE, PT_SIZE*sizeof(float), (void*)(3*sizeof(float)));
	// glEnableVertexAttribArray(texAttrib);
}
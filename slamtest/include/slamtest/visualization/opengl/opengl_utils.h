#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define PI 3.141592f

#define GLM_FORCE_RADIANS
#define GLM_FORCE_SWIZZLE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/string_cast.hpp>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <string>
#include <vector>
#include <fstream>

const bool DEBUG_ON = true;

GLFWwindow* GL_GLFW_init(const char* window_name);
void GL_GLFW_del();

float rand01();

char* file_read(const char* filename);
void print_log(GLuint object);
GLuint create_shader(const char* filename, GLenum type);

GLuint InitShader(const char* vShaderFileName, const char* fShaderFileName, const char* gShaderFileName);
GLuint InitShader(const char* vShaderFileName, const char* fShaderFileName);

int loadModel(std::string fName, float *&model1);
// GLuint loadTexture(const char* file, int &width, int &height);
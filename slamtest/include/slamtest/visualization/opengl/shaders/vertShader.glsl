#version 150 core

in vec2 pos;

uniform mat4 view;
uniform mat4 proj;

void main() {
    gl_Position = proj * view * vec4(pos, 0.0,1.0);
}

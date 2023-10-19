#version 450 core            // minimal GL version support expected from the GPU

// input vertex data, different for all executions of this shader
layout(location=0) in vec3 vPositionModel;

// transformation bringing the scene into light's point of view
uniform mat4 depthMVP;
uniform mat4 shadowModel;

void main() {
  //gl_Position = depthMVP*vec4(vPositionModel, 1);
  gl_Position = depthMVP * shadowModel * vec4(vPositionModel, 1);
}
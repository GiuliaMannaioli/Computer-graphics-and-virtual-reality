#version 450 core            // minimal GL version support expected from the GPU

struct LightSource {
  vec3 position;
  vec3 color;
  float intensity;
  int isActive;
};

int numberOfLights = 3;
uniform LightSource lightSources[3];
uniform sampler2D depthTex[3];

float shadowMapCalculation(vec4 fPosLight, int i) {
  // normalized coordinates, [-1,1] range
  vec3 projCoord = fPosLight.xyz / fPosLight.w;

  // change to [0,1] range
  projCoord = projCoord * 0.5 + 0.5;

  // sample depth map, depth from the light's point of view
  float closestDepth = texture(depthTex[i], projCoord.xy).r;

  // get current depth at this fragment - distance between the light and the current fragment
  float currentDepth = projCoord.z - 0.005;

  // check whether current frag pos is in the shadow
  float shadow = currentDepth > closestDepth ? 1.0 : 0.0;

  return shadow;
}

struct Material {
  vec3 albedo;
  sampler2D normalMap;
  sampler2D colorTex;
  float useNormalMap;
};

uniform Material material;

uniform vec3 camPos;

in vec3 fPositionModel;
in vec3 fPosition;
in vec3 fNormal;
in vec2 fTexCoord;
in vec4 fPosLightSpace[3];

out vec4 colorOut; // shader output: the color response attached to this fragment

float pi = 3.1415927;

void main() {
  vec3 albedo;
  vec3 text;
  vec3 normal = normalize(fNormal);
  vec3 wo = normalize(camPos - fPosition); // unit vector pointing to the camera
  vec3 radiance = vec3(0, 0, 0);

  for(int i = 0; i < numberOfLights; ++i) {
    LightSource a_light = lightSources[i];

    // consider active lights only
    if(a_light.isActive == 1) {
      vec3 wi = normalize(a_light.position - fPosition); // unit vector pointing to the light
      vec3 Li = a_light.color * a_light.intensity;

      if (material.useNormalMap > 0) {
        // compute the normal to [-1, 1] and texture
        albedo = texture(material.normalMap, fTexCoord).rgb;
        text = texture(material.colorTex, fTexCoord).rgb;
      } else {
        albedo = material.albedo;
        text = vec3(1, 1, 1);
      }

      // calculate the shadows
      float shadow = shadowMapCalculation(fPosLightSpace[i], i);

      radiance += Li * (1.0 - shadow) * albedo * text * max(dot(normal, wi), 0);
    }
  }

  colorOut = vec4(radiance, 1.0); // build an RGBA value from an RGB one
}
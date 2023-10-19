#version 450 core            // minimal GL version support expected from the GPU

struct LightSource {
  vec3 position;
  vec3 color;
  float intensity;
  int isActive;
};

int numberOfLights = 3;
uniform LightSource lightSources[3];

// TODO: shadow maps

uniform sampler2D shadowMap0;
uniform sampler2D shadowMap1;
uniform sampler2D shadowMap2;


struct Material {
  vec3 albedo;
  int isBackwall; 
  sampler2D albedoTex; 
  sampler2D normalTex; 
};


uniform Material material;


uniform vec3 camPos;


in vec3 fPositionModel;
in vec3 fPosition;
in vec3 fNormal;
in vec2 fTexCoord;

in vec4 lightsPos[3];

out vec4 colorOut; // shader output: the color response attached to this fragment

float pi = 3.1415927;
float normal = 1.0;

// TODO: shadows


float shadow = 1.0;
float visibility = 1.0;

float ShadowCalculation(int i){

  float bias = 0.05;
  float closestDepth = 0;

  vec4 lightViewPos = lightsPos[i];
  vec3 projCoords = lightViewPos.xyz / lightViewPos.w;
  projCoords = projCoords * 0.5 + 0.5;


  if (i == 0)
    closestDepth = texture(shadowMap0, projCoords.xy).r;  
  else if (i == 1)
    closestDepth = texture(shadowMap1, projCoords.xy).r;
  else
    closestDepth = texture(shadowMap2, projCoords.xy).r;

  float currentDepth = projCoords.z -bias;

  

  if (closestDepth <  currentDepth){ 
    shadow -= 0.5;
    }

  
  return shadow;
}


void main() {
  vec3 radiance = vec3(0, 0, 0);
  vec3 n = normalize(fNormal);
  vec3 albedo = material.albedo;



  if (material.isBackwall == 1){
    albedo = texture(material.albedoTex, fTexCoord).rgb;
    normal = texture(material.normalTex, fTexCoord).r; 
   }
 

  vec3 wo = normalize(camPos - fPosition); // unit vector pointing to the camera

  for(int i=0; i<numberOfLights; ++i) {

    LightSource a_light = lightSources[i];

    if(a_light.isActive == 1) { // consider active lights only

      vec3 wi = normalize(a_light.position - fPosition); // unit vector pointing to the light
      vec3 Li = a_light.color*a_light.intensity;
      shadow = ShadowCalculation(i);
      	
     radiance *= normal;
     radiance += Li*albedo*max(dot(n, wi), 0); 

    }
  }

  colorOut = visibility*shadow*vec4(radiance, 1.0); // build an RGBA value from an RGB one
}

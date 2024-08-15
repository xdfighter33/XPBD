#version 410 core
out vec4 FragColor;

in vec2 TexCoords;
in vec3 Normal;  
in vec3 FragPos;  
in vec4 FragPosLightspace;


uniform sampler2D diffuseTexture;
uniform sampler2D shadowMap;
  
uniform vec3 lightPos; 
uniform vec3 viewPos; 
uniform vec3 lightColor;
uniform vec3 objectColor;


float ShadowCalculation(vec4 fragPosLightSpace)
{
    // perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    float closestDepth = texture(shadowMap, projCoords.xy).r; 
    // get depth of current fragment from light's perspective
    float currentDepth = projCoords.z;
    // check whether current frag pos is in shadow
    float shadow = currentDepth > closestDepth  ? 1.0 : 0.0;

    return shadow;
}  


void main()
{
    // ambient
    float ambientStrength = 0.75;
    vec3 ambient = ambientStrength * lightColor;
  	
    vec3 color = texture(diffuseTexture,TexCoords).rgb;


    // diffuse 
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    
    // specular
    float specularStrength = 100000.0;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);  
    vec3 halfwayDir = normalize(lightDir + viewDir);
    //phong MOdel 
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);

    float blin_spec = pow(max(dot(norm,halfwayDir),0),100);
    vec3 specular = lightColor * blin_spec;  
    
    float shadow = ShadowCalculation(FragPosLightspace);
    vec3 result = (ambient + diffuse + specular) * objectColor;

    vec3 lighting = (ambient + (1.0 - shadow) * (diffuse + specular)) * color;    
    FragColor = vec4(result, 1.0);
} 
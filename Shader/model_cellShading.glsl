#version 410 core

in vec3 FragPos;
in vec3 Normal;

out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform vec3 lightColor;
uniform vec3 objectColor;

vec3 calculateRimLighting(vec3 normal,vec3 viewDirection) {
    // Calculate the dot product between the surface normal and the view direction
    float rimFactor = dot(normalize(normal), normalize(viewDirection));
    
    // Apply threshold to rimFactora
    float rimThreshold = .5;

    rimFactor = clamp((rimFactor - rimThreshold) / (1.0 - rimThreshold), 0.0, 1.0);
    
    // Scale rimFactor to control intensity
    float rimIntensity = 0.25;
    rimFactor *= rimIntensity;
    
    // Combine rim lighting color with base color
    vec3 rimColor = vec3(1.0, 1.0, 1.0); // Rim lighting color (usually white)
    return rimColor * rimFactor;
}



void main()
{
    vec3 lightDir = normalize(lightPos - FragPos);
    vec3 norm = normalize(Normal);
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 rim_lighting = calculateRimLighting(norm, viewDir);

    // Ambient
    float ambientStrength = 0.25;
    vec3 ambient = ambientStrength * lightColor;
  	
    // Diffuse 
    float diff = max(dot(norm, lightDir), 0.0);
    float diff2 = max(dot(norm, lightDir), 0.0);
    vec4 color;
    if(diff2 > 0.5)
        color = vec4(1.0, 1.0, 1.0, 1.0);
    else if(diff2 > 0.0)
        color = vec4(0.33, 0.33, 0.33, 1.0);
    else
        color = vec4(0.0, 0.0, 0.0, 1.0);

    // Cel-shading step function
    if (diff > 0.8) diff = 1.0;
    else if (diff > 0.5) diff = 0.6;
    else if (diff > 0.2) diff = 0.4;
    else diff = 0.2;
    
    vec3 diffuse = diff * lightColor;
    
    // Specular
    float specularStrength = 100.5;
    vec3 reflectDir = reflect(-lightDir, norm);  
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(norm, halfwayDir), 0.0), 32);
    
    // Cel-shading step function for specular
  //  spec = (spec > 0.5) ? 1.0 : 0.0;
    
    vec3 specular = spec * lightColor;
        
    // Combine results
    vec3 result = (ambient + diffuse + specular) * objectColor;
    
    // Simple outline
    float outline = dot(viewDir, norm);
    outline = smoothstep(0.2, 0.4, outline);
    
    // Apply outline
    // result = mix(vec3(0.0), result, outline);

    // Add rim lighting
    result += rim_lighting;
    
    FragColor = vec4(result, 1.0);
}

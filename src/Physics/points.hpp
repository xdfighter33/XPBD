#pragma once 

#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/matrix_decompose.hpp>
struct Particle {
    glm::vec3 position;
    glm::vec3 old_position;
    glm::vec3 velocity;
    float invMass;

};

struct Constraint{

size_t p1, p2;
float restLength;
float stiffness;
};

struct XPBD_Constraints{
  
    size_t p1, p2;
    float restLength;
    float stiffness; // Inverse stiffness needs to be between 0 and 1 (0 for infinitely stiff objects ) // This is for the distance cosntraint 
    float volume_stifness; 
    float lambda; // For XPBD
};


struct XPBD_VolumeConstraints{

    size_t p1, p2, p3;
    float volume_stiffness; 
    float lambda; 

};


struct BendingConstraint {
    size_t p1, p2, p3, p4;  // Indices of the four particles involved
    float restAngle;        // The initial dihedral angle
    float stiffness;        // How strictly to enforce this constraint (0-1)
};

struct XPBD_BendingConstraint {
    size_t p1, p2, p3, p4;
    float restAngle;
    float stiffness;
    float lambda;
};

struct XPBD_ShearingConstraint {
    size_t p1, p2, p3;
    float restArea;
    float stiffness;
    float lambda;
};


float calculateDihedralAngle(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& p4) {
    glm::vec3 n1 = glm::normalize(glm::cross(p2 - p1, p3 - p1));
    glm::vec3 n2 = glm::normalize(glm::cross(p2 - p4, p3 - p4));
    float d = glm::dot(n1, n2);
    d = glm::clamp(d, -1.0f, 1.0f);
    return std::acos(d);
}


float calculateTriangleContribution(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3) {
    return glm::dot(p1, glm::cross(p2 - p1, p3 - p1));
}



glm::mat3 normalizeMatrix(const glm::mat3& A) {
    glm::mat3 Q;
    for (int i = 0; i < 3; ++i) {
        float norm = glm::length(A[i]);
        Q[i] = A[i] / (norm > 0.0f ? norm : 1.0f);
    }
    return Q;
}




glm::mat3 Polar_decomp(glm::mat3 v){
    //V is your input matrix 

    glm::mat3 Q; // Rotation Matrix 
    glm::mat3 P; // Streching Matirx 
    
    glm::vec3 scale;
    glm::quat rotation;
    glm::vec3 translation;
    glm::vec3 skew;
    glm::vec4 perspective;

    bool success = glm::decompose(glm::mat4(v), scale, rotation, translation, skew, perspective);


    
}
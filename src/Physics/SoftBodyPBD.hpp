
#pragma once 
#define GLM_ENABLE_EXPERIMENTAL
#include "points.hpp"
#include <glm/glm.hpp>
#include <glm/gtx/polar_coordinates.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_factorisation.hpp>
#include <glm/gtx/norm.hpp>
#include "../Model_loading/Models.hpp"
#include <vector>


class SoftBodyPBD{
    private:
    std::vector<glm::vec3> initialRelativePositions;
    std::vector<Particle> particles;
    std::vector<Constraint> constraints;
    std::vector<BendingConstraint> B_constraints;
    std::vector<glm::vec3> qVector; // for shape matching
    std::vector<glm::vec3> updatedNormals;
    std::vector<glm::vec3> originalNormals;
    glm::vec3 centerOfMass;

    float stiffness;
    float initialVolume;
    float volumeStiffness;



    float restVolume;
    float currentVolume;
    float pressureStiffness;
    std::vector<glm::vec3> surfaceNormals;


    public: 
    SoftBodyPBD(const Model& model){
        initializeFromModel(model); 
    };


    void translate(const glm::vec3& pos){
        for(auto& p : particles){
            p.position += pos;
            p.old_position += pos;
        }
        centerOfMass += pos;
    }


    void setPosition(const glm::vec3& newPosition) {
        glm::vec3 translation = newPosition - centerOfMass;
        translate(translation);
        }

    void initializeFromModel(const Model& model) {
        // Initialize particles
        const auto& vertices = model.getVertices();
        particles.reserve(vertices.size());
        for (const auto& vertex : vertices) {
            particles.push_back({
                vertex.Position,
                vertex.Position,
                glm::vec3(0.0f),
                1.0f // inverse mass
            });
        }

    updatedNormals.reserve(vertices.size());
    originalNormals.reserve(vertices.size());
    for (const auto& vertex : vertices) {
        updatedNormals.push_back(vertex.Normal);
        originalNormals.push_back(vertex.Normal);
    }


    // INdices 
       
        const auto& indices = model.getIndices();
         for (size_t i = 0; i < indices.size(); i += 3) {
            for (int j = 0; j < 3; ++j) {
                size_t i1 = indices[i + j];
                size_t i2 = indices[i + (j + 1) % 3];
                float restLength = glm::distance(particles[i1].position, particles[i2].position);
                constraints.push_back({i1, i2, restLength,0.1f});
                
            }
        }
    

    initializeBendingConstraints(model);

    restVolume = calculateMeshVolume();
  
    pressureStiffness = 0.5f;  // Adjust this value as needed
    surfaceNormals.resize(particles.size(), glm::vec3(0.0f));

    //Volume calculations
    volumeStiffness = 1.0f;

    initialVolume = 0.0f;
    for (size_t i = 0; i < indices.size(); i += 3) {
        glm::vec3 p1 = particles[indices[i]].position;
        glm::vec3 p2 = particles[indices[i+1]].position;
        glm::vec3 p3 = particles[indices[i+2]].position;
        glm::vec3 normal = glm::cross(p2 - p1, p3 - p1);
        initialVolume += glm::dot(p1, normal) / 6.0f;
    }


      currentVolume = initialVolume;


        // Match the shape of the data (Get Initial distance constraint)
        centerOfMass = glm::vec3(0.0f);
         glm::vec3 initialCenterOfMass = glm::vec3(0.0f);
        for (const auto& p : particles) {
            centerOfMass += p.position;
            initialCenterOfMass += p.position;
        }
        centerOfMass /= particles.size();
        initialCenterOfMass /= particles.size();

        qVector.resize(particles.size());
        for (size_t i = 0; i < particles.size(); ++i) {
            qVector[i] = particles[i].position - centerOfMass;
        }

       

    // Store initial relative positions
        initialRelativePositions.resize(particles.size());
         for (size_t i = 0; i < particles.size(); ++i) {
            initialRelativePositions[i] = particles[i].position - initialCenterOfMass;
    }


    }


    void update(float dt){
        const int solveIt = 80;
        const float damp = 0.99f;

        // Add Forces 
        for(auto& p : particles){
                if(p.invMass > 0){
                    p.velocity += glm::vec3(0,-9.8f,0) * dt;
            }
        }

        // Set old position to new position
        // Apply velocity 
        for(auto& p : particles){
            p.old_position = p.position;
            p.position += p.velocity * dt;
        }

           
        // Apply  distance constraint
        for(int i{0}; i < solveIt; ++i){
            
            for(const auto& c : constraints){
                glm::vec3 delta = particles[c.p2].position - particles[c.p1].position;
                float deltaLength = glm::length(delta);

                if(deltaLength == 0.0f) continue;

                float correction = (deltaLength - c.restLength);
                auto correctionVector = correction;


            
                float w1 = particles[c.p1].invMass /  ( particles[c.p1].invMass + particles[c.p2].invMass );
                float w2 = particles[c.p2].invMass /  ( particles[c.p1].invMass + particles[c.p2].invMass );
            
                auto n = delta / deltaLength;


                // particles[c.p1].position += particles[c.p1].invMass * correctionVector;
                // particles[c.p2].position -= particles[c.p2].invMass * correctionVector;
                
                particles[c.p1].position += w1 * c.stiffness * correctionVector * n;
                particles[c.p2].position -= w2 * c.stiffness * correctionVector * n;

            }
           solveBendingConstraints();
        //    solveVolumeConstraint();
            
        }
    

        //Ground Constraints 
        float groundY = 0.35f;
        float restitution = 0.5f;
      for (auto& p : particles) {
        if (p.position.y < groundY) {
            // Collision response
            float penetration = groundY - p.position.y;
            p.position.y = groundY + penetration * restitution;

            // Reflect velocity with damping
            if (p.velocity.y < 0) {
                p.velocity.y = -p.velocity.y * restitution;
            }

            // Apply friction
            const float friction = 0.0f;
            glm::vec2 horizontalVelocity(p.velocity.x, p.velocity.z);
            if (glm::length(horizontalVelocity) > 0.001f) {
                glm::vec2 frictionForce = -glm::normalize(horizontalVelocity) * friction;
                p.velocity.x += frictionForce.x * dt;
                p.velocity.z += frictionForce.y * dt;
            }
        }
    }



        glm::vec3 newCenterOfMass = glm::vec3(0);
        for(const auto& p : particles){
            newCenterOfMass += p.position;
        }


        // Calculate stifness of shape 
        newCenterOfMass /= particles.size();

        glm::mat3 A(0), R;
        for (size_t i = 0; i < particles.size(); ++i) {
            glm::vec3 p = particles[i].position - newCenterOfMass;
            A += glm::outerProduct(p, qVector[i]);
        }

        //Rotation Matrix (THIS IS hideious please fix )
        // Extract rotation using QR decomposition
        glm::mat3 Q;
        glm::vec3 S; // We won't use this, but it's required for the function call
        glm::qr_decompose(A, Q, R);


        // Q is our rotation matrix
        //Helps keey the shape 
        glm::mat3 rotationMatrix = Q;

        for (size_t i = 0; i < particles.size(); ++i) {
            glm::vec3 goal = newCenterOfMass + Q * qVector[i];
           particles[i].position = particles[i].position + (goal - particles[i].position) * 0.75f;
            }

 

        // Dampen Particles 
        for (auto& p : particles) {
            if (p.invMass > 0) {
                p.velocity = (p.position - p.old_position) / dt;
                p.velocity *= damp;
            }
            }

           recalculateNormals();

    }

void reset(const glm::vec3& newCenterOfMass) {
    // Reset particles to their initial relative positions
    for (size_t i = 0; i < particles.size(); ++i) {
        particles[i].position = newCenterOfMass + initialRelativePositions[i];
        particles[i].old_position = particles[i].position;
        particles[i].velocity = glm::vec3(0.0f);
    }

    // Reset center of mass
    centerOfMass = newCenterOfMass;

    // Reset qVector for shape matching
    for (size_t i = 0; i < particles.size(); ++i) {
        qVector[i] = initialRelativePositions[i];
    }
}

void reset() {
    reset(centerOfMass);
}



void solveVolumeConstraint() {
    float currentVolume = 0.0f;
    std::vector<glm::vec3> gradients(particles.size(), glm::vec3(0.0f));

    // Calculate current volume and gradients
    for (size_t i = 0; i < constraints.size(); i += 3) {
        size_t i1 = constraints[i].p1;
        size_t i2 = constraints[i].p2;
        size_t i3 = constraints[i+1].p2;

        glm::vec3& p1 = particles[i1].position;
        glm::vec3& p2 = particles[i2].position;
        glm::vec3& p3 = particles[i3].position;

        glm::vec3 normal = glm::cross(p2 - p1, p3 - p1);
        float signedVolume = glm::dot(p1, normal) / 6.0f;
        currentVolume += signedVolume;

        gradients[i1] += glm::cross(p2 - p3, normal) / 6.0f;
        gradients[i2] += glm::cross(p3 - p1, normal) / 6.0f;
        gradients[i3] += glm::cross(p1 - p2, normal) / 6.0f;
    }

    float volumeError = currentVolume - initialVolume;

    // Calculate lambda
    float totalGradientMagnitude = 0.0f;
    for (size_t i = 0; i < particles.size(); ++i) {
        totalGradientMagnitude += particles[i].invMass * glm::length2(gradients[i]);
    }

    if (totalGradientMagnitude > 0.0f) {
        float lambda = -volumeError / (totalGradientMagnitude + 1e-6f);

        // Apply volume correction
        for (size_t i = 0; i < particles.size(); ++i) {
            glm::vec3 correction = volumeStiffness * lambda * particles[i].invMass * gradients[i];
            particles[i].position += correction;
        }
    }
}


void recalculateNormals() {
    // Reset normals
    std::fill(updatedNormals.begin(), updatedNormals.end(), glm::vec3(0.0f));

    // Calculate face normals and accumulate
    for (size_t i = 0; i < constraints.size(); i += 3) {
        size_t i1 = constraints[i].p1;
        size_t i2 = constraints[i].p2;
        size_t i3 = constraints[(i+1)%3].p2;

        glm::vec3 v1 = particles[i2].position - particles[i1].position;
        glm::vec3 v2 = particles[i3].position - particles[i1].position;
        glm::vec3 faceNormal = glm::cross(v1, v2);

        updatedNormals[i1] += faceNormal;
        updatedNormals[i2] += faceNormal;
        updatedNormals[i3] += faceNormal;
    }

    // Normalize
    for (auto& normal : updatedNormals) {
        normal = glm::normalize(normal);
    }
}



const std::vector<Particle>& getParticles() const {
        return particles;
    }




const std::vector<glm::vec3>& getUpdatedNormals() const {
    return updatedNormals;
}

void initializeBendingConstraints(const Model& model) {
    const auto& indices = model.getIndices();
    std::vector<std::vector<size_t>> adjacentTriangles(particles.size());

    // Build adjacency list
    for (size_t i = 0; i < indices.size(); i += 3) {
        for (int j = 0; j < 3; ++j) {
            adjacentTriangles[indices[i + j]].push_back(i / 3);
        }
    }

    // Find adjacent triangles and create bending constraints
    for (size_t i = 0; i < indices.size(); i += 3) {
        for (int j = 0; j < 3; ++j) {
            size_t p1 = indices[i + j];
            size_t p2 = indices[i + (j + 1) % 3];

            // Find a triangle that shares this edge
            for (size_t t1 : adjacentTriangles[p1]) {
                for (size_t t2 : adjacentTriangles[p2]) {
                    if (t1 != t2 && t1 != i / 3 && t2 != i / 3) {
                        size_t p3 = indices[i + (j + 2) % 3];
                        size_t p4 = indices[3 * t2];
                        if (p4 == p1 || p4 == p2) p4 = indices[3 * t2 + 1];
                        if (p4 == p1 || p4 == p2) p4 = indices[3 * t2 + 2];

                        float restAngle = calculateDihedralAngle(
                            particles[p1].position, particles[p2].position,
                            particles[p3].position, particles[p4].position);

                        B_constraints.push_back({p1, p2, p3, p4, restAngle, 1.0f});
                    }
                }
            }
        }
    }
}

void solveBendingConstraints() {
    for (const auto& constraint : B_constraints) {
        Particle& p1 = particles[constraint.p1];
        Particle& p2 = particles[constraint.p2];
        Particle& p3 = particles[constraint.p3];
        Particle& p4 = particles[constraint.p4];

        glm::vec3 n1 = glm::normalize(glm::cross(p2.position - p1.position, p3.position - p1.position));
        glm::vec3 n2 = glm::normalize(glm::cross(p2.position - p4.position, p3.position - p4.position));

        float d = glm::dot(n1, n2);
        d = glm::clamp(d, -1.0f, 1.0f);
        float angle = std::acos(d);

        float angleDiff = angle - constraint.restAngle;

        glm::vec3 q3 = (glm::cross(p2.position - p1.position, n1) + glm::cross(n1, p3.position - p1.position)) / glm::length(glm::cross(p3.position - p1.position, p2.position - p1.position));
        glm::vec3 q4 = (glm::cross(p2.position - p4.position, n2) + glm::cross(n2, p3.position - p4.position)) / glm::length(glm::cross(p3.position - p4.position, p2.position - p4.position));
        glm::vec3 q2 = -(q3 + q4);
        glm::vec3 q1 = -q2 - q3 - q4;

        float sum = p1.invMass + p2.invMass + p3.invMass + p4.invMass;
        if (sum == 0.0f) continue;

        float factor = -constraint.stiffness * angleDiff / sum;

        p1.position += factor * p1.invMass * q1;
        p2.position += factor * p2.invMass * q2;
        p3.position += factor * p3.invMass * q3;
        p4.position += factor * p4.invMass * q4;
    }
}


float calculateMeshVolume() {
    float volume = 0.0f;
    for (size_t i = 0; i < constraints.size(); i += 3) {
        const glm::vec3& p1 = particles[constraints[i].p1].position;
        const glm::vec3& p2 = particles[constraints[i].p2].position;
        const glm::vec3& p3 = particles[constraints[i+1].p2].position;
        volume += glm::dot(p1, glm::cross(p2 - p1, p3 - p1));
    }
    return std::abs(volume) / 6.0f;
}

void updateSurfaceNormals() {
    std::fill(surfaceNormals.begin(), surfaceNormals.end(), glm::vec3(0.0f));

    for (size_t i = 0; i < constraints.size(); i += 3) {
        const glm::vec3& p1 = particles[constraints[i].p1].position;
        const glm::vec3& p2 = particles[constraints[i].p2].position;
        const glm::vec3& p3 = particles[constraints[i+1].p2].position;

        glm::vec3 normal = glm::cross(p2 - p1, p3 - p1);

        surfaceNormals[constraints[i].p1] += normal;
        surfaceNormals[constraints[i].p2] += normal;
        surfaceNormals[constraints[i+1].p2] += normal;
    }

    for (auto& normal : surfaceNormals) {
        normal = glm::normalize(normal);
    }
}


void preserveVolume(float dt) {
    currentVolume = calculateMeshVolume();
    float volumeDifference = restVolume - currentVolume;

    updateSurfaceNormals();

    float pressureMagnitude = pressureStiffness * volumeDifference / restVolume;

    for (size_t i = 0; i < particles.size(); ++i) {
        glm::vec3 pressureForce = pressureMagnitude * surfaceNormals[i];
        particles[i].velocity += pressureForce * dt * particles[i].invMass;
    }
}




};



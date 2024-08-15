#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include "points.hpp"
#include <glm/glm.hpp>
#include <glm/gtx/polar_coordinates.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_factorisation.hpp>
#include <glm/gtx/polar_coordinates.hpp>
#include <glm/gtx/norm.hpp>
#include "../Model_loading/Models.hpp"
#include <vector>

class SoftBodyXPBD{

private:
        std::vector<Particle> particles;
        std::vector<XPBD_Constraints> Constraints;
        std::vector<glm::vec3> initialPositions;
    std::vector<XPBD_BendingConstraint> bending_constraints;
        std::vector<XPBD_ShearingConstraint> shearing_constraints;
        std::vector<XPBD_VolumeConstraints> volume_constraints; 


        //Plastic Deformations 

        float PlasticThreshHold = 0.5f;
        float PlasricCreep = 0.1f;
        std::vector<glm::vec3> PlasticOffsets; 



        // Misc 
        float dt;
        int solveIt; //Solver Iterations 
        float initialVolume;
        float volumeStiffness = 0.0f;
        float inverse_distance;
        float temp_volume;

        //shape matching 
    glm::vec3 restCenterOfMass;
    std::vector<glm::vec3> restPositions;
    float shapeFactor = 0.5f;


public: 
    SoftBodyXPBD(const Model& model, float timeStep, float Iterations)
        : dt(timeStep), solveIt(Iterations) {
        initializeFromModel(model);
  
    
        initialVolume = calculateMeshVolume();

    }

    
    
    void initializeShapeMatching() {
    // Calculate rest center of mass
    glm::vec3 com(0.0f);
    for (const auto& p : particles) {
        com += p.position;
    }
    com /= static_cast<float>(particles.size());

    // Store rest positions relative to center of mass
    restPositions.resize(particles.size());
    for (size_t i = 0; i < particles.size(); ++i) {
        restPositions[i] = particles[i].position - com;
    }

    restCenterOfMass = com;
}

    void initalizeVolumeConstraints(){

        for(size_t  i = 0; i < Constraints.size(); i+= 3){
            size_t p1 = Constraints[i].p1;
            size_t p2 = Constraints[i].p2;
            size_t p3 = Constraints[i+1].p2;

            volume_constraints.push_back({p1,p2,p3,temp_volume});



        }
    }
    void initializeShearingConstraints() {
    for (size_t i = 0; i < Constraints.size(); i += 3) {
        size_t p1 = Constraints[i].p1;
        size_t p2 = Constraints[i].p2;
        size_t p3 = Constraints[i+1].p2;

        glm::vec3 edge1 = particles[p2].position - particles[p1].position;
        glm::vec3 edge2 = particles[p3].position - particles[p1].position;
        float restArea = 0.5f * glm::length(glm::cross(edge1, edge2));

        shearing_constraints.push_back({p1, p2, p3, restArea, 0.0f, 0.0f});
    }
}

    void initializeBendingConstraints() {
    for (size_t i = 0; i < Constraints.size(); i += 3) {
        for (int j = 0; j < 3; ++j) {
            size_t e1 = Constraints[i + j].p1;
            size_t e2 = Constraints[i + j].p2;
            size_t e3 = Constraints[i + (j + 1) % 3].p2;
            size_t e4 = Constraints[i + (j + 2) % 3].p2;

            glm::vec3 n1 = glm::cross(particles[e2].position - particles[e1].position,
                                      particles[e3].position - particles[e1].position);
            glm::vec3 n2 = glm::cross(particles[e2].position - particles[e4].position,
                                      particles[e3].position - particles[e4].position);

            float restAngle = std::acos(glm::dot(n1, n2) / (glm::length(n1) * glm::length(n2)));

            bending_constraints.push_back({e1, e2, e3, e4, restAngle, 0.1f, 0.0f});
        }
    }
}


void solveVolumeConstraint(){

for(auto& c : Constraints){

    Particle& p1 = particles[c.p1];
    Particle& p2 = particles[c.p2];
    Particle& p3 = particles[c.p2 + 1];

    float currentVol = calculateMeshVolume();

    float volume_diff = currentVol - initialVolume;

    if(volume_diff <= 0)
        continue;

    // Compute denom 

 



}


}
void preserveVolume() {
    float currentVolume = calculateMeshVolume();
    std::vector<glm::vec3> gradients(particles.size(), glm::vec3(0.0f));
    
    glm::vec3 centroid(0.0f);
    for (const auto& p : particles) {
        centroid += p.position;
    }
    centroid /= static_cast<float>(particles.size());

    for (size_t i = 0; i < Constraints.size(); i += 3) {
        size_t i1 = Constraints[i].p1;
        size_t i2 = Constraints[i].p2;
        size_t i3 = Constraints[i+1].p2;

        const glm::vec3& p1 = particles[i1].position - centroid;
        const glm::vec3& p2 = particles[i2].position - centroid;
        const glm::vec3& p3 = particles[i3].position - centroid;

        glm::vec3 normal = glm::cross(p2 - p1, p3 - p1);
        float area = glm::length(normal);
        normal = glm::normalize(normal);

        gradients[i1] += area * normal / 3.0f;
        gradients[i2] += area * normal / 3.0f;
        gradients[i3] += area * normal / 3.0f;
    }

    float volumeDifference = currentVolume - initialVolume;
    float sum_gradient_squared = 0.0f;




    for (size_t i = 0; i < particles.size(); ++i) {
        sum_gradient_squared += particles[i].invMass * glm::length2(gradients[i]);
    }

    float alpha = volumeStiffness / ( dt * dt);
    float lambda = 0.0;
    float deltaLambda = -(volumeDifference - alpha + lambda)  / (sum_gradient_squared + alpha);
    lambda += deltaLambda;


    
    for (size_t i = 0; i < particles.size(); ++i) {
        particles[i].position += particles[i].invMass * deltaLambda * gradients[i];
    }
}

    void initializeFromModel(const Model& model){

        const auto& vertices = model.getVertices();
        particles.reserve(vertices.size());

        for(auto& vertex : vertices){
        particles.push_back({
            vertex.Position,
            vertex.Position,
            glm::vec3(0.0f),
            1.0f // Inverse mass its 1 / n 
        });    
        }


        const auto& indices = model.getIndices();
         for (size_t i = 0; i < indices.size(); i += 3) {
            for (int j = 0; j < 3; ++j) {
                size_t i1 = indices[i + j];
                size_t i2 = indices[i + (j + 1) % 3];
                float restLength = glm::distance(particles[i1].position, particles[i2].position);
                Constraints.push_back({
                    i1, 
                    i2,
                    restLength,
                    0.0f,
                    0.0f
                });
                
            }
        }


        initialPositions.reserve(vertices.size());
            for (const auto& vertex : vertices) {
                initialPositions.push_back(vertex.Position);
        }



        initializeBendingConstraints();
        initializeShearingConstraints();
        initializeShapeMatching();
        for (auto& constraint : Constraints) {
            constraint.lambda = 0.0f;
        }


        PlasticOffsets.resize(particles.size(), glm::vec3(0.0f));

    }


        void update() {

            ApplyGravity();
            predictPosition();
        for(int i = 0; i < solveIt; i++){
            stretchingConstraint();
            // solveShearingConstraints();
            preserveVolume();
            solveShapeMatching();
            }

        GroundConstraint();
        updateVelocitiesAndPositions();
        }


  
    void ApplyGravity(){
        for(auto& p : particles){
             if(p.invMass > 0){
                 p.velocity += glm::vec3(0,-9.8f,0) * dt;
        }}
    }

    void predictPosition(){
        for(auto& p : particles){
            p.old_position = p.position;
            p.position += p.velocity * dt;
        }
    }
    

     void updateVelocitiesAndPositions() {
        float alpha = 0.99f; // Damping factor
        for (auto& p : particles) {
            if (p.invMass > 0) {
                p.velocity = alpha * (p.position - p.old_position) / dt;
                p.old_position = p.position;
            }
        }
    }
 

void solveShapeMatching() {
    // Calculate the current center of mass
    glm::vec3 com(0.0f);
    for (const auto& p : particles) {
        com += p.position;
    }
    com /= static_cast<float>(particles.size());

    // Calculate the matrix A for the shape matching
    glm::mat3 A(0.0f);
    for (size_t i = 0; i < particles.size(); ++i) {
        glm::vec3 q = particles[i].position - com;
        glm::vec3 p = restPositions[i];
        A += glm::outerProduct(q, p);
    }

    // Orthonormalize matrix A to find the rotation matrix R
    glm::mat3 R = glm::mat3(1.0f);
    glm::vec3 col0 = glm::normalize(glm::vec3(A[0][0], A[1][0], A[2][0]));
    glm::vec3 col1 = glm::normalize(glm::vec3(A[0][1], A[1][1], A[2][1]) - glm::dot(col0, glm::vec3(A[0][1], A[1][1], A[2][1])) * col0);
    glm::vec3 col2 = glm::normalize(glm::vec3(A[0][2], A[1][2], A[2][2]) - glm::dot(col0, glm::vec3(A[0][2], A[1][2], A[2][2])) * col0 - glm::dot(col1, glm::vec3(A[0][2], A[1][2], A[2][2])) * col1);

    R[0] = glm::vec4(col0, 0.0f);
    R[1] = glm::vec4(col1, 0.0f);
    R[2] = glm::vec4(col2, 0.0f);

    // Apply shape matching
    for (size_t i = 0; i < particles.size(); ++i) {
        glm::vec3 goal = com + R * restPositions[i];
        glm::vec3 displacement = goal - particles[i].position;
        particles[i].position += shapeFactor * displacement;
    }
}




void solveShearingConstraints() {
    for (auto& c : shearing_constraints) {
        Particle& p1 = particles[c.p1];
        Particle& p2 = particles[c.p2];
        Particle& p3 = particles[c.p3];

        glm::vec3 edge1 = p2.position - p1.position;
        glm::vec3 edge2 = p3.position - p1.position;
        glm::vec3 normal = glm::cross(edge1, edge2);
        float currentArea = glm::length(normal);

        if (currentArea == 0.0f) continue;

        normal = glm::normalize(normal);

        glm::vec3 grad1 = glm::cross(edge2, normal);
        glm::vec3 grad2 = glm::cross(normal, edge1);
        glm::vec3 grad3 = -grad1 - grad2;

        float constraint = currentArea - c.restArea;
        float sum = p1.invMass * glm::length2(grad1) +
                    p2.invMass * glm::length2(grad2) +
                    p3.invMass * glm::length2(grad3);

        float alpha = c.stiffness / (dt * dt);
        float deltaLambda = -(constraint + alpha * c.lambda) / (sum + alpha);
        c.lambda += deltaLambda;

        p1.position += p1.invMass * deltaLambda * grad1;
        p2.position += p2.invMass * deltaLambda * grad2;
        p3.position += p3.invMass * deltaLambda * grad3;
    }
}



void DistanceConstraints() {
            for (auto& c : Constraints) {
                Particle& p1 = particles[c.p1];
                Particle& p2 = particles[c.p2];

                glm::vec3 delta = p2.position - p1.position;
                float deltaLength = glm::length(delta);
                if(deltaLength == 0.0f) continue;

                glm::vec3 n = delta / deltaLength;

                float alpha = c.stiffness / (dt * dt);
                float denominator = (p1.invMass + p2.invMass) + alpha;
                float dLambda = (-deltaLength + c.restLength  - alpha * c.lambda) / denominator;

                c.lambda += dLambda;


                glm::vec3 dp1 = -p1.invMass * dLambda * n;
                glm::vec3 dp2 = p2.invMass * dLambda * n;
                p1.position += dp1;
                p2.position += dp2;
            }
    }

// Bending Constraints 

void BendingConstraints(){
    for(auto& c : bending_constraints){
        Particle& p1 = particles[c.p1];
        Particle& p2 = particles[c.p2];
        Particle& p3 = particles[c.p3];
        Particle& p4 = particles[c.p4];


        
        
    }

}
// Ground  constraint


void stretchingConstraint(){

    for(auto& c : Constraints){
            Particle& p1 = particles[c.p1];
            Particle& p2 = particles[c.p2];

            glm::vec3 delta = p2.position - p1.position;
            float Delta_length = glm::length(delta); // caluclate the length of the delta and put this number into a float 

            if(Delta_length < 0) continue;

            glm::vec3 dir = delta / Delta_length;


            //Calculate the stretch value 
            float constraint = Delta_length - c.restLength;

            float alpha = c.stiffness / dt * dt; 

            float sum = p1.invMass + p2.invMass; 



            float deltaLambda = -(constraint + alpha * c.lambda) / (sum + alpha);
            c.lambda += deltaLambda;

            glm::vec3 correction = deltaLambda * dir;
            p1.position -= p1.invMass * correction;
            p2.position += p2.invMass * correction;


    }

}
void GroundConstraint() {
    float groundY = 0.35f;  // Adjust this to your ground level
    float restitution = 0.5f;  // Coefficient of restitution (0.0 to 1.0)
    float friction = 0.3f;  // Coefficient of friction

    for (auto& p : particles) {
        if (p.position.y < groundY) {
            // Normal collision response
            float penetration = groundY - p.position.y;
            p.position.y = groundY;

            // Apply impulse in the normal direction (bounce)
            float velocityAlongNormal = p.velocity.y;
            if (velocityAlongNormal < 0) {
                float j = -(1 + restitution) * velocityAlongNormal;
                p.velocity.y += j;
            }

            // Apply friction
            glm::vec2 tangentVelocity(p.velocity.x, p.velocity.z);
            float tangentSpeed = glm::length(tangentVelocity);

            if (tangentSpeed > 0.001f) {
                glm::vec2 frictionImpulse = -glm::normalize(tangentVelocity) * std::min(friction * penetration, tangentSpeed);
                p.velocity.x += frictionImpulse.x;
                p.velocity.z += frictionImpulse.y;
            }

            // Velocity damping for stability
            p.velocity *= 0.99f;
        }
    }
}


// Movement of particle

void setPosition(const glm::vec3& newPosition) {
    if (particles.empty()) return;

    // Calculate the current center of mass
    glm::vec3 centerOfMass(0.0f);
    for (const auto& p : particles) {
        centerOfMass += p.position;
    }
    centerOfMass /= static_cast<float>(particles.size());

    // Calculate the translation vector
    glm::vec3 translation = newPosition - centerOfMass;

    // Apply the translation to all particles
    for (auto& p : particles) {
        p.position += translation;
        p.old_position += translation;
    }
}

// Reset every constraint in the Array 
void SetDistaneConstraint(float t){

    for(auto& c  : Constraints){
        c.stiffness = t;
    }
}

void setShearingConstraints(float t){

    for(auto& c : shearing_constraints){
        c.stiffness = t;
    }
}

void setVolumeStifness(float t){
    for(auto& c : Constraints){
        c.volume_stifness = t;
    }
}


void setShapeFactor(float factor) {
    shapeFactor = glm::clamp(factor, 0.0f, 1.0f);
}




float calculateMeshVolume() {
    float volume = 0.0f;
    glm::vec3 centroid(0.0f);
    for (const auto& p : particles) {
        centroid += p.position;
    }
    centroid /= static_cast<float>(particles.size());

for (size_t i = 0; i < Constraints.size(); i += 3) {
    const glm::vec3& p1 = particles[Constraints[i].p1].position - centroid;
    const glm::vec3& p2 = particles[Constraints[i].p2].position - centroid;
    const glm::vec3& p3 = particles[Constraints[i+1].p2].position - centroid;
    volume += glm::dot(p1, glm::cross(p2 - p1, p3 - p1)) / 6.0f;
}
    return std::abs(volume);
}

void set_vol(float t){
    volumeStiffness = t;
}

void reset() {
    if (particles.size() != initialPositions.size()) return;

    for (size_t i = 0; i < particles.size(); ++i) {
        particles[i].position = initialPositions[i];
        particles[i].old_position = initialPositions[i];
        particles[i].velocity = glm::vec3(0.0f);
    }


        initializeShapeMatching();

        
    // Reset lambdas in constraints
    for (auto& c : Constraints) {
        c.lambda = 0.0f;
    }
}


void resetToPosition(const glm::vec3& newPosition) {
    reset();
    setPosition(newPosition);
}


const std::vector<Particle>& getParticles() const {
        return particles;
    }


};
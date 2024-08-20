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
        float volumeStiffness = 0.5f;
        float inverse_distance;
        float temp_volume;

        //shape matching 
    glm::vec3 restCenterOfMass;
    std::vector<glm::vec3> restPositions;
    float shapeFactor = 0.5f;

    glm::mat3 restShape;
    float mu = 1.0f;
    float lambda = 1.0f;

public: 
    SoftBodyXPBD(const Model& model, float timeStep, float Iterations)
        : dt(timeStep), solveIt(Iterations) {
        initializeFromModel(model);
  
    
        initialVolume = calculateMeshVolume();

        PlasticOffsets.resize(particles.size(), glm::vec3(0.0f));

    }

    
    void initializeNeoHookeanParameters() {
        restShape = glm::mat3(0.0f);
        for (size_t i = 0; i < particles.size(); ++i) {
            glm::vec3 q = particles[i].position - restCenterOfMass;
            restShape += glm::outerProduct(q, q);
        }
        restShape = glm::inverse(restShape);
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

    void initializeVolumeConstraint(){

        for(size_t  i = 0; i < Constraints.size(); i+= 3){
            size_t p1 = Constraints[i].p1;
            size_t p2 = Constraints[i].p2;
            size_t p3 = Constraints[i+1].p2;




            volume_constraints.push_back({p1,p2,p3,volumeStiffness,0.0f});



        }
    }
    void initializeShearingConstraints() {
        for (size_t i = 0; i < Constraints.size(); i += 3) {
            size_t p1 = Constraints[i].p1;
            size_t p2 = Constraints[i].p2;
            size_t p3 = Constraints[i + 1].p2;
            glm::vec3 edge1 = particles[p2].position - particles[p1].position;
            glm::vec3 edge2 = particles[p3].position - particles[p1].position;
            float cos_angle = glm::dot(glm::normalize(edge1), glm::normalize(edge2));
            float rest_angle = std::acos(glm::clamp(cos_angle, -1.0f, 1.0f));
            shearing_constraints.push_back({ p1, p2, p3, rest_angle, 0.0f,0.0f});
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
    Particle& p3 = particles[c.p2 + 2];

    float currentVol = calculateMeshVolume();

    double volume_diff = currentVol - initialVolume;

    if(volume_diff <= 0)
        continue;

    // Compute denom 

 



}


}
void preserveVolume() {
    double currentVolume = calculateMeshVolume();
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

    double volumeDifference = currentVolume - initialVolume;
    float sum_gradient_squared = 0.0f;




    for (size_t i = 0; i < particles.size(); ++i) {
        sum_gradient_squared += particles[i].invMass * glm::length2(gradients[i]);
    }

    float alpha = volumeStiffness / ( dt * dt);
    float lambda = 0.0;
    float deltaLambda = -(volumeDifference)  / (sum_gradient_squared + alpha);
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
        initializeVolumeConstraint();

        initializeShapeMatching();
        initializeNeoHookeanParameters();

        for (auto& constraint : Constraints) {
            constraint.lambda = 0.0f;
        }


        PlasticOffsets.resize(particles.size(), glm::vec3(0.0f));

    }


        void update() {

            ApplyGravity();
            predictPosition();
        for(int i = 0; i < solveIt; i++){
          
          //  stretchingConstraint();
            DistanceConstraints();
            solveNeoHookeanShapeMatching();
          //  solveShearConstraints();
            preserveVolume();
          //   BendingConstraints();
           
            //solveShapeMatching();
           // updatePlasticDeformations();
           // applyPlasticOffsets();
            }



        GroundConstraint();
        updateVelocitiesAndPositions();
        }


  
    void ApplyGravity(){
        for(auto& p : particles){
             if(p.invMass > 0){
                 p.velocity += glm::vec3(0,-19.8f,0) * dt;
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
 
     void updatePlasticDeformations() {
         for (size_t i = 0; i < Constraints.size(); ++i) {
             auto& c = Constraints[i];
             Particle& p1 = particles[c.p1];
             Particle& p2 = particles[c.p2];

             glm::vec3 currentVector = p2.position - p1.position;
             float currentLength = glm::length(currentVector);
             float strain = (currentLength - c.restLength) / c.restLength;

             if (std::abs(strain) > PlasticThreshHold) {
                 float plasticStrain = (std::abs(strain) - PlasticThreshHold) * PlasricCreep;
                 float sign = strain > 0 ? 1.0f : -1.0f;

                 // Update rest length
                 c.restLength *= (1.0f + sign * plasticStrain);

                 // Update plastic offsets
                 glm::vec3 offset = 0.5f * sign * plasticStrain * currentVector;
                 PlasticOffsets[c.p1] += offset;
                 PlasticOffsets[c.p2] -= offset;
             }
         }
     }


     void applyPlasticOffsets() {
         for (size_t i = 0; i < particles.size(); ++i) {
             particles[i].position += PlasticOffsets[i];
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



void solveNeoHookeanShapeMatching() {
    glm::vec3 com(0.0f);
    glm::mat3 P(0.0f), A(0.0f);

    for (const auto& p : particles) {
        com += p.position;
    }
    com /= static_cast<float>(particles.size());


    for (size_t i = 0; i < particles.size(); ++i) {
        glm::vec3 q = particles[i].position - com;
        glm::vec3 p = restPositions[i];
        P += glm::outerProduct(q, p);
        A += glm::outerProduct(q, q);
    }


    glm::mat3 R = glm::mat3(1.0f);
    glm::vec3 col0 = glm::normalize(glm::vec3(A[0][0], A[1][0], A[2][0]));
    glm::vec3 col1 = glm::normalize(glm::vec3(A[0][1], A[1][1], A[2][1]) - glm::dot(col0, glm::vec3(A[0][1], A[1][1], A[2][1])) * col0);
    glm::vec3 col2 = glm::normalize(glm::vec3(A[0][2], A[1][2], A[2][2]) - glm::dot(col0, glm::vec3(A[0][2], A[1][2], A[2][2])) * col0 - glm::dot(col1, glm::vec3(A[0][2], A[1][2], A[2][2])) * col1);

    R[0] = glm::vec4(col0, 0.0f);
    R[1] = glm::vec4(col1, 0.0f);
    R[2] = glm::vec4(col2, 0.0f);


    glm::mat3 F = P * restShape;


    glm::mat3 C = glm::transpose(F) * F;


    float I1 = trace(C);
    float J = glm::determinant(F);


    float energy = (mu / 2.0f) * (I1 - 3.0f) - mu * std::log(J) + (lambda / 2.0f) * std::pow(std::log(J), 2.0f);


    //Calculate the Piola Kirchioff Stress Tensor (Match go BRRR)
    glm::mat3 P_stress = mu * (F - glm::transpose(cofactor(F)) / J) + lambda * std::log(J) * cofactor(F) / J;



    for (size_t i = 0; i < particles.size(); ++i) {
        glm::vec3 goal = com + R * restPositions[i];
        glm::vec3 neoHookeanForce = P_stress * restPositions[i];
        glm::vec3 displacement = (goal - particles[i].position) + neoHookeanForce * dt * dt;
        particles[i].position += shapeFactor * displacement;
    }


}
void solveShearConstraints() {


    const float epsilon = 1e-6f;
    const float maxDisplacement = 0.1f;

    for (auto& c : shearing_constraints) {
        Particle& p1 = particles[c.p1];
        Particle& p2 = particles[c.p2];
        Particle& p3 = particles[c.p3];

        glm::vec3 edge1 = p2.position - p1.position;
        glm::vec3 edge2 = p3.position - p1.position;

        float edge1Length = glm::length(edge1);
        float edge2Length = glm::length(edge2);

        if (edge1Length < epsilon || edge2Length < epsilon) continue;

        glm::vec3 edge1Norm = edge1 / edge1Length;
        glm::vec3 edge2Norm = edge2 / edge2Length;

        float cosAngle = glm::dot(edge1Norm, edge2Norm);
        cosAngle = glm::clamp(cosAngle, -1.0f + epsilon, 1.0f - epsilon);
        float currentAngle = std::acos(cosAngle);

        float constraint = currentAngle - c.restArea;

        // Calculate gradients
        glm::vec3 grad1 = (edge2Norm - cosAngle * edge1Norm) / (edge1Length * std::sqrt(1 - cosAngle * cosAngle));
        glm::vec3 grad3 = (edge1Norm - cosAngle * edge2Norm) / (edge2Length * std::sqrt(1 - cosAngle * cosAngle));
        glm::vec3 grad2 = -grad1 - grad3;

        float sum = p1.invMass * glm::length2(grad1) +
            p2.invMass * glm::length2(grad2) +
            p3.invMass * glm::length2(grad3);

        if (sum < epsilon) continue;

        float alpha = 1.0f / (c.stiffness * dt * dt);
        float deltaLambda = -(constraint + alpha * c.lambda) / (sum + alpha);

        // Limit displacement
        float scale = 1.0f;
        glm::vec3 disp1 = p1.invMass * deltaLambda * grad1;
        glm::vec3 disp2 = p2.invMass * deltaLambda * grad2;
        glm::vec3 disp3 = p3.invMass * deltaLambda * grad3;

        float maxDisp = glm::max(glm::length(disp1), glm::max(glm::length(disp2), glm::length(disp3)));
        if (maxDisp > maxDisplacement) {
            scale = maxDisplacement / maxDisp;
        }

        c.lambda += deltaLambda * scale;

        p1.position += scale * disp1;
        p2.position += scale * disp2;
        p3.position += scale * disp3;
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


// Ground  constraint


void stretchingConstraint() {
    for (auto& c : Constraints) {
        Particle& p1 = particles[c.p1];
        Particle& p2 = particles[c.p2];

        glm::vec3 delta = p2.position - p1.position;
        float deltaLength = glm::length(delta);

        // Check for very small delta to avoid division by zero
        if (deltaLength < 1e-6f) continue;

        glm::vec3 dir = delta / deltaLength;

        // Calculate the stretch value 
        float constraint = deltaLength - c.restLength;

        float alpha = c.stiffness / (dt * dt);
        float sum = p1.invMass + p2.invMass;

        // Avoid division by zero
        if (sum < 1e-6f) continue;

        float deltaLambda = -(constraint + alpha * c.lambda) / (sum + alpha);
        c.lambda += deltaLambda;

        glm::vec3 correction = deltaLambda * dir;
        p1.position -= p1.invMass * correction;
        p2.position += p2.invMass * correction;
    }
}
void GroundConstraint() {
    float groundY = 0.0f;  // Adjust this to your ground level
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

void solveVolumeConstraints() {
    const float epsilon = 1e-6f;
    const float maxDisplacement = 0.01f;  // Reduced from previous version
    const float safetyFactor = 0.1f;  // Apply only a fraction of the calculated correction


    glm::vec3 com(0.0f);

    for (const auto& p : particles) {
        com += p.position;
    }

    com /= static_cast<float>(particles.size());

    float currentVolume{ 0.0f };

    std::vector<float> triangleVolumes(volume_constraints.size());

    for (size_t i{ 0 }; i < volume_constraints.size(); ++i) {
        const auto& vc = volume_constraints[i];

        const glm::vec3& p1 = particles[vc.p1].position;
        const glm::vec3& p2 = particles[vc.p2].position;
        const glm::vec3& p3 = particles[vc.p3].position;


        float signedVolume = glm::dot(p1 - com, glm::cross(p2 - p1, p3 - p1) / 6.0f);

        triangleVolumes[i] = signedVolume;

        currentVolume += signedVolume;



    }
    float volumeDifference = currentVolume - initialVolume;

    for (size_t i{ 0 }; i < volume_constraints.size(); ++i) {
        auto& vc = volume_constraints[i];

        auto& p1 = particles[vc.p1];
        auto& p2 = particles[vc.p2];
        auto& p3 = particles[vc.p3];


        glm::vec3 grad1 = glm::cross(p2.position - com, p3.position - com) / 6.0f;
        glm::vec3 grad2 = glm::cross(p3.position - com, p1.position - com) / 6.0f;
        glm::vec3 grad3 = glm::cross(p1.position - com, p2.position - com) / 6.0f;

        // Calculate the denominator for lambda
        float sum = p1.invMass * glm::length2(grad1) +
            p2.invMass * glm::length2(grad2) +
            p3.invMass * glm::length2(grad3);

        float alpha = volumeStiffness / (dt * dt);
        float dLambda = -(volumeDifference + alpha * vc.lambda) / (sum + alpha);
        vc.lambda += dLambda;

        // Apply position corrections
        p1.position += p1.invMass * dLambda * grad1;
        p2.position += p2.invMass * dLambda * grad2;
        p3.position += p3.invMass * dLambda * grad3;


        if (glm::any(glm::isnan(p1.position)) || glm::any(glm::isnan(p2.position)) || glm::any(glm::isnan(p3.position))) {
            std::cout << "NaN detected in particle positions!" << std::endl;
            // You might want to reset the simulation or take other corrective action here
        }

    }




}
void BendingConstraints() {
    for (auto& c : bending_constraints) {
        Particle& p1 = particles[c.p1];
        Particle& p2 = particles[c.p2];
        Particle& p3 = particles[c.p3];
        Particle& p4 = particles[c.p4];

        glm::vec3 n1 = glm::cross(p2.position - p1.position, p3.position - p1.position);
        glm::vec3 n2 = glm::cross(p2.position - p4.position, p3.position - p4.position);

        float n1_length = glm::length(n1);
        float n2_length = glm::length(n2);

        if (n1_length < 1e-6f || n2_length < 1e-6f) continue;

        n1 /= n1_length;
        n2 /= n2_length;

        float d = glm::dot(n1, n2);
        d = glm::clamp(d, -1.0f, 1.0f);
        float angle = std::acos(d);

        float targetAngle = c.restAngle;
        float constraint = angle - targetAngle;

        glm::vec3 q3 = (p2.position - p1.position) / glm::length(p2.position - p1.position);
        glm::vec3 q4 = (p3.position - p1.position) / glm::length(p3.position - p1.position);
        glm::vec3 q5 = (p4.position - p1.position) / glm::length(p4.position - p1.position);

        glm::vec3 dndx1 = -glm::cross(q3, q4);
        glm::vec3 dndx2 = glm::cross(q4 - q3, q5);
        glm::vec3 dndx3 = glm::cross(q5, q3);
        glm::vec3 dndx4 = -glm::cross(q5, q4 - q3);

        float invMass = p1.invMass + p2.invMass + p3.invMass + p4.invMass;
        if (invMass == 0.0f) continue;

        float alpha = c.stiffness / (dt * dt);
        float lambda = -(constraint + alpha * c.lambda) / (invMass + alpha);
        c.lambda += lambda;

        p1.position += p1.invMass * lambda * dndx1;
        p2.position += p2.invMass * lambda * dndx2;
        p3.position += p3.invMass * lambda * dndx3;
        p4.position += p4.invMass * lambda * dndx4;
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





double calculateMeshVolume(double scale = 1.0) {
    double totalVolume = 0.0;
    glm::dvec3 centroid(0.0);  // Use double precision

    // Calculate centroid
    for (const auto& p : particles) {
        centroid += glm::dvec3(p.position);
    }
    centroid /= static_cast<double>(particles.size());

 //   std::cout << "Centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")\n";
 //   std::cout << "Number of constraints: " << Constraints.size() << "\n";

    for (size_t i = 0; i < Constraints.size(); i += 3) {
        glm::dvec3 p1 = glm::dvec3(particles[Constraints[i].p1].position) - centroid;
        glm::dvec3 p2 = glm::dvec3(particles[Constraints[i].p2].position) - centroid;
        glm::dvec3 p3 = glm::dvec3(particles[Constraints[i + 1].p2].position) - centroid;

        // Scale the vectors
        p1 *= scale;
        p2 *= scale;
        p3 *= scale;

        // Calculate the signed volume of the tetrahedron
        double signedVolume = glm::dot(p1, glm::cross(p2, p3)) / 6.0;
        totalVolume += signedVolume;


    }

  //  std::cout << "Total raw volume: " << totalVolume << "\n";
    return std::abs(totalVolume);
}

void setPlasticThreshold(float threshold) {
    PlasticThreshHold = glm::clamp(threshold, 0.0f, 1.0f);
}

void setPlasticCreep(float creep) {
    PlasricCreep = glm::clamp(creep, 0.0f, 1.0f);
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
        particles[i].velocity = glm::vec3(0.0f);


    }


        initializeShapeMatching();

        
    // Reset lambdas in constraints
    for (auto& c : Constraints) {
        c.lambda = 0.0f;
        c.restLength = glm::distance(particles[c.p1].position, particles[c.p2].position);
    }

    for (auto& c : volume_constraints) {
        c.lambda = 0.0f;


    }

    for (auto& c : shearing_constraints) {
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
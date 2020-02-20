#pragma once
#include "NBodyAmp.h"

class NBodyAmpMy : public INBodyAmp{
private:
    float m_softeningSquared;
    float m_dampingFactor;
    float m_deltaTime;
    float m_particleMass;

public:
    NBodyAmpMy(float softeningSquared, float dampingFactor, float deltaTime, float particleMass) :
        m_softeningSquared(softeningSquared),
        m_dampingFactor(dampingFactor),
        m_deltaTime(deltaTime),
        m_particleMass(particleMass){}

    inline int TileSize() const{ return 1; }   //  No tiling.

    void Integrate(const std::vector<std::shared_ptr<TaskData>>& particleData,
                   int numParticles) const{
        assert(numParticles > 0);
        assert((numParticles % 4) == 0);

        ParticlesAmp particlesIn = *particleData[0]->DataOld;
        ParticlesAmp particlesOut = *particleData[0]->DataNew;

        extent<1> computeDomain(numParticles);
        const float softeningSquared = m_softeningSquared;
        const float dampingFactor = m_dampingFactor;
        const float deltaTime = m_deltaTime;
        const float particleMass = m_particleMass;

        parallel_for_each(computeDomain, [=](index<1> idx) restrict(amp){
            float_3 pos = particlesIn.pos[idx];
            float_3 vel = particlesIn.vel[idx];
            float_3 acc = 0.0f;

            // Update current Particle using all other particles
            for(int j = 0; j < numParticles; ++j)
                BodyBodyInteraction(acc, pos, particlesIn.pos[j], softeningSquared, particleMass);

            vel += acc * deltaTime;
            vel *= dampingFactor;
            pos += vel * deltaTime;

            particlesOut.pos[idx] = pos;
            particlesOut.vel[idx] = vel;
        });
    }
}; // *************************************************************************************************

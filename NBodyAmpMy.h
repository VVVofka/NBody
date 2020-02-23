#pragma once
#include "NBodyAmp.h"
#include <crtdbg.h>
class NBodyAmpMy : public INBodyAmp {
private:
	float m_softeningSquared;
	float m_dampingFactor;
	float m_deltaTime;
	float m_particleMass;
	float_3 sizes;
	std::vector<std::shared_ptr<TaskData>>* g_deviceData;
	std::vector<float_3> cpupos;
	std::vector<int_3> cpuintend;

public:
	NBodyAmpMy(float softeningSquared,
		float dampingFactor,
		float deltaTime,
		float particleMass, 
		std::vector<std::shared_ptr<TaskData>>* pgdData
	) :
		m_softeningSquared(softeningSquared),
		m_dampingFactor(dampingFactor),
		m_deltaTime(deltaTime),
		m_particleMass(particleMass) {
		sizes = float_3(1025, 513, 257);
		g_deviceData = pgdData;
	} // //////////////////////////////////////////////////////////////////////////////////////////////
	inline int TileSize() const { return 1; }   //  No tiling.

	void Integrate(const std::vector<std::shared_ptr<TaskData>>& particleData,
		int numParticles) const {
		assert(numParticles > 0);              // 512
		assert((numParticles % 4) == 0);

		ParticlesAmp particlesIn = *particleData[0]->DataOld;
		ParticlesAmp particlesOut = *particleData[0]->DataNew;

		extent<1> computeDomain(numParticles);
		const float softeningSquared = m_softeningSquared;
		const float dampingFactor = m_dampingFactor;
		const float deltaTime = m_deltaTime;
		const float particleMass = m_particleMass;

		parallel_for_each(computeDomain, [=](index<1> idx) restrict(amp) {
			float_3 pos = particlesIn.pos[idx];
			float_3 vel = particlesIn.vel[idx];
			float_3 acc = 0.0f;

			// Update current Particle using all other particles
			for (int j = 0; j < numParticles; ++j)
				BodyBodyInteraction(acc, pos, particlesIn.pos[j], softeningSquared, particleMass);

			vel += acc * deltaTime;
			vel *= dampingFactor;
			pos += vel * deltaTime;
			//pos.x = 0.0f;
			//pos.y = 0.0f;
			//pos.z = 0.0f;
			if (pos.x < -100.0f)
				vel.x = -vel.x;
			if (pos.x > 100.0f)
				vel.x = -vel.x;

			if (pos.y < -100.0f)
				vel.y = -vel.y;
			if (pos.y > 100.0f)
				vel.y = -vel.y;

			if (pos.z < -100.0f)
				vel.z = -vel.z;
			if (pos.z > 100.0f)
				vel.z = -vel.z;
			//if(pos.get_x() > 0.3f)                 pos.set_x(0.3f);

			particlesOut.pos[idx] = pos;   // *particleData[0]->DataNew
			particlesOut.vel[idx] = vel;   // *particleData[0]->DataNew
			});
		//_RPT1(0, "%f\n", particlesOut.pos(0).get_x());
	} // /////////////////////////////////////////////////////////////////////////////////
	void LoadParticles() {
		// Create particles in CPU memory.
		//ParticlesCpu particles(g_maxParticles);    // g_maxParticles = 58368
		std::random_device rd;
		std::default_random_engine engine(rd());
		for (int i = 0; i < g_maxParticles; i += g_particleNumStepSize) {
			std::uniform_real_distribution<float> randRadius(0.0f, __min(sizes.x, __min(sizes.y, sizes.z)));
			std::uniform_real_distribution<float> randTheta(-1.0f, 1.0f);
			std::uniform_real_distribution<float> randPhi(0.0f, 2.0f * static_cast<float>(std::_Pi));

			for (int i = offset; i < (offset + size); ++i) {
				float_3 delta = PolarToCartesian(randRadius(engine), acos(randTheta(engine)), randPhi(engine));
				particles.pos[i] = center + delta;
				//_RPT3(0, "%f\t%f\t%f\n", particles.pos[i].x, particles.pos[i].y, particles.pos[i].z);
				particles.vel[i] = velocity;
			}; . -

			LoadClusterParticles(
				i, 
				(g_particleNumStepSize / 2),
				float_3(centerSpread, 0.0f, 0.0f),
				float_3(0, 0, -20),
				g_Spread);
			LoadClusterParticles(particles, (i + g_particleNumStepSize / 2), ((g_particleNumStepSize + 1) / 2),
				float_3(-centerSpread, 0.0f, 0.0f),
				float_3(0, 0, 20),
				g_Spread);
		}
		// Copy particles to GPU memory.
		index<1> begin(0);
		extent<1> end(g_maxParticles);        // g_maxParticles = 58368
		size_t iall = g_deviceData.size();    // iall = 1
		for (size_t i = 0; i < iall; ++i) {
			array_view<float_3, 1> posView = g_deviceData[i]->DataOld->pos.section(index<1>(begin), extent<1>(end));
			posView = g_deviceData[i]->DataOld->pos.section(index<1>(begin), extent<1>(end));
			copy(particles.pos.begin(), posView);
			array_view<float_3, 1> velView = g_deviceData[i]->DataOld->vel.section(index<1>(begin), extent<1>(end));
			velView = g_deviceData[i]->DataOld->vel.section(index<1>(begin), extent<1>(end));
			copy(particles.vel.begin(), velView);
		}
	}//--------------------------------------------------------------------------------------
	void LoadClusterParticles(int offset, int size, float_3 center, float_3 velocity, float spread) {
		std::random_device rd;
		std::default_random_engine engine(rd());
		std::uniform_real_distribution<float> randRadius(0.0f, spread);
		std::uniform_real_distribution<float> randTheta(-1.0f, 1.0f);
		std::uniform_real_distribution<float> randPhi(0.0f, 2.0f * static_cast<float>(std::_Pi));

		for (int i = offset; i < (offset + size); ++i) {
			float_3 delta = PolarToCartesian(randRadius(engine), acos(randTheta(engine)), randPhi(engine));
			particles.pos[i] = center + delta;
			//_RPT3(0, "%f\t%f\t%f\n", particles.pos[i].x, particles.pos[i].y, particles.pos[i].z);
			particles.vel[i] = velocity;
		};.-
	} // /////////////// LoadClusterParticles()  ////////////////////////////////////////////////////
}; // *************************************************************************************************

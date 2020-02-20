// Microsoft Press
// C++ AMP: Accelerated Massive Parallelism with Microsoft Visual C++
// Copyright (c) 2012-2013 Ade Miller & Kate Gregory.  All rights reserved.
// Microsoft Public License (Ms-PL), http://ampbook.codeplex.com/license.
//===============================================================================
#pragma once
#include <math.h>
#include <ppl.h>
#include <concrtrm.h>
#include <amprt.h>
#include <assert.h>
#include <atlbase.h>
#include <random>
#include <amp.h>
#include <amp_graphics.h>
#include <amp_math.h>

#include "Common.h"
#include "INBodyAmp.h"
#include "AmpUtilities.h"

using namespace concurrency;
using namespace concurrency::graphics;

//--------------------------------------------------------------------------------------
//  Particle data structures.
//--------------------------------------------------------------------------------------

//  Data structure for storing particles on the CPU. Used during data initialization, which is
//  executed on the CPU. Also used for swapping partial results between multiple accelerators.
//
//  This is an struct of arrays, rather than the more conventional array of structs used by
//  the n-body CPU example. In general structs of arrays are more efficient for GPU programming.
#ifndef MY
struct ParticlesCpu{
	std::vector<float_3> pos;
	std::vector<float_3> vel;

	ParticlesCpu(int size) : pos(size), vel(size){}

	inline int size() const{
		assert(pos.size() == vel.size());
		return static_cast<int>(pos.size());
	}
}; // *************************************************************************************************
#else // !MY
struct ParticlesCpuMy{
	std::vector<int_3> pos;
	std::vector<float_3> intend;
	Concurrency::array<int, 3> area;

	ParticlesCpuMy(int size, int_3 sizes) :
		pos(size),
		intend(size),
		area(sizes.get_x(), sizes.get_y(), sizes.get_z()){}

	inline int size() const{
		assert(pos.size() == intend.size());
		return static_cast<int>(pos.size());
	}
}; // *************************************************************************************************
#endif // !MY
//  Data structure for storing particles on the C++ AMP accelerator.
//  This is an struct of arrays, rather than the more conventional array of structs used by
//  the n-body CPU example. In general structs of arrays are more efficient for GPU programming.
#ifndef MY
struct ParticlesAmp{
	array<float_3, 1>& pos;
	array<float_3, 1>& vel;
public:
	ParticlesAmp(array<float_3, 1>& pos, array<float_3, 1>& vel) : pos(pos), vel(vel){}
	inline int size() const{ return pos.extent.size(); }
}; // ****************************************************************************************
#else // !MY
struct ParticlesAmpMy{
	array<int_3, 1>& pos;
	array<float_3, 1>& intend;
	array<int, 3>& area;
public:
	ParticlesAmpMy(array<int_3, 1>& pos, array<float_3, 1>& intend, array<int, 3>& area) :
		pos(pos),
		intend(intend),
		area(area){}
	inline int size() const{ return pos.extent.size(); }
}; // ****************************************************************************************
#endif // !MY
//  Structure storing all the data associated with processing a subset of 
//  particles on a single C++ AMP accelerator.
#ifndef MY
struct TaskData{
public:
	accelerator Accelerator;
	std::shared_ptr<ParticlesAmp> DataOld;      // These hold references to the data
	std::shared_ptr<ParticlesAmp> DataNew;
private:
	array<float_3, 1> m_posOld;                 // These hold the actual data.
	array<float_3, 1> m_posNew;
	array<float_3, 1> m_velOld;
	array<float_3, 1> m_velNew;
public:
	TaskData(int size, accelerator_view view, accelerator acc) :
		Accelerator(acc),
		m_posOld(size, view),
		m_velOld(size, view),
		m_posNew(size, view),
		m_velNew(size, view),
		DataOld(new ParticlesAmp(m_posOld, m_velOld)),
		DataNew(new ParticlesAmp(m_posNew, m_velNew)){}
}; // *******************************************************************************************
#else // !MY
struct TaskDataMy{
public:
	accelerator Accelerator;
	std::shared_ptr<ParticlesAmpMy> DataOld;      // These hold references to the data
	std::shared_ptr<ParticlesAmpMy> DataNew;
private:
	array<int_3, 1> m_posOld;                 // These hold the actual data.
	array<int_3, 1> m_posNew;
	array<float_3, 1> m_intendOld;
	array<float_3, 1> m_intendNew;
	array<int, 3> m_arrayOld;
	array<int, 3> m_arrayNew;
public:
	TaskDataMy(int size, int_3 sizes, accelerator_view view, accelerator acc) :
		Accelerator(acc),
		m_posOld(size, view),
		m_intendOld(size, view),
		m_posNew(size, view),
		m_intendNew(size, view),
		m_arrayOld(sizes.get_x(), sizes.get_y(), sizes.get_z(), view),
		m_arrayNew(sizes.get_x(), sizes.get_y(), sizes.get_z(), view),
		DataOld(new ParticlesAmpMy(m_posOld, m_intendOld, m_arrayOld)),
		DataNew(new ParticlesAmpMy(m_posNew, m_intendNew, m_arrayNew)){}
}; // *******************************************************************************************
#endif // !MY
#ifndef MY
std::vector<std::shared_ptr<TaskData>> CreateTasks(int numParticles,
												   accelerator_view renderView){
	std::vector<accelerator> gpuAccelerators = AmpUtils::GetGpuAccelerators();
	std::vector<std::shared_ptr<TaskData>> tasks;
	tasks.reserve(gpuAccelerators.size());

	if(!gpuAccelerators.empty()){
		//  Create first accelerator attached to main view. This will attach the C++ AMP 
		//  array<float_3> to the D3D buffer on the first GPU.
		tasks.push_back(std::make_shared<TaskData>(numParticles, renderView, gpuAccelerators[0]));

		//  All other GPUs are associated with their default view.
		std::for_each(gpuAccelerators.cbegin() + 1, gpuAccelerators.cend(),
					  [=, &tasks](const accelerator& d){
			tasks.push_back(std::make_shared<TaskData>(numParticles, d.default_view, d));
		});
	}
	if(tasks.empty()){
		OutputDebugStringW(L"WARNING: No C++ AMP capable accelerators available, using REF.");
		accelerator a = accelerator(accelerator::default_accelerator);
		tasks.push_back(std::make_shared<TaskData>(numParticles, renderView, a));
	}
	AmpUtils::DebugListAccelerators(gpuAccelerators);
	return tasks;
}//--------------------------------------------------------------------------------------
#else // !MY
std::vector<std::shared_ptr<TaskDataMy>> CreateTasksMy(int numParticles, int_3 sizes,
													   accelerator_view renderView){
	std::vector<accelerator> gpuAccelerators = AmpUtils::GetGpuAccelerators();
	std::vector<std::shared_ptr<TaskDataMy>> tasks;
	tasks.reserve(gpuAccelerators.size());

	if(!gpuAccelerators.empty()){
		//  Create first accelerator attached to main view. This will attach the C++ AMP 
		//  array<float_3> to the D3D buffer on the first GPU.
		tasks.push_back(std::make_shared<TaskDataMy>(numParticles, sizes, renderView, gpuAccelerators[0]));

		//  All other GPUs are associated with their default view.
		std::for_each(gpuAccelerators.cbegin() + 1, gpuAccelerators.cend(),
					  [=, &tasks](const accelerator& d){
			tasks.push_back(std::make_shared<TaskDataMy>(numParticles, sizes, d.default_view, d));
		});
	}
	if(tasks.empty()){
		OutputDebugStringW(L"WARNING My: No C++ AMP capable accelerators available, using REF.");
		accelerator a = accelerator(accelerator::default_accelerator);
		tasks.push_back(std::make_shared<TaskDataMy>(numParticles, sizes, renderView, a));
	}
	AmpUtils::DebugListAccelerators(gpuAccelerators);
	return tasks;
}//--------------------------------------------------------------------------------------
#endif // !MY
//  Calculate the acceleration (force * mass) change for a pair of particles.
void BodyBodyInteraction(float_3& acc, const float_3 particlePosition,
						 const float_3 otherParticlePosition,
						 float softeningSquared, float particleMass) restrict(amp){
	float_3 r = otherParticlePosition - particlePosition;
	float distSqr = SqrLength(r) + softeningSquared;
	float invDist = concurrency::fast_math::rsqrt(distSqr);
	float invDistCube = invDist * invDist * invDist;
	float s = particleMass * invDistCube;
	acc += r * s;
}//--------------------------------------------------------------------------------------
//  Utility functions.
#ifndef MY
void LoadClusterParticles(ParticlesCpu& particles, int offset, int size, float_3 center, float_3 velocity, float spread){
	std::random_device rd;
	std::default_random_engine engine(rd());
	std::uniform_real_distribution<float> randRadius(0.0f, spread);
	std::uniform_real_distribution<float> randTheta(-1.0f, 1.0f);
	std::uniform_real_distribution<float> randPhi(0.0f, 2.0f * static_cast<float>(std::_Pi));

	for(int i = offset; i < (offset + size); ++i){
		float_3 delta = PolarToCartesian(randRadius(engine), acos(randTheta(engine)), randPhi(engine));
		particles.pos[i] = center + delta;
		particles.vel[i] = velocity;
	};
} // //////////////////////////////////////////////////////////////////////////////////////////
#else // !MY
void LoadClusterParticlesMy(ParticlesCpuMy& particlesMy, int_3 sizes){
	std::random_device rd;
	std::default_random_engine engine(rd());
	std::uniform_int_distribution<int> randX(0, sizes.get_x());
	std::uniform_int_distribution<int> randY(0, sizes.get_y());
	std::uniform_int_distribution<int> randZ(0, sizes.get_z());

	//int xup = sizes.get_x();
	//int yup = sizes.get_y();
	//int zup = sizes.get_z();
	//for(int x = 0; x < xup; x++)
	//	for(int y = 0; y < yup; y++)
	//		for(int z = 0; z < zup; z++)
	//			particlesMy.area[x][y][z] = 0;
	int szup = particlesMy.size();
	for(int i = 0; i < szup; ++i){
		int x, y, z;// , res;
		//do{
			x = randX(engine);
			y = randY(engine);
			z = randZ(engine);
			//res = particlesMy.area.data[x][y][z];
		//} while(res != 0);

		particlesMy.pos[i] = int_3(x, y, z);
		particlesMy.intend[i] = float_3(0.0f, 0.0f, 0.0f);
		//particlesMy.area.data[x][y][z] = i;
	}
} // //////////////////////////////////////////////////////////////////////////////////////////
#endif // !MY
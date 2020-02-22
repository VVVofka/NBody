// C++ AMP: Accelerated Massive Parallelism with Microsoft Visual C++
// Copyright (c) 2012-2013 Ade Miller & Kate Gregory.  All rights reserved.
// Microsoft Public License (Ms-PL), http://ampbook.codeplex.com/license.
#pragma once
#include <vector>
#include <memory>
//--------------------------------------------------------------------------------------
//  Interface for all classes that implement n-body calculations.
//--------------------------------------------------------------------------------------
//  Each class implements at least one overload of the Integrate method, for single or multi-accelerator.
//  They take one or more TaskData structs containing the data relating to all particles.

struct TaskData;
class INBodyAmp {
public:
	//  Each integrator exposes a tile size. This is used by the main client code to 
	//  ensure that the number of bodies is always a whole number of tiles.
	virtual int TileSize() const = 0;

	//  Integrate for all implementations. 
	//  This takes an array of TaskData but NBodyAmpSimple and NBodyAmpTiled only use the first element.
	virtual void Integrate(const std::vector<std::shared_ptr<TaskData>>& particleData,
		int numParticles) const = 0;

}; // **** class INBodyAmp *********************************************************************************************
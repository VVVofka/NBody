// Microsoft Press
// C++ AMP: Accelerated Massive Parallelism with Microsoft Visual C++
// Copyright (c) 2012-2013 Ade Miller & Kate Gregory.  All rights reserved.
// Microsoft Public License (Ms-PL), http://ampbook.codeplex.com/license.
//===============================================================================
#pragma once
#include <amp_graphics.h>
#include <d3dx9math.h>
//#define MY
using namespace concurrency::graphics;
//--------------------------------------------------------------------------------------
//  Utility functions for vector calculations.
inline const float SqrLength(const float_3& r) restrict(amp, cpu) {
	return r.x * r.x + r.y * r.y + r.z * r.z;
}//--------------------------------------------------------------------------------------
template<typename T>
inline float_3 PolarToCartesian(T r, T theta, T phi) {
	return float_3(r * sin(theta) * cos(phi), r * sin(theta) * sin(phi), r * cos(theta));
}//--------------------------------------------------------------------------------------
//  D3D related data structures used by the GUI.
struct ParticleVertex {
	D3DXCOLOR color;
};//--------------------------------------------------------------------------------------
struct ResourceData {
	D3DXMATRIX worldViewProj;
	D3DXMATRIX inverseView;
	D3DXCOLOR color;            // color value for changing particles color
};//--------------------------------------------------------------------------------------
//  Custom deleter for smart pointers to handle 
template <typename T>
struct FreeDeleter {
	FreeDeleter() throw() // VC++ does not yet support C++11 noexcept.
	{}

	template <typename U>
	FreeDeleter(const FreeDeleter<U>&, typename std::enable_if<std::is_convertible<U*, T*>::value, void>::type** = nullptr) throw() {}

	void operator()(T* const ptr) const throw() {
		static_assert(std::is_trivially_destructible<T>::value, "Cannot free memory for a type with a non-trivial destructor, use delete.");
		std::free(ptr);
	}//--------------------------------------------------------------------------------------
}; // ***************************************************************************************************

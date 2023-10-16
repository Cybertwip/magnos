
//Bullet Continuous Collision Detection and Physics Library
//Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

//
// cbtAxisSweep3
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
#include "cbtAxisSweep3.h"

cbtAxisSweep3::cbtAxisSweep3(const cbtVector3& worldAabbMin, const cbtVector3& worldAabbMax, unsigned short int maxHandles, cbtOverlappingPairCache* pairCache, bool disableRaycastAccelerator)
	: cbtAxisSweep3Internal<unsigned short int>(worldAabbMin, worldAabbMax, 0xfffe, 0xffff, maxHandles, pairCache, disableRaycastAccelerator)
{
	// 1 handle is reserved as sentinel
	cbtAssert(maxHandles > 1 && maxHandles < 32767);
}

bt32BitAxisSweep3::bt32BitAxisSweep3(const cbtVector3& worldAabbMin, const cbtVector3& worldAabbMax, unsigned int maxHandles, cbtOverlappingPairCache* pairCache, bool disableRaycastAccelerator)
	: cbtAxisSweep3Internal<unsigned int>(worldAabbMin, worldAabbMax, 0xfffffffe, 0x7fffffff, maxHandles, pairCache, disableRaycastAccelerator)
{
	// 1 handle is reserved as sentinel
	cbtAssert(maxHandles > 1 && maxHandles < 2147483647);
}

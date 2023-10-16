/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
StackAlloc extracted from GJK-EPA collision solver by Nathanael Presson
Nov.2006
*/

#ifndef BT_STACK_ALLOC
#define BT_STACK_ALLOC

#include "cbtScalar.h"  //for cbtAssert
#include "cbtAlignedAllocator.h"

///The cbtBlock class is an internal structure for the cbtStackAlloc memory allocator.
struct cbtBlock
{
	cbtBlock* previous;
	unsigned char* address;
};

///The StackAlloc class provides some fast stack-based memory allocator (LIFO last-in first-out)
class cbtStackAlloc
{
public:
	cbtStackAlloc(unsigned int size)
	{
		ctor();
		create(size);
	}
	~cbtStackAlloc() { destroy(); }

	inline void create(unsigned int size)
	{
		destroy();
		data = (unsigned char*)cbtAlignedAlloc(size, 16);
		totalsize = size;
	}
	inline void destroy()
	{
		cbtAssert(usedsize == 0);
		//Raise(L"StackAlloc is still in use");

		if (usedsize == 0)
		{
			if (!ischild && data)
				cbtAlignedFree(data);

			data = 0;
			usedsize = 0;
		}
	}

	int getAvailableMemory() const
	{
		return static_cast<int>(totalsize - usedsize);
	}

	unsigned char* allocate(unsigned int size)
	{
		const unsigned int nus(usedsize + size);
		if (nus < totalsize)
		{
			usedsize = nus;
			return (data + (usedsize - size));
		}
		cbtAssert(0);
		//&& (L"Not enough memory"));

		return (0);
	}
	SIMD_FORCE_INLINE cbtBlock* beginBlock()
	{
		cbtBlock* pb = (cbtBlock*)allocate(sizeof(cbtBlock));
		pb->previous = current;
		pb->address = data + usedsize;
		current = pb;
		return (pb);
	}
	SIMD_FORCE_INLINE void endBlock(cbtBlock* block)
	{
		cbtAssert(block == current);
		//Raise(L"Unmatched blocks");
		if (block == current)
		{
			current = block->previous;
			usedsize = (unsigned int)((block->address - data) - sizeof(cbtBlock));
		}
	}

private:
	void ctor()
	{
		data = 0;
		totalsize = 0;
		usedsize = 0;
		current = 0;
		ischild = false;
	}
	unsigned char* data;
	unsigned int totalsize;
	unsigned int usedsize;
	cbtBlock* current;
	bool ischild;
};

#endif  //BT_STACK_ALLOC

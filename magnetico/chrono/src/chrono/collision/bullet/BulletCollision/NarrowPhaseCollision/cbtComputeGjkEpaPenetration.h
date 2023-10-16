/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_GJK_EPA_PENETATION_CONVEX_COLLISION_H
#define BT_GJK_EPA_PENETATION_CONVEX_COLLISION_H

#include "LinearMath/cbtTransform.h"  // Note that cbtVector3 might be double precision...
#include "cbtGjkEpa3.h"
#include "cbtGjkCollisionDescription.h"
#include "BulletCollision/NarrowPhaseCollision/cbtVoronoiSimplexSolver.h"

template <typename cbtConvexTemplate>
bool cbtGjkEpaCalcPenDepth(const cbtConvexTemplate& a, const cbtConvexTemplate& b,
						  const cbtGjkCollisionDescription& colDesc,
						  cbtVector3& v, cbtVector3& wWitnessOnA, cbtVector3& wWitnessOnB)
{
	(void)v;

	//	const cbtScalar				radialmargin(cbtScalar(0.));

	cbtVector3 guessVector(b.getWorldTransform().getOrigin() - a.getWorldTransform().getOrigin());  //?? why not use the GJK input?

	cbtGjkEpaSolver3::sResults results;

	if (cbtGjkEpaSolver3_Penetration(a, b, guessVector, results))

	{
		//	debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,cbtVector3(255,0,0));
		//resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
		wWitnessOnA = results.witnesses[0];
		wWitnessOnB = results.witnesses[1];
		v = results.normal;
		return true;
	}
	else
	{
		if (cbtGjkEpaSolver3_Distance(a, b, guessVector, results))
		{
			wWitnessOnA = results.witnesses[0];
			wWitnessOnB = results.witnesses[1];
			v = results.normal;
			return false;
		}
	}
	return false;
}

template <typename cbtConvexTemplate, typename cbtGjkDistanceTemplate>
int cbtComputeGjkEpaPenetration(const cbtConvexTemplate& a, const cbtConvexTemplate& b, const cbtGjkCollisionDescription& colDesc, cbtVoronoiSimplexSolver& simplexSolver, cbtGjkDistanceTemplate* distInfo)
{
	bool m_catchDegeneracies = true;
	cbtScalar m_cachedSeparatingDistance = 0.f;

	cbtScalar distance = cbtScalar(0.);
	cbtVector3 normalInB(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));

	cbtVector3 pointOnA, pointOnB;
	cbtTransform localTransA = a.getWorldTransform();
	cbtTransform localTransB = b.getWorldTransform();

	cbtScalar marginA = a.getMargin();
	cbtScalar marginB = b.getMargin();

	int m_curIter = 0;
	int gGjkMaxIter = colDesc.m_maxGjkIterations;  //this is to catch invalid input, perhaps check for #NaN?
	cbtVector3 m_cachedSeparatingAxis = colDesc.m_firstDir;

	bool isValid = false;
	bool checkSimplex = false;
	bool checkPenetration = true;
	int m_degenerateSimplex = 0;

	int m_lastUsedMethod = -1;

	{
		cbtScalar squaredDistance = BT_LARGE_FLOAT;
		cbtScalar delta = cbtScalar(0.);

		cbtScalar margin = marginA + marginB;

		simplexSolver.reset();

		for (;;)
		//while (true)
		{
			cbtVector3 seperatingAxisInA = (-m_cachedSeparatingAxis) * localTransA.getBasis();
			cbtVector3 seperatingAxisInB = m_cachedSeparatingAxis * localTransB.getBasis();

			cbtVector3 pInA = a.getLocalSupportWithoutMargin(seperatingAxisInA);
			cbtVector3 qInB = b.getLocalSupportWithoutMargin(seperatingAxisInB);

			cbtVector3 pWorld = localTransA(pInA);
			cbtVector3 qWorld = localTransB(qInB);

			cbtVector3 w = pWorld - qWorld;
			delta = m_cachedSeparatingAxis.dot(w);

			// potential exit, they don't overlap
			if ((delta > cbtScalar(0.0)) && (delta * delta > squaredDistance * colDesc.m_maximumDistanceSquared))
			{
				m_degenerateSimplex = 10;
				checkSimplex = true;
				//checkPenetration = false;
				break;
			}

			//exit 0: the new point is already in the simplex, or we didn't come any closer
			if (simplexSolver.inSimplex(w))
			{
				m_degenerateSimplex = 1;
				checkSimplex = true;
				break;
			}
			// are we getting any closer ?
			cbtScalar f0 = squaredDistance - delta;
			cbtScalar f1 = squaredDistance * colDesc.m_gjkRelError2;

			if (f0 <= f1)
			{
				if (f0 <= cbtScalar(0.))
				{
					m_degenerateSimplex = 2;
				}
				else
				{
					m_degenerateSimplex = 11;
				}
				checkSimplex = true;
				break;
			}

			//add current vertex to simplex
			simplexSolver.addVertex(w, pWorld, qWorld);
			cbtVector3 newCachedSeparatingAxis;

			//calculate the closest point to the origin (update vector v)
			if (!simplexSolver.closest(newCachedSeparatingAxis))
			{
				m_degenerateSimplex = 3;
				checkSimplex = true;
				break;
			}

			if (newCachedSeparatingAxis.length2() < colDesc.m_gjkRelError2)
			{
				m_cachedSeparatingAxis = newCachedSeparatingAxis;
				m_degenerateSimplex = 6;
				checkSimplex = true;
				break;
			}

			cbtScalar previousSquaredDistance = squaredDistance;
			squaredDistance = newCachedSeparatingAxis.length2();
#if 0
            ///warning: this termination condition leads to some problems in 2d test case see Bullet/Demos/Box2dDemo
            if (squaredDistance>previousSquaredDistance)
            {
                m_degenerateSimplex = 7;
                squaredDistance = previousSquaredDistance;
                checkSimplex = false;
                break;
            }
#endif  //

			//redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

			//are we getting any closer ?
			if (previousSquaredDistance - squaredDistance <= SIMD_EPSILON * previousSquaredDistance)
			{
				//				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
				checkSimplex = true;
				m_degenerateSimplex = 12;

				break;
			}

			m_cachedSeparatingAxis = newCachedSeparatingAxis;

			//degeneracy, this is typically due to invalid/uninitialized worldtransforms for a cbtCollisionObject
			if (m_curIter++ > gGjkMaxIter)
			{
#if defined(DEBUG) || defined(_DEBUG)

				printf("cbtGjkPairDetector maxIter exceeded:%i\n", m_curIter);
				printf("sepAxis=(%f,%f,%f), squaredDistance = %f\n",
					   m_cachedSeparatingAxis.getX(),
					   m_cachedSeparatingAxis.getY(),
					   m_cachedSeparatingAxis.getZ(),
					   squaredDistance);
#endif

				break;
			}

			bool check = (!simplexSolver.fullSimplex());
			//bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > SIMD_EPSILON * m_simplexSolver->maxVertex());

			if (!check)
			{
				//do we need this backup_closest here ?
				//				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
				m_degenerateSimplex = 13;
				break;
			}
		}

		if (checkSimplex)
		{
			simplexSolver.compute_points(pointOnA, pointOnB);
			normalInB = m_cachedSeparatingAxis;

			cbtScalar lenSqr = m_cachedSeparatingAxis.length2();

			//valid normal
			if (lenSqr < 0.0001)
			{
				m_degenerateSimplex = 5;
			}
			if (lenSqr > SIMD_EPSILON * SIMD_EPSILON)
			{
				cbtScalar rlen = cbtScalar(1.) / cbtSqrt(lenSqr);
				normalInB *= rlen;  //normalize

				cbtScalar s = cbtSqrt(squaredDistance);

				cbtAssert(s > cbtScalar(0.0));
				pointOnA -= m_cachedSeparatingAxis * (marginA / s);
				pointOnB += m_cachedSeparatingAxis * (marginB / s);
				distance = ((cbtScalar(1.) / rlen) - margin);
				isValid = true;

				m_lastUsedMethod = 1;
			}
			else
			{
				m_lastUsedMethod = 2;
			}
		}

		bool catchDegeneratePenetrationCase =
			(m_catchDegeneracies && m_degenerateSimplex && ((distance + margin) < 0.01));

		//if (checkPenetration && !isValid)
		if (checkPenetration && (!isValid || catchDegeneratePenetrationCase))
		{
			//penetration case

			//if there is no way to handle penetrations, bail out

			// Penetration depth case.
			cbtVector3 tmpPointOnA, tmpPointOnB;

			m_cachedSeparatingAxis.setZero();

			bool isValid2 = cbtGjkEpaCalcPenDepth(a, b,
												 colDesc,
												 m_cachedSeparatingAxis, tmpPointOnA, tmpPointOnB);

			if (isValid2)
			{
				cbtVector3 tmpNormalInB = tmpPointOnB - tmpPointOnA;
				cbtScalar lenSqr = tmpNormalInB.length2();
				if (lenSqr <= (SIMD_EPSILON * SIMD_EPSILON))
				{
					tmpNormalInB = m_cachedSeparatingAxis;
					lenSqr = m_cachedSeparatingAxis.length2();
				}

				if (lenSqr > (SIMD_EPSILON * SIMD_EPSILON))
				{
					tmpNormalInB /= cbtSqrt(lenSqr);
					cbtScalar distance2 = -(tmpPointOnA - tmpPointOnB).length();
					//only replace valid penetrations when the result is deeper (check)
					if (!isValid || (distance2 < distance))
					{
						distance = distance2;
						pointOnA = tmpPointOnA;
						pointOnB = tmpPointOnB;
						normalInB = tmpNormalInB;

						isValid = true;
						m_lastUsedMethod = 3;
					}
					else
					{
						m_lastUsedMethod = 8;
					}
				}
				else
				{
					m_lastUsedMethod = 9;
				}
			}
			else

			{
				///this is another degenerate case, where the initial GJK calculation reports a degenerate case
				///EPA reports no penetration, and the second GJK (using the supporting vector without margin)
				///reports a valid positive distance. Use the results of the second GJK instead of failing.
				///thanks to Jacob.Langford for the reproduction case
				///http://code.google.com/p/bullet/issues/detail?id=250

				if (m_cachedSeparatingAxis.length2() > cbtScalar(0.))
				{
					cbtScalar distance2 = (tmpPointOnA - tmpPointOnB).length() - margin;
					//only replace valid distances when the distance is less
					if (!isValid || (distance2 < distance))
					{
						distance = distance2;
						pointOnA = tmpPointOnA;
						pointOnB = tmpPointOnB;
						pointOnA -= m_cachedSeparatingAxis * marginA;
						pointOnB += m_cachedSeparatingAxis * marginB;
						normalInB = m_cachedSeparatingAxis;
						normalInB.normalize();

						isValid = true;
						m_lastUsedMethod = 6;
					}
					else
					{
						m_lastUsedMethod = 5;
					}
				}
			}
		}
	}

	if (isValid && ((distance < 0) || (distance * distance < colDesc.m_maximumDistanceSquared)))
	{
		m_cachedSeparatingAxis = normalInB;
		m_cachedSeparatingDistance = distance;
		distInfo->m_distance = distance;
		distInfo->m_normalBtoA = normalInB;
		distInfo->m_pointOnB = pointOnB;
		distInfo->m_pointOnA = pointOnB + normalInB * distance;
		return 0;
	}
	return -m_lastUsedMethod;
}

#endif  //BT_GJK_EPA_PENETATION_CONVEX_COLLISION_H

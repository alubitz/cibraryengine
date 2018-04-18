#include "StdAfx.h"
#include "Random3D.h"

#include "Vector.h"
#include "Quaternion.h"

namespace CibraryEngine
{
	/*
	 * Random3D methods
	 */
	float Random3D::Rand() { return (float)rand() / RAND_MAX; }
	float Random3D::Rand(float max) { return Rand() * max; }
	float Random3D::Rand(float min, float max) { return Rand() * (max - min) + min; }

	int Random3D::RandInt() { return rand(); }
	int Random3D::RandInt(int min, int max) { return rand() % (max - min + 1) + min; }
	int Random3D::RandInt(int max_plus_one) { return rand() % max_plus_one; }

	unsigned int Random3D::BigRand(unsigned int max_plus_one)
	{
		if(max_plus_one == 0)
			return 0;
		if(max_plus_one <= RAND_MAX)
			return Random3D::RandInt(max_plus_one);

		unsigned int choice;
		do
		{
			choice = Random3D::RandInt();
			unsigned int max_possible = RAND_MAX;
			while(max_possible < max_plus_one)
			{
				max_possible <<= 1;
				choice <<= 1;
				choice |= Random3D::RandInt() % 2;		// TODO: do this more efficiently?
			}
		} while(choice >= max_plus_one);

		return choice;
	}

	Vec3 Random3D::RandomNormalizedVector(float len)
	{
		while(true)
		{
			float x = Rand(-1, 1), y = Rand(-1, 1), z = Rand(-1, 1);
			float mag_sq = x * x + y * y + z * z;
			if(mag_sq == 0 || mag_sq > 1)
				continue;
			float inv = len / sqrtf(mag_sq);
			return Vec3(x * inv, y * inv, z * inv);
		}
	}

	Vec2 Random3D::RandomNormalizedVec2(float len)
	{
		while(true)
		{
			float x = Rand(-1, 1), y = Rand(-1, 1);
			float mag_sq = x * x + y * y;
			if(mag_sq == 0 || mag_sq > 1)
				continue;
			float inv = len / sqrtf(mag_sq);
			return Vec2(x * inv, y * inv);
		}
	}

	Quaternion Random3D::RandomQuaternionRotation()
	{
		Quaternion temp = Quaternion::Identity();
		for(int i = 0; i < 3; ++i)
		{
			Vec3 axis = RandomNormalizedVector(1.0f);
			temp *= Quaternion::FromAxisAngle(axis.x, axis.y, axis.z, Rand((float)(16.0 * M_PI)));
		}
		return temp;
	}
}

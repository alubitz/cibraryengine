#include "StdAfx.h"
#include "MassInfo.h"

#include "CollisionShape.h"
#include "Serialize.h"

namespace CibraryEngine
{
	/*
	 * MassInfo methods
	 */
	MassInfo::MassInfo() : mass(0), com(), moi() { }
	MassInfo::MassInfo(float mass) : mass(mass), com(), moi() { }
	MassInfo::MassInfo(const Vec3& point, float mass) : mass(mass), com(point), moi() { }

	void MassInfo::operator +=(const MassInfo& right)
	{
		float totalmass = mass + right.mass;
		Vec3 new_com = (com * mass + right.com * right.mass) / totalmass;

		float left_moi[9];
		float right_moi[9];
		GetAlternatePivotMoI(new_com - com, moi, mass, left_moi);
		GetAlternatePivotMoI(new_com - right.com, right.moi, right.mass, right_moi);

		mass = totalmass;
		com = new_com;
		for(int i = 0; i < 9; ++i)
			moi[i] = left_moi[i] + right_moi[i];
	}
	MassInfo MassInfo::operator +(const MassInfo& other) { MassInfo temp = *this; temp += other; return temp; }

	void MassInfo::operator *=(float coeff)
	{
		mass *= coeff;

		for(int i = 0; i < 9; ++i)
			moi[i] *= coeff;
	}
	MassInfo MassInfo::operator *(float coeff) { MassInfo temp = *this; temp *= coeff; return temp; }

	void MassInfo::GetAlternatePivotMoI(const Vec3& a, const float* I, float m, float* result)
	{
		float xx = a.x * a.x;
		float yy = a.y * a.y;
		float zz = a.z * a.z;

		*(result++)	= *(I++)	+ m * (yy + zz);
		*(result++)	= *(I++)	- m * a.x * a.y;
		*(result++)	= *(I++)	- m * a.x * a.z;

		*(result++)	= *(I++)	- m * a.y * a.x;
		*(result++)	= *(I++)	+ m * (xx + zz);
		*(result++)	= *(I++)	- m * a.y * a.z;

		*(result++)	= *(I++)	- m * a.z * a.x;
		*(result++)	= *(I++)	- m * a.z * a.y;
		*result		= *I		+ m * (xx + yy);
	}

	MassInfo MassInfo::FromCollisionShape(CollisionShape* shape, float mass) { return shape->ComputeMassInfo() * mass; }

	MassInfo MassInfo::ReadMassInfo(istream& stream)
	{
		MassInfo result;

		result.mass = ReadSingle(stream);
		result.com = ReadVec3(stream);

		for(char i = 0; i < 9; ++i)
			result.moi[i] = ReadSingle(stream);

		return result;
	}

	void MassInfo::Write(ostream& stream)
	{
		WriteSingle(mass, stream);
		WriteVec3(com, stream);

		for(char i = 0; i < 9; ++i)
			WriteSingle(moi[i], stream);
	}
}

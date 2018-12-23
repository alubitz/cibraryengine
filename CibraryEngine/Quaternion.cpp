#include "StdAfx.h"
#include "Quaternion.h"
#include "Matrix.h"
#include "Vector.h"

#include "Serialize.h"

namespace CibraryEngine
{
	/*
	 * Quaternion methods
	 */
	Mat3 Quaternion::ToMat3() const
	{
		static const float root_two = sqrtf(2);				// only computed once! this lets us do 4 multiplications instead of 9 additions

		float X = x * root_two;
		float Y = y * root_two;
		float Z = z * root_two;
		float W = w * root_two;

		float xx = X * X;
		float xy = X * Y;
		float xz = X * Z;
		float yy = Y * Y;
		float yz = Y * Z;
		float zz = Z * Z;
		float wx = W * X;
		float wy = W * Y;
		float wz = W * Z;

		return Mat3(
			1.0f - yy - zz,		xy - wz,			xz + wy,
			xy + wz,			1.0f - xx - zz,		yz - wx,
			xz - wy,			yz + wx,			1.0f - xx - yy
		);
	}

	static Quaternion GetQFRMGivenBiggestPoint(const Mat3& mat, const Vec3& axis, Vec3& dir, float magsq)
	{
		if(magsq == 0)
			return Quaternion::Identity();
		else
		{
			dir /= sqrtf(magsq);

			Vec3 other_axis = Vec3::Cross(axis, dir);
			Vec3 mx = mat * dir;

			float angle = atan2f(Vec3::Dot(mx, other_axis), Vec3::Dot(mx, dir));
			float half  = angle * 0.5f;
			float sine  = sinf(half);

			return Quaternion(cosf(half), axis.x * sine, axis.y * sine, axis.z * sine);
		}
	}

	Quaternion Quaternion::FromRotationMatrix(const Mat3& mat)
	{
		// TODO: why does this matrix not work: Quaternion::FromRVec(0.25f * float(M_PI), 0, 0.25f * float(M_PI)).ToMat3() ???
		const float* arr = mat.values;

		float t = arr[0] + arr[4] + arr[8] + 1.0f;
		if(t > 0)
		{
#if 0
			float s = 2.0f * sqrtf(t);
			float inv_s = 1.0f / s;
			return Quaternion::Normalize(Quaternion(
				0.25f * s,
				(arr[7] - arr[5]) * inv_s,
				(arr[2] - arr[6]) * inv_s,
				(arr[3] - arr[1]) * inv_s
			));
#else
			Vec3 dx(arr[0] - 1, arr[3],     arr[6]    );
			Vec3 dy(arr[1],     arr[4] - 1, arr[7]    );
			Vec3 dz(arr[2],     arr[5],     arr[8] - 1);

			Vec3 axis = Vec3::Cross(dx, dy) + Vec3::Cross(dx, dz) + Vec3::Cross(dz, dy);
			if(float magsq = axis.ComputeMagnitudeSquared())
			{
				axis /= sqrtf(magsq);

				float dxsq = dx.ComputeMagnitudeSquared(), dysq = dy.ComputeMagnitudeSquared(), dzsq = dz.ComputeMagnitudeSquared();

				if(dxsq >= dysq && dxsq >= dzsq)
					return GetQFRMGivenBiggestPoint(mat, axis, dx, dxsq);
				else if(dysq >= dxsq && dysq >= dzsq)
					return GetQFRMGivenBiggestPoint(mat, axis, dy, dysq);
				else
					return GetQFRMGivenBiggestPoint(mat, axis, dz, dzsq);
			}
			else
				return Quaternion::Identity();

			
#endif
		}
		else if(arr[0] > arr[4] && arr[0] > arr[8])
		{
 			float s = 2.0f * sqrtf(1.0f + arr[0] - arr[4] - arr[8]);
			float inv_s = 1.0f / s;
			return Quaternion::Normalize(Quaternion(
				(arr[5] + arr[7]) * inv_s,
				0.25f * s,
				(arr[1] + arr[3]) * inv_s,
				(arr[2] + arr[6]) * inv_s
			));
		}
		else if(arr[4] > arr[0] && arr[4] > arr[8])
		{
			float s = 2.0f * sqrtf(1.0f + arr[4] - arr[0] - arr[8]);
			float inv_s = 1.0f / s;
			return Quaternion::Normalize(Quaternion(
				(arr[2] + arr[6]) * inv_s,
				(arr[1] + arr[3]) * inv_s,
				0.25f * s,
				(arr[5] + arr[7]) * inv_s
			));
		}
		else
		{
			float s = 2.0f * sqrtf(1.0f + arr[8] - arr[0] - arr[4]);
			float inv_s = 1.0f / s;
			return Quaternion::Normalize(Quaternion(
				(arr[1] + arr[3]) * inv_s,
				(arr[2] + arr[6]) * inv_s,
				(arr[5] + arr[7]) * inv_s,
				0.25f * s
			));
		}
	}




	/*
	 * Quaternion serialization functions
	 */
	void WriteQuaternion(const Quaternion& q, ostream& stream)
	{
		WriteSingle(q.w, stream);
		WriteSingle(q.x, stream);
		WriteSingle(q.y, stream);
		WriteSingle(q.z, stream);
	}

	Quaternion ReadQuaternion(istream& stream)
	{
		float w = ReadSingle(stream);
		float x = ReadSingle(stream);
		float y = ReadSingle(stream);
		float z = ReadSingle(stream);
		return Quaternion(w, x, y, z);
	}

}
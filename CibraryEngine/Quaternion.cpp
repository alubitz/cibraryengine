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
		float xx = x * x;	xx += xx;
		float xy = x * y;	xy += xy;
		float xz = x * z;	xz += xz;
		float yy = y * y;	yy += yy;
		float yz = y * z;	yz += yz;
		float zz = z * z;	zz += zz;
		float wx = w * x;	wx += wx;
		float wy = w * y;	wy += wy;
		float wz = w * z;	wz += wz;

		return Mat3(
			1.0f - yy - zz,		xy - wz,			xz + wy,
			xy + wz,			1.0f - xx - zz,		yz - wx,
			xz - wy,			yz + wx,			1.0f - xx - yy
		);
	}

	Quaternion Quaternion::FromRotationMatrix(const Mat3& mat)
	{
		const float* arr = mat.values;

		float t = arr[0] + arr[4] + arr[8] + 1.0f;
		if(t > 0)
		{
			float s = 0.5f / sqrtf(t);
			return Quaternion::Normalize(Quaternion(
				0.25f / s,
				(arr[7] - arr[5]) * s,
				(arr[2] - arr[6]) * s,
				(arr[3] - arr[1]) * s
			));
		}
		else
		{
			if(arr[0] > arr[4] && arr[0] > arr[8])
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
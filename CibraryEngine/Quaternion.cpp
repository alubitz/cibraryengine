#include "StdAfx.h"
#include "Quaternion.h"
#include "Matrix.h"
#include "Vector.h"

#include "Serialize.h"

namespace CibraryEngine
{
	Quaternion::Quaternion() { x = y = z = w = 0.0; }
	Quaternion::Quaternion(float w_, float x_, float y_, float z_) {w = w_;  x = x_; y = y_; z = z_; }

	float Quaternion::Norm() const { return sqrtf(w * w + x * x + y * y + z * z); }

	Vec3 Quaternion::ToPYR() const
	{
		Vec3 axis(x, y, z);
		float sine = axis.ComputeMagnitude();			// doesn't cover the possibility of a negative sine
		if(sine == 0.0)
			return Vec3();
		else
		{
			float half = asin(sine);
			float angle = half * 2.0f;
			return Vec3::Normalize(axis, angle);
		}
	}

	Mat3 Quaternion::ToMat3() const
	{
		Quaternion n = Normalize(*this);				// normalized copy

		float W = n.w, X = n.x, Y = n.y, Z = n.z;

		return Mat3(
			1.0f - 2.0f * Y * Y - 2.0f * Z * Z,	2.0f * (X * Y - W * Z),				2.0f * (X * Z + W * Y),
			2.0f * (X * Y + W * Z),				1.0f - 2.0f * X * X - 2.0f * Z * Z,	2.0f * (Y * Z - W * X),
			2.0f * (X * Z - W * Y),				2.0f * (Y * Z + W * X),				1.0f - 2.0f * X * X - 2.0f * Y * Y
		);
	}

	Vec3 Quaternion::operator *(const Vec3& right) const { return ToMat3() * right; }
	Quaternion Quaternion::operator -() const { return Quaternion(-w, -x, -y, -z); }

	Quaternion Quaternion::operator *(const Quaternion& right) const { Quaternion temp(*this); temp *= right; return temp; }
	void Quaternion::operator *=(const Quaternion& right)
	{
		float w_ = w * right.w - x * right.x - y * right.y - z * right.z;
		float x_ = w * right.x + x * right.w + y * right.z - z * right.y;
		float y_ = w * right.y + y * right.w + z * right.x - x * right.z;
		float z_ = w * right.z + z * right.w + x * right.y - y * right.x;

		w = w_; x = x_; y = y_; z = z_;
	}

	Quaternion Quaternion::operator *(float right) const { Quaternion temp(*this); temp *= right; return temp; }
	void Quaternion::operator *=(float right) { w *= right; x *= right; y *= right; z *= right; }

	Quaternion Quaternion::operator /(float right) const { Quaternion temp(*this); temp /= right; return temp; }
	void Quaternion::operator /=(float right) { *this *= (1.0f / right); }

	Quaternion Quaternion::operator +(const Quaternion& right) const { Quaternion temp(*this); temp += right; return temp; }
	void Quaternion::operator +=(const Quaternion& right) { w += right.w; x += right.x; y += right.y; z += right.z; }

	Quaternion Quaternion::operator -(const Quaternion& right) const { Quaternion temp(*this); temp -= right; return temp; }
	void Quaternion::operator -=(const Quaternion& right) { w -= right.w; x -= right.x; y -= right.y; z -= right.z; }

	Quaternion Quaternion::Identity() { return Quaternion(1.0f, 0.0, 0.0, 0.0); }

	Quaternion Quaternion::FromRotationMatrix(const Mat3& mat)
	{
		const float* arr = mat.values;

		float t = arr[0] + arr[4] + arr[8] + 1.0f;
		if (t > 0)
		{
			float s = 0.5f / sqrt(t);
			return Quaternion::Normalize(Quaternion(
				0.25f / s,
				(arr[7] - arr[5]) * s,
				(arr[2] - arr[6]) * s,
				(arr[3] - arr[1]) * s
			));
		}
		else
		{
			if (arr[0] > arr[4] && arr[0] > arr[8])
			{
				float s = 2.0f * sqrt(1.0f + arr[0] - arr[4] - arr[8]);
				return Quaternion::Normalize(Quaternion(
					(arr[5] + arr[7]) / s,
					0.25f * s,
					(arr[1] + arr[3]) / s,
					(arr[2] + arr[6]) / s
				));
			}
			else if (arr[4] > arr[0] && arr[4] > arr[8])
			{
				float s = 2.0f * sqrt(1.0f + arr[4] - arr[0] - arr[8]);
				return Quaternion::Normalize(Quaternion(
					(arr[2] + arr[6]) / s,
					(arr[1] + arr[3]) / s,
					0.25f * s,
					(arr[5] + arr[7]) / s
				));
			}
			else
			{
				float s = 2.0f * sqrt(1.0f + arr[8] - arr[0] - arr[4]);
				return Quaternion::Normalize(Quaternion(
					(arr[1] + arr[3]) / s,
					(arr[2] + arr[6]) / s,
					(arr[5] + arr[7]) / s,
					0.25f * s
				));
			}
		}
	}

	Quaternion Quaternion::FromAxisAngle(float x, float y, float z, float angle)
	{
		float half = angle * 0.5f, sine = sin(half);
		return Quaternion(
			cos(half),
			x * sine,
			y * sine,
			z * sine
		);
	}

	Quaternion Quaternion::FromPYR(const Vec3& pyrVector) { return Quaternion::FromPYR(pyrVector.x, pyrVector.y, pyrVector.z); }
	Quaternion Quaternion::FromPYR(float p, float y, float r)
	{
		float mag = Vec3::Magnitude(p, y, r), inv = (mag == 0 ? 1.0f : 1.0f / mag);
		return Quaternion::FromAxisAngle(p * inv, y * inv, r * inv, mag);
	}

	Quaternion Quaternion::Normalize(const Quaternion& q) { return q / q.Norm(); }




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
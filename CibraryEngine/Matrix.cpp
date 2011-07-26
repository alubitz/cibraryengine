#include "Matrix.h"
#include "Vector.h"
#include "Quaternion.h"

namespace CibraryEngine
{
	/*
	 * Mat3 methods
	 */
	Mat3::Mat3()
	{
		for(int i = 0; i < 9; i++)
			values[i] = 0.0;
	}

	Mat3::Mat3(float values_[9])
	{
		for(int i = 0; i < 9; i++)
			values[i] = values_[i];
	}

	float& Mat3::operator [](int index)
	{
		return values[index];
	}

	Mat3 Mat3::Transpose()
	{
		float values_[] = { values[0], values[3], values[6], values[1], values[4], values[7], values[2], values[5], values[8] };
		return Mat3(values_);
	}

	float Mat3::Determinant()
	{
		return values[0] * values[4] * values[8] + values[1] * values[5] * values[6] + values[2] * values[3] * values[7] - values[0] * values[5] * values[7] - values[1] * values[3] * values[8] - values[2] * values[5] * values[7];
	}

	Mat3 Mat3::FromAxisAngle(float x, float y, float z, float angle)
	{
		float costheta = cos(angle);
		float sintheta = sin(angle);
		float oneminuscostheta = 1.0 - costheta;
		float xy = x * y;
		float xz = x * z;
		float yz = y * z;
		float values[] =
		{
			(float)(costheta + oneminuscostheta * x * x),
			(float)(oneminuscostheta * xy - z * sintheta),
			(float)(oneminuscostheta * xz + sintheta * y),
			(float)(oneminuscostheta * xy + sintheta * z),
			(float)(costheta + oneminuscostheta * y * y),
			(float)(oneminuscostheta * yz - sintheta * x),
			(float)(oneminuscostheta * xz - sintheta * y),
			(float)(oneminuscostheta * yz + sintheta * x),
			(float)(costheta + oneminuscostheta * z * z)
		};
		return Mat3(values);
	}

	Mat3 Mat3::FromScaledAxis(Vec3 xyz) { return FromScaledAxis(xyz.x, xyz.y, xyz.z); }
	Mat3 Mat3::FromScaledAxis(float x, float y, float z)
	{
		float mag = sqrt(x * x + y * y + z * z);

		if (mag == 0)
			return Identity();

		float inv = 1.0 / mag;
		return FromAxisAngle(x * inv, y * inv, z * inv, mag);
	}

	Mat3 Mat3::Identity()
	{
		float values[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
		return Mat3(values);
	}

	Mat3 Mat3::Normalize(Mat3 mat)
	{
		Vec3 a(mat.values[0], mat.values[1], mat.values[2]);
		Vec3 b(mat.values[3], mat.values[4], mat.values[5]);

		Vec3 c = Vec3::Cross(a, b);

		a /= a.ComputeMagnitude();
		float values[] = { a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z };
		return Mat3(values);
	}

	Mat3 Mat3::Invert(Mat3 a)
	{
		float inv = 1.0 / a.Determinant();
		Vec3 x0(a[0], a[3], a[6]);
		Vec3 x1(a[1], a[4], a[7]);
		Vec3 x2(a[2], a[5], a[8]);
		Vec3 y0 = Vec3::Cross(x1, x2);
		Vec3 y1 = Vec3::Cross(x2, x0);
		Vec3 y2 = Vec3::Cross(x0, x1);
		float values[] = {
			y0.x * inv, y0.y * inv, y0.z * inv,
			y1.x * inv, y1.y * inv, y1.z * inv,
			y2.x * inv, y2.y * inv, y2.z * inv
		};
		return Mat3(values);
	}
	
	/*
	 * Mat3 methods
	 */
	Vec3 operator *(Mat3 a, Vec3 b)
	{
		float* avals = &a.values[0];
		return Vec3(
			b.x * avals[0] + b.y * avals[1] + b.z * avals[2],
			b.x * avals[3] + b.y * avals[4] + b.z * avals[5],
			b.x * avals[6] + b.y * avals[7] + b.z * avals[8]
		);
	}

	void operator *=(Mat3& a, Mat3 b)
	{
		float* avals = &a.values[0];
		float* bvals = &b.values[0];
		float nuvals[9];

		nuvals[0] = avals[0] * bvals[0] + avals[1] * bvals[3] + avals[2] * bvals[6];
		nuvals[1] = avals[0] * bvals[1] + avals[1] * bvals[4] + avals[2] * bvals[7];
		nuvals[2] = avals[0] * bvals[2] + avals[1] * bvals[5] + avals[2] * bvals[8];
		nuvals[3] = avals[3] * bvals[0] + avals[4] * bvals[3] + avals[5] * bvals[6];
		nuvals[4] = avals[3] * bvals[1] + avals[4] * bvals[4] + avals[5] * bvals[7];
		nuvals[5] = avals[3] * bvals[2] + avals[4] * bvals[5] + avals[5] * bvals[8];
		nuvals[6] = avals[6] * bvals[0] + avals[7] * bvals[3] + avals[8] * bvals[6];
		nuvals[7] = avals[6] * bvals[1] + avals[7] * bvals[4] + avals[8] * bvals[7];
		nuvals[8] = avals[6] * bvals[2] + avals[7] * bvals[5] + avals[8] * bvals[8];

		for(int i = 0; i < 9; i++)
			avals[i] = nuvals[i];
	}
	Mat3 operator *(Mat3 a, Mat3 b) { Mat3 result(a); result *= b; return result; }




	/*
	 * Mat4 methods
	 */
	Mat4::Mat4()
	{
		for(int i = 0; i < 16; i++)
			values[i] = 0.0;
	}

	Mat4::Mat4(float values_[16])
	{
		for(int i = 0; i < 16; i++)
			values[i] = values_[i];
	}

	float& Mat4::operator[](int index)
	{
		return values[index];
	}

	Mat4 Mat4::Transpose()
	{
		float values_[] = {
			values[0],	values[4],	values[8],	values[12],
			values[1],	values[5],	values[9],	values[13],
			values[2],	values[6],	values[10],	values[14],
			values[3],	values[7],	values[11],	values[15]
		};
		return Mat4(values_);
	}

	Vec3 Mat4::TransformVec3(Vec3 xyz, float w) { return TransformVec3(xyz.x, xyz.y, xyz.z, w); }
	Vec3 Mat4::TransformVec3(float x, float y, float z, float w)
	{
		Vec4 xformed = *this * Vec4(x, y, z, w);
		float inv_w = xformed.w == 0 ? 1.0f : 1.0f / xformed.w;
		return Vec3(xformed.x * inv_w, xformed.y * inv_w, xformed.z * inv_w);
	}

	Mat4 Mat4::Identity()
	{
		float values[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
		return Mat4(values);
	}

	Mat4 Mat4::FromMat3(Mat3 mat)
	{
		float values[] = {
			mat[0],	mat[1],	mat[2],		0,
			mat[3],	mat[4],	mat[5],		0,
			mat[6],	mat[7],	mat[8],		0,
			0,		0,			0,		1
		};
		return Mat4(values);
	}

	Mat4 Mat4::FromQuaternion(Quaternion q) { return Mat4::FromMat3(q.ToMat3()); }

	Mat4 Mat4::FromPositionAndOrientation(Vec3 pos, Quaternion ori) { return Mat4::FromPositionAndOrientation(pos, ori.ToMat3()); }
	Mat4 Mat4::FromPositionAndOrientation(Vec3 pos, Mat3 mat)
	{
		float values[] = 
		{
			mat[0],	mat[3],	mat[6],	pos.x,
			mat[1],	mat[4],	mat[7],	pos.y,
			mat[2],	mat[5],	mat[8],	pos.z,
			0,		0,		0,		1
		};
		return Mat4(values);
	}

	Mat4 Mat4::FromPosOriScale(Vec3 pos, Quaternion ori, float scale) { return Mat4::FromPosOriScale(pos, ori.ToMat3(), scale); }
	Mat4 Mat4::FromPosOriScale(Vec3 pos, Mat3 ori, float scale)
	{
		float values[] = {
			ori[0] * scale,	ori[3] * scale,	ori[6] * scale,	pos.x,
			ori[1] * scale,	ori[4] * scale,	ori[7] * scale,	pos.y,
			ori[2] * scale,	ori[5] * scale,	ori[8] * scale,	pos.z,
			0,				0,				0,				1
		};
		return Mat4(values);
	}

	Mat4 Mat4::Translation(Vec3 translation) { return Mat4::Translation(translation.x, translation.y, translation.z); }
	Mat4 Mat4::Translation(float x, float y, float z)
	{
		float values[] = {
			1, 0, 0, x,
			0, 1, 0, y,
			0, 0, 1, z,
			0, 0, 0, 1
		};
		return Mat4(values);
	}

	Mat4 Mat4::UniformScale(float scale) { return Mat4::Scale(scale, scale, scale); }
	Mat4 Mat4::Scale(float x, float y, float z)
	{
		float values[] = {
			x, 0, 0, 0,
			0, y, 0, 0,
			0, 0, z, 0,
			0, 0, 0, 1
		};
		return Mat4(values);
	}




	/*
	 * Mat4 math operators
	 */
	Vec4 operator *(Mat4 a, Vec4 b)
	{
		float* values = &a.values[0];
		return Vec4(
			b.x * values[0] + b.y * values[1] + b.z * values[2] + b.w * values[3],
			b.x * values[4] + b.y * values[5] + b.z * values[6] + b.w * values[7],
			b.x * values[8] + b.y * values[9] + b.z * values[10] + b.w * values[11],
			b.x * values[12] + b.y * values[13] + b.z * values[14] + b.w * values[15]
		);
	}

	void operator *=(Mat4& a, Mat4 b)
	{
		float* avals = &a.values[0];
		float* bvals = &b.values[0];
		float nuvals[] =
		{
			avals[0] * bvals[0] + avals[1] * bvals[4] + avals[2] * bvals[8] + avals[3] * bvals[12],
			avals[0] * bvals[1] + avals[1] * bvals[5] + avals[2] * bvals[9] + avals[3] * bvals[13],
			avals[0] * bvals[2] + avals[1] * bvals[6] + avals[2] * bvals[10] + avals[3] * bvals[14],
			avals[0] * bvals[3] + avals[1] * bvals[7] + avals[2] * bvals[11] + avals[3] * bvals[15],

			avals[4] * bvals[0] + avals[5] * bvals[4] + avals[6] * bvals[8] + avals[7] * bvals[12],
			avals[4] * bvals[1] + avals[5] * bvals[5] + avals[6] * bvals[9] + avals[7] * bvals[13],
			avals[4] * bvals[2] + avals[5] * bvals[6] + avals[6] * bvals[10] + avals[7] * bvals[14],
			avals[4] * bvals[3] + avals[5] * bvals[7] + avals[6] * bvals[11] + avals[7] * bvals[15],

			avals[8] * bvals[0] + avals[9] * bvals[4] + avals[10] * bvals[8] + avals[11] * bvals[12],
			avals[8] * bvals[1] + avals[9] * bvals[5] + avals[10] * bvals[9] + avals[11] * bvals[13],
			avals[8] * bvals[2] + avals[9] * bvals[6] + avals[10] * bvals[10] + avals[11] * bvals[14],
			avals[8] * bvals[3] + avals[9] * bvals[7] + avals[10] * bvals[11] + avals[11] * bvals[15],

			avals[12] * bvals[0] + avals[13] * bvals[4] + avals[14] * bvals[8] + avals[15] * bvals[12],
			avals[12] * bvals[1] + avals[13] * bvals[5] + avals[14] * bvals[9] + avals[15] * bvals[13],
			avals[12] * bvals[2] + avals[13] * bvals[6] + avals[14] * bvals[10] + avals[15] * bvals[14],
			avals[12] * bvals[3] + avals[13] * bvals[7] + avals[14] * bvals[11] + avals[15] * bvals[15]
		};
		for(int i = 0; i < 16; i++)
			avals[i] = nuvals[i];
	}
	Mat4 operator *(Mat4 a, Mat4 b) { Mat4 result(a); result *= b; return result; }

	void operator *=(Mat4& a, float b)
	{
		float* avals = &a.values[0];
		for(int i = 0; i < 16; i++)
			avals[i] *= b;
	}
	Mat4 operator *(Mat4 a, float b) { Mat4 result(a); result *= b; return result; }

	void operator +=(Mat4& a, Mat4 b)
	{
		float* avals = &a.values[0];
		float* bvals = &b.values[0];
		for(int i = 0; i < 16; i++)
			avals[i] += bvals[i];
	}
	Mat4 operator +(Mat4 a, Mat4 b) { Mat4 result(a); result *= b; return result; }

}

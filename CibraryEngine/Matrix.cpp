#include "StdAfx.h"
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
		for(int i = 0; i < 9; ++i)
			values[i] = 0.0;
	}

	Mat3::Mat3(float values_[9])
	{
		for(int i = 0; i < 9; ++i)
			values[i] = values_[i];
	}

	float& Mat3::operator [](int index)
	{
		return values[index];
	}

	Mat3 Mat3::Transpose() const
	{
		float values_[] = { values[0], values[3], values[6], values[1], values[4], values[7], values[2], values[5], values[8] };
		return Mat3(values_);
	}

	float Mat3::Determinant() const
	{
		return values[0] * values[4] * values[8] + values[1] * values[5] * values[6] + values[2] * values[3] * values[7] - values[0] * values[5] * values[7] - values[1] * values[3] * values[8] - values[2] * values[5] * values[7];
	}

	Mat3 Mat3::FromAxisAngle(float x, float y, float z, float angle)
	{
		float costheta = cosf(angle);
		float sintheta = sinf(angle);
		float oneminuscostheta = 1.0f - costheta;
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

	Mat3 Mat3::FromScaledAxis(const Vec3& xyz) { return FromScaledAxis(xyz.x, xyz.y, xyz.z); }
	Mat3 Mat3::FromScaledAxis(float x, float y, float z)
	{
		float mag = sqrt(x * x + y * y + z * z);

		if (mag == 0)
			return Identity();

		float inv = 1.0f / mag;
		return FromAxisAngle(x * inv, y * inv, z * inv, mag);
	}

	Mat3 Mat3::Identity()
	{
		float values[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
		return Mat3(values);
	}

	Mat3 Mat3::Normalize(const Mat3& mat)
	{
		Vec3 a(mat.values[0], mat.values[1], mat.values[2]);
		Vec3 b(mat.values[3], mat.values[4], mat.values[5]);

		Vec3 c = Vec3::Cross(a, b);

		a /= a.ComputeMagnitude();
		float values[] = { a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z };
		return Mat3(values);
	}

	Mat3 Mat3::Invert(const Mat3& a)
	{
		float inv = 1.0f / a.Determinant();
		const float* avals = a.values;
		Vec3 x0(avals[0], avals[3], avals[6]);
		Vec3 x1(avals[1], avals[4], avals[7]);
		Vec3 x2(avals[2], avals[5], avals[8]);
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
	 * Mat3 operators
	 */
	Vec3 Mat3::operator *(const Vec3& b) const
	{
		return Vec3(
			b.x * values[0] + b.y * values[1] + b.z * values[2],
			b.x * values[3] + b.y * values[4] + b.z * values[5],
			b.x * values[6] + b.y * values[7] + b.z * values[8]
		);
	}
	void Mat3::operator *=(const Mat3& b)
	{
		const float* bvals = b.values;

		float nuvals[] =
		{
			values[0] * bvals[0] + values[1] * bvals[3] + values[2] * bvals[6],
			values[0] * bvals[1] + values[1] * bvals[4] + values[2] * bvals[7],
			values[0] * bvals[2] + values[1] * bvals[5] + values[2] * bvals[8],
			values[3] * bvals[0] + values[4] * bvals[3] + values[5] * bvals[6],
			values[3] * bvals[1] + values[4] * bvals[4] + values[5] * bvals[7],
			values[3] * bvals[2] + values[4] * bvals[5] + values[5] * bvals[8],
			values[6] * bvals[0] + values[7] * bvals[3] + values[8] * bvals[6],
			values[6] * bvals[1] + values[7] * bvals[4] + values[8] * bvals[7],
			values[6] * bvals[2] + values[7] * bvals[5] + values[8] * bvals[8]
		};

		for(int i = 0; i < 9; ++i)
			values[i] = nuvals[i];
	}
	Mat3 Mat3::operator *(const Mat3& b) const { Mat3 result(*this); result *= b; return result; }




	/*
	 * Mat4 methods
	 */
	Mat4::Mat4()
	{
		for(int i = 0; i < 16; ++i)
			values[i] = 0.0;
	}

	Mat4::Mat4(float values_[16])
	{
		for(int i = 0; i < 16; ++i)
			values[i] = values_[i];
	}

	float& Mat4::operator[](int index) { return values[index]; }

	Mat4 Mat4::Transpose() const
	{
		float values_[] = {
			values[0],	values[4],	values[8],	values[12],
			values[1],	values[5],	values[9],	values[13],
			values[2],	values[6],	values[10],	values[14],
			values[3],	values[7],	values[11],	values[15]
		};
		return Mat4(values_);
	}

	Vec3 Mat4::TransformVec3(const Vec3& xyz, float w) const { return TransformVec3(xyz.x, xyz.y, xyz.z, w); }
	Vec3 Mat4::TransformVec3(float x, float y, float z, float w) const
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

	Mat4 Mat4::FromMat3(const Mat3& mat)
	{
		const float* arr = mat.values;
		float values[] = {
			arr[0],	arr[1],	arr[2],		0,
			arr[3],	arr[4],	arr[5],		0,
			arr[6],	arr[7],	arr[8],		0,
			0,		0,			0,		1
		};
		return Mat4(values);
	}

	Mat4 Mat4::FromQuaternion(const Quaternion& q) { return Mat4::FromMat3(q.ToMat3()); }

	Mat4 Mat4::FromPositionAndOrientation(const Vec3& pos, const Quaternion& ori) { return Mat4::FromPositionAndOrientation(pos, ori.ToMat3()); }
	Mat4 Mat4::FromPositionAndOrientation(const Vec3& pos, const Mat3& mat)
	{
		const float* arr = mat.values;
		float values[] = 
		{
			arr[0],	arr[3],	arr[6],	pos.x,
			arr[1],	arr[4],	arr[7],	pos.y,
			arr[2],	arr[5],	arr[8],	pos.z,
			0,		0,		0,		1
		};
		return Mat4(values);
	}

	Mat4 Mat4::FromPosOriScale(const Vec3& pos, const Quaternion& ori, float scale) { return Mat4::FromPosOriScale(pos, ori.ToMat3(), scale); }
	Mat4 Mat4::FromPosOriScale(const Vec3& pos, const Mat3& ori, float scale)
	{
		const float* arr = ori.values;
		float values[] = {
			arr[0] * scale,	arr[3] * scale,	arr[6] * scale,	pos.x,
			arr[1] * scale,	arr[4] * scale,	arr[7] * scale,	pos.y,
			arr[2] * scale,	arr[5] * scale,	arr[8] * scale,	pos.z,
			0,				0,				0,				1
		};
		return Mat4(values);
	}

	Mat4 Mat4::Translation(const Vec3& translation) { return Mat4::Translation(translation.x, translation.y, translation.z); }
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
	Mat4 Mat4::Scale(const Vec3& vec) { return Mat4::Scale(vec.x, vec.y, vec.z); }
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

	void row_swap(float*& r1, float*& r2)
	{
		float* temp = r1;
		r1 = r2;
		r2 = temp;
	}
	void row_mult(float* row, float coeff)
	{
		for(int i = 0; i < 8; ++i)
			row[i] *= coeff;
	}
	void row_combine(float* from, float* to, float coeff)
	{
		for(int i = 0; i < 8; ++i)
			to[i] += from[i] * coeff;
	}

	Mat4 Mat4::Invert(const Mat4& a)
	{
		const float* v = a.values;
		float ra[] = {	v[0],	v[1],	v[2],	v[3],	1, 0, 0, 0 };
		float rb[] = {	v[4],	v[5],	v[6],	v[7],	0, 1, 0, 0 };
		float rc[] = {	v[8],	v[9],	v[10],	v[11],	0, 0, 1, 0 };
		float rd[] = {	v[12],	v[13],	v[14],	v[15],	0, 0, 0, 1 };

		float *r[] = { ra, rb, rc, rd };

		// working on first column...
		for(int i = 0; i < 3; ++i)
			for(int j = 0; j < 3 - i; ++j)
				if(fabs(r[j][0]) < fabs(r[j + 1][0]))
					row_swap(r[j], r[j + 1]);
		row_mult(r[0], 1.0f / r[0][0]);
		if(r[1][0] != 0)
			row_combine(r[0], r[1], -r[1][0]);
		if(r[2][0] != 0)
			row_combine(r[0], r[2], -r[2][0]);
		if(r[3][0] != 0)
			row_combine(r[0], r[3], -r[3][0]);

		// working on second column
		for(int i = 0; i < 2; ++i)
			for(int j = 1; j < 3 - i; ++j)
				if(fabs(r[j][1]) < fabs(r[j + 1][1]))
					row_swap(r[j], r[j + 1]);
		row_mult(r[1], 1.0f / r[1][1]);
		if(r[0][1] != 0)
			row_combine(r[1], r[0], -r[0][1]);
		if(r[2][1] != 0)
			row_combine(r[1], r[2], -r[2][1]);
		if(r[3][1] != 0)
			row_combine(r[1], r[3], -r[3][1]);

		// working on third column
		if(fabs(r[2][2]) < fabs(r[3][2]))
			row_swap(r[2], r[3]);
		row_mult(r[2], 1.0f / r[2][2]);
		if(r[0][2] != 0)
			row_combine(r[2], r[0], -r[0][2]);
		if(r[1][2] != 0)
			row_combine(r[2], r[1], -r[1][2]);
		if(r[3][2] != 0)
			row_combine(r[2], r[3], -r[3][2]);

		// working on fourth column
		if(r[3][3] == 0)
			return Mat4();					// SINGULAR MATRIX!
		row_mult(r[3], 1.0f / r[3][3]);
		if(r[0][3] != 0)
			row_combine(r[3], r[0], -r[0][3]);
		if(r[1][3] != 0)
			row_combine(r[3], r[1], -r[1][3]);
		if(r[2][3] != 0)
			row_combine(r[3], r[2], -r[2][3]);

		// consolidating results
		float result[] =
		{
			r[0][4],	r[0][5],	r[0][6],	r[0][7],
			r[1][4],	r[1][5],	r[1][6],	r[1][7],
			r[2][4],	r[2][5],	r[2][6],	r[2][7],
			r[3][4],	r[3][5],	r[3][6],	r[3][7]
		};

		return Mat4(result);
	}




	/*
	 * Mat4 math operators
	 */
	Vec4 Mat4::operator *(const Vec4& b) const
	{
		return Vec4(
			b.x * values[0] + b.y * values[1] + b.z * values[2] + b.w * values[3],
			b.x * values[4] + b.y * values[5] + b.z * values[6] + b.w * values[7],
			b.x * values[8] + b.y * values[9] + b.z * values[10] + b.w * values[11],
			b.x * values[12] + b.y * values[13] + b.z * values[14] + b.w * values[15]
		);
	}

	void Mat4::operator *=(const Mat4& b)
	{
		const float* bvals = b.values;

		float nuvals[] =
		{
			values[0] * bvals[0] + values[1] * bvals[4] + values[2] * bvals[8] + values[3] * bvals[12],
			values[0] * bvals[1] + values[1] * bvals[5] + values[2] * bvals[9] + values[3] * bvals[13],
			values[0] * bvals[2] + values[1] * bvals[6] + values[2] * bvals[10] + values[3] * bvals[14],
			values[0] * bvals[3] + values[1] * bvals[7] + values[2] * bvals[11] + values[3] * bvals[15],

			values[4] * bvals[0] + values[5] * bvals[4] + values[6] * bvals[8] + values[7] * bvals[12],
			values[4] * bvals[1] + values[5] * bvals[5] + values[6] * bvals[9] + values[7] * bvals[13],
			values[4] * bvals[2] + values[5] * bvals[6] + values[6] * bvals[10] + values[7] * bvals[14],
			values[4] * bvals[3] + values[5] * bvals[7] + values[6] * bvals[11] + values[7] * bvals[15],

			values[8] * bvals[0] + values[9] * bvals[4] + values[10] * bvals[8] + values[11] * bvals[12],
			values[8] * bvals[1] + values[9] * bvals[5] + values[10] * bvals[9] + values[11] * bvals[13],
			values[8] * bvals[2] + values[9] * bvals[6] + values[10] * bvals[10] + values[11] * bvals[14],
			values[8] * bvals[3] + values[9] * bvals[7] + values[10] * bvals[11] + values[11] * bvals[15],

			values[12] * bvals[0] + values[13] * bvals[4] + values[14] * bvals[8] + values[15] * bvals[12],
			values[12] * bvals[1] + values[13] * bvals[5] + values[14] * bvals[9] + values[15] * bvals[13],
			values[12] * bvals[2] + values[13] * bvals[6] + values[14] * bvals[10] + values[15] * bvals[14],
			values[12] * bvals[3] + values[13] * bvals[7] + values[14] * bvals[11] + values[15] * bvals[15]
		};
		for(int i = 0; i < 16; ++i)
			values[i] = nuvals[i];
	}
	Mat4 Mat4::operator *(const Mat4& b) const { Mat4 result(*this); result *= b; return result; }

	void Mat4::operator *=(float b)
	{
		for(int i = 0; i < 16; ++i)
			values[i] *= b;
	}
	Mat4 Mat4::operator *(float b) const { Mat4 result(*this); result *= b; return result; }

	void Mat4::operator +=(const Mat4& b)
	{
		const float* bvals = b.values;
		for(int i = 0; i < 16; ++i)
			values[i] += bvals[i];
	}
	Mat4 Mat4::operator +(const Mat4& b) const { Mat4 result(*this); result *= b; return result; }

}

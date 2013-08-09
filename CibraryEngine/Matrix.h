#pragma once

#include "StdAfx.h"
#include "Vector.h"

namespace CibraryEngine
{
	using namespace std;

	struct Quaternion;

	/** Class representing a 3x3 matrix */
	struct Mat3
	{
		/** The 9 elements of the matrix */
		float values[9];

		/** Initializes a zero matrix */
		Mat3() { for(int i = 0; i < 9; ++i) { values[i] = 0.0; } }
		/** Initializes a matrix with the specified values */
		Mat3(const float values_[9]) { for(int i = 0; i < 9; ++i) { values[i] = values_[i]; } }
		/** Initializes a matrix with the specified values */
		Mat3(float v11, float v12, float v13, float v21, float v22, float v23, float v31, float v32, float v33)
		{
			values[0] = v11; values[1] = v12; values[2] = v13;
			values[3] = v21; values[4] = v22; values[5] = v23;
			values[6] = v31; values[7] = v32; values[8] = v33;
		}

		/** Returns a reference to the specified element of the matrix */
		float& operator[](int index)	{ return values[index]; }

		/** Returns the transpose of this matrix */
		Mat3 Transpose() const			{ return Mat3(values[0], values[3], values[6], values[1], values[4], values[7], values[2], values[5], values[8]); }
		/** Returns the determinant of this matrix */
		float Determinant() const		{ return values[0] * values[4] * values[8] + values[1] * values[5] * values[6] + values[2] * values[3] * values[7] - values[0] * values[5] * values[7] - values[1] * values[3] * values[8] - values[2] * values[5] * values[7]; }

		/** Returns a 3x3 matrix which represents a rotation about the specified axis, by the specified angle. The axis should be normalized. */
		static Mat3 FromAxisAngle(float x, float y, float z, float angle)
		{
			float costheta = cosf(angle);
			float sintheta = sinf(angle);
			float oneminuscostheta = 1.0f - costheta;
			float cxy = oneminuscostheta * x * y;
			float cxz = oneminuscostheta * x * z;
			float cyz = oneminuscostheta * y * z;
			float sx = sintheta * x;
			float sy = sintheta * y;
			float sz = sintheta * z;
			return Mat3(
				costheta + oneminuscostheta * x * x,	cxy - sz,								cxz + sy,
				cxy + sz,								costheta + oneminuscostheta * y * y,	cyz - sx,
				cxz - sy,								cyz + sx,								costheta + oneminuscostheta * z * z
			);
		}
		/** Returns a 3x3 matrix which represents a rotation about the specified axis, where the angle of rotation is the magnitude of the axis vector */
		static Mat3 FromScaledAxis(float x, float y, float z)
		{
			if(float magsq = Vec3::MagnitudeSquared(x, y, z))
			{
				float mag = sqrtf(magsq), inv = 1.0f / mag;
				return FromAxisAngle(x * inv, y * inv, z * inv, mag);
			}
			else
				return Identity();
		}
		/** Returns a 3x3 matrix which represents a rotation about the specified axis, where the angle of rotation is the magnitude of the axis vector */
		static Mat3 FromScaledAxis(const Vec3& xyz)		{ return FromScaledAxis(xyz.x, xyz.y, xyz.z); }
		/** Returns the identity matrix */
		static Mat3 Identity()							{ return Mat3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0); }

		/** Returns an orthonormal 3x3 matrix approximately matching the one given */
		static Mat3 Normalize(const Mat3& mat)
		{
			Vec3 a(mat.values[0], mat.values[1], mat.values[2]);
			Vec3 b(mat.values[3], mat.values[4], mat.values[5]);

			Vec3 c = Vec3::Cross(a, b);

			a /= a.ComputeMagnitude();

			return Mat3(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
		}

		/** Returns the matrix inverse of the given matrix */
		static Mat3 Invert(const Mat3& a);

		/** Transforms a 3-component vector by this matrix (i.e. matrix multiplication) */
		Vec3 operator *(const Vec3& b) const
		{
			return Vec3(
				b.x * values[0] + b.y * values[1] + b.z * values[2],
				b.x * values[3] + b.y * values[4] + b.z * values[5],
				b.x * values[6] + b.y * values[7] + b.z * values[8]);
		}
		/** Transforms a 3x3 matrix by another 3x3 matrix */
		void operator *=(const Mat3& b)
		{
			const float* bvals = b.values;

			*this = Mat3(
				values[0] * bvals[0] + values[1] * bvals[3] + values[2] * bvals[6],
				values[0] * bvals[1] + values[1] * bvals[4] + values[2] * bvals[7],
				values[0] * bvals[2] + values[1] * bvals[5] + values[2] * bvals[8],
				values[3] * bvals[0] + values[4] * bvals[3] + values[5] * bvals[6],
				values[3] * bvals[1] + values[4] * bvals[4] + values[5] * bvals[7],
				values[3] * bvals[2] + values[4] * bvals[5] + values[5] * bvals[8],
				values[6] * bvals[0] + values[7] * bvals[3] + values[8] * bvals[6],
				values[6] * bvals[1] + values[7] * bvals[4] + values[8] * bvals[7],
				values[6] * bvals[2] + values[7] * bvals[5] + values[8] * bvals[8]
			);
		}
		/** Transforms a 3x3 matrix by another 3x3 matrix */
		Mat3 operator *(const Mat3& b) const { Mat3 result(*this); result *= b; return result; }
		/** Scales the components of a 3x3 matrix by the specified amount */
		void operator *=(float b)						{ for(int i = 0; i < 9; ++i) { values[i] *= b; } }
		/** Scales the components of a 3x3 matrix by the specified amount */
		Mat3 operator *(float b) const					{ Mat3 result(*this); result *= b; return result; }
		/** Adds two matrices */
		void operator +=(const Mat3& b)					{ const float* bvals = b.values; for(int i = 0; i < 9; ++i) { values[i] += bvals[i]; } }
		/** Adds two matrices */
		Mat3 operator +(const Mat3& b) const			{ Mat3 result(*this); result += b; return result; }
		/** Subtracts two matrices */
		void operator -=(const Mat3& b)					{ const float* bvals = b.values; for(int i = 0; i < 9; ++i) { values[i] -= bvals[i]; } }
		/** Subtractss two matrices */
		Mat3 operator -(const Mat3& b) const			{ Mat3 result(*this); result -= b; return result; }
	};

	/** Class representing a 4x4 matrix */
	struct Mat4
	{
		/** The 16 elements of the matrix */
		float values[16];

		/** Initializes a zero matrix */
		Mat4() { for(int i = 0; i < 16; ++i) { values[i] = 0.0; } }
		/** Initializes a matrix with the specified values */
		Mat4(float values_[16]) { for(int i = 0; i < 16; ++i) { values[i] = values_[i]; } }
		/** Initializes a matrix with the specified values */
		Mat4(float v11, float v12, float v13, float v14, float v21, float v22, float v23, float v24, float v31, float v32, float v33, float v34, float v41, float v42, float v43, float v44)
		{
			values[0] = v11;	values[1] = v12;	values[2] = v13;	values[3] = v14;
			values[4] = v21;	values[5] = v22;	values[6] = v23;	values[7] = v24;
			values[8] = v31;	values[9] = v32;	values[10] = v33;	values[11] = v34;
			values[12] = v41;	values[13] = v42;	values[14] = v43;	values[15] = v44;
		}

		/** Returns a reference to the specified element of the matrix */
		float& operator[](int index) { return values[index]; }

		

		/** Returns the transpose of this matrix */
		Mat4 Transpose() const
		{
			return Mat4(
				values[0],	values[4],	values[8],	values[12],
				values[1],	values[5],	values[9],	values[13],
				values[2],	values[6],	values[10],	values[14],
				values[3],	values[7],	values[11],	values[15]
			);
		}

		/** Returns the result of transforming the 3-component vector xyz with homogeneous component w by this matrix, divided by the results' homogeneous component */
		Vec3 TransformVec3(const Vec3& xyz, float w) const		{ return TransformVec3(xyz.x, xyz.y, xyz.z, w); }
		/** Returns the result of transforming the 3-component vector xyz with homogeneous component w by this matrix, divided by the results' homogeneous component */
		Vec3 TransformVec3(float x, float y, float z, float w) const
		{
			Vec4 xformed = *this * Vec4(x, y, z, w);
			if(xformed.w == 0 || xformed.w == 1)
				return Vec3(xformed.x, xformed.y, xformed.z);
			else
			{
				float inv_w = 1.0f / xformed.w;
				return Vec3(xformed.x * inv_w, xformed.y * inv_w, xformed.z * inv_w);
			}
		}

		Vec3 TransformVec3_0(const Vec3& xyz) const				{ return TransformVec3_0(xyz.x, xyz.y, xyz.z); }
		Vec3 TransformVec3_1(const Vec3& xyz) const				{ return TransformVec3_1(xyz.x, xyz.y, xyz.z); }
		Vec3 TransformVec3_0(float x, float y, float z) const	{ Vec4 xformed = *this * Vec4(x, y, z, 0); return Vec3(xformed.x, xformed.y, xformed.z); }
		Vec3 TransformVec3_1(float x, float y, float z) const	{ Vec4 xformed = *this * Vec4(x, y, z, 1); return Vec3(xformed.x, xformed.y, xformed.z); }

		/** Returns the identity matrix */
		static Mat4 Identity()									{ return Mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1); }
		/** Returns a 4x4 matrix with the given 3x3 matrix as orientation */
		static Mat4 FromMat3(const Mat3& mat)
		{
			const float* arr = mat.values;
			return Mat4(
				arr[0],	arr[1],	arr[2],		0,
				arr[3],	arr[4],	arr[5],		0,
				arr[6],	arr[7],	arr[8],		0,
				0,		0,			0,		1
			);
		}
		/** Returns a 4x4 matrix with the given quaternion as orientation */
		static Mat4 FromQuaternion(const Quaternion& q);
		/** Returns a 4x4 matrix with the specified orientation and translation */
		static Mat4 FromPositionAndOrientation(const Vec3& pos, const Mat3& ori)
		{
			const float* arr = ori.values;
			return Mat4(
				arr[0],	arr[3],	arr[6],	pos.x,
				arr[1],	arr[4],	arr[7],	pos.y,
				arr[2],	arr[5],	arr[8],	pos.z,
				0,		0,		0,		1
			);
		}
		/** Returns a 4x4 matrix with the specified orientation and translation */
		static Mat4 FromPositionAndOrientation(const Vec3& pos, const Quaternion& ori);
		/** Returns a 4x4 matrix with the specified orientation, translation, and uniform scale */
		static Mat4 FromPosOriScale(const Vec3& pos, const Mat3& ori, float scale)
		{
			const float* arr = ori.values;
			return Mat4(
				arr[0] * scale,	arr[3] * scale,	arr[6] * scale,	pos.x,
				arr[1] * scale,	arr[4] * scale,	arr[7] * scale,	pos.y,
				arr[2] * scale,	arr[5] * scale,	arr[8] * scale,	pos.z,
				0,				0,				0,				1
			);
		}
		/** Returns a 4x4 matrix with the specified orientation, translation, and uniform scale */
		static Mat4 FromPosOriScale(const Vec3& pos, const Quaternion& ori, float scale);
		/** Returns a 4x4 matrix representing a rotation around the specified point */
		static Mat4 RotationAroundPoint(const Mat3& rot, const Vec3& point);
		/** Returns a translation matrix */
		static Mat4 Translation(const Vec3& translation)		{ return Mat4::Translation(translation.x, translation.y, translation.z); }
		/** Returns a translation matrix */
		static Mat4 Translation(float x, float y, float z)
		{
			return Mat4(
				1, 0, 0, x,
				0, 1, 0, y,
				0, 0, 1, z,
				0, 0, 0, 1
			);
		}
		/** Returns a uniform scale matrix */
		static Mat4 UniformScale(float scale)					{ return Mat4::Scale(scale, scale, scale); }
		/** Returns a non-uniform scale matrix, scaling each axis by the corresponding parameter */
		static Mat4 Scale(float x, float y, float z)
		{
			return Mat4(
				x, 0, 0, 0,
				0, y, 0, 0,
				0, 0, z, 0,
				0, 0, 0, 1
			);
		}
		/** Returns a non-uniform scale matrix, scaling each axis by the corresponding parameter */
		static Mat4 Scale(const Vec3& vec)						{ return Mat4::Scale(vec.x, vec.y, vec.z); }
		/** Returns the matrix inverse of the given matrix */
		static Mat4 Invert(const Mat4& a);

		/** Decomposes this matrix into a translation, orientation, and scale; assumes no perspective */
		void Decompose(Vec3& translation, Quaternion& orientation, Vec3& scale) const;

		/** Decomposes this matrix into a translation, and orientation; assumes no perspective and no scaling */
		void Decompose(Vec3& translation, Quaternion& orientation) const;

		/** Extracts the orientation component from this matrix transform; assumes no perspective and no scaling */
		Quaternion ExtractOrientation() const;



		/** Transforms a 4-component vector by a 4x4 matrix */
		Vec4 operator *(const Vec4& b) const
		{
			return Vec4(
				b.x * values[0] + b.y * values[1] + b.z * values[2] + b.w * values[3],
				b.x * values[4] + b.y * values[5] + b.z * values[6] + b.w * values[7],
				b.x * values[8] + b.y * values[9] + b.z * values[10] + b.w * values[11],
				b.x * values[12] + b.y * values[13] + b.z * values[14] + b.w * values[15]
			);
		}
		/** Transforms a 4x4 matrix by another 4x4 matrix */
		void operator *=(const Mat4& b)
		{
			const float* bvals = b.values;

			*this = Mat4(
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
			);
		}
		/** Transforms a 4x4 matrix by another 4x4 matrix */
		Mat4 operator *(const Mat4& b) const			{ Mat4 result(*this); result *= b; return result; }
		/** Scales the components of a 4x4 matrix by the specified amount */
		void operator *=(float b)						{ for(int i = 0; i < 16; ++i) { values[i] *= b; } }
		/** Scales the components of a 4x4 matrix by the specified amount */
		Mat4 operator *(float b) const					{ Mat4 result(*this); result *= b; return result; }
		/** Adds two matrices */
		void operator +=(const Mat4& b)					{ const float* bvals = b.values; for(int i = 0; i < 16; ++i) { values[i] += bvals[i]; } }
		/** Adds two matrices */
		Mat4 operator +(const Mat4& b) const			{ Mat4 result(*this); result += b; return result; }
		/** Subtracts two matrices */
		void operator -=(const Mat4& b)					{ const float* bvals = b.values; for(int i = 0; i < 16; ++i) { values[i] -= bvals[i]; } }
		/** Subtracts two matrices */
		Mat4 operator -(const Mat4& b) const			{ Mat4 result(*this); result -= b; return result; }
	};




	void WriteMat3(const Mat3& mat, ostream& stream);
	void WriteMat4(const Mat4& mat, ostream& stream);

	Mat3 ReadMat3(istream& stream);
	Mat4 ReadMat4(istream& stream);
}

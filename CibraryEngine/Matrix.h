#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	struct Vec3;
	struct Vec4;
	struct Quaternion;

	/** Class representing a 3x3 matrix */
	struct Mat3
	{
		/** The 9 elements of the matrix */
		float values[9];

		/** Initializes a zero matrix */
		Mat3();
		/** Initializes a matrix with the specified values */
		Mat3(float values_[9]);
		/** Initializes a matrix with the specified values */
		Mat3(float v11, float v12, float v13, float v21, float v22, float v23, float v31, float v32, float v33);

		/** Returns a reference to the specified element of the matrix */
		float& operator[](int index);

		/** Returns the transpose of this matrix */
		Mat3 Transpose() const;
		/** Returns the determinant of this matrix */
		float Determinant() const;

		/** Returns a 3x3 matrix which represents a rotation about the specified axis, by the specified angle. The axis should be normalized. */
		static Mat3 FromAxisAngle(float x, float y, float z, float angle);
		/** Returns a 3x3 matrix which represents a rotation about the specified axis, where the angle of rotation is the magnitude of the axis vector */
		static Mat3 FromScaledAxis(float x, float y, float z);
		/** Returns a 3x3 matrix which represents a rotation about the specified axis, where the angle of rotation is the magnitude of the axis vector */
		static Mat3 FromScaledAxis(const Vec3& xyz);
		/** Returns the identity matrix */
		static Mat3 Identity();

		/** Returns an orthonormal 3x3 matrix approximately matching the one given */
		static Mat3 Normalize(const Mat3& a);

		/** Returns the matrix inverse of the given matrix */
		static Mat3 Invert(const Mat3& a);



		/** Transforms a 3-component vector by this matrix (i.e. matrix multiplication) */
		Vec3 operator *(const Vec3& b) const;
		/** Transforms a 3x3 matrix by another 3x3 matrix */
		void operator *=(const Mat3& b);
		/** Transforms a 3x3 matrix by another 3x3 matrix */
		Mat3 operator *(const Mat3& b) const;
	};

	/** Class representing a 4x4 matrix */
	struct Mat4
	{
		/** The 16 elements of the matrix */
		float values[16];

		/** Initializes a zero matrix */
		Mat4();
		/** Initializes a matrix with the specified values */
		Mat4(float values_[16]);
		/** Initializes a matrix with the specified values */
		Mat4(float v11, float v12, float v13, float v14, float v21, float v22, float v23, float v24, float v31, float v32, float v33, float v34, float v41, float v42, float v43, float v44);

		/** Returns a reference to the specified element of the matrix */
		float& operator[](int index);

		

		/** Returns the transpose of this matrix */
		Mat4 Transpose() const;

		/** Returns the result of transforming the 3-component vector xyz with homogeneous component w by this matrix, divided by the results' homogeneous component */
		Vec3 TransformVec3(const Vec3& xyz, float w) const;
		/** Returns the result of transforming the 3-component vector xyz with homogeneous component w by this matrix, divided by the results' homogeneous component */
		Vec3 TransformVec3(float x, float y, float z, float w) const;

		Vec3 TransformVec3_0(const Vec3& xyz) const;
		Vec3 TransformVec3_1(const Vec3& xyz) const;
		Vec3 TransformVec3_0(float x, float y, float z) const;
		Vec3 TransformVec3_1(float x, float y, float z) const;

		/** Returns the identity matrix */
		static Mat4 Identity();
		/** Returns a 4x4 matrix with the given 3x3 matrix as orientation */
		static Mat4 FromMat3(const Mat3& mat);
		/** Returns a 4x4 matrix with the given quaternion as orientation */
		static Mat4 FromQuaternion(const Quaternion& q);
		/** Returns a 4x4 matrix with the specified orientation and translation */
		static Mat4 FromPositionAndOrientation(const Vec3& pos, const Mat3& ori);
		/** Returns a 4x4 matrix with the specified orientation and translation */
		static Mat4 FromPositionAndOrientation(const Vec3& pos, const Quaternion& ori);
		/** Returns a 4x4 matrix with the specified orientation, translation, and uniform scale */
		static Mat4 FromPosOriScale(const Vec3& pos, const Mat3& ori, float scale);
		/** Returns a 4x4 matrix with the specified orientation, translation, and uniform scale */
		static Mat4 FromPosOriScale(const Vec3& pos, const Quaternion& ori, float scale);
		/** Returns a 4x4 matrix representing a rotation around the specified point */
		static Mat4 RotationAroundPoint(const Mat3& rot, const Vec3& point);
		/** Returns a translation matrix */
		static Mat4 Translation(const Vec3& translation);
		/** Returns a translation matrix */
		static Mat4 Translation(float x, float y, float z);
		/** Returns a uniform scale matrix */
		static Mat4 UniformScale(float scale);
		/** Returns a non-uniform scale matrix, scaling each axis by the corresponding parameter */
		static Mat4 Scale(float x, float y, float z);
		/** Returns a non-uniform scale matrix, scaling each axis by the corresponding parameter */
		static Mat4 Scale(const Vec3& vec);
		/** Returns the matrix inverse of the given matrix */
		static Mat4 Invert(const Mat4& a);



		/** Transforms a 4-component vector by a 4x4 matrix */
		Vec4 operator *(const Vec4& b) const;
		/** Transforms a 4x4 matrix by another 4x4 matrix */
		void operator *=(const Mat4& b);
		/** Transforms a 4x4 matrix by another 4x4 matrix */
		Mat4 operator *(const Mat4& b) const;
		/** Scales the components of a 4x4 matrix by the specified amount */
		void operator *=(float b);
		/** Scales the components of a 4x4 matrix by the specified amount */
		Mat4 operator *(float b) const;
		/** Adds two matrices */
		void operator +=(const Mat4& b);
		/** Adds two matrices */
		Mat4 operator +(const Mat4& b) const;
	};
}

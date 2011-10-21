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

		/** Returns a reference to the specified element of the matrix */
		float& operator[](int index);

		/** Returns the transpose of this matrix */
		Mat3 Transpose();
		/** Returns the determinant of this matrix */
		float Determinant();

		/** Returns a 3x3 matrix which represents a rotation about the specified axis, by the specified angle. The axis should be normalized. */
		static Mat3 FromAxisAngle(float x, float y, float z, float angle);
		/** Returns a 3x3 matrix which represents a rotation about the specified axis, where the angle of rotation is the magnitude of the axis vector */
		static Mat3 FromScaledAxis(float x, float y, float z);
		/** Returns a 3x3 matrix which represents a rotation about the specified axis, where the angle of rotation is the magnitude of the axis vector */
		static Mat3 FromScaledAxis(Vec3 xyz);
		/** Returns the identity matrix */
		static Mat3 Identity();

		/** Returns an orthonormal 3x3 matrix approximately matching the one given */
		static Mat3 Normalize(Mat3 a);

		/** Returns the matrix inverse of the given matrix */
		static Mat3 Invert(Mat3 a);
	};

	/** Transforms a 3-component vector by this matrix (i.e. matrix multiplication) */
	Vec3 operator *(Mat3 a, Vec3 b);
	/** Transforms a 3x3 matrix by another 3x3 matrix */
	void operator *=(Mat3& a, Mat3 b);
	/** Transforms a 3x3 matrix by another 3x3 matrix */
	Mat3 operator *(Mat3 a, Mat3 b);

	/** Class representing a 4x4 matrix */
	struct Mat4
	{
		/** The 16 elements of the matrix */
		float values[16];

		/** Initializes a zero matrix */
		Mat4();
		/** Initializes a matrix with the specified values */
		Mat4(float values_[16]);

		/** Returns a reference to the specified element of the matrix */
		float& operator[](int index);

		

		/** Returns the transpose of this matrix */
		Mat4 Transpose();

		/** Returns the result of transforming the 3-component vector xyz with homogeneous component w by this matrix, divided by the results' homogeneous component */
		Vec3 TransformVec3(Vec3 xyz, float w);
		/** Returns the result of transforming the 3-component vector xyz with homogeneous component w by this matrix, divided by the results' homogeneous component */
		Vec3 TransformVec3(float x, float y, float z, float w);

		/** Returns the identity matrix */
		static Mat4 Identity();
		/** Returns a 4x4 matrix with the given 3x3 matrix as orientation */
		static Mat4 FromMat3(Mat3 mat);
		/** Returns a 4x4 matrix with the given quaternion as orientation */
		static Mat4 FromQuaternion(Quaternion q);
		/** Returns a 4x4 matrix with the specified orientation and translation */
		static Mat4 FromPositionAndOrientation(Vec3 pos, Mat3 ori);
		/** Returns a 4x4 matrix with the specified orientation and translation */
		static Mat4 FromPositionAndOrientation(Vec3 pos, Quaternion ori);
		/** Returns a 4x4 matrix with the specified orientation, translation, and uniform scale */
		static Mat4 FromPosOriScale(Vec3 pos, Mat3 ori, float scale);
		/** Returns a 4x4 matrix with the specified orientation, translation, and uniform scale */
		static Mat4 FromPosOriScale(Vec3 pos, Quaternion ori, float scale);
		/** Returns a 4x4 matrix representing a rotation around the specified point */
		static Mat4 RotationAroundPoint(Mat3 rot, Vec3 point);
		/** Returns a translation matrix */
		static Mat4 Translation(Vec3 translation);
		/** Returns a translation matrix */
		static Mat4 Translation(float x, float y, float z);
		/** Returns a uniform scale matrix */
		static Mat4 UniformScale(float scale);
		/** Returns a non-uniform scale matrix, scaling each axis by the corresponding parameter */
		static Mat4 Scale(float x, float y, float z);
		/** Returns a non-uniform scale matrix, scaling each axis by the corresponding parameter */
		static Mat4 Scale(Vec3 vec);
		/** Returns the matrix inverse of the given matrix */
		static Mat4 Invert(Mat4 a);
	};

	/** Transforms a 4-component vector by a 4x4 matrix */
	Vec4 operator *(Mat4 a, Vec4 b);
	/** Transforms a 4x4 matrix by another 4x4 matrix */
	void operator *=(Mat4& a, Mat4 b);
	/** Transforms a 4x4 matrix by another 4x4 matrix */
	Mat4 operator *(Mat4 a, Mat4 b);
	/** Scales the components of a 4x4 matrix by the specified amount */
	void operator *=(Mat4& a, float b);
	/** Scales the components of a 4x4 matrix by the specified amount */
	Mat4 operator *(Mat4 a, float b);
	/** Adds two matrices */
	void operator +=(Mat4& a, Mat4 b);
	/** Adds two matrices */
	Mat4 operator +(Mat4 a, Mat4 b);
}

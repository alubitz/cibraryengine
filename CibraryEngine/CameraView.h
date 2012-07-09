#pragma once

#include "StdAfx.h"

#include "MathTypes.h"

namespace CibraryEngine
{
	/** Class representing the frustum information of a camera */
	class CameraView
	{
		private:

			Vec3 position, forward, up;
			float left, right, top, bottom, _near, _far;	// beat up whatever idiot made 'near' and 'far' invalid variable names
			Mat4 view, proj;
			bool view_valid, proj_valid;					// if these are false, treat view and proj matrices as invalid and in need of recalculation
			Plane planes[6];
			bool planes_valid[6];							// if these are false, treat the corresponding planes as invalid and in need of recalculation

			void InvalidatePlanes();

		public:

			/** Initialize a CameraView specifying the positon, forward and up vectors, and floats for each of the planes of the frustum */
			CameraView(const Vec3& position_, const Vec3& forward_, const Vec3& up_, float left_, float right_, float top_, float bottom_, float near_, float far_);
			/** Initialize a CameraView specifying the positon, forward and up vectors, and floats for zoom and aspect ratio */
			CameraView(const Vec3& position_, const Vec3& forward_, const Vec3& up_, float zoom, float aspect_ratio);
			/** Initialize a CameraView specifying the view matrix and floats for each of the planes of the frustum */
			CameraView(const Mat4& view_, float zoom, float aspect_ratio);

			/** Get the position of the camera */
			Vec3 GetPosition() const;
			/** Get the forward vector of the camera */
			Vec3 GetForward() const;
			/** Get the up vector of the camera */
			Vec3 GetUp() const;
			/** Get the right vector of the camera */
			Vec3 GetRight() const;
			/** Get the view matrix */
			Mat4 GetViewMatrix();
			/** Get the projection matrix */
			Mat4 GetProjectionMatrix();

			/** Get the left plane */
			Plane GetLeftPlane();
			/** Get the right plane */
			Plane GetRightPlane();
			/** Get the top plane */
			Plane GetTopPlane();
			/** Get the bottom plane */
			Plane GetBottomPlane();
			/** Get the near plane */
			Plane GetNearPlane();
			/** Get the far plane */
			Plane GetFarPlane();

			/** Get the origin and direction of a ray which is fired from the position, through a position in the frustum which is the specified horizontal and vertical fractions of its size */
			void GetRayFromDimCoeffs(float xfrac, float yfrac, Vec3& origin, Vec3& direction) const;

			/** Check whether any part of the given sphere is within the camera's frustum */
			bool CheckSphereVisibility(const Vec3& center, float radius);
			/** Check whether any part of the given sphere is within the camera's frustum */
			bool CheckSphereVisibility(const Sphere& sphere);

			// setters may (in)validate matrices and/or planes!

			/** Set the position of the camera */
			void SetPosition(const Vec3& position);
			/** Set the forward vector of the camera */
			void SetForward(const Vec3& forward);
			/** Set the up vector of the camera */
			void SetUp(const Vec3& up);
			/** Set the view matrix  */
			void SetViewMatrix(const Mat4& view);

			/** Set the projection based on a 4x4 projection matrix */
			void SetProjectionMatrix(const Mat4& proj);
			/** Set the projection based on a zoom and aspect ratio */
			void SetProjection(float zoom, float aspect_ratio);
	};
}

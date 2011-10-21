#include "StdAfx.h"
#include "CameraView.h"

#include "MathTypes.h"

namespace CibraryEngine
{
	CameraView::CameraView(Vec3 position_, Vec3 forward_, Vec3 up_, float left_, float right_, float top_, float bottom_, float near_, float far_) : position(position_), forward(forward_), up(up_), left(left_), right(right_), top(top_), bottom(bottom_), _near(near_), _far(far_), view_valid(false), proj_valid(false) { }
	CameraView::CameraView(Vec3 position_, Vec3 forward_, Vec3 up_, float zoom, float aspect_ratio) : position(position_), forward(forward_), up(up_), view_valid(false), proj_valid(false) { SetProjection(zoom, aspect_ratio); }
	CameraView::CameraView(Mat4 view_, float zoom, float aspect_ratio) : view_valid(false), proj_valid(false) { SetViewMatrix(view_); SetProjection(zoom, aspect_ratio); }

	void CameraView::InvalidatePlanes() { planes_valid[0] = planes_valid[1] = planes_valid[2] = planes_valid[3] = planes_valid[4] = planes_valid[5] = false; }

	Vec3 CameraView::GetPosition() { return position; }
	Vec3 CameraView::GetForward() { return forward; }
	Vec3 CameraView::GetUp() { return up; }

	Vec3 CameraView::GetRight() { return Vec3::Normalize(Vec3::Cross(forward, up)); }

	Mat4 CameraView::GetViewMatrix()
	{
		if (!view_valid)
		{
			// Normalize direction vectors
			Vec3 reverse = -forward;
			Vec3 right = GetRight();

			// Construct rotation matrix from direction vectors
			float values[] = { 
				right.x,	right.y,	right.z,
				up.x,		up.y,		up.z,
				reverse.x,	reverse.y,	reverse.z
			};
			Mat3 rot = Mat3(values);

			// Generate and return the view matrix
			view = Mat4::Translation(rot * -position) * Mat4::FromMat3(rot);
			view_valid = true;
		}
		return view;
	}

	Mat4 CameraView::GetProjectionMatrix()
	{
		if (!proj_valid)
		{
			float inv_width = 1.0f / (right - left), inv_height = 1.0f / (top - bottom), inv_depth = -1.0f / (_far - _near);

			float a = (right + left) * inv_width;
			float b = (top + bottom) * inv_height;
			float c = (_far + _near) * inv_depth;
			float d = 2.0f * _far * _near * inv_depth;
			float e = 2.0f * _near * inv_width;
			float f = 2.0f * _near * inv_height;

			float values[] = {
				e,	0,	a,	0,
				0,	f,	b,	0,
				0,	0,	c,	d,
				0,	0,	-1,	0
			};
			proj = Mat4(values);
			proj_valid = true;
		}
		return proj;
	}

	Plane CameraView::GetLeftPlane()
	{
		if (!planes_valid[0])
		{
			Vec3 leftward = GetRight() * left + forward * _near;
			planes[0] = Plane::FromPositionNormal(position, Vec3::Cross(leftward, up));
			planes_valid[0] = true;
		}
		return planes[0];
	}
	Plane CameraView::GetRightPlane()
	{
		if (!planes_valid[1])
		{
			Vec3 rightward = GetRight() * right + forward * _near;
			planes[1] = Plane::FromPositionNormal(position, Vec3::Cross(up, rightward));
			planes_valid[1] = true;
		}
		return planes[1];
	}
	Plane CameraView::GetTopPlane()
	{
		if (!planes_valid[2])
		{
			Vec3 upward = up * top + forward * _near;
			planes[2] = Plane::FromPositionNormal(position, Vec3::Cross(upward, GetRight()));
			planes_valid[2] = true;
		}
		return planes[2];
	}
	Plane CameraView::GetBottomPlane()
	{
		if (!planes_valid[3])
		{
			Vec3 downward = up * bottom + forward * _near;
			planes[3] = Plane::FromPositionNormal(position, Vec3::Cross(GetRight(), downward));
			planes_valid[3] = true;
		}
		return planes[3];
	}
	Plane CameraView::GetNearPlane()
	{
		if (!planes_valid[4])
		{
			planes[4] = Plane::FromPositionNormal(position + forward * _near, forward);
			planes_valid[4] = true;
		}
		return planes[4];
	}
	Plane CameraView::GetFarPlane()
	{
		if (!planes_valid[5])
		{
			planes[5] = Plane::FromPositionNormal(position + forward * _far, -forward);
			planes_valid[5] = true;
		}
		return planes[5];
	}

	void CameraView::GetRayFromDimCoeffs(float xfrac, float yfrac, Vec3& origin, Vec3& direction)
	{
		direction = GetRight() * (right * xfrac + left * (1.0f - xfrac)) + GetUp() * (top * yfrac + bottom * (1.0f - yfrac)) + forward * _near;
		origin = position + direction;
		direction = Vec3::Normalize(direction);
	}

	bool CameraView::CheckSphereVisibility(Vec3 center, float radius)
	{
		float n_radius = -radius;
		if (GetNearPlane().PointDistance(center) > n_radius)
			if (GetFarPlane().PointDistance(center) > n_radius)
				if (GetLeftPlane().PointDistance(center) > n_radius)
					if (GetRightPlane().PointDistance(center) > n_radius)
						if (GetTopPlane().PointDistance(center) > n_radius)
							if (GetBottomPlane().PointDistance(center) > n_radius)
								return true;
		return false;
	}
	bool CameraView::CheckSphereVisibility(Sphere sphere) { return CheckSphereVisibility(sphere.center, sphere.radius); }

	void CameraView::SetPosition(Vec3 position_)
	{
		position = position_;
		view_valid = false;
		InvalidatePlanes();
	}
	void CameraView::SetForward(Vec3 forward_)
	{
		forward = Vec3::Normalize(forward_);
		view_valid = false;
		InvalidatePlanes();
	}
	void CameraView::SetUp(Vec3 up_)
	{
		up = Vec3::Normalize(up_);
		view_valid = false;
		InvalidatePlanes();
	}

	void CameraView::SetViewMatrix(Mat4 view_)
	{
		position = Vec3(-view_[3], -view_[7], -view_[11]);
		forward = Vec3(-view_[8], -view_[9], -view_[10]);
		up = Vec3(view_[4], view_[5], view_[6]);
		Vec3 right = GetRight();

		float rm_values[] = { right.x, right.y, right.z, up.x, up.y, up.z, -forward.x, -forward.y, -forward.z };
		Mat3 rm = Mat3(rm_values).Transpose();
		position = rm * position;

		view = view_;
		view_valid = true;

		InvalidatePlanes();
	}
	void CameraView::SetProjectionMatrix(Mat4 proj_)
	{
		float a = proj_[2], b = proj_[6], c = proj_[10], d = proj_[11], e = proj_[0], f = proj[5];
		float alpha = (a + 1) / (a - 1), bravo = (b + 1) / (b - 1), charlie = (c + 1) / (c - 1);

		_near = (d * charlie - d) / (2 * charlie);
		_far = _near * charlie;
		left = 2 * _near / (e * (alpha - 1));
		right = left * alpha;
		bottom = 2 * _near / (f * (bravo - 1));
		top = bottom * bravo;

		proj = proj_;
		proj_valid = true;

		InvalidatePlanes();
	}

	void CameraView::SetProjection(float zoom, float aspect_ratio)
	{
		_near = 0.1f;						// near plane distance, arbitrarily
		_far = 16384.0f;					// far plane distance, arbitrarily
		top = _near / zoom;
		right = aspect_ratio * top;
		left = -right;
		bottom = -top;

		proj_valid = false;

		InvalidatePlanes();
	}
}

#include "StdAfx.h"
#include "GhostCamera.h"
namespace Test
{
	/*
	 * GhostCamera methods
	 */	
	Mat4 GhostCamera::GetViewMatrix()
	{
		ori = Quaternion::FromRVec(0, -yaw, 0) * Quaternion::FromRVec(pitch, 0, 0);
		Mat4 flip   = Mat4::FromQuaternion(Quaternion::FromRVec(0, float(M_PI), 0));
		Mat4 rotate = Mat4::FromQuaternion(Quaternion::Reverse(ori));
		Mat4 loc	= Mat4::Translation(-pos);
		return flip * rotate * loc;
	}
	void GhostCamera::Update(const TimingInfo& time)
	{
		vel *= expf(-10.0f * time.elapsed);
		Mat3 camera_rm = ori.ToMat3();
		vel += camera_rm * Vec3(-control_state->GetFloatControl("sidestep"), control_state->GetFloatControl("elevate"), control_state->GetFloatControl("forward")) * (50.0f * time.elapsed);
		pos += vel * time.elapsed;
		yaw += control_state->GetFloatControl("yaw");
		pitch = max(-float(M_PI_2), min(float(M_PI_2), pitch + control_state->GetFloatControl("pitch")));
		control_state->SetFloatControl("yaw", 0.0f);
		control_state->SetFloatControl("pitch", 0.0f);
	}
}
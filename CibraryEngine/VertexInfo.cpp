#include "StdAfx.h"
#include "VertexInfo.h"
#include "DebugLog.h"

#include "MathTypes.h"

namespace CibraryEngine
{
	/*
	 * VUVNTTC methods
	 */
	VUVNTTC::VUVNTTC() : x(0.0f, 0.0f, 0.0f), rgba(1.0f, 1.0f, 1.0f, 1.0f) { }
	VUVNTTC::VUVNTTC(Vec3 x, Vec3 uvw, Vec3 n) : x(x), uvw(uvw), n(n), rgba(1.0, 1.0, 1.0, 1.0) { }




	/*
	 * SkinVInfo methods
	 */
	SkinVInfo::SkinVInfo() : VUVNTTC(), indices(), weights() { weights[0] = 255; }

	SkinVInfo::SkinVInfo(Vec3 x, Vec3 uvw, Vec3 n) : VUVNTTC(x, uvw, n), indices(), weights() { weights[0] = 255; }

	SkinVInfo::SkinVInfo(VUVNTTC original) : VUVNTTC(original), indices(), weights() { weights[0] = 255; }

	SkinVInfo::SkinVInfo(Vec3 x, Vec3 uvw, Vec3 n, unsigned char* indices_, unsigned char* weights_) : VUVNTTC(x, uvw, n)
	{
		for(int i = 0; i < 4; i++)
		{
			indices[i] = indices_[i];
			weights[i] = weights_[i];
		}
	}
}

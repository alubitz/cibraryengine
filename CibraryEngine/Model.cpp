#include "StdAfx.h"
#include "Model.h"

#include "ModelLoader.h"
#include "SkeletalAnimation.h"

#include "VertexBuffer.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * SkinVInfo methods
	 */	
	SkinVInfo::SkinVInfo() : VTNTT(), indices(), weights() { weights[0] = 255; }
	SkinVInfo::SkinVInfo(Vec3 x, Vec3 uvw, Vec3 n) : VTNTT(x, uvw, n), indices(), weights() { weights[0] = 255; }
	SkinVInfo::SkinVInfo(VTNTT original) : VTNTT(original), indices(), weights() { weights[0] = 255; }
	SkinVInfo::SkinVInfo(Vec3 x, Vec3 uvw, Vec3 n, unsigned char* indices_, unsigned char* weights_) : 
		VTNTT(x, uvw, n), 
		indices(),
		weights()
	{
		for(int i = 0; i < 4; i++)
		{
			indices[i] = indices_[i];
			weights[i] = weights_[i];
		}
	}




	/*
	 * SkinnedModel methods
	 */
	SkinnedModel::SkinnedModel(vector<MaterialModelPair> material_model_pairs, vector<string> material_names, Skeleton* skeleton) :
		material_model_pairs(material_model_pairs),
		material_names(material_names),
		skeleton(skeleton)
	{
	}

	SkinnedModel* SkinnedModel::WrapVertexBuffer(VertexBuffer* model, string material_name)
	{
		vector<string> material_names;
		material_names.push_back(material_name);

		vector<MaterialModelPair> material_model_pairs;
		MaterialModelPair mmp = MaterialModelPair();
		mmp.material_index = 0;
		mmp.vbo = model;

		material_model_pairs.push_back(mmp);

		Skeleton* skeleton = new Skeleton();
		skeleton->bones.push_back(new Bone("root", NULL, Quaternion::Identity(), Vec3()));

		return new SkinnedModel(material_model_pairs, material_names, skeleton);
	}

	void SetVertexInfo(VertexBuffer* vbo, int index, VTNTT info)
	{
		float* xyz = vbo->GetFloatPointer("gl_Vertex");
		xyz[index * 3 + 0] = info.x.x;
		xyz[index * 3 + 1] = info.x.y;
		xyz[index * 3 + 2] = info.x.z;

		float* n = vbo->GetFloatPointer("gl_Normal");
		n[index * 3 + 0] = info.n.x;
		n[index * 3 + 1] = info.n.y;
		n[index * 3 + 2] = info.n.z;

		float* uvw = vbo->GetFloatPointer("gl_MultiTexCoord0");
		uvw[index * 3 + 0] = info.uvw.x;
		uvw[index * 3 + 1] = info.uvw.y;
		uvw[index * 3 + 2] = info.uvw.z;

		float* t1 = vbo->GetFloatPointer("gl_MultiTexCoord1");
		t1[index * 3 + 0] = info.tan_1.x;
		t1[index * 3 + 1] = info.tan_1.y;
		t1[index * 3 + 2] = info.tan_1.z;

		float* t2 = vbo->GetFloatPointer("gl_MultiTexCoord2");
		t2[index * 3 + 0] = info.tan_2.x;
		t2[index * 3 + 1] = info.tan_2.y;
		t2[index * 3 + 2] = info.tan_2.z;
	}

	void AddVertexInfo(VertexBuffer* vbo, VTNTT info)
	{
		int index = vbo->GetNumVerts();
		vbo->SetNumVerts(index + 1);
		SetVertexInfo(vbo, index, info);
	}

	void AddVertexInfo(VertexBuffer* vbo, SkinVInfo info)
	{
		int index = vbo->GetNumVerts();

		AddVertexInfo(vbo, (VTNTT)info);

		float* indices = vbo->GetFloatPointer("gl_MultiTexCoord3");
		float* weights = vbo->GetFloatPointer("gl_MultiTexCoord4");
		for(int i = 0; i < 4; i++)
		{
			indices[index * 4 + i] = info.indices[i];
			weights[index * 4 + i] = info.weights[i];
		}
	}

	void AddTriangleVertexInfo(VertexBuffer* vbo, VTNTT a, VTNTT b, VTNTT c)
	{
		Vec3 b_minus_a = b.x - a.x, c_minus_a = c.x - a.x;

		for (int i = 0; i < 3; i++)
		{
			VTNTT* info;
			switch (i)
			{
				case 0:
					info = &a;
					break;
				case 1:
					info = &b;
					break;
				case 2:
				default:
					info = &c;
					break;
			}

			Vec3 non_edge_ab = Vec3::Cross(info->n, c_minus_a);
			Vec3 non_edge_ac = Vec3::Cross(info->n, b_minus_a);
			float ab_denom = Vec3::Dot(non_edge_ab, b.x) - Vec3::Dot(non_edge_ab, a.x);
			float ac_denom = Vec3::Dot(non_edge_ac, c.x) - Vec3::Dot(non_edge_ac, a.x);
			non_edge_ab /= ab_denom;
			non_edge_ac /= ac_denom;
			info->tan_1 = non_edge_ab * (b.uvw.x - a.uvw.x) + non_edge_ac * (c.uvw.x - a.uvw.x);
			info->tan_2 = non_edge_ab * (b.uvw.y - a.uvw.y) + non_edge_ac * (c.uvw.y - a.uvw.y);
		}

		AddVertexInfo(vbo, a);
		AddVertexInfo(vbo, b);
		AddVertexInfo(vbo, c);
	}

	void AddTriangleVertexInfo(VertexBuffer* vbo, SkinVInfo a, SkinVInfo b, SkinVInfo c)
	{
		Vec3 b_minus_a = b.x - a.x, c_minus_a = c.x - a.x;

		for (int i = 0; i < 3; i++)
		{
			SkinVInfo* info;
			switch (i)
			{
				case 0:
					info = &a;
					break;
				case 1:
					info = &b;
					break;
				case 2:
				default:
					info = &c;
					break;
			}

			Vec3 non_edge_ab = Vec3::Cross(info->n, c_minus_a);
			Vec3 non_edge_ac = Vec3::Cross(info->n, b_minus_a);
			float ab_denom = Vec3::Dot(non_edge_ab, b.x) - Vec3::Dot(non_edge_ab, a.x);
			float ac_denom = Vec3::Dot(non_edge_ac, c.x) - Vec3::Dot(non_edge_ac, a.x);
			non_edge_ab /= ab_denom;
			non_edge_ac /= ac_denom;
			info->tan_1 = non_edge_ab * (b.uvw.x - a.uvw.x) + non_edge_ac * (c.uvw.x - a.uvw.x);
			info->tan_2 = non_edge_ab * (b.uvw.y - a.uvw.y) + non_edge_ac * (c.uvw.y - a.uvw.y);
		}

		AddVertexInfo(vbo, a);
		AddVertexInfo(vbo, b);
		AddVertexInfo(vbo, c);
	}

	VTNTT GetVTNTT(VertexBuffer* vbo, int index)
	{
		float* x = vbo->GetFloatPointer("gl_Vertex");
		float* n = vbo->GetFloatPointer("gl_Normal");
		float* uv = vbo->GetFloatPointer("gl_MultiTexCoord0");
		float* t1 = vbo->GetFloatPointer("gl_MultiTexCoord1");
		float* t2 = vbo->GetFloatPointer("gl_MultiTexCoord2");
		VTNTT result(
			Vec3(x[index * 3 + 0], x[index * 3 + 1], x[index * 3 + 2]),
			Vec3(uv[index * 3 + 0], uv[index * 3 + 1], uv[index * 3 + 2]),
			Vec3(n[index * 3 + 0], n[index * 3 + 1], n[index * 3 + 2]));
		result.tan_1 = Vec3(t1[index * 3 + 0], t1[index * 3 + 1], t1[index * 3 + 2]);
		result.tan_2 = Vec3(t2[index * 3 + 0], t2[index * 3 + 1], t2[index * 3 + 2]);
		return result;
	}

	SkinVInfo GetSkinVInfo(VertexBuffer* vbo, int index)
	{
		VTNTT vtn = GetVTNTT(vbo, index);
		float* indices = vbo->GetFloatPointer("gl_MultiTexCoord3");
		float* weights = vbo->GetFloatPointer("gl_MultiTexCoord4");
		SkinVInfo result(vtn);
		for(int i = 0; i < 4; i++)
		{
			result.indices[i] = (unsigned char)indices[index * 4 + i];
			result.weights[i] = (unsigned char)weights[index * 4 + i];
		}
		return result;
	}
}

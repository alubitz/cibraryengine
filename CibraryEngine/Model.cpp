#include "StdAfx.h"
#include "Model.h"

#include "ModelLoader.h"
#include "SkeletalAnimation.h"

#include "VertexInfo.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * SkinnedModel methods
	 */
	SkinnedModel::SkinnedModel(vector<MaterialModelPair> material_model_pairs, vector<string> material_names, Skeleton* skeleton) :
		material_model_pairs(material_model_pairs),
		material_names(material_names),
		skeleton(skeleton)
	{
	}

	SkinnedModel* SkinnedModel::CopyVTNModel(VTNModel* model, string material_name)
	{

		vector<string> material_names;
		material_names.push_back(material_name);

		vector<MaterialModelPair> material_model_pairs;
		MaterialModelPair mmp = MaterialModelPair();
		mmp.material_index = 0;
		mmp.vbo = new SkinVInfoVertexBuffer(*model->GetVBO());

		material_model_pairs.push_back(mmp);

		Skeleton* skeleton = new Skeleton();
		skeleton->bones.push_back(new Bone("root", NULL, Quaternion::Identity(), Vec3()));

		return new SkinnedModel(material_model_pairs, material_names, skeleton);
	}




	/*
	 * VertexBufferI methods
	 */
	void VertexBufferI::BuildAsNeeded()
	{
		if (!built)
			Build();
	}

	void VertexBufferI::InnerDispose() { GLCleanup(); }

	bool VertexBufferI::IsValid() { return built; }
	void VertexBufferI::Validate() { built = true; }
	void VertexBufferI::Invalidate() { built = false; }




	/*
	 * VUVNTTCVertexBuffer methods
	 */
	VUVNTTCVertexBuffer::VUVNTTCVertexBuffer() : VertexBuffer<VUVNTTC>() { vbo_id[0] = vbo_id[1] = 0; }

	void VUVNTTCVertexBuffer::Draw()
	{
		BuildAsNeeded();

		glBindBuffer(GL_ARRAY_BUFFER, vbo_id[0]);

		glEnable(GL_VERTEX_ARRAY);
		glEnable(GL_NORMAL_ARRAY);

		int count = vertex_infos.size();

		glVertexPointer(	3,	GL_FLOAT, 	0,	0);
		glNormalPointer(		GL_FLOAT, 	0,	(void*)(count * sizeof(float) * 3));

		glClientActiveTexture(GL_TEXTURE0);
		glEnable(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(	3,	GL_FLOAT,	0,	(void*)(count * sizeof(float) * 6));

		glClientActiveTexture(GL_TEXTURE1);
		glEnable(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(	3,	GL_FLOAT,	0,	(void*)(count * sizeof(float) * 9));

		glClientActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(	3,	GL_FLOAT,	0,	(void*)(count * sizeof(float) * 12));

		glDrawArrays(GL_TRIANGLES, 0, count);

		glDisable(GL_VERTEX_ARRAY);
		glClientActiveTexture(GL_TEXTURE0);
		glDisable(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTexture(GL_TEXTURE1);
		glDisable(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTexture(GL_TEXTURE2);
		glDisable(GL_TEXTURE_COORD_ARRAY);
		glDisable(GL_NORMAL_ARRAY);

		glClientActiveTexture(GL_TEXTURE0);			// get texcoords back to working "the normal way"

		glBindBuffer(GL_ARRAY_BUFFER, 0);			// don't leave hardware vbo on
	}

	Sphere VUVNTTCVertexBuffer::GetBoundingSphere()
	{
		bool valid = false;
		Sphere temp;
		for(vector<VUVNTTC>::iterator iter = vertex_infos.begin(); iter != vertex_infos.end(); iter++)
			if(valid)
				temp = Sphere::Expand(temp, iter->x);
			else
			{
				temp = Sphere(iter->x, 0);
				valid = true;
			}
		return temp;
	}

	void VUVNTTCVertexBuffer::AddTriangleVertexInfo(VUVNTTC a, VUVNTTC b, VUVNTTC c)
	{
		Vec3 b_minus_a = b.x - a.x, c_minus_a = c.x - a.x;

		for (int i = 0; i < 3; i++)
		{
			VUVNTTC* info;
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

		AddVertexInfo(a);
		AddVertexInfo(b);
		AddVertexInfo(c);
	}

	void VUVNTTCVertexBuffer::AddVertexInfo(VUVNTTC info)
	{
		vertex_infos.push_back(info);
		GLCleanup();									// this will call Invalidate()
	}

	void VUVNTTCVertexBuffer::Build()
	{
		int count = vertex_infos.size();

		float* vertices = new float[count * 3];
		float* normals = new float[count * 3];
		float* uvws = new float[count * 3];
		float* tangents_1 = new float[count * 3];
		float* tangents_2 = new float[count * 3];
		for (int i = 0; i < count; i++)
		{
			VUVNTTC v = vertex_infos[i];

			vertices[i * 3 + 0] = v.x.x;
			vertices[i * 3 + 1] = v.x.y;
			vertices[i * 3 + 2] = v.x.z;
			normals[i * 3 + 0] = v.n.x;
			normals[i * 3 + 1] = v.n.y;
			normals[i * 3 + 2] = v.n.z;
			uvws[i * 3 + 0] = v.uvw.x;
			uvws[i * 3 + 1] = v.uvw.y;
			uvws[i * 3 + 2] = v.uvw.z;
			tangents_1[i * 3 + 0] = v.tan_1.x;
			tangents_1[i * 3 + 1] = v.tan_1.y;
			tangents_1[i * 3 + 2] = v.tan_1.z;
			tangents_2[i * 3 + 0] = v.tan_2.x;
			tangents_2[i * 3 + 1] = v.tan_2.y;
			tangents_2[i * 3 + 2] = v.tan_2.z;
		}

		// warning! this will delete the referenced vbo_id's! if you copied them and don't want them deleted, clear the vbo_id array
		GLCleanup();

		glGenBuffers(2, vbo_id);

		glBindBuffer(GL_ARRAY_BUFFER, vbo_id[0]);

		glBindBuffer(GL_ARRAY_BUFFER, vbo_id[0]);
		glBufferData(GL_ARRAY_BUFFER,									15 * count * sizeof(float),	NULL, GL_STATIC_DRAW);
		glBufferSubData(GL_ARRAY_BUFFER,	0,							3 * count * sizeof(float),	&vertices[0]);
		glBufferSubData(GL_ARRAY_BUFFER,	3 * count * sizeof(float),	3 * count * sizeof(float),	&normals[0]);
		glBufferSubData(GL_ARRAY_BUFFER,	6 * count * sizeof(float),	3 * count * sizeof(float),	&uvws[0]);
		glBufferSubData(GL_ARRAY_BUFFER,	9 * count * sizeof(float),	3 * count * sizeof(float),	&tangents_1[0]);
		glBufferSubData(GL_ARRAY_BUFFER,	12 * count * sizeof(float),	3 * count * sizeof(float),	&tangents_2[0]);

		glBindBuffer(GL_ARRAY_BUFFER, 0);			// don't leave hardware vbo on

		delete[] vertices;
		delete[] normals;
		delete[] uvws;
		delete[] tangents_1;
		delete[] tangents_2;

		Validate();
	}

	void VUVNTTCVertexBuffer::GLCleanup()
	{
		if (IsValid())
		{
			glDeleteBuffers(2, &vbo_id[0]);
			Invalidate();
		}
	}




	/*
	 * SkinVInfoVertexBuffer methods
	 */
	SkinVInfoVertexBuffer::SkinVInfoVertexBuffer() : VertexBuffer<SkinVInfo>() { vbo_id[0] = vbo_id[1] = 0; }

	SkinVInfoVertexBuffer::SkinVInfoVertexBuffer(VUVNTTCVertexBuffer& other) : VertexBuffer<SkinVInfo>()
	{
		vbo_id[0] = vbo_id[1] = 0;

		vector<VUVNTTC>& other_vec = other.vertex_infos;
		for(vector<VUVNTTC>::iterator iter = other_vec.begin(); iter != other_vec.end(); iter++)
			vertex_infos.push_back(SkinVInfo(*iter));
	}

	void SkinVInfoVertexBuffer::Draw()
	{
		BuildAsNeeded();

		glBindBuffer(GL_ARRAY_BUFFER, vbo_id[0]);

		glEnable(GL_VERTEX_ARRAY);
		glEnable(GL_NORMAL_ARRAY);

		int count = vertex_infos.size();

		glVertexPointer(	3,	GL_FLOAT, 	0,	0);
		glNormalPointer(		GL_FLOAT, 	0,	(void*)(count * sizeof(float) * 3));

		glClientActiveTexture(GL_TEXTURE0);
		glEnable(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(	3,	GL_FLOAT,	0,	(void*)(count * sizeof(float) * 6));

		glClientActiveTexture(GL_TEXTURE1);
		glEnable(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(	3,	GL_FLOAT,	0,	(void*)(count * sizeof(float) * 9));

		glClientActiveTexture(GL_TEXTURE2);
		glEnable(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(	3,	GL_FLOAT,	0,	(void*)(count * sizeof(float) * 12));

		glClientActiveTexture(GL_TEXTURE3);
		glEnable(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(	4,	GL_FLOAT,	0,	(void*)(count * sizeof(float) * 15));

		glClientActiveTexture(GL_TEXTURE4);
		glEnable(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(	4,	GL_FLOAT,	0,	(void*)(count * sizeof(float) * 19));

		glDrawArrays(GL_TRIANGLES, 0, count);

		glDisable(GL_VERTEX_ARRAY);
		glClientActiveTexture(GL_TEXTURE0);
		glDisable(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTexture(GL_TEXTURE1);
		glDisable(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTexture(GL_TEXTURE2);
		glDisable(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTexture(GL_TEXTURE3);
		glDisable(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTexture(GL_TEXTURE4);
		glDisable(GL_TEXTURE_COORD_ARRAY);
		glDisable(GL_NORMAL_ARRAY);

		glClientActiveTexture(GL_TEXTURE0);			// get texcoords back to working "the normal way"

		glBindBuffer(GL_ARRAY_BUFFER, 0);			// don't leave hardware vbo on
	}

	Sphere SkinVInfoVertexBuffer::GetBoundingSphere()
	{
		bool valid = false;
		Sphere temp;
		for(vector<SkinVInfo>::iterator iter = vertex_infos.begin(); iter != vertex_infos.end(); iter++)
			if(valid)
				temp = Sphere::Expand(temp, iter->x);
			else
			{
				temp = Sphere(iter->x, 0);
				valid = true;
			}
		return temp;
	}


	void SkinVInfoVertexBuffer::AddTriangleVertexInfo(SkinVInfo a, SkinVInfo b, SkinVInfo c)
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

		AddVertexInfo(a);
		AddVertexInfo(b);
		AddVertexInfo(c);
	}

	void SkinVInfoVertexBuffer::AddVertexInfo(SkinVInfo info)
	{
		vertex_infos.push_back(info);
		GLCleanup();									// this will call Invalidate()
	}

	void SkinVInfoVertexBuffer::Build()
	{
		int count = vertex_infos.size();

		float* vertices = new float[count * 3];
		float* normals = new float[count * 3];
		float* uvws = new float[count * 3];
		float* tangents_1 = new float[count * 3];
		float* tangents_2 = new float[count * 3];
		float* bone_indices = new float[count * 4];
		float* bone_weights = new float[count * 4];
		for (int i = 0; i < count; i++)
		{
			SkinVInfo v = vertex_infos[i];

			vertices[i * 3 + 0] = v.x.x;
			vertices[i * 3 + 1] = v.x.y;
			vertices[i * 3 + 2] = v.x.z;
			normals[i * 3 + 0] = v.n.x;
			normals[i * 3 + 1] = v.n.y;
			normals[i * 3 + 2] = v.n.z;
			uvws[i * 3 + 0] = v.uvw.x;
			uvws[i * 3 + 1] = v.uvw.y;
			uvws[i * 3 + 2] = v.uvw.z;
			tangents_1[i * 3 + 0] = v.tan_1.x;
			tangents_1[i * 3 + 1] = v.tan_1.y;
			tangents_1[i * 3 + 2] = v.tan_1.z;
			tangents_2[i * 3 + 0] = v.tan_2.x;
			tangents_2[i * 3 + 1] = v.tan_2.y;
			tangents_2[i * 3 + 2] = v.tan_2.z;
			for(int j = 0; j < 4; j++)
			{
				bone_indices[i * 4 + j] = v.indices[j];
				bone_weights[i * 4 + j] = v.weights[j];
			}
		}

		// warning! this will delete the referenced vbo_id's! if you copied them and don't want them deleted, clear the vbo_id array
		GLCleanup();

		glGenBuffers(2, vbo_id);

		glBindBuffer(GL_ARRAY_BUFFER, vbo_id[0]);

		glBindBuffer(GL_ARRAY_BUFFER, vbo_id[0]);
		glBufferData(GL_ARRAY_BUFFER,									23 * count * sizeof(float),	NULL, GL_STATIC_DRAW);

		glBufferSubData(GL_ARRAY_BUFFER,	0,							3 * count * sizeof(float),	&vertices[0]);
		glBufferSubData(GL_ARRAY_BUFFER,	3 * count * sizeof(float),	3 * count * sizeof(float),	&normals[0]);
		glBufferSubData(GL_ARRAY_BUFFER,	6 * count * sizeof(float),	3 * count * sizeof(float),	&uvws[0]);
		glBufferSubData(GL_ARRAY_BUFFER,	9 * count * sizeof(float),	3 * count * sizeof(float),	&tangents_1[0]);
		glBufferSubData(GL_ARRAY_BUFFER,	12 * count * sizeof(float),	3 * count * sizeof(float),	&tangents_2[0]);

		glBufferSubData(GL_ARRAY_BUFFER,	15 * count * sizeof(float),	4 * count * sizeof(float),	&bone_indices[0]);
		glBufferSubData(GL_ARRAY_BUFFER,	19 * count * sizeof(float),	4 * count * sizeof(float),	&bone_weights[0]);

		glBindBuffer(GL_ARRAY_BUFFER, 0);			// don't leave hardware vbo on

		Validate();

		delete vertices;
		delete normals;
		delete uvws;
		delete tangents_1;
		delete tangents_2;
		delete bone_indices;
		delete bone_weights;
	}

	void SkinVInfoVertexBuffer::GLCleanup()
	{
		if (IsValid())
		{
			glDeleteBuffers(2, &vbo_id[0]);
			Invalidate();
		}
	}
}

#include "StdAfx.h"
#include "ConverterWhiz.h"

namespace Test
{
	btCollisionShape* ShapeFromVTNModel(VTNModel* model)
	{
		btTriangleMesh* mesh = new btTriangleMesh();
		mesh->m_weldingThreshold = 0.05f;
		VUVNTTCVertexBuffer* vbo = model->GetVBO();
		for(unsigned int i = 0; i < vbo->vertex_infos.size();)
		{
			VUVNTTC vinfo_a = vbo->vertex_infos[i++];
			VUVNTTC vinfo_b = vbo->vertex_infos[i++];
			VUVNTTC vinfo_c = vbo->vertex_infos[i++];

			btVector3 a = btVector3(vinfo_a.x.x, vinfo_a.x.y, vinfo_a.x.z);
			btVector3 b = btVector3(vinfo_b.x.x, vinfo_b.x.y, vinfo_b.x.z);
			btVector3 c = btVector3(vinfo_c.x.x, vinfo_c.x.y, vinfo_c.x.z);

			mesh->addTriangle(a, b, c, true);
		}
		return new btBvhTriangleMeshShape(mesh, false);
	}

	btCollisionShape* ShapeFromSkinnedModel(SkinnedModel* model)
	{
		btTriangleMesh* mesh = new btTriangleMesh();
		mesh->m_weldingThreshold = 0.05f;
		for(vector<MaterialModelPair>::iterator iter = model->material_model_pairs.begin(); iter != model->material_model_pairs.end(); iter++)
		{
			SkinVInfoVertexBuffer* vbo = (SkinVInfoVertexBuffer*)iter->vbo;

			for(unsigned int i = 0; i < vbo->vertex_infos.size();)
			{
				SkinVInfo vinfo_a = vbo->vertex_infos[i++];
				SkinVInfo vinfo_b = vbo->vertex_infos[i++];
				SkinVInfo vinfo_c = vbo->vertex_infos[i++];

				btVector3 a = btVector3(vinfo_a.x.x, vinfo_a.x.y, vinfo_a.x.z);
				btVector3 b = btVector3(vinfo_b.x.x, vinfo_b.x.y, vinfo_b.x.z);
				btVector3 c = btVector3(vinfo_c.x.x, vinfo_c.x.y, vinfo_c.x.z);

				mesh->addTriangle(a, b, c, true);
			}
		}
		return new btBvhTriangleMeshShape(mesh, false);
	}

	btCollisionShape* HullFromVTNModel(VTNModel* model)
	{
		
		btConvexHullShape* shape = new btConvexHullShape();

		VUVNTTCVertexBuffer* vbo = model->GetVBO();
		for(unsigned int i = 0; i < vbo->vertex_infos.size();)
		{
			VUVNTTC vinfo_a = vbo->vertex_infos[i++];
			VUVNTTC vinfo_b = vbo->vertex_infos[i++];
			VUVNTTC vinfo_c = vbo->vertex_infos[i++];

			shape->addPoint(btVector3(vinfo_a.x.x, vinfo_a.x.y, vinfo_a.x.z));
			shape->addPoint(btVector3(vinfo_b.x.x, vinfo_b.x.y, vinfo_b.x.z));
			shape->addPoint(btVector3(vinfo_c.x.x, vinfo_c.x.y, vinfo_c.x.z));
		}
		return shape;
	}
}

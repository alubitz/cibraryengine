#include "StdAfx.h"
#include "MultiSphereShape.h"

#include "Physics.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "DebugDrawMaterial.h"

#include "Serialize.h"

namespace CibraryEngine
{
	/*
	 * MultiSphereShape methods
	 */
	MultiSphereShape::MultiSphereShape() : CollisionShape(ST_MultiSphere), centers(NULL), radii(NULL), count(0) { }

	MultiSphereShape::MultiSphereShape(Vec3* centers_, float* radii_, unsigned int count) : CollisionShape(ST_MultiSphere), centers(NULL), radii(NULL), count(count)
	{
		if(count > 0)
		{
			centers = new Vec3[count];
			radii = new float[count];

			for(unsigned int i = 0; i < count; ++i)
			{
				centers[i] = centers_[i];
				radii[i] = radii_[i];
			}
		}
	}

	void MultiSphereShape::InnerDispose()
	{
		delete[] centers;
		centers = NULL;

		delete[] radii;	
		radii = NULL;

		count = 0;
	}

	MassInfo MultiSphereShape::ComputeMassInfo()
	{
		// TODO: implement this for real
		MassInfo temp;
		for(unsigned int i = 0; i < count; ++i)
			temp += MassInfo(centers[i], pow(radii[i], 3.0f));

		return temp;
	}

	void MultiSphereShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
	{
		// TODO: implement this for real
		float radius = 1.0f;

		Mat3 rm = ori.ToMat3();
		Vec3 x = Vec3(rm[0], rm[1], rm[2]) * radius;
		Vec3 y = Vec3(rm[3], rm[4], rm[5]) * radius;
		Vec3 z = Vec3(rm[6], rm[7], rm[8]) * radius;

		static const Vec3 r(1, 0, 0), g(0, 1, 0), b(0, 0, 1);

		renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - x, pos + x, r), 1.0f));
		renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - y, pos + y, g), 1.0f));
		renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - z, pos + z, b), 1.0f));
	}

	void MultiSphereShape::Write(ostream& stream)
	{
		WriteUInt32(count, stream);
		for(unsigned int i = 0; i < count; ++i)
		{
			WriteVec3(centers[i], stream);
			WriteSingle(radii[i], stream);
		}
	}

	unsigned int MultiSphereShape::Read(istream& stream)
	{
		count = ReadUInt32(stream);

		if(count > 0)
		{
			centers = new Vec3[count];
			radii = new float[count];

			for(unsigned int i = 0; i < count; ++i)
			{
				centers[i] = ReadVec3(stream);
				radii[i] = ReadSingle(stream);
			}
		}

		return stream.fail() ? 1 : 0;
	}
}

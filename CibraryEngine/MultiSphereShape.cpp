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

	MassInfo MultiSphereShape::ComputeMassInfo() { return CollisionShape::ComputeMassInfo(); }				// TODO: implement this

	void MultiSphereShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori) { }	// TODO: implement this

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

#include "StdAfx.h"
#include "MultiSphereShape.h"

#include "Physics.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "DebugDrawMaterial.h"

#include "Serialize.h"
#include "Util.h"

namespace CibraryEngine
{
	/*
	 * MultiSphereShape private implementation struct
	 */
	struct MultiSphereShape::Imp
	{
		Vec3* centers;
		float* radii;
		unsigned int count;

		Imp() : centers(NULL), radii(NULL), count(0) { }

		Imp(Vec3* centers_, float* radii_, unsigned int count) : centers(NULL), radii(NULL), count(count)
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

		~Imp()
		{
			delete[] centers;
			centers = NULL;

			delete[] radii;	
			radii = NULL;

			count = 0;
		}

		void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
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

		MassInfo ComputeMassInfo()
		{
			// TODO: implement this for real
			MassInfo temp;
			for(unsigned int i = 0; i < count; ++i)
				temp += MassInfo(centers[i], pow(radii[i], 3.0f));

			return temp;
		}

		bool Contains(const Vec3& point)
		{
			// TODO: implement this for real

			for(unsigned int i = 0; i < count; ++i)
			{
				Vec3 i_cen = centers[i];
				Vec3 dif = point - i_cen;
				float i_radius = radii[i];

				if(dif.ComputeMagnitudeSquared() < i_radius * i_radius)
					return true;
			}

			return false;
		}

		bool CollisionCheck(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: implement this
			return false;
		}

		bool CollisionCheck(const Sphere& sphere, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: implement this
			return false;
		}

		bool CollisionCheck(const Mat4& my_xform, const Plane& plane, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: maybe fudge a bit to make face/edge collisions less chaotic?
			float min = 0.0f;

			Vec3 plane_norm = plane.normal;
			float plane_offset = plane.offset;

			for(unsigned int sphere_num = 0; sphere_num < count; ++sphere_num)
			{
				Vec3 sphere_pos = my_xform.TransformVec3(centers[sphere_num], 1.0f);
				float radius = radii[sphere_num];

				float dist = Vec3::Dot(plane_norm, sphere_pos) - plane_offset - radius;

				if(dist < min)
				{
					min = dist;

					Vec3 a_pos = sphere_pos - plane_norm * radius;

					result = ContactPoint();
					result.a.obj = ibody;
					result.b.obj = jbody;
					result.a.pos = a_pos;
					result.b.pos = a_pos - plane_norm * (Vec3::Dot(a_pos, plane_norm) - plane_offset);
					result.b.norm = plane_norm;
					result.a.norm = -plane_norm;
				}
			}

			return min < 0.0f;
		}

		bool CollisionCheck(const Mat4& xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: implement this
			return false;
		}

		void Write(ostream& stream)
		{
			WriteUInt32(count, stream);
			for(unsigned int i = 0; i < count; ++i)
			{
				WriteVec3(centers[i], stream);
				WriteSingle(radii[i], stream);
			}
		}

		unsigned int Read(istream& stream)
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
	};




	/*
	 * MultiSphereShape methods
	 */
	MultiSphereShape::MultiSphereShape() : CollisionShape(ST_MultiSphere), imp(new Imp()) { }
	MultiSphereShape::MultiSphereShape(Vec3* centers, float* radii, unsigned int count) : CollisionShape(ST_MultiSphere), imp(new Imp(centers, radii, count)) { }

	void MultiSphereShape::InnerDispose() { delete imp; imp = NULL; }

	void MultiSphereShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori) { imp->DebugDraw(renderer, pos, ori); }

	MassInfo MultiSphereShape::ComputeMassInfo() { return imp->ComputeMassInfo(); }

	bool MultiSphereShape::Contains(const Vec3& point) { return imp->Contains(point); }

	bool MultiSphereShape::CollisionCheck(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(ray, result, time, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Mat4& my_xform, const Plane& plane, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(my_xform, plane, result, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Sphere& sphere, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(sphere, result, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Mat4& xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(xform, other, result, ibody, jbody); }

	void MultiSphereShape::Write(ostream& stream) { imp->Write(stream);}
	unsigned int MultiSphereShape::Read(istream& stream) { return imp->Read(stream); }	
}

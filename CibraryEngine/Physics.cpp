#include "StdAfx.h"
#include "Physics.h"

#include "RigidBody.h"

#include "CollisionShape.h"
#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#include "CollisionGraph.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Util.h"

#include "DebugDrawMaterial.h"

#define MAX_SEQUENTIAL_SOLVER_ITERATIONS 50

namespace CibraryEngine
{
	/*
	 * PhysicsWorld private implementation struct
	 */
	struct PhysicsWorld::Imp
	{
		boost::unordered_set<RigidBody*> rigid_bodies;											// List of all of the rigid bodies in the physical simulation

		boost::unordered_set<RigidBody*> shape_bodies[ST_ShapeTypeMax];							// Lists of rigid bodies by shape type

		Vec3 gravity;

		float internal_timer, timer_interval;

		Imp();
		~Imp();

		void AddRigidBody(RigidBody* body);
		bool RemoveRigidBody(RigidBody* body);

		void GetUseMass(const Vec3& direction, const ContactPoint& cp, float& A, float& B);

		// returns whether or not a collision response was actually needed (i.e. whether an impulse was applied to prevent interpenetration)
		bool DoCollisionResponse(const ContactPoint& cp);

		// do collision responses to make stuff work
		void SolveCollisionGraph(CollisionGraph& graph);

		void DoFixedStep();

		void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time = 1.0f, RigidBody* ibody = NULL);
	};




	/*
	 * PhysicsWorld and PhysicsWorld::Imp methods
	 */
	PhysicsWorld::Imp::Imp() : rigid_bodies(), shape_bodies(), gravity(0, -9.8f, 0), internal_timer(), timer_interval(1.0f / 60.0f) { }

	PhysicsWorld::Imp::~Imp()
	{
		// dispose rigid bodies
		for(boost::unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->Dispose();
		rigid_bodies.clear();
	}

	void PhysicsWorld::Imp::AddRigidBody(RigidBody* r)
	{
		if(r == NULL)
			return;

		if(rigid_bodies.find(r) != rigid_bodies.end())
			return;

		rigid_bodies.insert(r);
		if(r->GetCollisionShape() == NULL)
			Debug("RigidBody has a NULL collision shape!\n");

		ShapeType shape_type = r->GetCollisionShape()->GetShapeType();
		shape_bodies[shape_type].insert(r);

		// set gravity upon adding to world
		r->gravity = gravity;
	}
			
	bool PhysicsWorld::Imp::RemoveRigidBody(RigidBody* r)
	{
		if(r == NULL)
			return false;

		boost::unordered_set<RigidBody*>::iterator found = rigid_bodies.find(r);
		if(found != rigid_bodies.end())
		{
			rigid_bodies.erase(found);

			ShapeType shape_type = r->GetCollisionShape()->GetShapeType();
			shape_bodies[shape_type].erase(r);

			return true;
		}
		else
			return false;
	}

	// much cleaner than that quadratic formula nonsense
	void PhysicsWorld::Imp::GetUseMass(const Vec3& direction, const ContactPoint& cp, float& A, float& B)
	{
		RigidBody* ibody = cp.a.obj;
		RigidBody* jbody = cp.b.obj;

		float m1 = ibody->mass_info.mass, m2 = jbody->mass_info.mass;

		A = B = 0;

		if(jbody->can_move)
		{
			A = ibody->inv_mass + jbody->inv_mass;

			Vec3 i_lvel = ibody->vel, j_lvel = jbody->vel;
			B = Vec3::Dot(i_lvel, direction) - Vec3::Dot(j_lvel, direction);
		}
		else
		{
			A = ibody->inv_mass;
			B = Vec3::Dot(ibody->vel, direction);
		}

		if(ibody->can_rotate)
		{
			Vec3 nr1 = Vec3::Cross(direction, cp.a.pos - ibody->GetTransformationMatrix().TransformVec3(ibody->mass_info.com, 1.0f));
			A += Vec3::Dot(ibody->inv_moi * nr1, nr1);
			B += Vec3::Dot(ibody->rot, nr1);
		}

		if(jbody->can_rotate)
		{
			Vec3 nr2 = Vec3::Cross(direction, cp.b.pos - jbody->GetTransformationMatrix().TransformVec3(jbody->mass_info.com, 1.0f));
			A += Vec3::Dot(jbody->inv_moi * nr2, nr2);
			B -= Vec3::Dot(jbody->rot, nr2);
		}
	}

	bool PhysicsWorld::Imp::DoCollisionResponse(const ContactPoint& cp)
	{
		RigidBody* ibody = cp.a.obj;
		RigidBody* jbody = cp.b.obj;

		bool j_can_move = jbody->can_move;

		float m1 = ibody->mass_info.mass;
		float m2 = jbody->mass_info.mass;

		if(m1 + m2 > 0)
		{
			Vec3 i_poi = ibody->GetInvTransform().TransformVec3(cp.a.pos, 1.0f);
			Vec3 j_poi = jbody->GetInvTransform().TransformVec3(cp.b.pos, 1.0f);

			Vec3 i_v = ibody->GetLocalVelocity(cp.a.pos);
			Vec3 j_v = jbody->GetLocalVelocity(cp.b.pos);
					
			Vec3 dv = j_v - i_v;
			const Vec3& normal = Vec3::Normalize(cp.a.norm - cp.b.norm);

			float nvdot = Vec3::Dot(normal, dv);
			if(nvdot < 0.0f)
			{
				float A, B;
				GetUseMass(normal, cp, A, B);

				float use_mass = 1.0f / A;
				float bounciness = ibody->bounciness * jbody->bounciness;
				float impulse_mag = -(1.0f + bounciness) * B * use_mass;
				
				if(impulse_mag < 0)
				{
					Vec3 impulse = normal * impulse_mag;

					if(impulse.ComputeMagnitudeSquared() != 0)
					{
						ibody->ApplyImpulse(impulse, i_poi);
						if(j_can_move)
							jbody->ApplyImpulse(-impulse, j_poi);

						// applying this impulse means we need to recompute dv and nvdot!
						dv = jbody->GetLocalVelocity(cp.b.pos) - ibody->GetLocalVelocity(cp.a.pos);
						nvdot = Vec3::Dot(normal, dv);
					}

					float sfric_coeff = ibody->friction * jbody->friction;
					float kfric_coeff = 0.9f * sfric_coeff;

					Vec3 t_dv = dv - normal * nvdot;
					float t_dv_magsq = t_dv.ComputeMagnitudeSquared();

					if(t_dv_magsq > 0.001f)							// object is moving; apply kinetic friction
					{
						float t_dv_mag = sqrtf(t_dv_magsq), inv_tdmag = 1.0f / t_dv_mag;
						Vec3 u_tdv = t_dv * inv_tdmag;

						GetUseMass(u_tdv, cp, A, B);
						use_mass = 1.0f / A;

						Vec3 fric_impulse = t_dv * min(use_mass, fabs(impulse_mag * kfric_coeff * inv_tdmag));

						ibody->ApplyImpulse(fric_impulse, i_poi);
						if(j_can_move)
							jbody->ApplyImpulse(-fric_impulse, j_poi);
					}
					else											// object isn't moving; apply static friction
					{
						Vec3 df = jbody->applied_force - ibody->applied_force;
						float nfdot = Vec3::Dot(normal, df);

						Vec3 t_df = df - normal * nfdot;
						float t_df_mag = t_df.ComputeMagnitude();

						float fric_i_mag = min(impulse_mag * sfric_coeff, t_df_mag);
						if(fric_i_mag > 0)
						{
							Vec3 fric_impulse = t_df * (-fric_i_mag / t_df_mag);

							ibody->ApplyImpulse(fric_impulse, i_poi);
							if(j_can_move)
								jbody->ApplyImpulse(-fric_impulse, j_poi);
						}
					}

					return true;
				}
			}
		}

		return false;
	}

	void PhysicsWorld::Imp::SolveCollisionGraph(CollisionGraph& graph)
	{
		// break the graph into separate subgraphs
		struct Subgraph
		{
			boost::unordered_set<CollisionGraph::Node*> nodes;
			vector<ContactPoint*> contact_points;

			Subgraph() : contact_points() { }

			bool ContainsNode(CollisionGraph::Node* node) { return nodes.find(node) != nodes.end(); }
		};
		
		boost::unordered_set<Subgraph*> subgraphs;
		boost::unordered_map<RigidBody*, Subgraph*> body_subgraphs;
		
		for(map<RigidBody*, CollisionGraph::Node*>::iterator iter = graph.nodes.begin(); iter != graph.nodes.end(); ++iter)
		{
			boost::unordered_map<RigidBody*, Subgraph*>::iterator found = body_subgraphs.find(iter->first);
			
			if(found == body_subgraphs.end())
			{
				Subgraph* subgraph = new Subgraph();
				subgraphs.insert(subgraph);

				vector<CollisionGraph::Node*> fringe;
				fringe.push_back(iter->second);

				while(!fringe.empty())
				{
					CollisionGraph::Node* node = *fringe.rbegin();
					subgraph->nodes.insert(node);

					body_subgraphs[node->body] = subgraph;

					fringe.pop_back();

					for(vector<CollisionGraph::Edge>::iterator jter = node->edges.begin(); jter != node->edges.end(); ++jter)
					{
						subgraph->contact_points.push_back(jter->cp);

						if(CollisionGraph::Node* other = jter->other_node)
							if(!subgraph->ContainsNode(other))
								fringe.push_back(other);
					}
				}
			}
		}

		// now go through each subgraph and do as many iterations as are necessary
		for(boost::unordered_set<Subgraph*>::iterator iter = subgraphs.begin(); iter != subgraphs.end(); ++iter)
		{
			Subgraph& subgraph = **iter;

			for(int i = 0; i < MAX_SEQUENTIAL_SOLVER_ITERATIONS; ++i)
			{
				bool any = false;
				for(vector<ContactPoint*>::iterator jter = subgraph.contact_points.begin(); jter != subgraph.contact_points.end(); ++jter)
				{
					if(DoCollisionResponse(**jter))
					{
						any = true;

						if((*jter)->a.obj->collision_callback)
							(*jter)->a.obj->collision_callback->OnCollision(**jter);

						if((*jter)->b.obj->collision_callback)
							(*jter)->b.obj->collision_callback->OnCollision(**jter);
					}
				}

				if(!any)
					break;
			}
		}

		// clean up subgraphs
		for(boost::unordered_set<Subgraph*>::iterator iter = subgraphs.begin(); iter != subgraphs.end(); ++iter)
			delete *iter;
	}

	void PhysicsWorld::Imp::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time, RigidBody* ibody)
	{
		Ray ray;
		ray.origin = from;
		ray.direction = to - from;

		struct Hit
		{
			float t;
			ContactPoint p;

			Hit(float t, ContactPoint p) : t(t), p(p) { }
			bool operator <(Hit& h) { return t < h.t; }
		};

		list<Hit> hits;

		// ray-sphere collisions
		for(boost::unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_Sphere].begin(); jter != shape_bodies[ST_Sphere].end(); ++jter)
		{
			RigidBody* jbody = *jter;

			float first, second;

			if(Util::RaySphereIntersect(ray, Sphere(jbody->GetPosition(), ((SphereShape*)jbody->GetCollisionShape())->radius), first, second))
				if(first >= 0 && first < max_time)
				{
					ContactPoint p;

					p.a.obj = ibody;
					p.b.obj = jbody;
					p.a.pos = ray.origin + first * ray.direction;
					p.b.norm = Vec3::Normalize(p.a.pos - jbody->GetPosition(), 1.0f);

					hits.push_back(Hit(first, p));
				}
		}
				
		// ray-mesh collisions
		for(boost::unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_TriangleMesh].begin(); jter != shape_bodies[ST_TriangleMesh].end(); ++jter)
		{
			RigidBody* jbody = *jter;
			TriangleMeshShape* mesh = (TriangleMeshShape*)jbody->GetCollisionShape();

			Mat4 inv_mat = jbody->GetInvTransform();

			Ray ray_cut;
			ray_cut.origin = inv_mat.TransformVec3(ray.origin, 1.0f);
			ray_cut.direction = inv_mat.TransformVec3(ray.direction * max_time, 0.0f);

			vector<Intersection> mesh_hits = mesh->RayTest(ray_cut);

			for(vector<Intersection>::iterator kter = mesh_hits.begin(); kter != mesh_hits.end(); ++kter)
			{
				float t = kter->time * max_time;
				if(t >= 0 && t < max_time)
				{
					ContactPoint p;

					p.a.obj = ibody;
					p.b.obj = jbody;
					p.a.pos = ray.origin + t * ray.direction;
					p.b.norm = kter->normal;

					hits.push_back(Hit(t, p));
				}
			}
		}

		// ray-plane collisions
		for(boost::unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_InfinitePlane].begin(); jter != shape_bodies[ST_InfinitePlane].end(); ++jter)
		{
			RigidBody* jbody = *jter;

			InfinitePlaneShape* plane = (InfinitePlaneShape*)jbody->GetCollisionShape();

			float t = Util::RayPlaneIntersect(ray, plane->plane);
			if(t >= 0 && t < max_time)
			{
				ContactPoint p;

				p.a.obj = ibody;
				p.b.obj = jbody;
				p.a.pos = ray.origin + t * ray.direction;
				p.b.norm = plane->plane.normal;

				hits.push_back(Hit(t, p));
			}
		}

		// ray-multisphere collisions
		for(boost::unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_MultiSphere].begin(); jter != shape_bodies[ST_MultiSphere].end(); ++jter)
		{
			RigidBody* jbody = *jter;
			MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

			Mat4 inv_mat = jbody->GetInvTransform();

			Ray nu_ray;
			nu_ray.origin = inv_mat.TransformVec3(ray.origin, 1.0f);
			nu_ray.direction = inv_mat.TransformVec3(ray.direction * max_time, 0.0f);

			ContactPoint p;
			float t;
			if(shape->CollisionCheck(nu_ray, p, t, ibody, jbody))
			{
				p.a.pos = jbody->GetTransformationMatrix().TransformVec3(p.b.pos, 1.0f);
				hits.push_back(Hit(t * max_time, p));
			}
		}

		if(!hits.empty())
		{
			hits.sort();

			for(list<Hit>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
				if(callback.OnCollision(jter->p))
					break;
		}
	}

	void PhysicsWorld::Imp::DoFixedStep()
	{
		float timestep = timer_interval;

		// set forces to what was applied by gravity / user forces
		for(boost::unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->UpdateVel(timestep);

		// handle all the collisions involving rays
		for(boost::unordered_set<RigidBody*>::iterator iter = shape_bodies[ST_Ray].begin(); iter != shape_bodies[ST_Ray].end(); ++iter)
		{
			RigidBody* ibody = *iter;

			struct RayCallback : public CollisionCallback
			{
				Imp* imp;
				CollisionCallback* callback;
				RayCallback(Imp* imp, CollisionCallback* callback) : imp(imp), callback(callback) { }

				bool OnCollision(const ContactPoint& cp)			// return value controls whether to continue iterating through ray collisions
				{
					if(callback && !callback->OnCollision(cp))
						return true;
					else
					{
						imp->DoCollisionResponse(cp);

						return true;
					}
				}
			} my_callback(this, ibody->GetCollisionCallback());

			RayTest(ibody->GetPosition(), ibody->GetPosition() + ibody->GetLinearVelocity(), my_callback, timestep, ibody);
		}


		CollisionGraph collision_graph;

		// handle all the collisions involving spheres
		for(boost::unordered_set<RigidBody*>::iterator iter = shape_bodies[ST_Sphere].begin(); iter != shape_bodies[ST_Sphere].end(); ++iter)
		{
			RigidBody* ibody = *iter;
			float radius = ((SphereShape*)ibody->GetCollisionShape())->radius;

			Vec3 pos = ibody->GetPosition();
			Vec3 vel = ibody->GetLinearVelocity();

			list<ContactPoint> hits;

			// sphere-sphere collisions
			for(boost::unordered_set<RigidBody*>::iterator jter = iter; jter != shape_bodies[ST_Sphere].end(); ++jter)
			{
				if(iter != jter)
				{
					RigidBody* jbody = *jter;
					float sr = radius + ((SphereShape*)jbody->GetCollisionShape())->radius;

					Vec3 other_pos = jbody->GetPosition();
					Vec3 other_vel = jbody->GetLinearVelocity();

					Ray ray;
					ray.origin = pos - other_pos;
					ray.direction = vel - other_vel;

					float first = 0, second = 0;
					if(ray.origin.ComputeMagnitudeSquared() < sr * sr || Util::RaySphereIntersect(ray, Sphere(Vec3(), sr), first, second))
						if(first >= 0 && first <= timestep)
						{
							ContactPoint p;

							p.a.obj = ibody;
							p.b.obj = jbody;
							p.a.pos = pos + first * vel;
							p.b.pos = other_pos + first * other_vel;
							p.b.norm = Vec3::Normalize(p.a.pos - other_pos, 1.0f);
							p.a.norm = -p.b.norm;

							hits.push_back(p);
						}
				}
			}

			// sphere-mesh collisions
			for(boost::unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_TriangleMesh].begin(); jter != shape_bodies[ST_TriangleMesh].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				TriangleMeshShape* shape = (TriangleMeshShape*)jbody->GetCollisionShape();

				Ray ray;
				ray.origin = pos;
				ray.direction = vel;

				vector<unsigned int> relevant_triangles = shape->GetRelevantTriangles(AABB(pos, radius));
				for(vector<unsigned int>::iterator kter = relevant_triangles.begin(); kter != relevant_triangles.end(); ++kter)
				{
					TriangleMeshShape::TriCache tri = shape->GetTriangleData(*kter);
							
					float dist = tri.DistanceToPoint(pos);
					if(dist < radius)
					{
						ContactPoint p;

						p.a.obj = ibody;
						p.b.obj = jbody;
						p.a.pos = pos - tri.plane.normal * radius;
						p.b.pos = p.a.pos;
						p.a.norm = -tri.plane.normal;
						p.b.norm = tri.plane.normal;

						hits.push_back(p);
					}
				}
			}

			// sphere-plane collisions
			for(boost::unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_InfinitePlane].begin(); jter != shape_bodies[ST_InfinitePlane].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				InfinitePlaneShape* shape = (InfinitePlaneShape*)jbody->GetCollisionShape();

				Vec3 plane_norm = shape->plane.normal;
				float plane_offset = shape->plane.offset;

				float center_offset = Vec3::Dot(plane_norm, pos) - plane_offset;
				float vel_dot = Vec3::Dot(plane_norm, vel);

				// if the sphere is moving away from the plane, don't bounce!
				if(vel_dot <= 0.0f)
				{
					float dist = fabs(center_offset) - radius;
					float t = center_offset - radius < 0.0f ? 0.0f : dist / fabs(vel_dot);
					if(t >= 0 && t <= timestep)
					{
						ContactPoint p;

						p.a.obj = ibody;
						p.b.obj = jbody;
						p.a.pos = pos - plane_norm * radius;
						p.b.pos = p.a.pos - plane_norm * (Vec3::Dot(p.a.pos, plane_norm) - plane_offset);
						p.b.norm = plane_norm;
						p.a.norm = -plane_norm;

						hits.push_back(p);
					}
				}
			}

			// sphere-multisphere collisions
			for(boost::unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_MultiSphere].begin(); jter != shape_bodies[ST_MultiSphere].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

				Mat4 inv_xform = Mat4::Invert(jbody->GetTransformationMatrix());
				Vec3 nu_pos = inv_xform.TransformVec3(pos, 1.0f);

				ContactPoint cp;
				if(shape->CollisionCheck(Sphere(nu_pos, radius), cp, ibody, jbody))
					hits.push_back(cp);
			}

			if(!hits.empty())
				for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
					collision_graph.AddContactPoint(*jter);
		}

		// handle all the collisions involving multispheres
		for(boost::unordered_set<RigidBody*>::iterator iter = shape_bodies[ST_MultiSphere].begin(); iter != shape_bodies[ST_MultiSphere].end(); ++iter)
		{
			RigidBody* ibody = *iter;
			MultiSphereShape* ishape = (MultiSphereShape*)ibody->GetCollisionShape();

			Vec3 vel = ibody->GetLinearVelocity();
			Vec3 pos = ibody->GetPosition();
			Mat4 xform = ibody->GetTransformationMatrix();

			AABB i_aabb = ishape->GetAABB();
			list<ContactPoint> hits;
			
			// multisphere-mesh collisions
			for(boost::unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_TriangleMesh].begin(); jter != shape_bodies[ST_TriangleMesh].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				TriangleMeshShape* jshape = (TriangleMeshShape*)jbody->GetCollisionShape();

				Mat4 inv_net_xform = jbody->GetInvTransform() * ibody->GetTransformationMatrix();
				AABB xformed_aabb = i_aabb.GetTransformedAABB(inv_net_xform);						// the AABB of the multisphere in the coordinate system of the mesh

				vector<unsigned int> relevant_triangles = jshape->GetRelevantTriangles(xformed_aabb);
				if(relevant_triangles.empty())
					continue;

				ContactPoint p;
				for(vector<unsigned int>::iterator kter = relevant_triangles.begin(); kter != relevant_triangles.end(); ++kter)
				{
					TriangleMeshShape::TriCache tri = jshape->GetTriangleData(*kter);

					ContactPoint p;
					if(ishape->CollisionCheck(inv_net_xform, tri, p, ibody, jbody))
					{
						Mat4 j_xform = jbody->GetTransformationMatrix();

						p.a.pos = p.b.pos = j_xform.TransformVec3(p.a.pos, 1.0f);
						p.a.norm = j_xform.TransformVec3(p.a.norm, 0.0f);
						p.b.norm = -p.a.norm;

						hits.push_back(p);
					}
				}
			}

			// multisphere-plane collisions
			for(boost::unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_InfinitePlane].begin(); jter != shape_bodies[ST_InfinitePlane].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				InfinitePlaneShape* jshape = (InfinitePlaneShape*)jbody->GetCollisionShape();

				ContactPoint p;
				if(ishape->CollisionCheck(xform, jshape->plane, p, ibody, jbody))
					hits.push_back(p);
			}

			// multisphere-multisphere collisions
			for(boost::unordered_set<RigidBody*>::iterator jter = iter; jter != shape_bodies[ST_MultiSphere].end(); ++jter)
			{
				if(iter != jter)
				{
					RigidBody* jbody = *jter;
					MultiSphereShape* jshape = (MultiSphereShape*)jbody->GetCollisionShape();

					Mat4 net_xform = ibody->GetInvTransform() * jbody->GetTransformationMatrix();

					ContactPoint p;
					if(ishape->CollisionCheck(net_xform, jshape, p, ibody, jbody))
					{
						p.a.pos = p.b.pos = xform.TransformVec3(p.a.pos, 1.0f);
						p.a.norm = xform.TransformVec3(p.a.norm, 0.0f);
						p.b.norm = -p.a.norm;

						hits.push_back(p);
					}
				}
			}

			if(!hits.empty())
				for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
					collision_graph.AddContactPoint(*jter);
		}

		SolveCollisionGraph(collision_graph);

		// update positions
		for(boost::unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->UpdatePos(timestep);
	}

	

	PhysicsWorld::PhysicsWorld() : imp(new Imp()) { }

	void PhysicsWorld::InnerDispose() { delete imp; imp = NULL; }

	void PhysicsWorld::AddRigidBody(RigidBody* r) { imp->AddRigidBody(r); }
	bool PhysicsWorld::RemoveRigidBody(RigidBody* r) { return imp->RemoveRigidBody(r); }

	void PhysicsWorld::Update(TimingInfo time)
	{
		imp->internal_timer += time.elapsed;
		while(imp->internal_timer >= imp->timer_interval)
		{
			imp->DoFixedStep();
			imp->internal_timer -= imp->timer_interval;
		}

		for(boost::unordered_set<RigidBody*>::iterator iter = imp->rigid_bodies.begin(); iter != imp->rigid_bodies.end(); ++iter)
			(*iter)->ResetForces();
	}

	void PhysicsWorld::DebugDrawWorld(SceneRenderer* renderer)
	{
		for(boost::unordered_set<RigidBody*>::iterator iter = imp->rigid_bodies.begin(); iter != imp->rigid_bodies.end(); ++iter)
			(*iter)->DebugDraw(renderer);

		renderer->Render();
		renderer->Cleanup();
	}

	Vec3 PhysicsWorld::GetGravity() { return imp->gravity; }
	void PhysicsWorld::SetGravity(const Vec3& gravity)
	{
		imp->gravity = gravity;

		// set gravity of all rigid bodies within the world
		for(boost::unordered_set<RigidBody*>::iterator iter = imp->rigid_bodies.begin(); iter != imp->rigid_bodies.end(); ++iter)
			(*iter)->gravity = gravity;
	}

	void PhysicsWorld::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback) { imp->RayTest(from, to, callback); }
}

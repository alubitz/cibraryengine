#include "StdAfx.h"
#include "Physics.h"

#include "RigidBody.h"
#include "PhysicsRegion.h"

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
	using boost::unordered_set;
	using boost::unordered_map;

	// forward declare a few functions
	static void DoSphereSphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, list<ContactPoint>& hits);
	static void DoSphereMesh(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, list<ContactPoint>& hits);
	static void DoSpherePlane(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, list<ContactPoint>& hits);
	static void DoSphereMultisphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, list<ContactPoint>& hits);

	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const AABB& i_aabb, list<ContactPoint>& hits);
	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, list<ContactPoint>& hits);
	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, list<ContactPoint>& hits);




	/*
	 * PhysicsWorld private implementation struct
	 */
	struct PhysicsWorld::Imp
	{
		unordered_set<PhysicsRegion*> regions;
		PhysicsRegion* region;

		Vec3 gravity;

		float internal_timer, timer_interval;

		Imp() : regions(), gravity(0, -9.8f, 0), internal_timer(), timer_interval(1.0f / 60.0f) { region = new PhysicsRegion(); regions.insert(region); }
		~Imp()
		{
			// dispose rigid bodies
			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
			{
				(*iter)->Dispose();
				delete *iter;
			}
			regions.clear();

			region = NULL;
		}

		// much cleaner than that quadratic formula nonsense
		void GetUseMass(const Vec3& direction, const ContactPoint& cp, float& A, float& B)
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

		// returns whether or not a collision response was actually needed (i.e. whether an impulse was applied to prevent interpenetration)
		bool DoCollisionResponse(const ContactPoint& cp)
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

		// do collision responses to make stuff work
		void SolveCollisionGraph(CollisionGraph& graph)
		{
			// break the graph into separate subgraphs
			struct Subgraph
			{
				unordered_set<CollisionGraph::Node*> nodes;
				vector<ContactPoint*> contact_points;

				Subgraph() : contact_points() { }

				bool ContainsNode(CollisionGraph::Node* node) { return nodes.find(node) != nodes.end(); }
			};
		
			unordered_set<Subgraph*> subgraphs;
			unordered_map<RigidBody*, Subgraph*> body_subgraphs;
		
			for(map<RigidBody*, CollisionGraph::Node*>::iterator iter = graph.nodes.begin(); iter != graph.nodes.end(); ++iter)
			{
				unordered_map<RigidBody*, Subgraph*>::iterator found = body_subgraphs.find(iter->first);
			
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
			for(unordered_set<Subgraph*>::iterator iter = subgraphs.begin(); iter != subgraphs.end(); ++iter)
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
			for(unordered_set<Subgraph*>::iterator iter = subgraphs.begin(); iter != subgraphs.end(); ++iter)
				delete *iter;
		}



		void DoFixedStep()
		{
			float timestep = timer_interval;

			// set forces to what was applied by gravity / user forces
			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
				(*iter)->UpdateVel(timestep);

			// reset cache of what objects are overlapping into nearby regions
			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
				(*iter)->ResetOverlappingObjects();

			// do broadphase; this will also update the overlapping objects
			NearPairs near_pairs;
			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
				(*iter)->AddNearPairs(timestep, near_pairs);

			// handle all the collisions involving rays
			struct RayCallback : public CollisionCallback
			{
				Imp* imp;
				RayCallback(Imp* imp) : imp(imp) { }

				bool OnCollision(const ContactPoint& cp)			// return value controls whether to continue iterating through ray collisions
				{
					CollisionCallback* callback = cp.a.obj->GetCollisionCallback();

					if(callback && !callback->OnCollision(cp))
						return true;
					else
					{
						imp->DoCollisionResponse(cp);

						return true;
					}
				}
			} ray_callback(this);

			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
				(*iter)->DoRayUpdates(timestep, ray_callback);

			// do collision detection for all of the pairs we found, and populate a CollisionGraph with the results
			CollisionGraph collision_graph;

			// sphere-sphere
			{
				unordered_map<RigidBody*, unordered_set<RigidBody*> > sphere_sphere_pairs;
				NearPairs::Data::iterator sphere_sphere_iter = near_pairs.GetSet(ST_Sphere, ST_Sphere);

				for(unordered_set<pair<RigidBody*, RigidBody*> >::iterator iter = sphere_sphere_iter->second.begin(); iter != sphere_sphere_iter->second.end(); ++iter)
				{
					if(sphere_sphere_pairs.find(iter->first) == sphere_sphere_pairs.end())
						sphere_sphere_pairs[iter->first] = unordered_set<RigidBody*>();
					sphere_sphere_pairs[iter->first].insert(iter->second);
				}

				for(unordered_map<RigidBody*, unordered_set<RigidBody*> >::iterator iter = sphere_sphere_pairs.begin(); iter != sphere_sphere_pairs.end(); ++iter)
				{
					RigidBody* ibody = iter->first;
					SphereShape* ishape = (SphereShape*)ibody->GetCollisionShape();

					float radius = ishape->radius;
					Vec3 pos = ibody->GetPosition();
					Vec3 vel = ibody->GetLinearVelocity();

					list<ContactPoint> hits;

					for(unordered_set<RigidBody*>::iterator jter = iter->second.begin(); jter != iter->second.end(); ++jter)
						DoSphereSphere(ibody, *jter, radius, pos, vel, timestep, hits);

					for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
						collision_graph.AddContactPoint(*jter);
				}
			}


			// sphere-mesh
			{
				unordered_map<RigidBody*, unordered_set<RigidBody*> > sphere_mesh_pairs;
				NearPairs::Data::iterator sphere_mesh_iter = near_pairs.GetSet(ST_Sphere, ST_TriangleMesh);

				for(unordered_set<pair<RigidBody*, RigidBody*> >::iterator iter = sphere_mesh_iter->second.begin(); iter != sphere_mesh_iter->second.end(); ++iter)
				{
					if(sphere_mesh_pairs.find(iter->first) == sphere_mesh_pairs.end())
						sphere_mesh_pairs[iter->first] = unordered_set<RigidBody*>();
					sphere_mesh_pairs[iter->first].insert(iter->second);
				}

				for(unordered_map<RigidBody*, unordered_set<RigidBody*> >::iterator iter = sphere_mesh_pairs.begin(); iter != sphere_mesh_pairs.end(); ++iter)
				{
					RigidBody* ibody = iter->first;
					SphereShape* ishape = (SphereShape*)ibody->GetCollisionShape();

					float radius = ishape->radius;
					Vec3 pos = ibody->GetPosition();
					Vec3 vel = ibody->GetLinearVelocity();

					list<ContactPoint> hits;

					for(unordered_set<RigidBody*>::iterator jter = iter->second.begin(); jter != iter->second.end(); ++jter)
						DoSphereMesh(ibody, *jter, radius, pos, vel, timestep, hits);

					for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
						collision_graph.AddContactPoint(*jter);
				}
			}

			// sphere-plane
			{
				unordered_map<RigidBody*, unordered_set<RigidBody*> > sphere_plane_pairs;
				NearPairs::Data::iterator sphere_plane_iter = near_pairs.GetSet(ST_Sphere, ST_InfinitePlane);

				for(unordered_set<pair<RigidBody*, RigidBody*> >::iterator iter = sphere_plane_iter->second.begin(); iter != sphere_plane_iter->second.end(); ++iter)
				{
					if(sphere_plane_pairs.find(iter->first) == sphere_plane_pairs.end())
						sphere_plane_pairs[iter->first] = unordered_set<RigidBody*>();
					sphere_plane_pairs[iter->first].insert(iter->second);
				}

				for(unordered_map<RigidBody*, unordered_set<RigidBody*> >::iterator iter = sphere_plane_pairs.begin(); iter != sphere_plane_pairs.end(); ++iter)
				{
					RigidBody* ibody = iter->first;
					SphereShape* ishape = (SphereShape*)ibody->GetCollisionShape();

					float radius = ishape->radius;
					Vec3 pos = ibody->GetPosition();
					Vec3 vel = ibody->GetLinearVelocity();

					list<ContactPoint> hits;

					for(unordered_set<RigidBody*>::iterator jter = iter->second.begin(); jter != iter->second.end(); ++jter)
						DoSpherePlane(ibody, *jter, radius, pos, vel, timestep, hits);

					for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
						collision_graph.AddContactPoint(*jter);
				}
			}

			// sphere-multisphere
			{
				unordered_map<RigidBody*, unordered_set<RigidBody*> > sphere_msphere_pairs;
				NearPairs::Data::iterator sphere_msphere_iter = near_pairs.GetSet(ST_Sphere, ST_MultiSphere);

				for(unordered_set<pair<RigidBody*, RigidBody*> >::iterator iter = sphere_msphere_iter->second.begin(); iter != sphere_msphere_iter->second.end(); ++iter)
				{
					if(sphere_msphere_pairs.find(iter->first) == sphere_msphere_pairs.end())
						sphere_msphere_pairs[iter->first] = unordered_set<RigidBody*>();
					sphere_msphere_pairs[iter->first].insert(iter->second);
				}

				for(unordered_map<RigidBody*, unordered_set<RigidBody*> >::iterator iter = sphere_msphere_pairs.begin(); iter != sphere_msphere_pairs.end(); ++iter)
				{
					RigidBody* ibody = iter->first;
					SphereShape* ishape = (SphereShape*)ibody->GetCollisionShape();

					float radius = ishape->radius;
					Vec3 pos = ibody->GetPosition();
					Vec3 vel = ibody->GetLinearVelocity();

					list<ContactPoint> hits;

					for(unordered_set<RigidBody*>::iterator jter = iter->second.begin(); jter != iter->second.end(); ++jter)
						DoSphereMultisphere(ibody, *jter, radius, pos, vel, timestep, hits);

					for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
						collision_graph.AddContactPoint(*jter);
				}
			}

			// multisphere-mesh
			{
				unordered_map<RigidBody*, unordered_set<RigidBody*> > msphere_mesh_pairs;
				NearPairs::Data::iterator msphere_mesh_iter = near_pairs.GetSet(ST_MultiSphere, ST_TriangleMesh);

				for(unordered_set<pair<RigidBody*, RigidBody*> >::iterator iter = msphere_mesh_iter->second.begin(); iter != msphere_mesh_iter->second.end(); ++iter)
				{
					if(msphere_mesh_pairs.find(iter->first) == msphere_mesh_pairs.end())
						msphere_mesh_pairs[iter->first] = unordered_set<RigidBody*>();
					msphere_mesh_pairs[iter->first].insert(iter->second);
				}

				for(unordered_map<RigidBody*, unordered_set<RigidBody*> >::iterator iter = msphere_mesh_pairs.begin(); iter != msphere_mesh_pairs.end(); ++iter)
				{
					RigidBody* ibody = iter->first;
					MultiSphereShape* ishape = (MultiSphereShape*)ibody->GetCollisionShape();

					AABB i_aabb = ishape->GetAABB();

					list<ContactPoint> hits;

					for(unordered_set<RigidBody*>::iterator jter = iter->second.begin(); jter != iter->second.end(); ++jter)
						DoMultisphereMesh(ibody, *jter, ishape, i_aabb, hits);

					for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
						collision_graph.AddContactPoint(*jter);
				}
			}

			// multisphere-plane
			{
				unordered_map<RigidBody*, unordered_set<RigidBody*> > msphere_plane_pairs;
				NearPairs::Data::iterator msphere_mesh_iter = near_pairs.GetSet(ST_MultiSphere, ST_InfinitePlane);

				for(unordered_set<pair<RigidBody*, RigidBody*> >::iterator iter = msphere_mesh_iter->second.begin(); iter != msphere_mesh_iter->second.end(); ++iter)
				{
					if(msphere_plane_pairs.find(iter->first) == msphere_plane_pairs.end())
						msphere_plane_pairs[iter->first] = unordered_set<RigidBody*>();
					msphere_plane_pairs[iter->first].insert(iter->second);
				}

				for(unordered_map<RigidBody*, unordered_set<RigidBody*> >::iterator iter = msphere_plane_pairs.begin(); iter != msphere_plane_pairs.end(); ++iter)
				{
					RigidBody* ibody = iter->first;
					MultiSphereShape* ishape = (MultiSphereShape*)ibody->GetCollisionShape();

					Mat4 xform = ibody->GetTransformationMatrix();
					list<ContactPoint> hits;

					for(unordered_set<RigidBody*>::iterator jter = iter->second.begin(); jter != iter->second.end(); ++jter)
						DoMultispherePlane(ibody, *jter, ishape, xform, hits);

					for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
						collision_graph.AddContactPoint(*jter);
				}
			}

			// multisphere-multisphere
			{
				unordered_map<RigidBody*, unordered_set<RigidBody*> > msphere_msphere_pairs;
				NearPairs::Data::iterator msphere_msphere_iter = near_pairs.GetSet(ST_MultiSphere, ST_MultiSphere);

				for(unordered_set<pair<RigidBody*, RigidBody*> >::iterator iter = msphere_msphere_iter->second.begin(); iter != msphere_msphere_iter->second.end(); ++iter)
				{
					if(msphere_msphere_pairs.find(iter->first) == msphere_msphere_pairs.end())
						msphere_msphere_pairs[iter->first] = unordered_set<RigidBody*>();
					msphere_msphere_pairs[iter->first].insert(iter->second);
				}

				for(unordered_map<RigidBody*, unordered_set<RigidBody*> >::iterator iter = msphere_msphere_pairs.begin(); iter != msphere_msphere_pairs.end(); ++iter)
				{
					RigidBody* ibody = iter->first;
					MultiSphereShape* ishape = (MultiSphereShape*)ibody->GetCollisionShape();

					Mat4 xform = ibody->GetTransformationMatrix();
					list<ContactPoint> hits;

					for(unordered_set<RigidBody*>::iterator jter = iter->second.begin(); jter != iter->second.end(); ++jter)
						DoMultisphereMultisphere(ibody, *jter, ishape, xform, hits);

					for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
						collision_graph.AddContactPoint(*jter);
				}
			}


			SolveCollisionGraph(collision_graph);

			// update positions
			for(unordered_set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
				(*iter)->UpdatePos(timestep);
		}

		PhysicsRegion* SelectRegion(const Vec3& pos) { return region; }

		void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time = 1.0f, RigidBody* ibody = NULL) { SelectRegion(from)->RayTest(from, to, callback, max_time, ibody); }
	};




	/*
	 * PhysicsWorld methods
	 */
	PhysicsWorld::PhysicsWorld() : imp(new Imp()) { }

	void PhysicsWorld::InnerDispose() { delete imp; imp = NULL; }

	void PhysicsWorld::AddRigidBody(RigidBody* r) { r->gravity = imp->gravity; imp->SelectRegion(r->pos)->AddRigidBody(r); }
	bool PhysicsWorld::RemoveRigidBody(RigidBody* r)
	{
		if(PhysicsRegion* region = r->region)
			return region->RemoveRigidBody(r);
		else
			return false;
	}

	void PhysicsWorld::Update(TimingInfo time)
	{
		imp->internal_timer += time.elapsed;
		while(imp->internal_timer >= imp->timer_interval)
		{
			imp->DoFixedStep();
			imp->internal_timer -= imp->timer_interval;
		}

		for(unordered_set<PhysicsRegion*>::iterator iter = imp->regions.begin(); iter != imp->regions.end(); ++iter)
			(*iter)->ResetForces();
	}

	void PhysicsWorld::DebugDrawWorld(SceneRenderer* renderer)
	{
		for(unordered_set<PhysicsRegion*>::iterator iter = imp->regions.begin(); iter != imp->regions.end(); ++iter)
			(*iter)->DebugDrawRegion(renderer);

		renderer->Render();
		renderer->Cleanup();
	}

	Vec3 PhysicsWorld::GetGravity() { return imp->gravity; }
	void PhysicsWorld::SetGravity(const Vec3& gravity)
	{
		imp->gravity = gravity;

		for(unordered_set<PhysicsRegion*>::iterator iter = imp->regions.begin(); iter != imp->regions.end(); ++iter)
			(*iter)->SetGravity(gravity);
	}

	void PhysicsWorld::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback) { imp->RayTest(from, to, callback); }




	/*
	 * NearPairs methods
	 */
	NearPairs::Data::iterator NearPairs::GetSet(ShapeType a, ShapeType b)
	{
		pair<ShapeType, ShapeType> key(a, b);	

		Data::iterator found = pairs.find(key);
		if(found != pairs.end())
			return found;
		else
		{
			pairs[key] = unordered_set<pair<RigidBody*, RigidBody*> >();
			return pairs.find(key);
		}
	}

	void NearPairs::AddPair(Data::iterator target, RigidBody* a, RigidBody* b) { target->second.insert(pair<RigidBody*, RigidBody*>(a, b)); }







	/*
	 * SphereShape collision functions
	 */
	static void DoSphereSphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, list<ContactPoint>& hits)
	{
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

	static void DoSphereMesh(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, list<ContactPoint>& hits)
	{
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

	static void DoSpherePlane(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, list<ContactPoint>& hits)
	{
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

	static void DoSphereMultisphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, list<ContactPoint>& hits)
	{
		MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

		Mat4 inv_xform = Mat4::Invert(jbody->GetTransformationMatrix());
		Vec3 nu_pos = inv_xform.TransformVec3(pos, 1.0f);

		ContactPoint cp;
		if(shape->CollisionCheck(Sphere(nu_pos, radius), cp, ibody, jbody))
			hits.push_back(cp);
	}




	/*
	 * MultiSphereShape collision functions
	 */
	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const AABB& i_aabb, list<ContactPoint>& hits)
	{
		TriangleMeshShape* jshape = (TriangleMeshShape*)jbody->GetCollisionShape();

		Mat4 inv_net_xform = jbody->GetInvTransform() * ibody->GetTransformationMatrix();
		AABB xformed_aabb = i_aabb.GetTransformedAABB(inv_net_xform);						// the AABB of the multisphere in the coordinate system of the mesh

		vector<unsigned int> relevant_triangles = jshape->GetRelevantTriangles(xformed_aabb);
		if(relevant_triangles.empty())
			return;

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

	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, list<ContactPoint>& hits)
	{
		InfinitePlaneShape* jshape = (InfinitePlaneShape*)jbody->GetCollisionShape();

		ContactPoint p;
		if(ishape->CollisionCheck(xform, jshape->plane, p, ibody, jbody))
			hits.push_back(p);
	}

	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, list<ContactPoint>& hits)
	{
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

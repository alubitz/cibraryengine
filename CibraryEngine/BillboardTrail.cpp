#include "StdAfx.h"
#include "BillboardTrail.h"
#include "BillboardMaterial.h"

#include "CameraView.h"
#include "RenderNode.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * BillboardTrail
	 */
	BillboardTrail::BillboardTrail(GameState* gs, TrailHead* trailhead, BillboardMaterial* material, float width) :
		Entity(gs),
		node_count(0),
		trail(),
		nu_trail(),
		shrinking(false),
		material(material),
		width(width),
		bs(Vec3(), -1),
		trailhead(trailhead)
	{
		AddNode();
	}

	bool BillboardTrail::AddNode()
	{
		TrailNode head;
		if((*trailhead)(head))
		{
			node_count++;
			nu_trail.push_back(trail.size());
			trail.push_back(head);

			InvalidateBoundingSphere();
			return true;
		}

		return false;
	}

	void BillboardTrail::InvalidateBoundingSphere() { bs = Sphere(Vec3(), -1); }

	void BillboardTrail::Update(TimingInfo time)
	{
		float timestep = time.elapsed;

		// collapse unused stuff
		unsigned int next = 0;
		for(unsigned int i = 0; i < node_count; i++)
		{
			TrailNode& node = trail[i];
			if(node.age <= node.max_age)
				nu_trail[next++] = i;
		}
		if(next < node_count)
		{
			InvalidateBoundingSphere();
			node_count = next;
		}
		for(unsigned int i = 0; i < node_count; i++)
			trail[i] = trail[nu_trail[i]];

		// age the nodes
		for(vector<TrailNode>::iterator iter = trail.begin(); iter != trail.end(); iter++)
			iter->age += timestep;

		// add new nodes / kill the chain
		if (!shrinking)
		{
			if(!AddNode())
				shrinking = true;
		}
		else if (node_count == 0)
			is_valid = false;
	}

	Sphere BillboardTrail::GetBoundingSphere()
	{
		if(bs.radius < 0 && node_count > 0)
		{
			bs = Sphere(trail[0].pos, width);

			for (unsigned int i = 1; i < trail.size(); i++)
				bs = Sphere::Expand(bs, Sphere(trail[i].pos, width));
		}
		return bs;
	}

	void BillboardTrail::Vis(SceneRenderer* renderer)
	{
		//if (renderer->camera->CheckSphereVisibility(GetBoundingSphere()))
		{
			for (unsigned int i = 0; i + 1 < node_count; i++)
			{
				TrailNode& a = trail[i + 1];
				TrailNode& b = trail[i];

				Vec3 front = a.pos, rear = b.pos;

				Vec3 cameraPos = renderer->camera->GetPosition();
				Vec3 fromCamera = a.pos - cameraPos;

				BillboardMaterial::NodeData* node_data = new BillboardMaterial::NodeData(front, rear, width);

				node_data->front_u = a.age / a.max_age;
				node_data->back_u = b.age / b.max_age;

				renderer->objects.push_back(RenderNode(material, node_data, Vec3::Dot(renderer->camera->GetForward(), front)));
			}
		}
	}
}

#include "StdAfx.h"
#include "EntityList.h"

#include "Entity.h"

namespace CibraryEngine
{
	/*
	 * EntityList methods
	 */
	EntityList::EntityList() : entities() { }
	EntityList::EntityList(vector<Entity*> entities) : entities(entities) { }

	unsigned int EntityList::Count() { return entities.size(); }

	Entity* EntityList::operator [](unsigned int index) { return entities[index]; }

	EntityList EntityList::Union(EntityList& a, EntityList& b)
	{
		// figure out which one is smaller, so we can have the smaller loop inside the bigger one ... i think it's faster this way?
		EntityList& smaller = a.entities.size() < b.entities.size() ? a : b;
		EntityList& bigger = a.entities.size() < b.entities.size() ? b : a;

		unsigned int smaller_size = smaller.entities.size();
		unsigned int bigger_size = bigger.entities.size();

		EntityList result = EntityList();

		// add all of the smaller list
		for(unsigned int i = 0; i < smaller_size; ++i)
			result.entities.push_back(smaller.entities[i]);

		// add whatever of the bigger list is not in the smaller list
		for(unsigned int i = 0; i < bigger_size; ++i)
		{
			Entity* ent = bigger.entities[i];
			unsigned int j;
			for(j = 0; j < smaller_size; ++j)
				if(smaller.entities[j] == ent)
					break;
			if(j == smaller_size)
				result.entities.push_back(ent);
		}

		return result;
	}

	EntityList EntityList::Intersection(EntityList& a, EntityList& b)
	{
		EntityList result = EntityList();

		unsigned int a_size = a.entities.size(), b_size = b.entities.size();

		for(unsigned int i = 0; i < a_size; ++i)
		{
			Entity* ent = a.entities[i];
			for(unsigned int j = 0; j < b_size; ++j)
				if(b.entities[j] == ent)
				{
					result.entities.push_back(ent);
					break;
				}
		}

		return result;
	}

	EntityList EntityList::Subtract(EntityList& a, EntityList& b)
	{
		EntityList result = EntityList();

		unsigned int a_size = a.entities.size(), b_size = b.entities.size();
		for(unsigned int i = 0; i < a_size; ++i)
		{
			Entity* ent = a.entities[i];
			unsigned int j;
			for(j = 0; j < b_size; ++j)
				if(b.entities[j] == ent)
					break;
			if(j == b_size)
				result.entities.push_back(ent);
		}

		return result;
	}
}

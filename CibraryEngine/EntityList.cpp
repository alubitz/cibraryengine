#include "StdAfx.h"
#include "EntityList.h"

#include "Entity.h"

namespace CibraryEngine
{
	/*
	 * EntityList methods
	 */
	EntityList::EntityList() : entities() { }
	EntityList::EntityList(const vector<Entity*>& entities) : entities(entities) { }

	unsigned int EntityList::Count() const { return entities.size(); }

	Entity* EntityList::operator [](unsigned int index) const { return entities[index]; }

	EntityList EntityList::Union(const EntityList& a, const EntityList& b)
	{
		// figure out which one is smaller, so we can have the smaller loop inside the bigger one ... i think it's faster this way?
		const EntityList& smaller = a.entities.size() < b.entities.size() ? a : b;
		const EntityList& bigger = a.entities.size() < b.entities.size() ? b : a;

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

	EntityList EntityList::Intersection(const EntityList& a, const EntityList& b)
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

	EntityList EntityList::Subtract(const EntityList& a, const EntityList& b)
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

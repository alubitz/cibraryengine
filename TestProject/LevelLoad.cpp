#include "LevelLoad.h"

#include "TestGame.h"
#include "StaticLevelGeometry.h"
#include "WorldBoundary.h"

namespace Test
{
	/*
	 * Main level loader function
	 */
	unsigned int LoadLevel(TestGame* game, string level_name)
	{
		string filename = "Files/Levels/" + level_name + ".txt";

		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		NamedItemDictionaryTableParser parser(&file);

		StaticGeometrySetter static_geoms(&file, game);
		WorldBoundarySetter world_bounds(&file, game);

		parser.field_setters["static_geometry"] = &static_geoms;
		parser.field_setters["world_boundary"] = &world_bounds;
		parser.ParseTable();

		return 0;
	}
}

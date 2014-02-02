#pragma once

namespace CibraryEngine
{
	using namespace std;

	struct ContentMetadata
	{
		string name;

		bool fail;

		bool needed_recent;
		bool needed_soon;

		ContentMetadata() :
			name(),
			fail(false),
			needed_recent(false),
			needed_soon(false)
		{
		}

		ContentMetadata(const string& name) :
			name(name),
			fail(false),
			needed_recent(false),
			needed_soon(false)
		{
		}
	};
}
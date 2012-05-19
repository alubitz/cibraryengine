#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	/** Table of external strings referred to internally by unsigned integers; 0 is not a valid value */
	struct StringTable
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			StringTable();
			~StringTable();

			bool StringExists(const string& str) const;
			bool IntExists(unsigned int i) const;

			/** Creates if not already present */
			unsigned int StringToInt(const string& str);

			string IntToString(unsigned int i) const;

			// wrap the above two functions
			unsigned int operator[](const string& str);
			string operator[](unsigned int i) const;
	};
}

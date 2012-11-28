#include "StdAfx.h"
#include "StringTable.h"

namespace CibraryEngine
{
	using namespace std;
	using boost::unordered_map;

	/*
	 * StringTable private implementation struct
	 */
	struct StringTable::Imp
	{
		unordered_map<string, unsigned int> str_to_i;
		unordered_map<unsigned int, string> i_to_str;

		unsigned int next_id;

		Imp() : str_to_i(), i_to_str(), next_id(1)
		{
			str_to_i[""] = NULL;
			i_to_str[NULL] = "";
		}
		~Imp() { }
	};




	/*
	 * StringTable methods
	 */
	StringTable::StringTable() : imp(new Imp()) { }
	StringTable::~StringTable()									{ if(imp) { delete imp; imp = NULL; } }

	bool StringTable::StringExists(const string& str) const		{ return imp->str_to_i.find(str) != imp->str_to_i.end(); }
	bool StringTable::IntExists(unsigned int i) const			{ return imp->i_to_str.find(i) != imp->i_to_str.end(); }

	unsigned int StringTable::StringToInt(const string& str)
	{
		unordered_map<string, unsigned int>::iterator found = imp->str_to_i.find(str);
		if(found == imp->str_to_i.end())
		{
			unsigned int result = imp->next_id++;

			imp->str_to_i[str] = result;
			imp->i_to_str[result] = str;

			return result;
		}
		else
			return found->second;
	}

	string StringTable::IntToString(unsigned int i) const
	{
		unordered_map<unsigned int, string>::iterator found = imp->i_to_str.find(i);
		if(found != imp->i_to_str.end())
			return found->second;
		else
			return string();			// maybe you should have checked first
	}

	unsigned int StringTable::operator[] (const string& str)	{ return StringToInt(str); }
	string StringTable::operator[] (unsigned int i) const		{ return IntToString(i); }
}

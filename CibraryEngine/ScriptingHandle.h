#pragma once 

#include "StdAfx.h"

namespace CibraryEngine
{
	/** Basically a wrapper for a pointer-to-pointer-to-T */
	template <class T> struct ScriptingHandle
	{
		private:
			
			ScriptingHandle(T** ptr) : ptr(ptr) { }

			T** ptr;

		public:

			/** Creates a scripting handle with a NULL pointer */
			static ScriptingHandle Default() { return ScriptingHandle(NULL); }

			/** Creates a scripting handle with a new pointer to the given object pointer */
			static ScriptingHandle Wrap(T* t) { return ScriptingHandle(new T*(t)); }

			/** If true, a pointer has been created for this scripting handle */
			bool HandleExists() { return ptr != NULL; }

			/** If true, the object pointed to by this scripting handle has not yet been deleted */
			bool IsValid() { return ptr != NULL && *ptr != NULL; }

			/** Get the object pointed to by this scripting handle */
			T* operator *() { return ptr == NULL ? NULL : *ptr; }

			/** Call this when you delete or dispose of the object */
			void ObjectDeleted() { if(ptr != NULL) { *ptr = NULL; } }

			/** Call this when the object is no longer referenced script-side */
			void GarbageCollect() { delete ptr; ptr = NULL; }

			void PushHandle(lua_State* L) { lua_pushlightuserdata(L, ptr); }
	};
}

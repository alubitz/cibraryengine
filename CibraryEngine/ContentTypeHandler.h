#pragma once

namespace CibraryEngine
{
	class ContentMan;
	struct ContentMetadata;

	class ContentTypeHandlerBase
	{
		public:

			/** Pointer to the content manager */
			ContentMan* man;

			/** Initializes a new content type handler */
			ContentTypeHandlerBase(ContentMan* man) : man(man) { }
	};

	/** Struct which handles a certain type of content within the overall content management system */
	template <class T> class ContentTypeHandler : public ContentTypeHandlerBase
	{
		public:

			/** Initializes a new content type handler */
			ContentTypeHandler(ContentMan* man) : ContentTypeHandlerBase(man) { }

			/**
			 * Loads some content
			 * @param what Metadata specifying which asset to load
			 * @return The data pointer for some content (returned by ContentMan::GetObject). The default implementation returns NULL.
			 */
			virtual T* Load(ContentMetadata& what) { return NULL; }

			/**
			 * Unloads some content, providing a place for you to dispose of resources and delete pointers.
			 * The data pointer (returned by ContentMan::GetObject) will be set to NULL when the content manager calls this function.
			 *
			 * @param content The data pointer for some content. If you want to delete this, you should do it here.
			 * @param meta Metadata associated with the content
			 */
			virtual void Unload(T* content, ContentMetadata& meta) { }
	};
}

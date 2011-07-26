#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	/** Class representing data about an event; subclass this with your own specific event type */
	class Event
	{
		public:

			Event();
			virtual ~Event();
	};

	// override HandleEvent with your own event handler function
	// you can get away with not extending this; the default implementation just does nothing
	/** Class which causes code to execute when an Event occurs */
	class EventHandler
	{
		public:

			EventHandler();
			virtual ~EventHandler();

			/** Function to be called when an event occurs; cast the paramater as the appropriate type */
			virtual void HandleEvent(Event* evt);
	};

	/** Class which registers event handlers and calls their HandleEvent functions when you tell it to */
	class EventDispatcher
	{
		private:

			set<EventHandler*> event_handlers;

		public:

			/** Initiliazes a new EventDispatcher with no event handlers */
			EventDispatcher();

			/** Adds an EventHandler; don't pass a "new" EventHandler, pass a pointer to an existing one you plan to delete later */
			void operator +=(EventHandler* h);
			/** Removes an EventHandler */
			void operator -=(EventHandler* h);

			/** Calls DispatchEvent */
			void operator ()(Event* evt);

			/** Calls HandleEvent on all of the registered event handlers */
			void DispatchEvent(Event* evt);
	};
}

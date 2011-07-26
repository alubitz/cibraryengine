#include "Events.h"

namespace CibraryEngine
{
	/*
	 * Event methods
	 */
	Event::Event() { }
	Event::~Event() { }



	/*
	 * EventHandler methods
	 */
	EventHandler::EventHandler() { }
	EventHandler::~EventHandler() { }

	void EventHandler::HandleEvent(Event* evt) { }




	/*
	 * EventDispatcher methods
	 */
	EventDispatcher::EventDispatcher() : event_handlers() { }

	void EventDispatcher::operator +=(EventHandler* h) { event_handlers.insert(h); }
	void EventDispatcher::operator -=(EventHandler* h) { event_handlers.erase(h); }

	void EventDispatcher::DispatchEvent(Event* evt)
	{
		for(set<EventHandler*>::iterator iter = event_handlers.begin(); iter != event_handlers.end(); iter++)
			(*iter)->HandleEvent(evt);
	}
	void EventDispatcher::operator()(Event* evt) { DispatchEvent(evt); }
}

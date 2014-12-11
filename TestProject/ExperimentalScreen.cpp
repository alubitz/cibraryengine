#include "StdAfx.h"
#include "ExperimentalScreen.h"

namespace Test
{
	/*
	 * ExperimentalScreen private implementation struct
	 */
	struct ExperimentalScreen::Imp
	{
		ExperimentalScreen* scr;
		vector<AutoMenuItem*> auto_menu_items;

		AutoMenuItem* output_text[6];

		boost::thread* my_thread;
		boost::mutex   mutex;

		bool abort;
		bool aborted;

		ProfilingTimer my_timer;
		float time_spent;

		struct BackButton : public AutoMenuItem
		{
			BackButton(ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		Imp(ExperimentalScreen* scr) : scr(scr), auto_menu_items(), my_thread(NULL), mutex(), abort(false), aborted(false)
		{
			// init screen stuff
			ContentMan* content = scr->content;

			unsigned int row = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Experimental Screen", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", row++, false));

			for(unsigned int i = 0; i < 6; ++i)
				auto_menu_items.push_back(output_text[i] = new AutoMenuItem(content, "", row++, false));

			auto_menu_items.push_back(new BackButton  (content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				scr->AddItem(auto_menu_items[i]);

			time_spent = 0.0f;

			thread_helper.imp = this;
			my_thread = new boost::thread(thread_helper);
		}

		~Imp()
		{
			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
			{
				AutoMenuItem* item = auto_menu_items[i];

				item->Dispose();
				delete item;
			}
			auto_menu_items.clear();

			if(boost::thread* temp = my_thread)
			{
				{
					boost::mutex::scoped_lock lock(mutex);
					my_thread = NULL;
				}

				temp->join();
				delete temp;
			}
		}

		void SetAbort()	    { boost::mutex::scoped_lock lock(mutex); abort = true; }
		bool CheckAbort()   { boost::mutex::scoped_lock lock(mutex); return abort; }
		void SetAborted()   { boost::mutex::scoped_lock lock(mutex); aborted = true; }
		bool CheckAborted() { boost::mutex::scoped_lock lock(mutex); return aborted; }

		void ThreadsafeSetText(unsigned int index, const string& text) { boost::mutex::scoped_lock lock(mutex); output_text[index]->SetText(text); }

		void ThreadAction()
		{
			my_timer.Start();

			unsigned int iteration = 0;
			while(!CheckAbort())
			{
				time_spent += my_timer.GetAndRestart();

				++iteration;
			}

			my_timer.Stop();

			SetAborted();
		}

		struct ThreadHelper
		{
			Imp* imp;
			void operator()() { imp->ThreadAction(); }
		} thread_helper;
	};




	/*
	 * ExperimentalScreen methods
	 */
	ExperimentalScreen::ExperimentalScreen(ProgramWindow* win, ProgramScreen* previous) : MenuScreen(win, previous), imp(NULL) { }

	void ExperimentalScreen::Activate()
	{
		MenuScreen::Activate();

		if(imp == NULL)
			imp = new Imp(this);
	}

	void ExperimentalScreen::Deactivate()
	{
		MenuScreen::Deactivate();

		if(imp)
		{
			imp->SetAbort();
			while(!imp->CheckAborted()) { Sleep(1); }			// don't busy wait

			delete imp; imp = NULL;
		}
	}

	void ExperimentalScreen::Draw(int width, int height)
	{
		if(imp != NULL)
		{
			boost::mutex::scoped_lock lock(imp->mutex);
			MenuScreen::Draw(width, height);
		}
		else
			MenuScreen::Draw(width, height);
	}

	ProgramScreen* ExperimentalScreen::Update(const TimingInfo& time) { Sleep(1); return MenuScreen::Update(time); }
}

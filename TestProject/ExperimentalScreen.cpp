#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "DynamicBrain.h"

namespace Test
{
	/*
	 * ExperimentalScreen private implementation struct
	 */
	struct ExperimentalScreen::Imp
	{
		ExperimentalScreen* scr;
		vector<AutoMenuItem*> auto_menu_items;

		static const unsigned int num_output_texts = 20;
		AutoMenuItem* output_text[num_output_texts];

		boost::thread* my_thread;
		boost::mutex   mutex;

		bool abort;
		bool aborted;

		ProfilingTimer my_timer;
		float time_spent;

		struct DBDemoSimulation
		{
			float value;
			float guess;
			float up, down;
			float score;

			DBDemoSimulation() : value(0), guess(0), up(0), down(0), score(0) { value = Random3D::Rand(); }

			float GetInputValue (unsigned int index) const
			{
				switch(index)
				{
				case 0:
					return tanhf((guess - value) * 3.0f) * 0.5f + 0.5f;

				case 1:
					return tanhf((value - guess) * 3.0f) * 0.5f + 0.5f;

				default:
					return 1.0f;
				}
			}
			float GetGoalScore  (unsigned int category) const { return score; }
			void  SetOutputValue(unsigned int index, float v) { (index == 1 ? down : up) = v; }
			
			void IncrementWeightedTotal(float& tot, float& wtot, float weight, float amount)
			{
				wtot += weight;
				tot  += weight * amount;
			}

			void Simulate()
			{
				if(Random3D::RandInt() % 60 == 0)
					value = Random3D::Rand();
				
				float tot = 0.0f, wtot = 0.0f;

				IncrementWeightedTotal( tot, wtot,         2, guess );
				IncrementWeightedTotal( tot, wtot,   10 * up,     1 );
				IncrementWeightedTotal( tot, wtot, 10 * down,     0 );

				guess = tot / wtot;
			
				float error = value - guess;
				score = error * error;
			}
		};

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

			for(unsigned int i = 0; i < num_output_texts; ++i)
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

		void ThreadsafeSetText(unsigned int index, const string& text) { if(index >= num_output_texts) return; boost::mutex::scoped_lock lock(mutex); output_text[index]->SetText(text); }

		string GetFixedLengthString(float f)
		{
			float a;
			string s = "  .      ";
			if(f < 0)
			{
				a = -f;
				s[0] = '-';
			}
			else
			{
				a = f;
				s[1] = ' ';
			}

			if(a >= 10)
				a -= 10 * (int)(a / 10);

			s[1] = '0' + (int)a;
			for(unsigned int i = 0; i < 6; ++i)
			{
				a -= (int)a;
				a *= 10;
				s[i + 3] = '0' + (int)a;
			}

			return s;
		}

		void ThreadAction()
		{
			srand((unsigned int)time(NULL));

			my_timer.Start();
			
			static const unsigned int num_input_neurons    = 2;
			static const unsigned int num_output_neurons   = 2;
			static const unsigned int num_misc_neurons     = 2;
			static const unsigned int total_neurons        = num_input_neurons + num_output_neurons + num_misc_neurons;

			static const unsigned int max_synapses         = total_neurons * (total_neurons - num_input_neurons);

			static const unsigned int num_score_categories = 1;
			bool n = false, Y = true;
			bool included_synapses[max_synapses] = 
			{
				Y, Y, Y, Y,
				Y, Y, Y, Y,
				n, n, Y, n,
				n, n, n, Y,
				Y, n, Y, n,
				n, Y, n, Y
			};
			unsigned int scount = 0;
			for(unsigned int i = 0; i < max_synapses; ++i)
				if(included_synapses[i])
					++scount;		

			DynamicBrain brain(total_neurons, scount, num_score_categories);

			DynamicBrain::Synapse* sptr = brain.synapses;
			for(unsigned int y = 0; y < total_neurons; ++y)
				for(unsigned int x = num_input_neurons; x < total_neurons; ++x)
				{
					if(included_synapses[y * (num_output_neurons + num_misc_neurons) + (x - num_input_neurons)])
					{
						DynamicBrain::Synapse& s = *sptr;
						s.from = &brain.neurons[y];
						s.to = &brain.neurons[x];

						++sptr;
					}
				}

			DBDemoSimulation sim;
			static const unsigned int mavg_over_n = 60;
			float mavg_win[mavg_over_n];
			memset(mavg_win, 0, mavg_over_n * sizeof(float));
			float mavg_tot = 0.0f;

			for(unsigned int j = 0; j < num_input_neurons; ++j)
				brain.neurons[j].value = sim.GetInputValue(j);

			unsigned int iteration = 0;
			while(!CheckAbort())
			{
				float goal_scores[num_score_categories];
				for(unsigned int j = 0; j < num_score_categories; ++j)
					goal_scores[j] = sim.GetGoalScore(j);

				brain.Update(goal_scores);

				for(unsigned int j = 0; j < num_output_neurons; ++j)
					sim.SetOutputValue(j, brain.neurons[j + num_input_neurons].value);

				sim.Simulate();

				for(unsigned int j = 0; j < num_input_neurons; ++j)
					brain.neurons[j].value = sim.GetInputValue(j);

				mavg_tot -= mavg_win[iteration % mavg_over_n];
				mavg_tot += mavg_win[iteration % mavg_over_n] = sim.score;

				ThreadsafeSetText(0, ((stringstream&)(stringstream() << "i = " << iteration << "; guess = " << GetFixedLengthString(sim.guess) << "; score = " << GetFixedLengthString(sim.score) << "; moving average of score over past " << mavg_over_n << " = " << GetFixedLengthString(mavg_tot / min(iteration + 1, mavg_over_n)))).str());
				for(unsigned int i = 0; i < brain.num_neurons; ++i)
				{
					const DynamicBrain::Neuron& n = brain.neurons[i];
					ThreadsafeSetText(i + 1, ((stringstream&)(stringstream() << "\tneuron " << i << ": value = " << GetFixedLengthString(n.value))).str());
				}
				for(unsigned int i = 0; i < brain.num_synapses; ++i)
				{
					const DynamicBrain::Synapse& s = brain.synapses[i];
					unsigned int from = s.from - brain.neurons;
					unsigned int to   = s.to   - brain.neurons;
					ThreadsafeSetText(i + 1 + brain.num_neurons, ((stringstream&)(stringstream() << "\tsynapse from " << from << " to " << to << ": coeff = " << GetFixedLengthString(s.coeff))).str());
				}

				time_spent += my_timer.GetAndRestart();

				++iteration;

				Sleep(16);
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

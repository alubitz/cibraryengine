#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "MultiLayerBrain.h"

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

		thread* my_thread;
		mutex   mutex;

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

			for(unsigned int i = 0; i < num_output_texts; ++i)
				auto_menu_items.push_back(output_text[i] = new AutoMenuItem(content, "", row++, false));

			auto_menu_items.push_back(new BackButton  (content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				scr->AddItem(auto_menu_items[i]);

			time_spent = 0.0f;

			thread_helper.imp = this;
			my_thread = new thread(thread_helper);
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

			if(thread* temp = my_thread)
			{
				{
					unique_lock<std::mutex> lock(mutex);
					my_thread = NULL;
				}

				temp->join();
				delete temp;
			}
		}

		void SetAbort()	    { unique_lock<std::mutex> lock(mutex); abort = true; }
		bool CheckAbort()   { unique_lock<std::mutex> lock(mutex); return abort; }
		void SetAborted()   { unique_lock<std::mutex> lock(mutex); aborted = true; }
		bool CheckAborted() { unique_lock<std::mutex> lock(mutex); return aborted; }

		void ThreadsafeSetText(unsigned int index, const string& text) { if(index >= num_output_texts) return; unique_lock<std::mutex> lock(mutex); output_text[index]->SetText(text); }

		void ThreadAction()
		{
			static const unsigned int num_strict_inputs = 2;
			static const unsigned int num_strict_outputs = 1;
			static const unsigned int num_feedbacks = 1;
			static const unsigned int num_hidden_layers = 1;
			static const unsigned int hidden_layer_size = 25;

			static const unsigned int batch_size = 200;
			static const unsigned int item_ticks = 10;

			static const float initial_randomness = 0.01f;
			static const float learning_rate = 0.1f;

			srand((unsigned int)time(NULL));

			my_timer.Start();

			vector<unsigned int> layer_sizes;
			layer_sizes.push_back(num_strict_inputs + num_feedbacks);
			for(unsigned int i = 0; i < num_hidden_layers; ++i)
				layer_sizes.push_back(hidden_layer_size);
			layer_sizes.push_back(num_strict_outputs + num_feedbacks);

			MultiLayerBrain* nn = new MultiLayerBrain(layer_sizes);
			MultiLayerBrain* gradient = new MultiLayerBrain(layer_sizes);

			nn->Randomize(initial_randomness);
			//for(int i = 0; i < 100; ++i)
			//	nn->RandomizeOne(initial_randomness);

			vector<vector<float>> saved_inputs;
			vector<vector<float>> saved_targets;

			vector<float> inputs(num_strict_inputs + num_feedbacks);
			vector<float> outputs(num_strict_outputs + num_feedbacks);
			vector<float> scratch;
			vector<float> correct_outputs(num_strict_outputs);
			vector<float> feedbacks(num_feedbacks);

			unsigned int batch = 0;
			unsigned int item = 0;
			unsigned int tick = 0;
			float batch_tot = 0.0f;
			float prev_batch_tot = 0.0f;

			float grad_mag = 0.0f;
			float grad_magsq = 0.0f;

			float random_offset;

			while(!CheckAbort())
			{
				if(tick == 0)
					random_offset = (float)item;//Random3D::Rand(float(M_PI) * 2.0f);

				Vec2 test_point = GenerateTestPoint(random_offset, tick, item_ticks);
				inputs[0] = 1.0f;
				inputs[1] = test_point.x;
				//inputs[2] = tick == 0 ? 1.0f : 0.0f;
				correct_outputs[0] = test_point.y;
				if(tick == 0)
					feedbacks[0] = correct_outputs[0];
				memcpy(inputs.data() + num_strict_inputs, feedbacks.data(), num_feedbacks * sizeof(float));

				saved_inputs.push_back(inputs);
				saved_targets.push_back(correct_outputs);

				nn->Evaluate(inputs, outputs, scratch);
				memcpy(feedbacks.data(), outputs.data() + num_strict_outputs, num_feedbacks * sizeof(float));

				for(unsigned int i = 0; i < num_strict_outputs; ++i)
				{
					float err = correct_outputs[i] - outputs[i];
					batch_tot += err * err;
				}

				//vector<float> junk;
				//float errorsq = nn->AddGradient(num_strict_outputs, inputs, outputs, junk, scratch, correct_outputs, *gradient);
				//batch_tot += errorsq;

				float nntot, nnsqtot;
				nn->GetCoeffTotals(nntot, nnsqtot);

				/*ThreadsafeSetText( 0, ((stringstream&)(stringstream() << "batch " << batch)).str());
				ThreadsafeSetText( 1, ((stringstream&)(stringstream() << "item  " << item)).str());
				ThreadsafeSetText( 2, ((stringstream&)(stringstream() << "tick  " << tick)).str());
				ThreadsafeSetText( 3, ((stringstream&)(stringstream() << "batch average  = " << batch_tot / (item * item_ticks + tick + 1))).str());
				ThreadsafeSetText( 4, ((stringstream&)(stringstream() << "prev batch avg = " << prev_batch_tot / (item_ticks * batch_size))).str());
				ThreadsafeSetText( 5, ((stringstream&)(stringstream() << "expected = " << correct_outputs[0])).str());
				ThreadsafeSetText( 6, ((stringstream&)(stringstream() << "actual   = " << outputs[0])).str());
				ThreadsafeSetText( 8, ((stringstream&)(stringstream() << "nn tot   = " << nntot)).str());
				ThreadsafeSetText( 9, ((stringstream&)(stringstream() << "nn^2 tot = " << nnsqtot)).str());

				ThreadsafeSetText(11, ((stringstream&)(stringstream() << "grad tot   = " << grad_mag)).str());
				ThreadsafeSetText(12, ((stringstream&)(stringstream() << "grad^2 tot = " << grad_magsq)).str());*/

				time_spent += my_timer.GetAndRestart();

				++tick;

				if(tick == item_ticks)
				{
					ThreadsafeSetText( 0, ((stringstream&)(stringstream() << "batch " << batch)).str());
					ThreadsafeSetText( 1, ((stringstream&)(stringstream() << "item  " << item)).str());
					ThreadsafeSetText( 2, ((stringstream&)(stringstream() << "tick  " << tick)).str());
					ThreadsafeSetText( 3, ((stringstream&)(stringstream() << "batch average  = " << batch_tot / (item * item_ticks + tick + 1))).str());
					ThreadsafeSetText( 4, ((stringstream&)(stringstream() << "prev batch avg = " << prev_batch_tot / (item_ticks * batch_size))).str());
					ThreadsafeSetText( 5, ((stringstream&)(stringstream() << "expected = " << correct_outputs[0])).str());
					ThreadsafeSetText( 6, ((stringstream&)(stringstream() << "actual   = " << outputs[0])).str());
					ThreadsafeSetText( 8, ((stringstream&)(stringstream() << "nn tot   = " << nntot)).str());
					ThreadsafeSetText( 9, ((stringstream&)(stringstream() << "nn^2 tot = " << nnsqtot)).str());

					ThreadsafeSetText(11, ((stringstream&)(stringstream() << "grad tot   = " << grad_mag)).str());
					ThreadsafeSetText(12, ((stringstream&)(stringstream() << "grad^2 tot = " << grad_magsq)).str());

					vector<float> derrdin(num_strict_inputs + num_feedbacks);
					for(unsigned int i = item_ticks - 1; i < item_ticks; --i)		// deliberately inverted for-loop
					{
						vector<float> derrdout(num_strict_outputs + num_feedbacks);
						memcpy(derrdout.data(), saved_targets[i].data(), num_strict_outputs * sizeof(float));
						memcpy(derrdout.data() + num_strict_outputs, derrdin.data() + num_strict_inputs, num_feedbacks * sizeof(float));
						float grstep = nn->AddGradient(num_strict_outputs, saved_inputs[i], outputs, derrdin, scratch, derrdout, *gradient);
						//Debug(((stringstream&)(stringstream() << "i = " << i << "; grstep = " << grstep << endl)).str());
					}

					tick = 0;
					++item;

					if(item == batch_size)
					{
						item = 0;
						++batch;

						prev_batch_tot = batch_tot;
						batch_tot = 0.0f;

						gradient->GetCoeffTotals(grad_mag, grad_magsq);

						if(grad_mag == 0)
							nn->RandomizeOne(learning_rate);
						else
						{
							nn->Train(*gradient, learning_rate);// / grad_mag);
							gradient->SetZero();
						}
					}

					saved_inputs.clear();
					saved_targets.clear();
					feedbacks.clear();
					feedbacks.resize(num_feedbacks);

					Sleep(0);
				}

				//Sleep(0);
			}

			delete nn;
			delete gradient;

			my_timer.Stop();

			SetAborted();
		}

		Vec2 GenerateTestPoint(float offset, unsigned int i, unsigned int of) const
		{
			float theta = float(i % of) * float(M_PI) * 2.0f / of;
			theta *= 0.1f;
			theta += offset;

			float r = 0.5f;

			return Vec2(r * cosf(theta), r * sinf(theta));
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
			unique_lock<std::mutex> lock(imp->mutex);
			MenuScreen::Draw(width, height);
		}
		else
			MenuScreen::Draw(width, height);
	}

	ProgramScreen* ExperimentalScreen::Update(const TimingInfo& time) { Sleep(1); return MenuScreen::Update(time); }
}

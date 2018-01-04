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
			static const unsigned int num_inputs = 3;
			static const unsigned int num_outputs = 1;
			static const unsigned int num_hidden_layers = 5;
			static const unsigned int hidden_layer_size = 5;

			static const unsigned int batch_size = 100;

			static const float initial_randomness = 1.0f;
			static const float learning_rate = 0.05f / batch_size;

			srand((unsigned int)time(NULL));

			my_timer.Start();

			vector<unsigned int> layer_sizes;
			layer_sizes.push_back(num_inputs);
			for(unsigned int i = 0; i < num_hidden_layers; ++i)
				layer_sizes.push_back(hidden_layer_size);
			layer_sizes.push_back(num_outputs);

			MultiLayerBrain* nn = new MultiLayerBrain(layer_sizes);
			MultiLayerBrain* gradient = new MultiLayerBrain(layer_sizes);

			nn->Randomize(initial_randomness);

			vector<float> inputs(num_inputs);
			vector<float> outputs(num_outputs);
			vector<float> scratch;
			vector<float> correct_outputs(num_outputs);

			unsigned int batch = 0;
			unsigned int item = 0;
			float batch_tot = 0.0f;
			float prev_batch_tot = 0.0f;

			float grad_mag = 0.0f;
			float grad_magsq = 0.0f;

			while(!CheckAbort())
			{
				Vec3 test_point = GenerateTestPoint(item, batch_size);
				inputs[0] = test_point.x;
				inputs[1] = test_point.y;
				inputs[2] = 1.0f;
				correct_outputs[0] = test_point.z;

				float errorsq = nn->AddGradient(inputs, outputs, scratch, correct_outputs, *gradient);
				batch_tot += errorsq;

				float nntot, nnsqtot;
				nn->GetCoeffTotals(nntot, nnsqtot);

				ThreadsafeSetText( 0, ((stringstream&)(stringstream() << "batch " << batch)).str());
				ThreadsafeSetText( 1, ((stringstream&)(stringstream() << "item  " << item)).str());
				ThreadsafeSetText( 2, ((stringstream&)(stringstream() << "batch average  = " << batch_tot / (item + 1))).str());
				ThreadsafeSetText( 3, ((stringstream&)(stringstream() << "prev batch avg = " << prev_batch_tot / batch_size)).str());
				ThreadsafeSetText( 4, ((stringstream&)(stringstream() << "expected = " << correct_outputs[0])).str());
				ThreadsafeSetText( 5, ((stringstream&)(stringstream() << "actual   = " << outputs[0])).str());
				ThreadsafeSetText( 7, ((stringstream&)(stringstream() << "nn tot   = " << nntot)).str());
				ThreadsafeSetText( 8, ((stringstream&)(stringstream() << "nn^2 tot = " << nnsqtot)).str());

				ThreadsafeSetText(10, ((stringstream&)(stringstream() << "grad tot   = " << grad_mag)).str());
				ThreadsafeSetText(11, ((stringstream&)(stringstream() << "grad^2 tot = " << grad_magsq)).str());

				time_spent += my_timer.GetAndRestart();

				++item;

				if(item == batch_size)
				{
					gradient->GetCoeffTotals(grad_mag, grad_magsq);

					item = 0;
					++batch;

					prev_batch_tot = batch_tot;
					batch_tot = 0.0f;

					nn->Train(*gradient, learning_rate);
					gradient->SetZero();

					//if(batch % 50 == 0)
					//	nn->RandomizeOne(0.1f);
				}

				Sleep(0);
			}

			delete nn;
			delete gradient;

			my_timer.Stop();

			SetAborted();
		}

		Vec3 GenerateTestPoint(unsigned int i, unsigned int of) const
		{
#if 0
			float theta = Random3D::Rand(float(M_PI) * 2.0f);
			bool big = Random3D::RandInt() % 2 == 0;
#else
			unsigned int half = of / 2;
			float theta = float(i % half) * float(M_PI) * 2.0f / half;
			bool big = i < half;
#endif

			float r = big ? 1.0f : 0.5f;

			float correct_mag = 0.5f;

			return Vec3(r * cosf(theta), r * sinf(theta), big ? correct_mag : -correct_mag);
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

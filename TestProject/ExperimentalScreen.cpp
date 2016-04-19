#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "NeuralNet.h"

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

		struct DBDemoSimulation
		{
			float x, y, angle;
			float vx, vy, spin;

			float tx, ty;

			float ldx, ldy;

			float motors[2];
			float sensors[5];
			float old_sensors[5];

			float score;

			DBDemoSimulation() : x(0), y(0), angle(0), vx(0), vy(0), spin(0)
			{
				motors[0] = motors[1] = 0.0f;

				Randomize();

				ComputeInputs();
				for(unsigned int i = 0; i < 4; ++i)
					old_sensors[i] = sensors[i];
			}

			void Randomize()
			{
				tx = Random3D::Rand(-10, 10);
				ty = Random3D::Rand(-10, 10);
			}

			void Simulate()
			{
				static const float timestep = 1.0f / 60.0f;

				float hx = cosf(angle);
				float hy = sinf(angle);


				//float vdamp = expf(-10.0f * timestep);
				float vdamp = 0.0f;
				vx *= vdamp;
				vy *= vdamp;

				float dv = (motors[0] + motors[1]) * 60.0f * timestep;
				vx += hx * dv;
				vy += hy * dv;

				//spin *= expf(-10.0f * timestep);
				spin = 0.0f;
				spin += (motors[0] - motors[1]) * 60.0f * timestep;

				x += vx * timestep;
				y += vy * timestep;
				angle += spin * timestep;
			}

			void ComputeInputs()
			{
				float hx = cosf(angle);
				float hy = sinf(angle);
				float rx = -hy;
				float ry = hx;

				float dx = tx - x;
				float dy = ty - y;

				ldx = dx * rx + dy * ry;
				ldy = dx * hx + dy * hy;

				float dmagsq = Vec2::MagnitudeSquared(dx, dy); 
				score = expf(-dmagsq * 0.01f) * 0.95f + 0.05f * powf(ldy / sqrtf(dmagsq) * 0.5f + 0.5f, 2.0f);

				float sr[4] = { -0.1f, 0.1f, -0.1f,  0.1f };
				float sh[4] = {  0.1f, 0.1f, -0.1f, -0.1f };

				for(unsigned int i = 0; i < 5; ++i)
					old_sensors[i] = sensors[i];

				sensors[4] = 0.0f;
				for(unsigned int i = 0; i < 4; ++i)
				{
					float sx = sr[i] * rx + sh[i] * hx - dx;
					float sy = sr[i] * ry + sh[i] * hy - dy;

					float dsq = Vec2::MagnitudeSquared(sx, sy);
					sensors[i] = expf(-dsq * 0.05f) * 2.0f - 1.0f;
					sensors[4] += sensors[i];
				}
				sensors[4] /= 4.0f;
				for(unsigned int i = 0; i < 4; ++i)
				{
					sensors[i] -= sensors[4];
					sensors[i] *= 10.0f;
				}
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
			srand((unsigned int)time(NULL));

			my_timer.Start();
			
			static const unsigned int num_inputs    = 14;
			static const unsigned int num_outputs   = 3;
			static const unsigned int num_middles   = 50;
			
			NeuralNet* nn = NeuralNet::New(num_inputs, num_outputs, num_middles);
			nn->Randomize(0.1f);

			DBDemoSimulation sim;

			static const unsigned int mavg_over_n = 60;
			float mavg_win[mavg_over_n];
			memset(mavg_win, 0, mavg_over_n * sizeof(float));
			float mavg_tot = 0.0f;

			list<vector<float>> records;

			float nns = 0;
			unsigned int iteration = 0;
			while(!CheckAbort())
			{
				unsigned int use_iteration = iteration % 360;
				if(iteration % 360 == 0)
				{
					sim.x = sim.y = 0.0f;
					sim.Randomize();
				}

				sim.ComputeInputs();

				float score = sim.score;
				mavg_tot -= mavg_win[iteration % mavg_over_n];
				mavg_tot += mavg_win[iteration % mavg_over_n] = score;

				for(unsigned int i = 0; i < 5; ++i)
				{
					nn->inputs[i]     = tanhf(sim.sensors[i]);
					nn->inputs[i + 5] = tanhf((sim.sensors[i] - sim.old_sensors[i]) * 5.0f);
				}
				nn->inputs[10] = tanhf(sim.motors[0]);
				nn->inputs[11] = tanhf(sim.motors[1]);
				nn->inputs[12] = tanhf((1.0f - sim.score) * 0.25f);
				nn->inputs[13] = tanhf(sim.score);
				

				nn->Evaluate();

				static const float exploration_scale = 0.1f;
				sim.motors[0] = min(1.0f, max(-1.0f, nn->outputs[0] + Random3D::Rand(-exploration_scale, exploration_scale)));
				sim.motors[1] = min(1.0f, max(-1.0f, nn->outputs[1] + Random3D::Rand(-exploration_scale, exploration_scale)));

				vector<float> nrecord(15);
				for(unsigned int i = 0; i < 12; ++i)
					nrecord[i] = nn->inputs[i];
				nrecord[12] = sim.motors[0];
				nrecord[13] = sim.motors[1];
				nrecord[14] = sim.score;
				records.push_back(nrecord);

				sim.Simulate();

				if(records.size() == 5)
				{
					vector<float> rec = *records.begin();
					records.pop_front();

					for(unsigned int i = 0; i < 12; ++i)
						nn->inputs[i] = rec[i];
					nn->inputs[12] = tanhf(sim.score - rec[14]);
					nn->inputs[13] = tanhf(rec[14]);

					nn->correct_outputs[0] = rec[12];
					nn->correct_outputs[1] = rec[13];
					nn->correct_outputs[2] = sim.score * 2.0f - 1.0f;

					nn->MultiTrainBegin();
					nns = nn->MultiTrainNext();
					nn->MultiTrainApply(0.2f);
				}

				ThreadsafeSetText(0, ((stringstream&)(stringstream() << "i = " << use_iteration << "; score = " << score << "; moving average of score over past " << mavg_over_n << " = " << mavg_tot / min(iteration + 1, mavg_over_n))).str());
				ThreadsafeSetText(1, ((stringstream&)(stringstream() << "nn score = " << nns)).str());
				ThreadsafeSetText(2, ((stringstream&)(stringstream() << "ldx = " << sim.ldx)).str());
				ThreadsafeSetText(3, ((stringstream&)(stringstream() << "ldy = " << sim.ldy)).str());
				ThreadsafeSetText(4, ((stringstream&)(stringstream() << "lf = " << sim.sensors[0])).str());
				ThreadsafeSetText(5, ((stringstream&)(stringstream() << "rf = " << sim.sensors[1])).str());
				ThreadsafeSetText(6, ((stringstream&)(stringstream() << "lr = " << sim.sensors[2])).str());
				ThreadsafeSetText(7, ((stringstream&)(stringstream() << "rr = " << sim.sensors[3])).str());
				ThreadsafeSetText(8, ((stringstream&)(stringstream() << "c  = " << sim.sensors[4])).str());
				ThreadsafeSetText(9, ((stringstream&)(stringstream() << "lmotor = " << sim.motors[0])).str());
				ThreadsafeSetText(10, ((stringstream&)(stringstream() << "rmotor = " << sim.motors[1])).str());

				time_spent += my_timer.GetAndRestart();

				++iteration;

				Sleep(16);
			}

			NeuralNet::Delete(nn);
			nn = NULL;

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
			unique_lock<std::mutex> lock(imp->mutex);
			MenuScreen::Draw(width, height);
		}
		else
			MenuScreen::Draw(width, height);
	}

	ProgramScreen* ExperimentalScreen::Update(const TimingInfo& time) { Sleep(1); return MenuScreen::Update(time); }
}

#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "NeuralNet.h"

#include "ScaledIOBrain.h"

#define STARTING_MIDDLE_LAYER_NEURONS   5

#define INITIAL_RANDOMIZATION           0.01f

namespace Test
{
	/*
	 * ExperimentalScreen private implementation struct
	 */
	struct ExperimentalScreen::Imp
	{
		ScaledIOBrain* siob;

		ExperimentalScreen* scr;
		vector<AutoMenuItem*> auto_menu_items;

		AutoMenuItem* output_text[6];

		boost::thread* my_thread;
		boost::mutex   mutex;

		bool abort;
		bool aborted;

		bool splat_requested;

		int desired_size;
		int gran_exp;

		ProfilingTimer my_timer;
		float time_spent;

		struct BackButton : public AutoMenuItem
		{
			BackButton(ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		struct SplatButton : public AutoMenuItem
		{
			SplatButton(ContentMan* content, int row) : AutoMenuItem(content, "Splat!", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { ((ExperimentalScreen*)mse->menu)->imp->SetSplat(true); }
		};

		struct GranButton : public AutoMenuItem
		{
			int gran;
			GranButton(ContentMan* content, int row, int gran) : AutoMenuItem(content, (((stringstream&)(stringstream() << (gran > 0 ? "Granularity + " : "Granularity - ") << (gran > 0 ? gran : -gran))).str()), row, true), gran(gran) { }
			void DoAction(MenuSelectionEvent* mse) { ((ExperimentalScreen*)mse->menu)->imp->SetGranularity(gran); }
		};

		struct ResizeButton : public AutoMenuItem
		{
			int size;
			ResizeButton(ContentMan* content, int row, int size) : AutoMenuItem(content, (((stringstream&)(stringstream() << (size > 0 ? "Size + " : "Size - ") << (size > 0 ? size : -size))).str()), row, true), size(size) { }
			void DoAction(MenuSelectionEvent* mse) { ((ExperimentalScreen*)mse->menu)->imp->Resize(size); }
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

			auto_menu_items.push_back(new SplatButton (content, row++));
			auto_menu_items.push_back(new ResizeButton(content, row++, -5));
			auto_menu_items.push_back(new ResizeButton(content, row++, -1));
			auto_menu_items.push_back(new ResizeButton(content, row++,  1));
			auto_menu_items.push_back(new ResizeButton(content, row++,  5));
			auto_menu_items.push_back(new GranButton  (content, row++, -1));
			auto_menu_items.push_back(new GranButton  (content, row++,  1));
			auto_menu_items.push_back(new BackButton  (content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				scr->AddItem(auto_menu_items[i]);

			// init brain stuff
			LoadLogs();

			time_spent = 0.0f;

			if(state_floats > 0)
			{
				desired_size = STARTING_MIDDLE_LAYER_NEURONS;
				gran_exp = 0;
				splat_requested = false;

				siob = new ScaledIOBrain(NeuralNet::New(state_floats, 21, desired_size));
				siob->nn->Randomize(INITIAL_RANDOMIZATION);

				siob->input_centers  = state_centers;
				siob->input_scales   = state_scales;
				siob->output_centers = out_centers;
				siob->output_scales  = out_scales;

				thread_helper.imp = this;
				my_thread = new boost::thread(thread_helper);
			}
			else
				siob = NULL;
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

			if(siob != NULL)
			{
				SaveNeuralNet();

				delete siob;
				siob = NULL;
			}
		}

		struct TransIndices { unsigned int state1, state2; };

		unsigned int state_floats;
		vector<Vec3> untranslates;
		vector<Mat3> unrotates;
		vector<vector<float>> states;
		vector<vector<float>> outputs;
		vector<TransIndices> indices;
		vector<float> state_centers, state_scales;
		vector<float> out_centers, out_scales;

		void LoadLogs()
		{
			unsigned int original_count = 0;

			ifstream file("Files/Logs/learnme.statelog", ios::in | ios::binary);
			if(!file)
			{
				Debug("Unable to load state transitions logfile!\n");
				state_floats = 0;
			}
			else
			{
				unsigned int bone_floats      = ReadUInt32(file);
				unsigned int num_bones        = ReadUInt32(file);

				unsigned int num_joints       = ReadUInt32(file);

				unsigned int cp_floats        = ReadUInt32(file);
				unsigned int max_cps_per_foot = ReadUInt32(file);
				unsigned int num_cps          = ReadUInt32(file);

				state_floats = bone_floats * num_bones + 6 * num_joints + cp_floats * num_cps;

				unsigned int num_state_entries = ReadUInt32(file);
				for(unsigned int i = 0; i < num_state_entries; ++i)
				{
					untranslates.push_back(ReadVec3(file));
					unrotates.push_back(ReadMat3(file));

					vector<float> data(state_floats);
					for(unsigned int j = 0; j < state_floats; ++j)
						data[j] = ReadSingle(file);
					states.push_back(data);
				}

				// set up indices
				unsigned int num_trans_entries = ReadUInt32(file);
				for(unsigned int i = 0; i < num_trans_entries; ++i)
				{
					TransIndices index;
					index.state1 = ReadUInt32(file);
					index.state2 = ReadUInt32(file);
					indices.push_back(index);
				}

				file.close();
				

				// find output vector to match state2 vectors; search is needed so we can match up untransforms
				outputs.clear();
				for(unsigned int i = 0; i < num_state_entries; ++i)
				{
					vector<float> data(21);

					for(unsigned int j = 0; j < num_trans_entries; ++j)
						if(indices[j].state2 == i)
						{
							StateToOutput(indices[j], data.data());
							break;
						}

					outputs.push_back(data);
				}

				// scale everything to an appropriate range
				state_centers.clear();
				state_centers.reserve(state_floats);
				state_scales.clear();
				state_scales.reserve(state_floats);
				for(unsigned int i = 0; i < num_bones; ++i)
				{
					for(unsigned int j = 0; j < 4; ++j)		// ori
					{
						state_centers.push_back(0);
						state_scales.push_back(0.95f);
					}
					for(unsigned int j = 0; j < 3; ++j)		// pos
					{
						state_centers.push_back(0);
						state_scales.push_back(0.4f);
					}
					for(unsigned int j = 0; j < 3; ++j)		// vel
					{
						state_centers.push_back(0);
						state_scales.push_back(0.005f);
					}
					for(unsigned int j = 0; j < 3; ++j)		// rot
					{
						state_centers.push_back(0);
						state_scales.push_back(0.002f);
					}
				}

				for(unsigned int i = 0; i < num_joints; ++i)
					for(unsigned int j = 0; j < 6; ++j)
					{
						state_centers.push_back(0.5f);
						state_scales.push_back(0.315f);		// a little less than 1 over pi
					}

				for(unsigned int i = 0; i < num_cps; ++i)
				{
					// pos
					state_centers.push_back(0);
					state_scales.push_back(0.25f);
					state_centers.push_back(0);				// or do we want to use -1.0f?
					state_scales.push_back(0.25f);
					state_centers.push_back(0);
					state_scales.push_back(0.25f);

					for(unsigned int j = 0; j < 3; ++j)		// normal
					{
						state_centers.push_back(0);
						state_scales.push_back(0.95f);
					}
				}

				for(unsigned int i = 0; i < num_state_entries; ++i)
					for(unsigned int j = 0; j < state_floats; ++j)
					{
						float& f = states[i][j];
						f = (f - state_centers[j]) * state_scales[j];
						if(abs(f) > 0.95f)
							Debug(((stringstream&)(stringstream() << "state[" << i << "][" << j << "] = " << f << endl)).str());
					}


				out_centers.clear();
				out_scales.clear();
				out_centers.reserve(21);
				out_scales.reserve(21);
				for(unsigned int i = 0; i < 3; ++i)
				{
					// ori
					for(unsigned int j = 0; j < 4; ++j)
					{
						out_centers.push_back(0);
						out_scales.push_back(0.95f);
					}

					// pos
					out_centers.push_back(0);
					out_scales.push_back(0.25f);
					out_centers.push_back(0);
					out_scales.push_back(0.25f);
					out_centers.push_back(0);
					out_scales.push_back(0.25f);
				}

				for(unsigned int i = 0; i < outputs.size(); ++i)
					for(unsigned int j = 0; j < 21; ++j)
					{
						float& f = outputs[i][j];
						f = (f - out_centers[j]) * out_scales[j];
						if(abs(f) > 0.95f)
							Debug(((stringstream&)(stringstream() << "output[" << i << "][" << j << "] = " << f << endl)).str());
					}
				


				// select some subset of the transitions which should be included in the training set
				original_count = indices.size();
				vector<TransIndices> nu_indices;
				for(unsigned int i = 0; i < indices.size(); ++i)
					if(GetIndexTTL(i) <= Random3D::Rand(2.0f, 15.0f) || Random3D::RandInt() % 5 == 0)
						nu_indices.push_back(indices[i]);
				indices = nu_indices;
			}
			
			string str = ((stringstream&)(stringstream() << "dataset contains " << states.size() << " state records, between which there are " << original_count << " transitions, of which " << indices.size() << " have been included in the training set" << endl)).str();
			output_text[0]->SetText(str);
			Debug(str);
		}

		float GetIndexTTL(unsigned int index_index) const
		{
			float ttl = 0.0f;
			float children = 0.0f;
			TransIndices t = indices[index_index];
			for(unsigned int i = 0; i < indices.size(); ++i)
				if(indices[i].state1 == t.state2)
				{
					ttl += 1.0f + GetIndexTTL(i);
					++children;
				}
			return children == 0 ? 0.0f : ttl / children;
		}

		void SetGranularity(int gran) { boost::mutex::scoped_lock lock(mutex); gran_exp += gran; }

		void Resize(int size) { boost::mutex::scoped_lock lock(mutex); desired_size = max(1, desired_size + size); }

		void MaybeRandomizeCoeff(float& c)
		{
			if(Random3D::RandInt() % 5 == 0)
				c += (Random3D::Rand() * 2.0f - 1.0f) * 0.01f * pow(2.0f, gran_exp * 0.5f);
		}

		void StateToOutput(const TransIndices& index, float* twenty_one)
		{
			unsigned int a = index.state1;
			unsigned int b = index.state2;

			memcpy(twenty_one + 7 * 0, states[b].data() + 13 * 1, 7 * sizeof(float));
			memcpy(twenty_one + 7 * 1, states[b].data() + 13 * 2, 7 * sizeof(float));
			memcpy(twenty_one + 7 * 2, states[b].data() + 13 * 0, 7 * sizeof(float));

			Mat3 b_to_a = unrotates[a] * unrotates[b].Transpose();
			Quaternion bta_quat = Quaternion::FromRotationMatrix(b_to_a);

			struct OriPos
			{
				Quaternion ori;
				Vec3 pos;
			};

			OriPos* ops = (OriPos*)twenty_one;
			for(unsigned int i = 0; i < 3; ++i)
			{
				OriPos& op = ops[i];

				op.ori = bta_quat * op.ori;
				op.pos = b_to_a * (op.pos + untranslates[b]) - untranslates[a];
			}
		}

		float GetActualPredictionError(const float* prediction, const TransIndices& indices, bool verbose = false)
		{
			float* correct = outputs[indices.state2].data();

			float scaled_p[21];
			float scaled_c[21];

			for(unsigned int i = 0; i < 21; ++i)
			{
				scaled_p[i] = prediction[i] / siob->output_scales[i] + siob->output_centers[i];
				scaled_c[i] = correct[i]    / siob->output_scales[i] + siob->output_centers[i];
			}

			struct OriPos
			{
				Quaternion ori;
				Vec3 pos;
			};

			OriPos* op_p = (OriPos*)&scaled_p;
			OriPos* op_c = (OriPos*)&scaled_c;

			float tot = 0.0f;
			for(unsigned int i = 0; i < 3; ++i)
			{
				OriPos& op_pi = op_p[i];
				OriPos& op_ci = op_c[i];				// confirmed: quaternion's norm is already 1 in "correct output" values

				float pi_norm = op_pi.ori.Norm();
				if(pi_norm != 0)
					op_pi.ori /= pi_norm;
				else
					op_pi.ori = Quaternion::Identity();
				float ori_err = (op_ci.ori * Quaternion::Reverse(op_pi.ori)).GetRotationAngle();

				static const float foot_mult     = 1.0f;
				static const float pelvis_mult   = 1.0f;

				float mult = i == 2 ? pelvis_mult : foot_mult;
				tot += mult * ori_err * ori_err;
				float pos_errsq = (op_pi.pos - op_ci.pos).ComputeMagnitudeSquared();
				tot += mult * pos_errsq;

				if(verbose)
					Debug(((stringstream&)(stringstream() << "\tbone[" << i << "]: ori norm = " << pi_norm << "; ori error = " << ori_err << "; pos error = " << sqrtf(pos_errsq) << endl)).str());
			}

			return tot;
		}

		void PrepareRecord(const TransIndices& index)
		{
			memcpy(siob->nn->inputs,          states [index.state1].data(), sizeof(float) * siob->nn->num_inputs);
			memcpy(siob->nn->correct_outputs, outputs[index.state2].data(), sizeof(float) * 21);
		}

		void SetSplat(bool v) { boost::mutex::scoped_lock lock(mutex); splat_requested = v; }
		bool GetSplat()       { boost::mutex::scoped_lock lock(mutex); return splat_requested; }

		void SetAbort()	    { boost::mutex::scoped_lock lock(mutex); abort = true; }
		bool CheckAbort()   { boost::mutex::scoped_lock lock(mutex); return abort; }
		void SetAborted()   { boost::mutex::scoped_lock lock(mutex); aborted = true; }
		bool CheckAborted() { boost::mutex::scoped_lock lock(mutex); return aborted; }

		void ThreadsafeSetText(unsigned int index, const string& text) { boost::mutex::scoped_lock lock(mutex); output_text[index]->SetText(text); }

		bool ThreadsafeMaybeApplyResize()
		{
			boost::mutex::scoped_lock lock(mutex);

			if(desired_size != siob->nn->num_middles)
			{
				assert(desired_size > 0);
				unsigned int new_size = (unsigned)desired_size;

				NeuralNet* old = siob->nn;
				siob->nn = old->Resized(new_size);

				unsigned int old_size = old->num_middles;
				NeuralNet::Delete(old);

				if(new_size > old_size)
				{
					unsigned int num_inputs = siob->nn->num_inputs;
					for(unsigned int i = old_size; i < new_size; ++i)
						for(unsigned int j = 0; j < num_inputs; ++j)
							siob->nn->top_matrix[i * num_inputs + j] = Random3D::Rand(-INITIAL_RANDOMIZATION, INITIAL_RANDOMIZATION);
				}

				return true;
			}
			else
				return false;
		}

		struct NNCandidate
		{
			float score, disp;

			vector<float> top, bot;
		};

		void ThreadAction()
		{
			unsigned int num_records = indices.size();
			vector<unsigned int> shuffle;

			my_timer.Start();

			unsigned int iteration = 0;
			NNCandidate best;

			float NNCandidate::* optimize_me = &NNCandidate::score;			// set this to either ::score or ::disp

			while(!CheckAbort())
			{
				bool splat = GetSplat();
				if(ThreadsafeMaybeApplyResize() || iteration == 0 || splat)
				{
					if(splat)
					{
						static const float splat_size = 0.5f;

						SetSplat(false);

						for(unsigned int i = 0; i < 3; ++i)
						{
							siob->nn->top_matrix[Random3D::RandInt(siob->nn->top_matrix_size)] += Random3D::Rand(-splat_size, splat_size);
							siob->nn->bot_matrix[Random3D::RandInt(siob->nn->bot_matrix_size)] += Random3D::Rand(-splat_size, splat_size);
						}
					}

					GetOverallNNScore(num_records, best.score, best.disp);
					best.top.resize(siob->nn->top_matrix_size);
					best.bot.resize(siob->nn->bot_matrix_size);
					memcpy(best.top.data(), siob->nn->top_matrix, sizeof(float) * siob->nn->top_matrix_size);
					memcpy(best.bot.data(), siob->nn->bot_matrix, sizeof(float) * siob->nn->bot_matrix_size);
				}

				ThreadsafeSetText(4, ((stringstream&)(stringstream() << "middle layer size = " << siob->nn->num_middles)).str());
				ThreadsafeSetText(5, ((stringstream&)(stringstream() << "granularity exp   = " << gran_exp)).str());

				ThreadsafeSetText(1, ((stringstream&)(stringstream() << "iteration = " << iteration)).str());

				ThreadsafeSetText(2, ((stringstream&)(stringstream() << "base = " << best.disp << " (" << best.score << ")")).str());

				for(unsigned int j = 0; j < 10 && !CheckAbort(); ++j)
				{
					ThreadsafeSetText(1, ((stringstream&)(stringstream() << "iteration = " << iteration << "; random # " << j)).str());
					ThreadsafeSetText(3, ((stringstream&)(stringstream() << "best = " << best.disp << " (" << best.score << ")")).str());

					memcpy(siob->nn->top_matrix, best.top.data(), sizeof(float) * siob->nn->top_matrix_size);
					memcpy(siob->nn->bot_matrix, best.bot.data(), sizeof(float) * siob->nn->bot_matrix_size);

					NNCandidate nnc;

					{	// curly braces for scope; synchronizing scale_exp
						boost::mutex::scoped_lock lock(mutex);
						for(unsigned int i = 0; i < siob->nn->top_matrix_size; ++i)
							MaybeRandomizeCoeff(siob->nn->top_matrix[i]);
						for(unsigned int i = 0; i < siob->nn->bot_matrix_size; ++i)
							MaybeRandomizeCoeff(siob->nn->bot_matrix[i]);
					}

					nnc.top.resize(siob->nn->top_matrix_size);
					nnc.bot.resize(siob->nn->bot_matrix_size);
					memcpy(nnc.top.data(), siob->nn->top_matrix, sizeof(float) * siob->nn->top_matrix_size);
					memcpy(nnc.bot.data(), siob->nn->bot_matrix, sizeof(float) * siob->nn->bot_matrix_size);

					GetOverallNNScore(num_records, nnc.score, nnc.disp);

					if(nnc.*optimize_me < best.*optimize_me)
						best = nnc;
				}

				memcpy(siob->nn->top_matrix, best.top.data(), sizeof(float) * siob->nn->top_matrix_size);
				memcpy(siob->nn->bot_matrix, best.bot.data(), sizeof(float) * siob->nn->bot_matrix_size);

				ThreadsafeSetText(3, ((stringstream&)(stringstream() << "best = " << best.disp << " (" << best.score << ")")).str());

				time_spent += my_timer.GetAndRestart();
				Debug(((stringstream&)(stringstream() << time_spent << "\t" << best.*optimize_me << endl)).str());

				++iteration;
			}

			my_timer.Stop();

			SetAborted();
		}

		void GetOverallNNScore(unsigned int num_records, float& nn_scale, float& actual)
		{
			nn_scale = actual = 0.0f;
			for(unsigned int i = 0; i < num_records; ++i)
			{
				const TransIndices& index = indices[i];
				PrepareRecord(index);
				
				nn_scale += siob->nn->EvaluateAndScore();
				actual   += GetActualPredictionError(siob->nn->outputs, index);
			}

			nn_scale = sqrtf(nn_scale / num_records);
			actual   = sqrtf(actual   / num_records);
		}

		struct ThreadHelper
		{
			Imp* imp;
			void operator()() { imp->ThreadAction(); }
		} thread_helper;
			

		void SaveNeuralNet() const
		{
			time_t raw_time;
			time(&raw_time);
			tm now = *localtime(&raw_time);

			string filename = ((stringstream&)(stringstream() << "Files/Brains/neuralnet-" << now.tm_year + 1900 << "-" << now.tm_mon + 1 << "-" << now.tm_mday << "-" << now.tm_hour << "-" << now.tm_min << "-" << now.tm_sec << ".brain")).str();

			ofstream file(filename, ios::out | ios::binary);
			if(!file)
				Debug("Failed to save brain!\n");
			else
			{
				siob->Write(file);
				file.close();
			}
		}
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

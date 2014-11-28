#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "NeuralNet.h"

#include "ScaledIOBrain.h"

#define NUM_MIDDLE_LAYER_NEURONS   25

#define BASE_LEARNING_RATE         1.0f
#define INITIAL_RANDOMIZATION      0.0001f
#define SHAKE_AMOUNT               0.025f;

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

		AutoMenuItem* output_text[7];

		boost::thread* my_thread;
		boost::mutex   mutex;

		bool abort;
		bool aborted;

		struct BackButton : public AutoMenuItem
		{
			BackButton(ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		struct SpeedButton : public AutoMenuItem
		{
			int increment;
			SpeedButton(ContentMan* content, const string text, int increment, int row) : AutoMenuItem(content, text, row, true), increment(increment) { }
			void DoAction(MenuSelectionEvent* mse) { ((ExperimentalScreen*)mse->menu)->imp->ChangeSpeed(increment); }
		};

		struct ShakeButton : public AutoMenuItem
		{
			ShakeButton(ContentMan* content, int row) : AutoMenuItem(content, "Shake", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { ((ExperimentalScreen*)mse->menu)->imp->Shake(); }
		};

		Imp(ExperimentalScreen* scr) : scr(scr), auto_menu_items(), my_thread(NULL), mutex(), abort(false), aborted(false)
		{
			// init screen stuff
			ContentMan* content = scr->content;

			unsigned int row = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Experimental Screen", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", row++, false));

			for(unsigned int i = 0; i < 7; ++i)
				auto_menu_items.push_back(output_text[i] = new AutoMenuItem(content, "", row++, false));

			auto_menu_items.push_back(new SpeedButton(content, "Slower", -1, row++));
			auto_menu_items.push_back(new SpeedButton(content, "Faster",  1, row++));

			auto_menu_items.push_back(new ShakeButton(content, row++));
			auto_menu_items.push_back(new BackButton(content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				scr->AddItem(auto_menu_items[i]);

			// init brain stuff
			LoadLogs();

			step = 0;

			speed_exp = 0;

			if(state_floats > 0)
			{
				siob = new ScaledIOBrain(NeuralNet::New(state_floats, 21, NUM_MIDDLE_LAYER_NEURONS));
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

		unsigned int step;

		unsigned int state_floats;
		vector<Vec3> untranslates;
		vector<Mat3> unrotates;
		vector<vector<float>> states;
		vector<vector<float>> outputs;
		vector<TransIndices> indices;
		vector<float> state_centers, state_scales;
		vector<float> out_centers, out_scales;

		int speed_exp;

		static void SetTableScaling(vector<vector<float>>& data, vector<float>& centers, vector<float>& scales, unsigned int entry_floats)
		{
			unsigned int num_data = data.size();
			vector<float> *data_begin = data.data(), *data_end = data_begin + num_data;

			centers.resize(entry_floats);
			for(unsigned int i = 0; i < entry_floats; ++i)
			{
				float tot = 0.0f;
				for(vector<float>* iter = data_begin; iter != data_end; ++iter)
				{
					if(float& datum = (*iter)[i])
					{
						float inv_datum = 1.0f / datum;
						float product = datum * inv_datum;
						if(fabs(product - 1.0f) > 0.001f || (!(datum >= 0.0f) && !(datum < 0.0f)))
						{
							Debug(((stringstream&)(stringstream() << "bad datum = " << datum << "; product = " << product << endl)).str());
							datum = 0.0f;
						}
						tot += datum;
					}
				}

				float avg = centers[i] = tot / float(num_data);
				for(vector<float>* iter = data_begin; iter != data_end; ++iter)
					(*iter)[i] -= avg;
			}

			scales.resize(entry_floats);
			for(unsigned int i = 0; i < entry_floats; ++i)
			{
				float& scale = scales[i];
				
				scale = 0.01f;			// most it will ever scale something up by is 100x
				for(vector<float>* iter = data_begin; iter != data_end; ++iter)
					scale = max(scale, fabs((*iter)[i]));

				scale = scale > 0 ? 0.95f / scale : 1.0f;			// catch NAN, etc.
			}

			for(vector<vector<float>>::iterator iter = data.begin(); iter != data.end(); ++iter)
				for(unsigned int i = 0; i < entry_floats; ++i)
				{
					float& datum = (*iter)[i];

					datum *= scales[i];
					if(!(datum >= -1.0f && datum <= 1.0f))
					{
						Debug(((stringstream&)(stringstream() << "still bad datum = " << datum << endl)).str());
						datum = 0.0f;
					}
				}
		}

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

				unsigned int cp_floats        = ReadUInt32(file);
				unsigned int max_cps_per_foot = ReadUInt32(file);
				unsigned int num_cps          = ReadUInt32(file);

				state_floats = bone_floats * num_bones + cp_floats * num_cps;

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
				SetTableScaling(states,  state_centers, state_scales, state_floats);
				SetTableScaling(outputs, out_centers,   out_scales,   21);
				
				original_count = indices.size();
#if 0
				vector<TransIndices> nu_indices;
				for(unsigned int i = 0; i < indices.size(); ++i)
					if(GetIndexTTL(i) <= 5.0f || Random3D::RandInt() % 5 != 0)
						nu_indices.push_back(indices[i]);
				indices = nu_indices;
#endif
			}
			
			string str = ((stringstream&)(stringstream() << "dataset contains " << states.size() << " state records, between which there are " << original_count << " transitions, of which " << indices.size() << " have been selected" << endl)).str();
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

		void Shake()
		{
			if(siob == NULL)
				return;

			static const float shake_amount = SHAKE_AMOUNT;

			boost::mutex::scoped_lock lock(mutex); 

			for(float *fptr = siob->nn->top_matrix, *fend = fptr + siob->nn->top_matrix_size; fptr != fend; ++fptr)
				*fptr += Random3D::Rand(-shake_amount, shake_amount);
			for(float *fptr = siob->nn->bottom_matrix, *fend = fptr + siob->nn->bottom_matrix_size; fptr != fend; ++fptr)
				*fptr += Random3D::Rand(-shake_amount, shake_amount);
		}

		void ChangeSpeed(int increment) { boost::mutex::scoped_lock lock(mutex); speed_exp += increment; }

		void MaybeRandomizeCoeff(float& c)
		{
			if(Random3D::RandInt() % 50 == 0)
				c += (Random3D::Rand() * 2.0f - 1.0f) * 0.001f;
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
				op_pi.ori /= pi_norm;

				float ori_err = (op_ci.ori * Quaternion::Reverse(op_pi.ori)).GetRotationAngle();

				tot += ori_err * ori_err;
				float pos_errsq = (op_pi.pos - op_ci.pos).ComputeMagnitudeSquared();
				tot += pos_errsq;

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

		void SetAbort()	    { boost::mutex::scoped_lock lock(mutex); abort = true; }
		bool CheckAbort()   { boost::mutex::scoped_lock lock(mutex); return abort; }
		void SetAborted()   { boost::mutex::scoped_lock lock(mutex); aborted = true; }
		bool CheckAborted() { boost::mutex::scoped_lock lock(mutex); return aborted; }

		void ThreadsafeSetText(unsigned int index, const string& text) { boost::mutex::scoped_lock lock(mutex); output_text[index]->SetText(text); }

		void ThreadAction()
		{
			unsigned int num_records = indices.size();
			vector<unsigned int> shuffle;

			unsigned int pass = 0;
			unsigned int record = 0;

			while(!CheckAbort())
			{
				if(record == 0)
					siob->nn->MultiTrainBegin();
				
				if(Random3D::RandInt() % 10 == 0)
				{
					PrepareRecord(indices[record]);
					siob->nn->MultiTrainNext();
				}
				
				unsigned int pow = 1;
				for(int i = 0; i < abs(speed_exp); ++i)
					pow *= 2;
				string speed_pow_str = ((stringstream&)(stringstream() << pow)).str();
				string speed_str     = speed_exp >= 0 ? speed_pow_str : ((stringstream&)(stringstream() << "1 / " << speed_pow_str)).str();

				ThreadsafeSetText(1, ((stringstream&)(stringstream() << "pass = " << pass << "; record = " << record)).str());
				ThreadsafeSetText(6, ((stringstream&)(stringstream() << "speed multiplier = " << speed_str)).str());

				++record;
				if(record == num_records)
				{
					record = 0;

					float initial_score = GetOverallNNScore(num_records, false);
					float display_initial = GetOverallNNScore(num_records, true);

					ThreadsafeSetText(2, ((stringstream&)(stringstream() << "base           = " << display_initial<< " (" << initial_score << ")")).str());

					vector<float> base_top   (siob->nn->top_matrix_size   );
					vector<float> base_bottom(siob->nn->bottom_matrix_size);

					memcpy(base_top   .data(), siob->nn->top_matrix,    sizeof(float) * siob->nn->top_matrix_size   );
					memcpy(base_bottom.data(), siob->nn->bottom_matrix, sizeof(float) * siob->nn->bottom_matrix_size);

					vector<float> best_top    = base_top;
					vector<float> best_bottom = base_bottom;

					float slope_sq = 0.0f;
					for(float *iptr = siob->nn->temp_top, *iend = iptr + siob->nn->top_matrix_size; iptr != iend; ++iptr)
						slope_sq += *iptr * *iptr;
					for(float *iptr = siob->nn->temp_bottom, *iend = iptr + siob->nn->bottom_matrix_size; iptr != iend; ++iptr)
						slope_sq += *iptr * *iptr;
					float slope = sqrtf(slope_sq);

					ThreadsafeSetText(4, ((stringstream&)(stringstream() << "nn-scale slope = " << slope)).str());

					float best_score   = initial_score;
					float display_best = display_initial;
					float best_lr      = 0.0f;

					static const int exp_range = 16;
					for(int use_exp = -exp_range; use_exp <= exp_range && !CheckAbort(); ++use_exp)
					{
						ThreadsafeSetText(1, ((stringstream&)(stringstream() << "pass = " << pass << "; use exp = " << use_exp)).str());
						ThreadsafeSetText(3, ((stringstream&)(stringstream() << "best           = " << display_best << " (" << best_score << ")")).str());
						ThreadsafeSetText(5, ((stringstream&)(stringstream() << "learning rate  = " << best_lr)).str());

						memcpy(siob->nn->top_matrix,    base_top   .data(), sizeof(float) * siob->nn->top_matrix_size   );
						memcpy(siob->nn->bottom_matrix, base_bottom.data(), sizeof(float) * siob->nn->bottom_matrix_size);

						float use_learning_rate = BASE_LEARNING_RATE * powf(2.0f, speed_exp + use_exp * 0.0625f) / slope;
						siob->nn->MultiTrainApply(use_learning_rate);

						float score = GetOverallNNScore(num_records, false);
						float display_score = GetOverallNNScore(num_records, true);

						if(score < best_score)
						{
							display_best = display_score;
							best_score   = score;
							best_lr      = use_learning_rate;

							memcpy(best_top   .data(), siob->nn->top_matrix,    sizeof(float) * siob->nn->top_matrix_size   );
							memcpy(best_bottom.data(), siob->nn->bottom_matrix, sizeof(float) * siob->nn->bottom_matrix_size);
						}
					}

					for(unsigned int j = 0; j < 500 && !CheckAbort(); ++j)
					{
						ThreadsafeSetText(1, ((stringstream&)(stringstream() << "pass = " << pass << "; random # " << j)).str());
						ThreadsafeSetText(3, ((stringstream&)(stringstream() << "best           = " << display_best << " (" << best_score << ")")).str());

						memcpy(siob->nn->top_matrix,    best_top   .data(), sizeof(float) * siob->nn->top_matrix_size   );
						memcpy(siob->nn->bottom_matrix, best_bottom.data(), sizeof(float) * siob->nn->bottom_matrix_size);

						for(unsigned int i = 0; i < siob->nn->top_matrix_size; ++i)
							MaybeRandomizeCoeff(siob->nn->top_matrix[i]);
						for(unsigned int i = 0; i < siob->nn->bottom_matrix_size; ++i)
							MaybeRandomizeCoeff(siob->nn->bottom_matrix[i]);

						float score = GetOverallNNScore(num_records, false);
						float display_score = GetOverallNNScore(num_records, true);

						if(score < best_score)
						{
							display_best = display_score;
							best_score   = score;

							memcpy(best_top   .data(), siob->nn->top_matrix,    sizeof(float) * siob->nn->top_matrix_size   );
							memcpy(best_bottom.data(), siob->nn->bottom_matrix, sizeof(float) * siob->nn->bottom_matrix_size);
						}
					}

					memcpy(siob->nn->top_matrix,    best_top   .data(), sizeof(float) * siob->nn->top_matrix_size   );
					memcpy(siob->nn->bottom_matrix, best_bottom.data(), sizeof(float) * siob->nn->bottom_matrix_size);

					ThreadsafeSetText(3, ((stringstream&)(stringstream() << "best           = " << display_best << " (" << best_score << ")")).str());
					ThreadsafeSetText(5, ((stringstream&)(stringstream() << "learning rate  = " << best_lr)).str());

					Debug(((stringstream&)(stringstream() << "pass = " << pass << "; base = " << display_initial << " (" << initial_score << "); best = " << display_best << " (" << best_score << "); nn-scale slope = " << slope << "; learning rate = " << best_lr << endl)).str());

					++pass;
				}
			}

			SetAborted();
		}

		float GetOverallNNScore(unsigned int num_records, bool actual)
		{
			float score_tot = 0.0f;
			for(unsigned int i = 0; i < num_records; ++i)
			{
				const TransIndices& index = indices[i];
				PrepareRecord(index);
				if(actual)
				{
					siob->nn->EvaluateAndScore();
					score_tot += GetActualPredictionError(siob->nn->outputs, index);
				}
				else
					score_tot += siob->nn->EvaluateAndScore();
			}
			return score_tot;
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

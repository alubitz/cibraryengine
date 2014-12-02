#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "NeuralNet.h"

#include "ScaledIOBrain.h"

#define STARTING_MIDDLE_LAYER_NEURONS   25

#define BASE_LEARNING_RATE              0.01f
#define INITIAL_RANDOMIZATION           0.01f
#define SHAKE_AMOUNT                    0.025f

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

		int desired_size;
		int speed_exp;

		ProfilingTimer my_timer;
		float time_spent;

		struct BackButton : public AutoMenuItem
		{
			BackButton(ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		struct SpeedButton : public AutoMenuItem
		{
			int speed;
			SpeedButton(ContentMan* content, int row, int speed) : AutoMenuItem(content, (((stringstream&)(stringstream() << (speed > 0 ? "Speed + " : "Speed - ") << (speed > 0 ? speed : -speed))).str()), row, true), speed(speed) { }
			void DoAction(MenuSelectionEvent* mse) { ((ExperimentalScreen*)mse->menu)->imp->SetSpeed(speed); }
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

			auto_menu_items.push_back(new ResizeButton(content, row++, -5));
			auto_menu_items.push_back(new ResizeButton(content, row++, -1));
			auto_menu_items.push_back(new ResizeButton(content, row++,  1));
			auto_menu_items.push_back(new ResizeButton(content, row++,  5));
			auto_menu_items.push_back(new SpeedButton (content, row++, -1));
			auto_menu_items.push_back(new SpeedButton (content, row++,  1));
			auto_menu_items.push_back(new BackButton  (content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				scr->AddItem(auto_menu_items[i]);

			// init brain stuff
			LoadLogs();

			time_spent = 0.0f;

			if(state_floats > 0)
			{
				desired_size = STARTING_MIDDLE_LAYER_NEURONS;
				speed_exp = 0;

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

				float avg = centers[i] = 0.0f;//tot / float(num_data);
				for(vector<float>* iter = data_begin; iter != data_end; ++iter)
					(*iter)[i] -= avg;
			}

			scales.resize(entry_floats);
			for(unsigned int i = 0; i < entry_floats; ++i)
			{
				float& scale = scales[i];
				
				scale = 0.01f;			// most it will ever scale something up by is 95x
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
					if(GetIndexTTL(i) <= 2.0f || Random3D::RandInt() % 5 != 0)
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

			boost::mutex::scoped_lock lock(mutex); 

			for(float *fptr = siob->nn->top_matrix, *fend = fptr + siob->nn->top_matrix_size; fptr != fend; ++fptr)
				*fptr += Random3D::Rand(-SHAKE_AMOUNT, SHAKE_AMOUNT);
			for(float *fptr = siob->nn->bot_matrix, *fend = fptr + siob->nn->bot_matrix_size; fptr != fend; ++fptr)
				*fptr += Random3D::Rand(-SHAKE_AMOUNT, SHAKE_AMOUNT);
		}

		void SetSpeed(int speed) { boost::mutex::scoped_lock lock(mutex); speed_exp += speed; }

		void Resize(int size) { boost::mutex::scoped_lock lock(mutex); desired_size = max(1, desired_size + size); }

		void MaybeRandomizeCoeff(float& c)
		{
			if(Random3D::RandInt() % 10 == 0)
				c += (Random3D::Rand() * 2.0f - 1.0f) * 0.01f * pow(2.0f, speed_exp * 0.25f);
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

				op_pi.ori.w += 1.0f;

				float pi_norm = op_pi.ori.Norm();
				if(pi_norm != 0)
					op_pi.ori /= pi_norm;
				else
					op_pi.ori = Quaternion::Identity();
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

		bool ThreadsafeMaybeApplyResize()
		{
			boost::mutex::scoped_lock lock(mutex);

			if(desired_size != siob->nn->num_middles)
			{
				assert(desired_size > 0);

				NeuralNet* old = siob->nn;
				siob->nn = old->Resized(desired_size);

				NeuralNet::Delete(old);

				return true;
			}
			else
				return false;
		}

		struct LineSearchPoint
		{
			float lr, score, disp;

			vector<float> top, bot;
		};

		void ThreadAction()
		{
			unsigned int num_records = indices.size();
			vector<unsigned int> shuffle;

			unsigned int pass = 0;
			unsigned int record = 0;

			my_timer.Start();

			while(!CheckAbort())
			{
				if(ThreadsafeMaybeApplyResize())
					record = 0;

				if(record == 0)
				{
					shuffle.clear();
					for(unsigned int i = 0; i < num_records; ++i)
						shuffle.push_back(i);
					for(unsigned int i = 0; i < num_records; ++i)
						swap(shuffle[i], shuffle[Random3D::RandInt(num_records)]);
				}

				ThreadsafeSetText(4, ((stringstream&)(stringstream() << "size = " << siob->nn->num_middles << "; speed exp = " << speed_exp)).str());

				//siob->nn->MultiTrainBegin();
				//PrepareRecord(indices[shuffle[record]]);
				//siob->nn->MultiTrainNext();

				ThreadsafeSetText(1, ((stringstream&)(stringstream() << "pass = " << pass << "; record[" << record << "] = " << shuffle[record])).str());

				//++record;
				//if(record == num_records)
				{
					//record = 0;

					vector<float> base_top(siob->nn->top_matrix_size);
					vector<float> base_bot(siob->nn->bot_matrix_size);

					memcpy(base_top.data(), siob->nn->top_matrix, sizeof(float) * siob->nn->top_matrix_size);
					memcpy(base_bot.data(), siob->nn->bot_matrix, sizeof(float) * siob->nn->bot_matrix_size);

					LineSearchPoint initial;
					GetOverallNNScore(num_records, initial.score, initial.disp);
					initial.lr  = 0.0f;
					initial.top = base_top;
					initial.bot = base_bot;

					ThreadsafeSetText(2, ((stringstream&)(stringstream() << "base           = " << initial.disp << " (" << initial.score << ")")).str());

					float slope_sq = 0.0f;
					for(float *iptr = siob->nn->temp_top, *iend = iptr + siob->nn->top_matrix_size; iptr != iend; ++iptr)
						slope_sq += *iptr * *iptr;
					for(float *iptr = siob->nn->temp_bot, *iend = iptr + siob->nn->bot_matrix_size; iptr != iend; ++iptr)
						slope_sq += *iptr * *iptr;
					float slope = sqrtf(slope_sq);

					//ThreadsafeSetText(4, ((stringstream&)(stringstream() << "nn-scale slope = " << slope)).str());

					LineSearchPoint best = initial;
					if(slope != 0.0f)
					{
						//LineSearchPoint lsp_far = TryLearningRate(4.0f / slope, base_top, base_bot, num_records);
						//if(lsp_far.score < best.score)
						//best = lsp_far;
						//LineSearchPoint lsp_mid = TryLearningRate(BASE_LEARNING_RATE / slope, base_top, base_bot, num_records);
						//best = lsp_mid;
					
						/*
						if(lsp_mid.score < best.score)
							best = lsp_mid;
						list<LineSearchPoint> lsps;
						lsps.push_back(initial);
						lsps.push_back(lsp_mid);
						lsps.push_back(lsp_far);

						for(int i = 0; i < 10 && !CheckAbort(); ++i)
						{
							ThreadsafeSetText(1, ((stringstream&)(stringstream() << "pass = " << pass << "; record[" << record << "] = " << shuffle[record] << "; search i = " << i)).str());
							ThreadsafeSetText(3, ((stringstream&)(stringstream() << "best           = " << best.disp << " (" << best.score << ")")).str());
							ThreadsafeSetText(5, ((stringstream&)(stringstream() << "learning rate  = " << best.lr << "; mag = " << best.lr * slope)).str());

							unsigned int j = 0;
							for(list<LineSearchPoint>::iterator iter = lsps.begin(); iter != lsps.end(); ++iter, ++j)
							{
								if(iter->lr == best.lr)
								{
									float best_lr = best.lr;
									if(j != 0)
									{
										--iter;
										LineSearchPoint lsp = TryLearningRate((iter->lr + best_lr) * 0.5f, base_top, base_bot, num_records);
										if(lsp.score < best.score)
											best = lsp;
										++iter;
										lsps.insert(iter, lsp);
										++j;
									}
									if(j + 1 != lsps.size())
									{
										++iter;
										LineSearchPoint lsp = TryLearningRate((iter->lr + best_lr) * 0.5f, base_top, base_bot, num_records);
										if(lsp.score < best.score)
											best = lsp;
										lsps.insert(iter, lsp);
										--iter;
									}
								
									break;
								}
							}
						}
						*/
					}

					for(unsigned int j = 0; j < 10 && !CheckAbort(); ++j)
					{
						ThreadsafeSetText(1, ((stringstream&)(stringstream() << "pass = " << pass << "; record[" << record << "] = " << shuffle[record] << "; random # " << j)).str());
						ThreadsafeSetText(3, ((stringstream&)(stringstream() << "best           = " << best.disp << " (" << best.score << ")")).str());

						memcpy(siob->nn->top_matrix, best.top.data(), sizeof(float) * siob->nn->top_matrix_size);
						memcpy(siob->nn->bot_matrix, best.bot.data(), sizeof(float) * siob->nn->bot_matrix_size);

						LineSearchPoint lsp;
						lsp.lr = best.lr;

						{	// curly braces for scope; synchronizing scale_exp
							boost::mutex::scoped_lock lock(mutex);
							for(unsigned int i = 0; i < siob->nn->top_matrix_size; ++i)
								MaybeRandomizeCoeff(siob->nn->top_matrix[i]);
							for(unsigned int i = 0; i < siob->nn->bot_matrix_size; ++i)
								MaybeRandomizeCoeff(siob->nn->bot_matrix[i]);
						}

						lsp.top.resize(siob->nn->top_matrix_size);
						lsp.bot.resize(siob->nn->bot_matrix_size);
						memcpy(lsp.top.data(), siob->nn->top_matrix, sizeof(float) * siob->nn->top_matrix_size);
						memcpy(lsp.bot.data(), siob->nn->bot_matrix, sizeof(float) * siob->nn->bot_matrix_size);

						GetOverallNNScore(num_records, lsp.score, lsp.disp);

						if(lsp.disp < best.disp)
							best = lsp;
					}

					memcpy(siob->nn->top_matrix, best.top.data(), sizeof(float) * siob->nn->top_matrix_size);
					memcpy(siob->nn->bot_matrix, best.bot.data(), sizeof(float) * siob->nn->bot_matrix_size);

					ThreadsafeSetText(3, ((stringstream&)(stringstream() << "best           = " << best.disp << " (" << best.score << ")")).str());
					ThreadsafeSetText(5, ((stringstream&)(stringstream() << "learning rate  = " << best.lr << "; mag = " << best.lr * slope)).str());

					time_spent += my_timer.GetAndRestart();
					//Debug(((stringstream&)(stringstream() << "pass = " << pass << "; record[" << record << "] = " << shuffle[record] << "; base = " << initial.disp << " (" << initial.score << "); best = " << best.disp << " (" << best.score << "); nn-scale slope = " << slope << "; learning rate = " << best.lr << "; mag = " << best.lr * slope << endl)).str());
					Debug(((stringstream&)(stringstream() << time_spent << " " << initial.disp << endl)).str());

				//	++pass;
				}

				++record;
				if(record == num_records)
				{
					++pass;
					record = 0;
				}
			}

			my_timer.Stop();

			SetAborted();
		}

		LineSearchPoint TryLearningRate(	float use_learning_rate,
											const vector<float>& base_top,
											const vector<float>& base_bot,
											unsigned int num_records         )
		{
			memcpy(siob->nn->top_matrix, base_top.data(), sizeof(float) * siob->nn->top_matrix_size);
			memcpy(siob->nn->bot_matrix, base_bot.data(), sizeof(float) * siob->nn->bot_matrix_size);

			siob->nn->MultiTrainApply(use_learning_rate);

			LineSearchPoint result;
			result.lr = use_learning_rate;
			GetOverallNNScore(num_records, result.score, result.disp);

			result.top.resize(siob->nn->top_matrix_size);
			result.bot.resize(siob->nn->bot_matrix_size);
			memcpy(result.top.data(), siob->nn->top_matrix, sizeof(float) * siob->nn->top_matrix_size);
			memcpy(result.bot.data(), siob->nn->bot_matrix, sizeof(float) * siob->nn->bot_matrix_size);

			return result;
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

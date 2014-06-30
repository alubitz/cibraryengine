#include "StdAfx.h"
#include "ExperimentalScreen.h"

namespace Test
{
	struct Algo
	{
		struct Op
		{
			unsigned char type;

			struct Operand
			{
				unsigned char type;
				union
				{
					unsigned int input_index;
					unsigned int scratch_index;
					float        constant;
				};

				void Mutate(unsigned int index, unsigned int n_inputs)
				{
					if(index != 0 && Random3D::RandInt() % 5 == 0)			// can't reference a scratch index on the first op 
					{
						type = 1;
						scratch_index = Random3D::RandInt(index);
					}
					else
					{
						type = (Random3D::RandInt() % 2) * 2;
						switch(type)
						{
							case 0: { input_index = Random3D::RandInt(n_inputs);                                                  break; }
							case 2: { constant = Random3D::Rand(-2, 2) * Random3D::Rand(0.1f, 2.0f) * Random3D::Rand(0.5f, 1.5f); break; }
						}
					}
				}

				// only call this if you've already checked that the operand type is a scratch index
				void DecrementIndex(unsigned int removed_index, unsigned int index, unsigned int n_inputs)
				{
					if(scratch_index > removed_index)
						--scratch_index;
					else if(scratch_index == removed_index)
						Mutate(index, n_inputs);
				}

				// only call this if you've already checked that the operand type is a scratch index
				void IncrementIndex(unsigned int added_index)
				{
					if(scratch_index >= added_index)
						++scratch_index;
				}

			} a, b;

			void Mutate(unsigned int index, unsigned int n_inputs)
			{
				unsigned int mutation = Random3D::RandInt(1, 7);

				if(mutation & 0x1) { type = Random3D::RandInt() % 10 == 0 ? 3 : Random3D::RandInt(3); }			// don't try division as often as other ops
				if(mutation & 0x2) { a.Mutate(index, n_inputs); }
				if(mutation & 0x4) { b.Mutate(index, n_inputs); }
			}
		};
		vector<Op> ops;
		unsigned int n_inputs, n_outputs;

		unsigned int count;
		float score;
		float fitness;

		Algo() : ops(), n_inputs(0), n_outputs(0) { }
		Algo(unsigned int n_ops, unsigned int n_inputs, unsigned int n_outputs) : ops(n_inputs), n_inputs(n_inputs), n_outputs(n_outputs), count(0), score(0.0f), fitness(1.0f) { }

		void Evaluate(const float* inputs, float* outputs, vector<float>& scratch)
		{
			unsigned int n_ops = ops.size();

			scratch.resize(n_ops);

			unsigned int first_output = n_ops >= n_outputs ? n_ops - n_outputs : n_ops;

			for(unsigned int i = 0; i < n_ops; ++i)
			{
				const Op& op = ops[i];

				float a, b;
				switch(op.a.type)
				{
					case 0: { a = inputs[op.a.input_index];    break; }
					case 1: { a = scratch[op.a.scratch_index]; break; }
					case 2: { a = op.a.constant;               break; }
				}
				switch(op.b.type)
				{
					case 0: { b = inputs[op.b.input_index];    break; }
					case 1: { b = scratch[op.b.scratch_index]; break; }
					case 2: { b = op.b.constant;               break; }
				}

				switch(op.type)
				{
					case 0: { scratch[i] = a + b; break; }
					case 1: { scratch[i] = a - b; break; }
					case 2: { scratch[i] = a * b; break; }
					case 3: { scratch[i] = a / b; break; }
				}

				if(i >= first_output)
					outputs[i - first_output] = scratch[i];
			}
		}

		void DoSingleMutation()
		{
			if(Random3D::RandInt() % 5 == 0)
			{
				// remove an op at a random index
				unsigned int remove = Random3D::RandInt(ops.size());
				for(unsigned int i = remove + 1; i < ops.size(); ++i)
				{
					Op& op = ops[i - 1] = ops[i];
					if(op.a.type == 1)
						op.a.DecrementIndex(remove, i, n_inputs);
					if(op.b.type == 1)
						op.b.DecrementIndex(remove, i, n_inputs);
				}

				// insert a new op at another random index
				unsigned int insert = Random3D::RandInt(ops.size());
				for(unsigned int i = ops.size() - 1; i > insert; --i)
				{
					Op& op = ops[i] = ops[i - 1];
					if(op.a.type == 1)
						op.a.IncrementIndex(insert);
					if(op.b.type == 1)
						op.b.IncrementIndex(insert);
				}
				ops[insert].Mutate(insert, n_inputs);
			}
			else
			{
				// randomize one op
				unsigned int index = Random3D::RandInt(ops.size());
				ops[index].Mutate(index, n_inputs);
			}
		}
	};


	/*
	 * ExperimentalScreen private implementation struct
	 */
	struct ExperimentalScreen::Imp
	{
		vector<Algo*> algos;
		vector<float> scratch;

		ExperimentalScreen* scr;
		vector<AutoMenuItem*> auto_menu_items;

		AutoMenuItem *output1, *output2;

		struct BackButton : public AutoMenuItem
		{
			BackButton (ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		Imp(ExperimentalScreen* scr) : scr(scr), auto_menu_items(), output1(NULL), output2(NULL)
		{
			ContentMan* content = scr->content;

			unsigned int row = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Experimental Screen", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", row++, false));

			auto_menu_items.push_back(output1 = new AutoMenuItem(content, "", row++, false));
			auto_menu_items.push_back(output2 = new AutoMenuItem(content, "", row++, false));

			auto_menu_items.push_back(new BackButton(content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				scr->AddItem(auto_menu_items[i]);

			for(unsigned int i = 0; i < 200; ++i)
			{
				Algo* algo = new Algo(50, 6, 3);
				for(unsigned int j = 0; j < 20; ++j)
					algo->DoSingleMutation();
				algos.push_back(algo);
			}
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

			for(unsigned int i = 0; i < algos.size(); ++i)
				delete algos[i];
			algos.clear();
		}

		void Update(const TimingInfo& time)
		{
			// pick an existing algorithm to mutate (with odds of selection proportional to the algorithm's fitness)
			float tot = 0.0f;
			for(unsigned int i = 0; i < algos.size(); ++i)
				tot += algos[i]->fitness;

			unsigned int use_algo = 0;
			float pick = Random3D::Rand(tot);
			for(unsigned int i = 0; i < algos.size(); ++i)
			{
				pick -= algos[i]->fitness;
				if(pick <= 0.0f)
				{
					use_algo = i;
					break;
				}
			}

			// maybe do one or more mutations
			Algo test = Algo(*algos[use_algo]);
			if(Random3D::RandInt() % 2 == 0)
				do { test.DoSingleMutation(); } while(Random3D::RandInt() % 3 != 0);
			test.score = 0.0f;
			test.count = 0;

			// test this algorithm on some data
			for(unsigned int i = 0; i < 5000; ++i)
			{
				float inputs[6] = { Random3D::Rand(-2, 2), Random3D::Rand(-2, 2), Random3D::Rand(-2, 2), Random3D::Rand(-2, 2), Random3D::Rand(-2, 2), Random3D::Rand(-2, 2) };
				Vec3 a = (Vec3&)(inputs[0]);
				Vec3 b = (Vec3&)(inputs[3]);
				Vec3 xprod = Vec3::Cross(a, b);
				float correct_results[3] = { xprod.x, xprod.y, xprod.z };

				float outputs[3];
				test.Evaluate(inputs, outputs, scratch);

				for(unsigned int j = 0; j < 3; ++j)
				{
					float err = correct_results[j] - outputs[j];
					if(err > 0) { } else if(err <= 0) { } else
						err = 100;
					test.score += err * err;
				}

				++test.count;
			}

			float ratio = test.score / test.count;
			test.fitness = 1.0f / (ratio + 0.01f);

			*algos[Random3D::RandInt(algos.size())] = test;

			float avg = tot / algos.size();

			output1->SetText(((stringstream&)(stringstream() << "fitness = " << test.fitness)).str());
			output2->SetText(((stringstream&)(stringstream() << "average = " << avg)).str());
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

		if(imp) { delete imp; imp = NULL; }
	}

	ProgramScreen* ExperimentalScreen::Update(const TimingInfo& time) { imp->Update(time); return MenuScreen::Update(time); }
}

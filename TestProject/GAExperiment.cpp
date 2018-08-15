#include "StdAfx.h"
#include "GAExperiment.h"

#define NUM_ELITES						20

#define CROSSOVERS_PER_PAIR				2

#define MUTATION_COUNT					20
#define MUTATION_SCALE					0.05f

#define TRIALS_PER_SUBTEST				1

#define TARGET_PROGRAM_SIZE				250

#define NUM_CONSTANTS					64
#define NUM_BONE_INPUTS					91
#define NUM_BRAIN_INPUTS				4
#define NUM_SCRATCH						256		// was 64
#define NUM_BONE_MEM					0		// was 1
#define NUM_BRAIN_MEM					0		// was 4
#define NUM_BONE_OUTPUTS				5		// was 3

#define NUM_OPCODES						10

#define FLOAT_CONSTANT_RANGE			2.0f

#define FORCE_FIRST_GEN_MODIFICATION	0


namespace Test
{
	using namespace CibraryEngine;

	static unsigned char RandomOpCode()
	{
		// return Random3D::RandInt(NUM_OPCODES);			// TODO: change it back?

		unsigned int x = Random3D::RandInt(100);
		if(x < 10)
			return 4;
		else if(x < 55)
			return 2;
		else if(x < 80)
			return 0;
		else
			return 1;
	}


	void GPOperand::Randomize(bool brain, unsigned int my_index, const vector<GPOp>& ops)
	{
		if(brain)
		{
			unsigned int r = Random3D::RandInt(100);
			if(r < 5)
				src = 1;
			else if(r < 15)
				src = 0;
			else if(r < 25)
				src = 7;
#if NUM_BRAIN_MEM > 0
			else if(r < 40)
				src = 3;
#endif
			else if(r < 70)
				src = 6;
			else
				src = 2;

			unsigned int maxi;
			switch(src)
			{
				case 0: maxi = 255;              break;
				case 1: maxi = NUM_BRAIN_INPUTS; break;
				case 2: maxi = NUM_SCRATCH;      break;
				case 3: maxi = NUM_BRAIN_MEM;    break;
				case 4: maxi = NUM_SCRATCH;      break;
				case 7: maxi = NUM_CONSTANTS;    break;
			}

			set<unsigned char> scratches;
			if(src == 2 || src == 6)
			{
				bool need_brain = src == 6 ? false : true;
				for(unsigned int i = 0; i < my_index && i < ops.size(); ++i)
					if(ops[i].dst_class == 0 && (ops[i].brain == need_brain))
						scratches.insert(ops[i].dst_index);
			}

			if(scratches.empty())
			{
				if(src == 2 || src == 6)
				{
					src = 1;
					maxi = NUM_BRAIN_INPUTS;
				}

				index = Random3D::RandInt(maxi);
			}
			else
			{
				unsigned int r = Random3D::RandInt(scratches.size());
				for(set<unsigned char>::iterator iter = scratches.begin(); iter != scratches.end(); ++iter, --r)
					if(r == 0)
					{
						index = *iter;
						break;
					}
			}
		}
		else
		{
			if(Random3D::RandInt(5) == 0)
				src = my_index * 2 < ops.size() ? 5 : 4;
			else if(Random3D::RandInt(3) == 0)
			{
				src = Random3D::RandInt(1, 3);
				if(src == 3)
					src = 7;
			}
			else
				src = Random3D::RandInt(8);
#if NUM_BONE_MEM == 0
			if(src == 3)
				src = 0;
#endif

			unsigned int maxi;
			switch(src)
			{
				case 0: maxi = 255;             break;
				case 1: maxi = NUM_BONE_INPUTS; break;
				case 2: maxi = NUM_SCRATCH;     break;
				case 3: maxi = NUM_BONE_MEM;    break;
				case 4:	maxi = NUM_SCRATCH;     break;
				case 5:	maxi = NUM_SCRATCH;     break;
				case 6: maxi = NUM_SCRATCH;		break;
				case 7: maxi = NUM_CONSTANTS;   break;
			}

			set<unsigned char> scratches;
			if(SourceIsScratch())
			{
				bool need_brain = src == 6 ? true : false;
				for(unsigned int i = 0; i < my_index && i < ops.size(); ++i)
					if(ops[i].dst_class == 0 && (ops[i].brain == need_brain))
						scratches.insert(ops[i].dst_index);
			}

			if(scratches.empty())
			{
				if(SourceIsScratch())
				{
					src = 1;
					maxi = NUM_BONE_INPUTS;
				}
				index = Random3D::RandInt(maxi);
			}
			else
			{
				unsigned int r = Random3D::RandInt(scratches.size());
				for(set<unsigned char>::iterator iter = scratches.begin(); iter != scratches.end(); ++iter, --r)
					if(r == 0)
					{
						index = *iter;
						break;
					}
			}
		}
	}

	string GPOperand::SourceToString() const
	{
		switch(src)
		{
			case 0: return "int ";
			case 1: return "input";
			case 2: return "scratch";
			case 3: return "persist";
			case 4: return "parent";
			case 5: return "child";
			case 6: return "brain";
			case 7: return "const";
			default: return "?????";
		}
	}

	void GPOp::Randomize(unsigned int my_index, const vector<GPOp>& ops)
	{
		opcode = RandomOpCode();

		if(Random3D::RandInt(3) == 0 && my_index * 8 < ops.size() * 7)
		{
			brain = true;
#if NUM_BRAIN_MEM == 0
			dst_class = 0;
#else
			dst_class = Random3D::RandInt(5) == 0 ? 1 : 0;
#endif
			dst_index = Random3D::RandInt(dst_class == 0 ? NUM_SCRATCH : NUM_BRAIN_MEM);
			arg1.Randomize(true, my_index, ops);
			if(!GPOp::IsUnary(opcode))
				arg2.Randomize(true, my_index, ops);
		}
		else
		{
			brain = false;

			if(Random3D::RandInt(5) == 0)
				if(my_index * 8 < ops.size() * 7)
					dst_class = Random3D::RandInt(2);
				else
					dst_class = Random3D::RandInt(3);
			else
				dst_class = 0;

#if NUM_BONE_MEM == 0
			if(dst_class == 1)
				dst_class = 0;
#endif

			dst_index = Random3D::RandInt(dst_class == 0 ? NUM_SCRATCH : dst_class == 1 ? NUM_BONE_MEM : NUM_BONE_OUTPUTS);
			arg1.Randomize(false, my_index, ops);
			if(!GPOp::IsUnary(opcode))
				arg2.Randomize(false, my_index, ops);
		}
	}

	string GPOp::ToString() const
	{
		stringstream ss;

		ss << (brain ? "*\t" : "\t");
		ss << (dst_class == 0 ? "scratch" : dst_class == 1 ? "persist" : "outputs");
		ss << '\t' << (int)dst_index << '\t';
		switch(opcode)
		{
			case 0: ss << "add\t";	break;
			case 1: ss << "sub\t";	break;
			case 2: ss << "mul\t";	break;
			case 3: ss << "div\t";	break;
			case 4: ss << "tanh";	break;
			case 5: ss << "avg\t";	break;
			case 6: ss << "comp";	break;
			case 7: ss << "sqrt";	break;
			case 8: ss << "square";	break;
			case 9: ss << "pow\t";	break;
		}
		ss << '\t' << arg1.SourceToString() << '\t' << (int)arg1.index;
		if(!GPOp::IsUnary(opcode))
			ss << '\t' << arg2.SourceToString() << '\t' << (int)arg2.index;
		//ss << endl;

		return ss.str();
	}




	/*
	 * GACandidate methods
	 */
	GACandidate::GACandidate(unsigned int id) : id(id), p1(0), p2(0), ops(), compiled_flag(false), compiled(), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		memset(constants, 0, sizeof(constants));
		memset(mutations, 0, sizeof(mutations));
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& other) : id(id), p1(other.id), p2(other.id), ops(other.ops), compiled_flag(false), compiled(), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		memcpy(constants, other.constants, sizeof(constants));
		memcpy(mutations, other.mutations, sizeof(mutations));		// or should it memset(0)?
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& p1, const GACandidate& p2) : id(id), p1(p1.id), p2(p2.id), ops(), compiled_flag(false), compiled(), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		for(unsigned int i = 0; i < NUM_CONSTANTS; ++i)
			constants[i] = Crossover(p1.constants[i], p2.constants[i]);

		memset(mutations, 0, sizeof(mutations));

#if 0
		const vector<GPOp>* a = &p1.ops;
		const vector<GPOp>* b = &p2.ops;
#else
		const vector<GPOp>* a = &p1.compiled;
		const vector<GPOp>* b = &p2.compiled;
#endif
		if(Random3D::RandInt(2) == 0 && !b->empty())
			swap(a, b);

		unsigned int insert_at = Random3D::RandInt(a->size() + 1);
		for(unsigned int i = 0; i < insert_at; ++i)
			ops.push_back(a->at(i));

		if(!b->empty())
		{
			unsigned int insert_count = Random3D::RandInt(1, b->size());
			unsigned int insert_from = insert_count == b->size() ? 0 : Random3D::RandInt(0, b->size() - insert_count);
			unsigned int insert_to = insert_from + insert_count;

			for(unsigned int i = insert_from; i < insert_to; ++i)
				ops.push_back(b->at(i));
		}

		for(unsigned int i = insert_at; i < a->size(); ++i)
			ops.push_back(a->at(i));

		// if the result is too long, remove elements at random until it's just the right length
		if(ops.size() > TARGET_PROGRAM_SIZE)
		{
			unsigned int remove_n = ops.size() - TARGET_PROGRAM_SIZE;

			unordered_set<unsigned int> remaining;
			vector<unsigned int> remaining_indices;
			for(unsigned int i = 0; i < ops.size(); ++i)
			{
				remaining.insert(i);
				remaining_indices.push_back(i);
			}

			for(unsigned int i = 0; i < remove_n; ++i)
			{
				unsigned int j = Random3D::RandInt(remaining.size());
				remaining.erase(remaining_indices[j]);
				swap(remaining_indices[j], remaining_indices[remaining_indices.size() - 1]);
				remaining_indices.pop_back();
			}

			vector<GPOp> temp;
			for(unsigned int i = 0; i < ops.size(); ++i)
				if(remaining.find(i) != remaining.end())
					temp.push_back(ops[i]);
			ops = temp;
		}

#if 1
		// find broken references and replace them with something randomly
		set<unsigned char> brain_scratch;
		set<unsigned char> bone_scratch;

		for(unsigned int i = 0; i < ops.size(); ++i)
		{
			GPOp& op = ops[i];

			if(op.arg1.SourceIsScratch())
			{
				set<unsigned char>* used_scratch = ((op.arg1.src == 6) == op.brain) ? &bone_scratch : &brain_scratch;
				if(used_scratch->find(op.arg1.index) == used_scratch->end())
				{
					op.arg1.Randomize(op.brain, i, ops);
					++mutations[0];
				}
			}

			if(!GPOp::IsUnary(op.opcode) && op.arg2.SourceIsScratch())
			{
				set<unsigned char>* used_scratch = ((op.arg2.src == 6) == op.brain) ? &bone_scratch : &brain_scratch;
				if(used_scratch->find(op.arg2.index) == used_scratch->end())
				{
					op.arg2.Randomize(op.brain, i, ops);
					++mutations[0];
				}
			}

			if(op.dst_class == 0)
				if(op.brain)
					brain_scratch.insert(op.dst_index);
				else
					bone_scratch.insert(op.dst_index);
		}
#endif

		// if any of the outputs aren't assigned to, make something up for each of them
		vector<unsigned int> last_outputs;
		unsigned int num_ops = ops.size();
		GetLastOutputs(ops, last_outputs);

		for(unsigned int i = 0; i < NUM_BONE_OUTPUTS; ++i)
			if(last_outputs[i] == num_ops)
			{
				GPOp op;
				op.brain = false;
				op.dst_class = 2;
				op.dst_index = i;
				op.opcode = RandomOpCode();
				op.arg1.Randomize(false, ops.size(), ops);
				if(!GPOp::IsUnary(op.opcode))
					op.arg2.Randomize(false, ops.size(), ops);
				ops.push_back(op);
			}
			else
			{
				GPOp op = ops[last_outputs[i]];
				if(op.opcode >= 4 && (op.opcode != 8 || op.arg1.src != 0 || op.arg1.index != 0))
				{
					if(Random3D::RandInt(5) == 0)
					{
						op.opcode = 0;
						op.arg1 = GPOperand(0, 0);
						op.arg2 = GPOperand(0, 0);
						ops[last_outputs[i]] = op;
					}
					else
					{
						unsigned char index = Random3D::RandInt(NUM_SCRATCH);		// if this scratch happens to be assigned to between now and the end of the program, oops
						ops[last_outputs[i]].dst_class = 0;
						ops[last_outputs[i]].dst_index = index;

						GPOp nop;
						nop.brain = false;
						nop.dst_class = 2;
						nop.dst_index = i;
						nop.opcode = Random3D::RandInt(3);
						nop.arg1 = GPOperand(2, index);					
						nop.arg2.Randomize(false, ops.size(), ops);
						if(Random3D::RandInt(2) == 0)
							swap(nop.arg1, nop.arg2);
						ops.push_back(nop);
					}
				}
			}

		Randomize(MUTATION_COUNT, MUTATION_SCALE);

#if FORCE_FIRST_GEN_MODIFICATION
		if(p1.p1 == 0 && p1.p2 == 0 || p2.p1 == 0 && p2.p2 == 0)
		{
			for(unsigned int i = 0; i < NUM_BONE_OUTPUTS; ++i)
			{
				//if(i % 3 == 0)
				//{
				//	GPOp oop;
				//	oop.brain = false;
				//	oop.dst_class = 2;
				//	oop.dst_index = i;
				//	oop.opcode = Random3D::RandInt(NUM_OPCODES);
				//	oop.arg1.Randomize(false, ops.size(), ops);
				//	if(!GPOp::IsUnary(oop.opcode))
				//		oop.arg2.Randomize(false, ops.size(), ops);
				//	ops.push_back(oop);
				//}
			}
		}
#endif
	}

	GACandidate::~GACandidate() { }

	void GACandidate::Randomize(unsigned int count, float scale)
	{
		float minx = -FLOAT_CONSTANT_RANGE;
		float maxx =  FLOAT_CONSTANT_RANGE;
		float rscale = scale * (maxx - minx);

		//Compile();
		//ops = compiled;

		bool ok;
		do
		{
			ok = false;

			unsigned int mutation_type = Random3D::RandInt(11);
			if(ops.size() <= NUM_BONE_OUTPUTS * 2)
			{
				if(Random3D::RandInt(8) != 0)
				{
					mutation_type = Random3D::RandInt(4, 7);
					if(mutation_type == 4)
						mutation_type = 3;
					else if(mutation_type == 7)
						mutation_type = 9;
				}
			}
			else if(Random3D::RandInt(5) == 0)
				if(ops.size() * 2 < TARGET_PROGRAM_SIZE)
					mutation_type = Random3D::RandInt(3, 4);
				else
					mutation_type = Random3D::RandInt(2) == 0 ? 4 : 9;

			switch(mutation_type)
			{
				case 0:		// insert a new op
				{
					if(ops.size() > TARGET_PROGRAM_SIZE && Random3D::RandInt(2) == 0)
						break;

					unsigned int index = ops.empty() ? 0 : Random3D::RandInt(ops.size());
					GPOp nuop;
					nuop.Randomize(index, ops);

					if(ops.empty())
						ops.push_back(nuop);
					else
					{
						ops.push_back(GPOp());
						for(unsigned int i = ops.size() - 1; i > index; --i)
							ops[i] = ops[i - 1];
						ops[index] = nuop;
					}

					++mutations[1];
					ok = true;
					break;
				}

				case 1:		// replace an existing op
				{
					unsigned int index = ops.empty() ? 0 : Random3D::RandInt(ops.size());
					GPOp nuop;
					nuop.Randomize(index, ops);

					if(ops.empty())
						ops.push_back(nuop);
					else
						ops[index] = nuop;

					++mutations[2];
					ok = true;
					break;
				}

				case 2:		// delete an existing op
				{
					//if(ops.size() < TARGET_PROGRAM_SIZE && Random3D::RandInt(2) == 0)
					//	break;
					//
					//if(!ops.empty())
					//{
					//	unsigned int index = Random3D::RandInt(ops.size());
					//	for(unsigned int i = index + 1; i < ops.size(); ++i)
					//		ops[i - 1] = ops[i];
					//	ops.pop_back();
					//
					//	++mutations[3];
					//	ok = true;
					//}
					break;
				}

				case 3:		// replace an operand with some function of that operand
				{
					if(!ops.empty())
					{
						if(ops.size() > TARGET_PROGRAM_SIZE && Random3D::RandInt(2) == 0)
							break;

						unsigned int index = Random3D::RandInt(ops.size());
						GPOp op = ops[index];
						GPOperand* operand;
						if(GPOp::IsUnary(op.opcode) || Random3D::RandInt(2) == 0)
							operand = &op.arg1;
						else
							operand = &op.arg2;

						unsigned int scratch = Random3D::RandInt(NUM_SCRATCH);
						GPOp nuop;
						nuop.opcode = RandomOpCode();
						nuop.brain = op.brain;
						nuop.dst_class = 0;
						nuop.dst_index = scratch;
						nuop.arg1 = *operand;
						nuop.arg2.Randomize(op.brain, index, ops);

						operand->src = 2;
						operand->index = scratch;

						ops.push_back(GPOp());
						for(unsigned int i = ops.size() - 1; i > index; --i)
							ops[i] = ops[i - 1];
						ops[index] = nuop;

						++mutations[4];
						ok = true;
					}
					break;
				}

				case 4:		// swap indices of 2 scratch computations (doesn't affect this genome, but will affect its future generations)
				{
					unsigned int sa = Random3D::RandInt(NUM_SCRATCH);
					unsigned int sb = Random3D::RandInt(NUM_SCRATCH);
					if(sb == sa)
						sb = (sa + 1) % NUM_SCRATCH;

					bool any = false;
					for(unsigned int i = 0; i < ops.size(); ++i)
					{
						GPOp& op = ops[i];
						if(op.dst_class == 0)
						{
							if(op.dst_index == sa)
							{
								op.dst_index = sb;
								any = true;
							}
							else if(op.dst_index == sb)
							{
								op.dst_index = sa;
								any = true;
							}
						}

						if(op.arg1.SourceIsScratch())
						{
							if(op.arg1.index == sa)
							{
								op.arg1.index = sb;
								any = true;
							}
							else if(op.arg1.index == sb)
							{
								op.arg1.index = sa;
								any = true;
							}
						}

						if(!GPOp::IsUnary(op.opcode) && op.arg2.SourceIsScratch())
						{
							if(op.arg2.index == sa)
							{
								op.arg2.index = sb;
								any = true;
							}
							else if(op.arg2.index == sb)
							{
								op.arg2.index = sa;
								any = true;
							}
						}
					}

					if(any)
					{
						++mutations[5];
						ok = true;
					}

					break;
				}

				case 5:		// mutate an operand of an existing op
				{
					if(!ops.empty())
					{
						unsigned int index = Random3D::RandInt(ops.size());
						GPOp& op = ops[index];
						if(Random3D::RandInt(2) == 1 && !GPOp::IsUnary(op.opcode))
							op.arg2.Randomize(op.brain, index, ops);
						else
							op.arg1.Randomize(op.brain, index, ops);
					
						++mutations[6];
						ok = true;
					}
					break;
				}

				case 6:		// change the opcode of an existing op (and/or reorder its operands)
				{
					if(!ops.empty())
					{
						unsigned int index = Random3D::RandInt(ops.size());
						GPOp& op = ops[index];
						unsigned int nucode = RandomOpCode();
						if(nucode == op.opcode)
						{
							if(GPOp::IsUnary(nucode))
								break;

							swap(op.arg1, op.arg2);	
						}
						else
						{
							if(GPOp::IsUnary(op.opcode))
								op.arg2.Randomize(op.brain, index, ops);
							op.opcode = nucode;
						}							

						++mutations[7];
						ok = true;
					}
					break;
				}

				case 7:		// move an op
				{
					if(ops.size() >= 2)
					{
						unsigned int a = Random3D::RandInt(ops.size());	// from
						unsigned int b = Random3D::RandInt(ops.size());	// to
						if(b == a)
							b = (a + 1) % ops.size();

						if(a > b)		// move left
						{
							for(unsigned int i = a; i > b; --i)
								swap(ops[i], ops[i - 1]);
						}
						else			// move right
						{
							for(unsigned int i = a; i < b; ++i)
								swap(ops[i], ops[i + 1]);
						}

						++mutations[8];
						ok = true;
					}
					break;
				}

				case 8:		// swap indices of two memory values (doesn't affect this genome, but will affect its future generations)
				{
#if NUM_BRAIN_MEM > 1 || NUM_BONE_MEM > 1
#if NUM_BRAIN_MEM <= 1
					bool brain = false;
#elif NUM_BONE_MEM <= 1
					bool brain = true;
#else
					bool brain = Random3D::RandInt(2) == 0;
#endif

					unsigned int max_cell = brain ? NUM_BRAIN_MEM : NUM_BONE_MEM;

					unsigned int ma = Random3D::RandInt(max_cell);
					unsigned int mb = Random3D::RandInt(max_cell);
					if(ma == mb)
						mb = (mb + 1) % max_cell;

					bool any = false;

					for(unsigned int i = 0; i < ops.size(); ++i)
					{
						GPOp& op = ops[i];
						if(op.brain == brain)
						{
							if(op.dst_class == 1)
							{
								if(op.dst_index == ma)
								{
									op.dst_index = mb;
									any = true;
								}
								else if(op.dst_index == mb)
								{
									op.dst_index = ma;
									any = true;
								}
							}

							if(op.arg1.src == 3)
							{
								if(op.arg1.index == ma)
								{
									op.arg1.index = mb;
									any = true;
								}
								else if(op.arg1.index == mb)
								{
									op.arg1.index = ma;
									any = true;
								}
							}

							if(!GPOp::IsUnary(op.opcode) && op.arg2.src == 3)
							{
								if(op.arg2.index == ma)
								{
									op.arg2.index = mb;
									any = true;
								}
								else if(op.arg2.index == mb)
								{
									op.arg2.index = ma;
									any = true;
								}
							}
						}
					}

					if(any)
					{
						ok = true;
						++mutations[9];
					}
#endif
					break;
				}

				case 9:					// replace an op (not an operand) with some function of its result
				{
					if(!ops.empty())
					{
						if(ops.size() > TARGET_PROGRAM_SIZE && Random3D::RandInt(2) == 0)
							break;

						unsigned int index = Random3D::RandInt(ops.size());
						ops.push_back(GPOp());
						for(unsigned int i = ops.size() - 1; i > index; --i)
							ops[i] = ops[i - 1];
						// now the op at index also appears at index+1
						ops[index].dst_class = 0;
						ops[index].dst_index = Random3D::RandInt(NUM_SCRATCH);

						ops[index + 1].opcode = RandomOpCode();
						ops[index + 1].arg1.src = 2;
						ops[index + 1].arg1.index = ops[index].dst_index;
						if(!GPOp::IsUnary(ops[index + 1].opcode))
						{
							ops[index + 1].arg2.Randomize(ops[index + 1].brain, index + 1, ops);
							if(Random3D::RandInt(2) == 0)
								swap(ops[index + 1].arg1, ops[index + 1].arg2);
						}

						ok = true;
						++mutations[10];
						
					}
					break;
				}

				case 10:				// mutate the constants table
				{
					set<unsigned char> indices;
					for(unsigned int i = 0; i < ops.size(); ++i)
					{
						const GPOp& op = ops[i];
						if(op.arg1.src == 7)
							indices.insert(op.arg1.index);
						if(!GPOp::IsUnary(op.opcode) && op.arg2.src == 7)
							indices.insert(op.arg2.index);
					}

					if(indices.empty())
						break;

					unsigned int index_index = Random3D::RandInt(indices.size());
					unsigned int index = indices.size();
					for(set<unsigned char>::iterator iter = indices.begin(); iter != indices.end(); ++iter, --index_index)
					{
						if(index_index == 0)
						{
							index = *iter;
							break;
						}
					}

					if(index != indices.size())
					{
						float* ptr = constants + Random3D::RandInt(NUM_CONSTANTS);
						*ptr = min(maxx, max(minx, *ptr + Random3D::Rand(-rscale, rscale)));
					
						ok = true;
						++mutations[11];
					}
					break;
				}
			}

			if(ok)
			{
				//Compile();
				//ops = compiled;
			}

		} while(!ok || Random3D::RandInt() % count != 0);

		//Compile();
		//ops = compiled;
	}

	float GACandidate::Crossover(float a, float b) { return a + Random3D::Rand() * (b - a); }

	void GACandidate::Compile()
	{
		// remove anything that assigns to an output which is later overwritten; also remove any ops that happen after the last assignment to an output		
		vector<unsigned int> last_outputs;
		GetLastOutputs(ops, last_outputs);
		RemoveUnreferencedScratchSteps(ops, compiled, last_outputs);

		// if anything reads from an unassigned variable or out-of-bounds input, replace it with the constant 0
		set<unsigned char> brain_scratch;
		set<unsigned char> bone_scratch;
		for(unsigned int i = 0; i < compiled.size(); ++i)
		{
			GPOp& op = compiled[i];					

			if(op.arg1.SourceIsScratch())
			{
				set<unsigned char>* used_scratch = ((op.arg1.src == 6) == op.brain) ? &bone_scratch : &brain_scratch;
				if(used_scratch->find(op.arg1.index) == used_scratch->end())
				{
					op.arg1.src = 0;
					op.arg1.index = 0;
				}
			}
			else if(op.arg1.src == 1 && op.arg1.index >= (op.brain ? NUM_BRAIN_INPUTS : NUM_BONE_INPUTS))
			{
				op.arg1.src = 0;
				op.arg1.index = 0;
			}
			else if(op.arg1.src == 3 && op.arg1.index >= (op.brain ? NUM_BRAIN_MEM : NUM_BONE_MEM))
			{
				op.arg1.src = 0;
				op.arg1.index = 0;
			}

			if(!GPOp::IsUnary(op.opcode))
			{
				if(op.arg2.SourceIsScratch())
				{
					set<unsigned char>* used_scratch = ((op.arg2.src == 6) == op.brain) ? &bone_scratch : &brain_scratch;
					if(used_scratch->find(op.arg2.index) == used_scratch->end())
					{
						op.arg2.src = 0;
						op.arg2.index = 0;
					}
				}
				else if(op.arg2.src == 1 && op.arg2.index >= (op.brain ? NUM_BRAIN_INPUTS : NUM_BONE_INPUTS))
				{
					op.arg2.src = 0;
					op.arg2.index = 0;
				}
				else if(op.arg2.src == 3 && op.arg2.index >= (op.brain ? NUM_BRAIN_MEM : NUM_BONE_MEM))
				{
					op.arg2.src = 0;
					op.arg2.index = 0;
				}
			}

			if(op.dst_class == 0)
				if(op.brain)
					brain_scratch.insert(op.dst_index);
				else
					bone_scratch.insert(op.dst_index);
		}

		// optimize! remove mutiplications by zero, additions/subtractions by zero, multiplications/divisions by one, etc.
		// note that some optimizations may ignore the fact that an operation can return nan/inf, e.g. 0*nan should be nan but will be optimized away as always 0
		unsigned int counter = 0;
		bool any;
		vector<GPOp> temp;
		do
		{
			//Debug(((stringstream&)(stringstream() << "id = " << id << ", raw size = " << ops.size() << ", compiled size = " << compiled.size() << ", counter = " << counter << endl)).str());

			any = false;
			temp.clear();

			for(unsigned int i = 0; i < compiled.size(); ++i)
			{
				bool push = true;
				bool cant_remove = false;

				GPOp& op = compiled[i];
				if(op.dst_class == 0)
				{
					GPOperand replace_with;

					if(false)
					{
					}
					if(op.opcode == 0 && op.arg1.src == 0 && op.arg1.index == 0)							// 0 + y = y
					{
						push = false;
						replace_with = op.arg2;
					}
					else if((op.opcode == 0 || op.opcode == 1) && op.arg2.src == 0 && op.arg2.index == 0)	// x + 0 = x; x - 0 = x
					{
						push = false;
						replace_with = op.arg1;
					}
					else if(op.opcode == 2 && (op.arg1.src == 0 && op.arg1.index == 0 || op.arg2.src == 0 && op.arg2.index == 0))	// x * 0 = 0; 0 * y = 0
						push = false;
					else if(op.opcode == 3 && op.arg1.src == 0 && op.arg1.index == 0)						// 0 / y = 0
						push = false;
					else if(op.opcode == 3 && op.arg2.src == 0 && op.arg2.index == 0)						// x / 0 = nan		(replaced with 0)
						push = false;
					else if(op.opcode == 2 && op.arg1.src == 0 && op.arg1.index == 1)						// 1 * y = y
					{
						push = false;
						replace_with = op.arg2;
					}
					else if((op.opcode == 2 || op.opcode == 3) && op.arg2.src == 0 && op.arg2.index == 1)	// x * 1 = x; x / 1 = x
					{
						push = false;
						replace_with = op.arg1;
					}
					else if((op.opcode == 4 || op.opcode == 7 || op.opcode == 8) && op.arg1.src == 0 && op.arg1.index == 0)	// tanh(0) = 0; sqrt(0) = 0; square(0) = 0
						push = false;
					else if((op.opcode == 7 || op.opcode == 8) && op.arg1.src == 0 && op.arg1.index == 1)					// sqrt(1) = 1; square(1) = 1
					{
						push = false;
						replace_with = op.arg1;
					}
					else if(op.opcode == 9 && op.arg1.src == 0 && (op.arg1.index == 0 || op.arg1.index == 1))				// 0^y = 0; 1^y = 1
					{
						push = false;
						replace_with = op.arg1;
					}
					else if(op.opcode == 9 && op.arg2.src == 0 && (op.arg2.index == 0 || op.arg2.index == 1))				// x^0 = 1; x^1 = x
					{
						push = false;
						if(op.arg2.index == 0)
							replace_with = GPOperand(0, 1);
						else
							replace_with = op.arg1;
					}
					else if((op.arg1.src == 0 || op.arg1.src == 7) && (GPOp::IsUnary(op.opcode) || (op.arg2.src == 0 || op.arg2.src == 7)))		// function of constant(s) returns a constant
					{
						float a = op.arg1.src == 0 ? float((signed)op.arg1.index) : constants[op.arg1.index];
						float b = GPOp::IsUnary(op.opcode) ? 0.0f : op.arg2.src == 0 ? float((signed)op.arg2.index) : constants[op.arg2.index];
						float temp;
						switch(op.opcode)
						{
							case 0: temp = a + b;				break;
							case 1: temp = a - b;				break;
							case 2: temp = a * b;				break;
							case 3: temp = a / b;				break;
							case 4: temp = tanhf(a);			break;
							case 5: temp = (a + b) * 0.5f;		break;
							case 6: temp = a > b ? 1.0f : 0.0f;	break;
							case 7: temp = sqrtf(a);			break;
							case 8: temp = a * a;				break;
							case 9: temp = powf(a, b);			break;
							default: temp = 0.0f;				break;
						}
						if(!isfinite(temp))
							temp = 0.0f;
						if(floor(temp) == temp && temp >= -128.0f && temp < 127.0f)
						{
							push = false;
							replace_with = GPOperand(0, (unsigned char)((char)temp));
						}
					}
					
					if(!push)
					{
						//Debug(((stringstream&)(stringstream() << "Dropping instruction at index " << i << " (" << op.ToString() << "); replacing references to it with " << replace_with.SourceToString() << " " << (unsigned int)replace_with.index << endl)).str());
						for(unsigned int j = i + 1; j < compiled.size(); ++j)
						{
							GPOp& jop = compiled[j];
							
							if(jop.arg1.SourceIsScratch() && jop.arg1.index == op.dst_index)
								if((jop.arg1.src == 6) != (jop.brain == op.brain))
								{
									if((replace_with.src == 1 || replace_with.src == 3 || replace_with.src == 4 || replace_with.src == 5) && jop.arg1.src != 2)
										cant_remove = true;
									else
									{
										jop.arg1 = replace_with;
										any = true;
									}
								}

							if(!GPOp::IsUnary(jop.opcode) && jop.arg2.SourceIsScratch() && jop.arg2.index == op.dst_index)
								if((jop.arg2.src == 6) != (jop.brain == op.brain))
								{
									if((replace_with.src == 1 || replace_with.src == 3 || replace_with.src == 4 || replace_with.src == 5) && jop.arg2.src != 2)
										cant_remove = true;
									else
									{
										jop.arg2 = replace_with;
										any = true;
									}
								}

							if(jop.dst_class == 0 && jop.dst_index == op.dst_index)
								break;
						}
					}
				}
		
				if(push || cant_remove)
					temp.push_back(op);
				else
					any = true;
			}

			if(any)
				compiled = temp;

			GetLastOutputs(compiled, last_outputs);
			if(RemoveUnreferencedScratchSteps(compiled, compiled, last_outputs))
				any = true;
		
			if(any)
				++counter;
		
		} while(any);

		compiled_flag = true;
	}

	void GACandidate::GetLastOutputs(const vector<GPOp>& ops, vector<unsigned int>& last_outputs) const
	{
		last_outputs.resize(NUM_BONE_OUTPUTS + NUM_BONE_MEM + NUM_BRAIN_MEM);
		for(unsigned int i = 0; i < last_outputs.size(); ++i)
			last_outputs[i] = ops.size();

		unsigned int remaining = last_outputs.size();
		for(unsigned int i = ops.size() - 1; i < ops.size(); --i)	// deliberately inverted for loop
		{
			const GPOp& op = ops[i];
			if(op.dst_class == 1 || op.dst_class == 2)
			{
				unsigned int index =
					op.dst_class == 2 ? op.dst_index < NUM_BONE_OUTPUTS ? op.dst_index : last_outputs.size()
					: op.brain ? op.dst_index < NUM_BRAIN_MEM ? NUM_BONE_OUTPUTS + op.dst_index : last_outputs.size()
					: op.dst_index < NUM_BONE_MEM ? NUM_BONE_OUTPUTS + NUM_BRAIN_MEM + op.dst_index : last_outputs.size();
				if(index < last_outputs.size())
				{
					if(last_outputs[index] == ops.size())
					{
						last_outputs[index] = i;
						--remaining;
						if(remaining == 0)
							break;
					}
				}
			}
		}
	}

	bool GACandidate::RemoveUnreferencedScratchSteps(const vector<GPOp>& ops_in, vector<GPOp>& ops_out, const vector<unsigned int>& last_outputs) const
	{
		vector<bool> dependencies(ops_in.size());
		for(unsigned int i = 0; i < last_outputs.size(); ++i)
			if(last_outputs[i] != ops_in.size())
				dependencies[last_outputs[i]] = true;

		for(unsigned int i = ops_in.size() - 1; i < ops_in.size(); --i)		// deliberately inverted for loop
			if(dependencies[i])
			{
				const GPOp& op = ops_in[i];

				if(op.arg1.SourceIsScratch())
				{
					for(unsigned int j = i - 1; j < i; --j)				// deliberately inverted for loop
					{
						if(ops_in[j].dst_class == 0 && ops_in[j].dst_index == op.arg1.index)
							if((op.arg1.src == 6) != (op.brain == ops_in[j].brain))
							{
								dependencies[j] = true;
								break;
							}
					}
				}

				if(!GPOp::IsUnary(op.opcode) && op.arg2.SourceIsScratch())
				{
					for(unsigned int j = i - 1; j < i; --j)				// deliberately inverted for loop
					{
						if(ops_in[j].dst_class == 0 && ops_in[j].dst_index == op.arg2.index)
							if((op.arg2.src == 6) != (op.brain == ops_in[j].brain))
							{
								dependencies[j] = true;
								break;
							}
					}
				}
			}

		bool skipped_any = false;
		vector<GPOp> temp;
		for(unsigned int i = 0; i < ops_in.size(); ++i)
			if(dependencies[i])
				temp.push_back(ops_in[i]);
			else
				skipped_any = true;
		
		ops_out = temp;
		return skipped_any;
	}

	string GACandidate::GetText() const
	{
		stringstream ss;
		ss << '(' << id << ", p " << p1 << ", " << p2 << ") score = " << score / TRIALS_PER_SUBTEST << " (";
		for(unsigned int i = 0; i < score_parts.size(); ++i)
		{
			if(i != 0)
				ss << ", ";
			ss << score_parts[i] / TRIALS_PER_SUBTEST;
		}
		ss << ')';

		return ss.str();
	}

	unsigned int GACandidate::Write(ostream& s)
	{
		stringstream ss;

		WriteUInt32(NUM_CONSTANTS, ss);
		for(unsigned int i = 0; i < NUM_CONSTANTS; ++i)
			WriteSingle(constants[i], ss);
		
		WriteUInt32(ops.size(), ss);
		for(unsigned int i = 0; i < ops.size(); ++i)
		{
			const GPOp& op = ops[i];
			WriteBool(op.brain, ss);
			WriteByte(op.opcode, ss);
			WriteByte(op.dst_class, ss);
			WriteByte(op.dst_index, ss);
			WriteByte(op.arg1.src, ss);
			WriteByte(op.arg1.index, ss);
			if(!GPOp::IsUnary(op.opcode))
			{
				WriteByte(op.arg2.src, ss);
				WriteByte(op.arg2.index, ss);
			}
		}

		WriteByte(0x5A, ss);

		
		BinaryChunk chunk("IDKOPS02");
		chunk.data = ss.str();
		chunk.Write(s);

		return 0;
	}

	unsigned int GACandidate::Read(istream& s, GACandidate*& result, unsigned int id)
	{
		result = nullptr;

		BinaryChunk chunk;
		chunk.Read(s);

		if(chunk.GetName() != "IDKOPS02")
			return 1;

		istringstream ss(chunk.data);

		GACandidate* candidate = new GACandidate(id);
		unsigned int constants = ReadUInt32(ss);
		for(unsigned int i = 0; i < constants; ++i)
		{
			float f = ReadSingle(ss);
			if(i < NUM_CONSTANTS)
				candidate->constants[i] = f;
		}

		unsigned int ops = ReadUInt32(ss);
		for(unsigned int i = 0; i < ops; ++i)
		{
			GPOp op;
			op.brain = ReadBool(ss);
			op.opcode = ReadByte(ss);
			op.dst_class = ReadByte(ss);
			op.dst_index = ReadByte(ss);
			op.arg1.src = ReadByte(ss);
			op.arg1.index = ReadByte(ss);
			if(!GPOp::IsUnary(op.opcode))
			{
				op.arg2.src = ReadByte(ss);
				op.arg2.index = ReadByte(ss);
			}
			candidate->ops.push_back(op);
		}

		unsigned char trailer = ReadByte(ss);
		if(trailer != 0x5A)
		{
			delete candidate;
			return 3;
		}

		if(s.bad())
		{
			delete candidate;
			return 2;
		}
		else
		{
			result = candidate;
			return 0;
		}
	}

	string GACandidate::CompiledToString(unsigned int id, const vector<GPOp>& ops)
	{
		stringstream ss;
		ss << "id = " << id << "; length = " << ops.size() << endl;

		for(unsigned int j = 0; j < ops.size(); ++j)
			ss << ops[j].ToString() << endl;

		return ss.str();
	}

	unsigned int GACandidate::GetInputCoverage(const vector<GPOp>& ops)
	{
		set<unsigned char> used;
		for(unsigned int i = 0; i < ops.size(); ++i)
		{
			const GPOp& op = ops[i];
			if(op.arg1.src == 1)
				used.insert(op.arg1.index);
			if(!GPOp::IsUnary(op.opcode) && op.arg2.src == 1)
				used.insert(op.arg2.index);
		}

		return used.size();
	}




	/*
	 * GAExperiment methods
	 */
	GAExperiment::GAExperiment(const string& filename) : mutex(), batch(0), next_id(1), subtests(), elites(), candidates(), tokens_not_started(), tokens_busy()
	{
		// load the saved best brain if possible
		ifstream file(filename, ios::in | ios::binary);
		if(!!file)
		{
			unsigned int num_genomes = ReadUInt32(file);
			for(unsigned int i = 0; i < num_genomes; ++i)
			{
				GACandidate* loadme = nullptr;
				if(unsigned int error = GACandidate::Read(file, loadme, next_id++))
				{
					Debug(((stringstream&)(stringstream() << "Error " << error << " loading GACandidate" << endl)).str());
					break;
				}
				else
					candidates.push_back(loadme);
			}

			if(!candidates.empty())
			{
				Debug(((stringstream&)(stringstream() << "Successfully loaded " << candidates.size() << " brains from genepool" << endl)).str());
				DebugGenerationStats();

				stringstream ss;
				for(unsigned int i = 0; i < candidates.size(); ++i)
				{
					GACandidate& c = *candidates[i];
					c.Compile();
					ss << c.CompiledToString(c.id, c.compiled);
				}
				Debug(ss.str());
			}

			file.close();
		}

		if(candidates.empty())
			MakeFirstGeneration();

		GenerateSubtestList();

		GenerateTrialTokens();

		Debug(((stringstream&)(stringstream() << "first generation will contain " << candidates.size() << " candidates (" << tokens_not_started.size() << " tokens)" << endl)).str());
	}

	GAExperiment::~GAExperiment()
	{
		subtests.clear();
		elites.clear();
		tokens_not_started.clear();
		tokens_busy.clear();

		for(unsigned int i = 0; i < candidates.size(); ++i)
			delete candidates[i];
		candidates.clear();
	}

	void GAExperiment::GenerateSubtestList()
	{
		static const unsigned int num_subtests = 100;
		static const float scalar = 0.1f;

		vector<float> floats;
		for(unsigned int j = 0; j < num_subtests / 2; ++j)
		{
			float x = scalar * float(j) / (float(num_subtests / 2) - 1);
			floats.push_back(x);
			floats.push_back(-x);
		}
		while(floats.size() < num_subtests)
			floats.push_back(0.0f);					// i.e. if the number of subtests is not a multiple of 2

		vector<float[6]> axes(num_subtests);
		for(unsigned int i = 0; i < 6; ++i)
		{
			vector<float> axis_floats = floats;
			for(unsigned int j = 0; j < num_subtests; ++j)
			{
				swap(axis_floats[axis_floats.size() - 1], axis_floats[Random3D::RandInt(axis_floats.size())]);
				axes[j][i] = axis_floats[axis_floats.size() - 1];
				axis_floats.pop_back();
			}
		}

		subtests.clear();
		for(unsigned int i = 0; i < num_subtests; ++i)
		{
			const Vec3* axes_vec = (Vec3*)&axes[i];
			subtests.push_back(GASubtest(i, axes_vec[0], axes_vec[1]));
			//subtests.push_back(GASubtest(i, Vec3(), Vec3()));
		}
	}

	void GAExperiment::GenerateTrialTokens()
	{
		assert(tokens_not_started.empty());
		assert(tokens_busy.empty());

		for(unsigned int left = 0, right = candidates.size() - 1; left < right; left++, right--)
			swap(candidates[left], candidates[right]);

		for(unsigned int i = 0; i < candidates.size(); ++i)
		{
			GACandidate* candidate = candidates[i];
			if(!candidate->compiled_flag)
				candidate->Compile();

			assert(candidate->tokens_not_finished.empty());
			assert(candidate->tokens_busy.empty());

			candidate->score = 0.0f;
			candidate->score_parts.clear();
			candidate->time_spent = 0;
			candidate->aborting = false;

			for(unsigned int j = 0; j < subtests.size(); ++j)
				for(unsigned int k = 0; k < TRIALS_PER_SUBTEST; ++k)
				{
					GATrialToken token;
					token.candidate = candidate;
					token.subtest = subtests[j];
					token.trial = k;

					candidate->tokens_not_finished.insert(token);
					tokens_not_started.push_back(token);
				}
		}
	}

	void GAExperiment::MakeFirstGeneration()
	{
		unsigned int count = NUM_ELITES * NUM_ELITES;
		for(unsigned int i = 0; i < count; ++i)
		{
			GACandidate* candidate = new GACandidate(next_id++);

			for(unsigned int j = 0; j < NUM_CONSTANTS; ++j)
				candidate->constants[j] = (float)(j + 1) * (FLOAT_CONSTANT_RANGE / NUM_CONSTANTS);
			
			if(i != 0)
			{
#if 1
				for(unsigned int j = 0; j < NUM_BONE_OUTPUTS; ++j)// + NUM_BONE_MEM + NUM_BRAIN_MEM; ++j)
				{
					GPOp op;
					op.brain = j < NUM_BONE_OUTPUTS + NUM_BONE_MEM ? false : true;
					op.dst_class = j < NUM_BONE_OUTPUTS ? 2 : 1;
					op.dst_index = j < NUM_BONE_OUTPUTS ? j : j < NUM_BONE_OUTPUTS + NUM_BONE_MEM ? j - NUM_BONE_OUTPUTS : j - NUM_BONE_OUTPUTS - NUM_BONE_MEM;
					op.opcode = 0;
					op.arg1.src = op.arg1.index = op.arg2.src = op.arg2.index = 0;
					candidate->ops.push_back(op);
				}

				candidate->Randomize(MUTATION_COUNT * 2, MUTATION_SCALE * 2);
#else
				unsigned int target = TARGET_PROGRAM_SIZE / 5 + 10;

				candidate->ops.resize(target + NUM_BONE_OUTPUTS + NUM_BONE_MEM + NUM_BRAIN_MEM);

				set<unsigned char> scratch_set;

				for(unsigned int j = 0; j < target; ++j)
				{
					GPOp op;
					op.brain = false;
					op.dst_class = 0;
					op.dst_index = Random3D::RandInt(NUM_SCRATCH);
					op.opcode = RandomOpCode();
					op.arg1.Randomize(false, j, candidate->ops);
					if(!GPOp::IsUnary(op.opcode))
						op.arg2.Randomize(false, j, candidate->ops);
					candidate->ops[j] = op;

					scratch_set.insert(op.dst_index);
				}

				vector<unsigned char> scratch;
				for(set<unsigned char>::iterator iter = scratch_set.begin(); iter != scratch_set.end(); ++iter)
					scratch.push_back(*iter);

				for(unsigned int j = 0; j < NUM_BONE_OUTPUTS; ++j)// + NUM_BONE_MEM + NUM_BRAIN_MEM; ++j)
				{
					GPOp op;
					op.brain = j < NUM_BONE_OUTPUTS + NUM_BONE_MEM ? false : true;
					op.dst_class = j < NUM_BONE_OUTPUTS ? 2 : 1;
					op.dst_index = j < NUM_BONE_OUTPUTS ? j : j < NUM_BONE_OUTPUTS + NUM_BONE_MEM ? j - NUM_BONE_OUTPUTS : j - NUM_BONE_OUTPUTS - NUM_BONE_MEM;

					op.opcode = 0;
					op.arg1.src = 2;
					op.arg1.index = scratch[Random3D::RandInt(scratch.size())];
					op.arg2.src = 2;
					op.arg2.index = scratch[Random3D::RandInt(scratch.size())];

					candidate->ops[j + target] = op;
				}
#endif
			}

			for(unsigned int i = 0; i < candidate->ops.size(); ++i)
			{
				GPOp& op = candidate->ops[i];
				if(op.dst_class != 2)
				{
					if(op.arg1.src == 1)
						if(Random3D::RandInt(3) == 0)
							op.arg1.index = Random3D::RandInt(2) * 3 + 70;
			
					if(!GPOp::IsUnary(op.opcode) && op.arg2.src == 1)
						if(Random3D::RandInt(3) == 0)
							op.arg2.index = Random3D::RandInt(2) * 3 + 70;
				}
			}

			candidates.push_back(candidate);
		}
	}

	void GAExperiment::MakeNextGeneration()
	{
		++batch;

		// delete old candidates that didn't make it into the elites
		set<GACandidate*> dead;
		for(unsigned int i = 0; i < candidates.size(); ++i)
			dead.insert(candidates[i]);
		for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
			dead.erase(*iter);

		for(set<GACandidate*>::iterator iter = dead.begin(); iter != dead.end(); ++iter)
			delete *iter;
		dead.clear();
		candidates.clear();

		// elites of the last generation are included in the next one
		for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
			candidates.push_back(*iter);

		// create 2 crossovers for each pair of elites
		for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
			for(list<GACandidate*>::iterator jter = iter; jter != elites.end(); ++jter)
				if(iter != jter)
				{
					for(unsigned int n = 0; n < CROSSOVERS_PER_PAIR; ++n)
					{
						GACandidate* candidate = new GACandidate(next_id++, **iter, **jter);
						candidates.push_back(candidate);
					}
				}

		elites.clear();
	}

	GATrialToken GAExperiment::GetNextTrial()
	{
		unique_lock<std::mutex> lock(mutex);

		if(tokens_not_started.empty())
			return GATrialToken();

		GATrialToken result = *tokens_not_started.rbegin();
		tokens_not_started.pop_back();

		result.candidate->tokens_busy.insert(result);
		tokens_busy.insert(result);

		//Debug(((stringstream&)(stringstream() << "returning trial: candidate = " << result.candidate->id << "; trial = " << result.trial << endl)).str()); 

		return result;
	}

	void GAExperiment::TrialFinished(GATrialToken token, float score, const vector<float>& score_parts, unsigned int time_spent)
	{
		unique_lock<std::mutex> lock(mutex);

		GACandidate& candidate = *token.candidate;
		candidate.score += score;

		if(candidate.score_parts.empty())
			candidate.score_parts.resize(score_parts.size());
		else if(score_parts.empty())
			assert(candidate.score_parts.size() == score_parts.size() || score_parts.empty());

#if FORCE_FIRST_GEN_MODIFICATION
		if(candidate.p1 == 0 && candidate.p2 == 0)
		{
			score += 10000;
			candidate.score += 10000;
			if(!candidate.score_parts.empty())
				candidate.score_parts[0] += 10000;
		}
#endif

		for(unsigned int i = 0; i < score_parts.size(); ++i)
			candidate.score_parts[i] += score_parts[i];

		candidate.time_spent += time_spent;

		candidate.tokens_busy.erase(token);
		candidate.tokens_not_finished.erase(token);

		tokens_busy.erase(token);

		if(!candidate.aborting)
		{
			if(elites.size() >= NUM_ELITES && candidate.score >= (*elites.rbegin())->score)
				candidate.aborting = true;
		}

		if(candidate.tokens_not_finished.empty())
		{
			stringstream ss;
			ss << candidate.GetText();

			list<GACandidate*>::iterator insert_where;
			for(insert_where = elites.begin(); insert_where != elites.end(); ++insert_where)
				if(!candidate.aborting && (*insert_where)->score > candidate.score)
					break;

			elites.insert(insert_where, token.candidate);

			if(*elites.begin() == token.candidate)
				ss << "; new best!";
			else if(*elites.rbegin() == token.candidate && elites.size() > NUM_ELITES)
				ss << "; fail (" << candidate.time_spent << " / x)";		// log early-abort savings

			ss << endl;
			Debug(ss.str());

			if(elites.size() > NUM_ELITES && *elites.rbegin() != token.candidate)
				elites.pop_back();

			stringstream ss2;
			ss2 << "batch " << batch << endl;
			for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
			{
				const GACandidate& c = **iter;
				if(*iter == token.candidate)
				{
					if(elites.size() > NUM_ELITES && &c == *elites.rbegin())
						ss2 << endl;
					ss2 << ">   ";
				}
				else
					ss2 << "    ";
				ss2 << c.GetText() << endl;
			}
			debug_text = ss2.str();

			if(elites.size() > NUM_ELITES)
				elites.pop_back();
		}

		if(tokens_busy.empty() & tokens_not_started.empty())
		{
			time_t raw_time;
			time(&raw_time);
			tm now = *localtime(&raw_time);
			string filename = ((stringstream&)(stringstream() << "Files/Brains/genepool-" << now.tm_year + 1900 << "-" << now.tm_mon + 1 << "-" << now.tm_mday << "-" << now.tm_hour << "-" << now.tm_min << "-" << now.tm_sec)).str();

			SaveElites(filename, true);
			SaveElites("Files/Brains/genepool");

			DebugGenerationStats();
			
			MakeNextGeneration();
			GenerateTrialTokens();

			Debug(((stringstream&)(stringstream() << "next generation will contain " << candidates.size() << " candidates (" << tokens_not_started.size() << " tokens)" << endl)).str());
		}
	}

	float GAExperiment::GetEarlyFailThreshold()
	{
		unique_lock<std::mutex> lock(mutex);

		if(elites.size() < NUM_ELITES)
			return -1;
		else
			return (*elites.rbegin())->score;
	}

	void GAExperiment::SaveElites(const string& filename, bool verbose)
	{
		ofstream file(filename, ios::out | ios::binary);
		if(!file)
			Debug("Failed to save brain!\n");
		else
		{
			WriteUInt32(elites.size(), file);

			for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
				(**iter).Write(file);

			file.close();

			if(verbose)
			{
				stringstream ss;
				ss << "Genepool saved to \"" << filename << "\"" << endl;
				ss << "batch " << batch << " elites:" << endl;

				for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
				{
					const GACandidate& c = **iter;
					ss << '\t' << c.GetText() << "; " << c.ops.size() << " ops (" << c.compiled.size() << "); mutations: [";
					for(unsigned int i = 0; i < sizeof(c.mutations) / sizeof(unsigned short); ++i)
					{
						if(i != 0)
							ss << ", ";
						ss << c.mutations[i];
					}
					ss << "]" << endl;
				}

				Debug(ss.str());
			}
		}
	}

	void GAExperiment::DebugGenerationStats() const
	{
		vector<GACandidate*> analyze_these;
		if(!elites.empty())
			analyze_these.assign(elites.begin(), elites.end());
		else if(!candidates.empty())
			analyze_these.assign(candidates.begin(), candidates.end());
		else
			return;

		//stringstream ss;
		//
		//unsigned int count = GACandidate::NumIndexedParams();
		//for(unsigned int i = 0; i < count; ++i)
		//{
		//	float tot = 0.0f;
		//	float min, max;
		//	for(unsigned int j = 0; j < analyze_these.size(); ++j)
		//	{
		//		float value = ((float*)&analyze_these[j]->params_prefix)[i + 1];
		//		tot += value;
		//		if(j == 0 || value < min)
		//			min = value;
		//		if(j == 0 || value > max)
		//			max = value;
		//	}
		//
		//	float avg = tot / analyze_these.size();
		//	ss << "\tcomponent " << i << ": MIN = " << GACandidate::min_values.GetIndexedParam(i) << ", min = " << min << ", avg = " << avg << ", max = " << max << ", MAX = " << GACandidate::max_values.GetIndexedParam(i) << endl;
		//}
		//
		//Debug(ss.str());
	}

	string GAExperiment::GetDebugText()
	{
		return debug_text;
	}
}

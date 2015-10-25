#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <queue>

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#define NUM_REGISTERS 8
#define DEPTH_PIPELINE 6
#define MAX_ENTRIES 100000
#define MAX_RS 32

using namespace std;

struct pipeline_instr {
	long long int pc;
	string IR;
	string display_IR;
	int opcode;
	bool immediate;
	int op1, op2, op3;
	int A, B, imm_field;
	int cond;
	int alu_output;
	int load_md;
	bool wait_for_op1;
	bool wait_for_op2;
	bool wait_for_op3;
	int id;
	long long int cycle_finish;
	int rs_tag;
	int broadcast_output;
	//copy constructor
};

struct rs_f {
	int tag;
	bool valid;
};

struct rs_entry {
	pipeline_instr instr;
	bool busy;
	bool ready;
	vector<rs_f> operand;
	int id;
};

class rob_entry {
  public:
	pipeline_instr instr;
	int id;
	bool busy;
	bool issued;
	bool finished;
	rob_entry() {
		busy = false;
		issued = false;
		finished = false;
	}
};

class store_buff{
public:
	int id;
	int addr;
	int val;
};

class Simulator{
	long long int m_clk;
	bool control_flag;
	bool control_stall;
	map<int, int> d_cache;
	map<long long int, string> i_cache;
	vector<store_buff> store_buffer;

	long long int pc;

	int register_file[NUM_REGISTERS];
	
	void next_clock_cycle(){
		m_clk ++;
	}

	bool prev_ins_decoded_is_branch;
	bool halt_flag;

	string input_code;

	void load_i_cache();
	void print_i_cache();
	void print_d_cache();
	void print_reg_file();

	bool flush_pipeline;
	void flush();
	long long int num_ins_executed;
	long long int num_stalls;
	long long int num_control_stalls;

	// SuperScalar Part
	int n_width;
	int unique_id;
	int rs_tag;
	int rob_width;
	int rs_width;
	bool eoc;//end of code flag

	rs_f arf_flags[NUM_REGISTERS];

	queue<pipeline_instr> n_fetch, n_decode, decode_buffer, dispatch_buffer;
	vector<rob_entry> rob;

	vector<rs_entry> RS[3];
	vector<pipeline_instr> execute_buffer[3];
	bool new_issue[3];
	int num_units[3];
	int latency[8];

	int generate_id();
	int generate_tag();
	int get_rob_entry(int id);
	int tomasulo_update();
	int process_store_buffer();
	pair<int,int> get_rs_entry(int tag);
	
public:
	Simulator(string input_file);
	void decode(pipeline_instr &p);
	int execute(pipeline_instr &p);
	int simulate();

	// SuperScalar Part
	int multi_fetch();
	int multi_decode();
	int dispatch();
	int process_rs();
	int multi_execute();
	int broadcast_cdb(int tag, int val);
	int complete_instr();
	int retire_instr();
	int find_store_buffer_entry(int id);
	int forward_from_store_buffer(int addr);
};

int gen_int_from_string(string s);
int get_twos_complement(string s);

#endif


#include "simulator.h"

#include<stdlib.h>

int get_int_from_string(string s){
	int num = 0;
	int len = s.length();
	int pow2 = 1;
	for(int i = len-1; i >= 0; i--){
		if(s[i] == '1')
			num += pow2;
		pow2 *= 2;

	}
	return num;
}

int get_twos_complement(string s) {
	int num = 0;
	if (s[0] == '0') {
		return get_int_from_string(s);
	}
	else {
		// if 2'complement of a negative no y is x then ~x + 1 = -y
		for(int i = 0; i < s.size(); ++i) {
			s[i] = '0' + (1 - (s[i] - '0'));
		}
		num = get_int_from_string(s);
		num++;
		return (-num);
	}
}

Simulator::Simulator(string input_file){
	//decode stage
	prev_ins_decoded_is_branch = false;
	control_flag = false;
	control_stall = false;
	m_clk = 0;

	pc = 0;
	for(int i = 0; i < NUM_REGISTERS; ++i) {
		register_file[i] = 0;
		arf_flags[i].valid = true;
	}

	for(int i = 0; i < 8; ++i) {
		latency[i] = 1;
	}
	for(int i = 0; i < 3; ++i) {
		num_units[i] = 2;
	}
	register_file[0] = 0;


	input_code = input_file;
	flush_pipeline = false;
	num_ins_executed = 0;
	num_stalls = 0;
	num_control_stalls = 0;

	rs_width = 4;
	n_width = 4;
	rob_width = 8;

	unique_id = 0;
	rs_tag = 0;

	for (int i = 0; i < 3; ++i) {
		new_issue[i] = false;
	}

	eoc = false;


}

// modify this function for superscalar
void Simulator::flush() {
	int i;
	while(decode_buffer.size() > 0)
		decode_buffer.pop();
	while(n_fetch.size() > 0)	
		n_fetch.pop();
}


void Simulator::load_i_cache() {
	ifstream in;
	in.open(input_code.c_str(),ios::in);
	long long int pc_value = 0;
	string instr;
	while (getline(in, instr)) {
		if (instr == "1110000000000000") {
			i_cache[pc_value] = instr;
			pc_value += 2;
		}
		else {
			i_cache[pc_value] = instr;
			pc_value += 2;
		}
	}
	in.close();
}

void Simulator::print_i_cache() {
	cout << "I Cache Size: " << i_cache.size() << endl;
	for(map<long long int, string>::iterator it = i_cache.begin(); it != i_cache.end(); ++it) {
		cout << it->first << ": " << it->second <<endl;
	}
}

void Simulator::print_d_cache() {
	cout << "D Cache\n";
	cout << d_cache.size() << endl;
	for(map<int, int>::iterator it = d_cache.begin(); it != d_cache.end(); ++it) {
		cout << it->first << ": " << it->second <<endl;
	}
}

void Simulator::print_reg_file() {
	cout << "Reg File\n";
	for (int i = 0; i < NUM_REGISTERS; ++i) {
		cout << i << ": " << register_file[i] << " " << arf_flags[i].valid << arf_flags [i]. tag << endl;
	}
}


void Simulator::decode(pipeline_instr& p){


	string IR = p.IR;
	p.opcode = get_int_from_string(IR.substr(0,3));
	p.immediate = (get_int_from_string(IR.substr(3,1)) >= 1)?true:false;

	if(p.opcode == 7 && p.immediate == 0)
		p.opcode = 7;

	if (p.opcode == 5){ // jump instruction
		p.op1 = get_twos_complement(IR.substr(4,8));
	}
	else if (p.opcode == 6){ // BEQZ
		p.op1 = get_int_from_string(IR.substr(4,4));
		p.op2 = get_twos_complement(IR.substr(8,8));

	}
	else if (p.opcode <= 4) { // add, sub, mul, ld,st
		p.op1 = get_int_from_string(IR.substr(4,4));
		p.op2 = get_int_from_string(IR.substr(8,4));
		if (p.immediate) {
			p.op3 = get_twos_complement(IR.substr(12,4));
		}
		else {
			p.op3 = get_int_from_string(IR.substr(12,4));
		}

	}
}

int Simulator::forward_from_store_buffer(int addr){
	for(int i=store_buffer.size()-1;i>=0;i--){
		if(store_buffer[i].addr == addr)
			return i;
	}
	return -1;
}

int Simulator::execute (pipeline_instr &p){
	long long int A = p.A;
	long long int B = p.B;
	long long int imm_field = p.imm_field;
	int temp;
	switch(p.opcode){

		case 0: if (p.immediate) p.alu_output = A + imm_field;
					else p.alu_output = A + B; 
					p.broadcast_output = p.alu_output;
					break;
		case 1: if (p.immediate) p.alu_output = A - imm_field;
					else p.alu_output = A - B; 
					p.broadcast_output = p.alu_output;
					break;
		case 2: if (p.immediate) p.alu_output = A * imm_field;
					else p.alu_output = A * B; 
					p.broadcast_output = p.alu_output;
					break;

		case 3: p.alu_output = 0 + A; 
				temp =  forward_from_store_buffer(p.alu_output);
				if(temp != -1){
					p.load_md = store_buffer[temp].val;

				}
				else{
					if (d_cache.count (p.alu_output) > 0) {
						p.load_md = d_cache[p.alu_output];
					}
					else {
						p.load_md = 0;
					}
					p.broadcast_output = p.load_md;
				}

				break;
		case 4: p.alu_output = 0 + A; break;

		case 5: p.alu_output = p.pc + (imm_field*2); p.cond = 1;  break;
		case 6: p.alu_output = p.pc + (imm_field*2); p.cond = (A == 0); break;
	}
	return 1;
}

int Simulator::generate_id() {
	int id = unique_id;
	unique_id = (unique_id + 1)%MAX_ENTRIES;
	return id;
}

int Simulator::generate_tag() {
	int tag = rs_tag;
	rs_tag = (rs_tag + 1)%MAX_RS;
	return tag;
}

int Simulator::get_rob_entry (int id) {
	for (int i = 0; i < rob.size(); ++i) {
		if (rob[i].id == id) {
			return i;
		}
	}
	return 0;
}
int Simulator::find_store_buffer_entry(int id){
	for (int i = 0; i < store_buffer.size(); ++i) {
		if (store_buffer[i].id == id) {
			return i;
		}
	}
	return -1;
}

pair<int, int> Simulator::get_rs_entry (int id) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < RS[i].size(); ++j) {
			if (RS[i][j].id == id) {
				return make_pair(i,j);
			}
		}
	}
	return make_pair(0,0);
}



int Simulator::multi_fetch() {
	if(control_stall == false && halt_flag == false){
		while (decode_buffer.size() < n_width && n_fetch.size() > 0) {
			decode_buffer.push(n_fetch.front());
			n_fetch.pop();
		}
		while (n_fetch.size() < n_width) {
			pipeline_instr p;
			if (i_cache.count(pc)>0) {
				p.IR = i_cache[pc];
			
				p.pc = pc;
				pc += 2;
			
				cout<<"Fetch : "<<p.IR<<endl;
				n_fetch.push(p);
			}
			if (i_cache.count(pc) == 0) {
				break;
			}
		}
	}
	return 1;
}


int Simulator::multi_decode() {
	while (dispatch_buffer.size() < n_width && n_decode.size() > 0) {
		dispatch_buffer.push(n_decode.front());
		n_decode.pop();
	}
	while (n_decode.size() < n_width && decode_buffer.size() > 0 && !control_stall) {
		
		pipeline_instr p = decode_buffer.front();
		decode_buffer.pop();
		decode(p);
		n_decode.push(p);
		cout<<"Decode: "<<p.IR<<endl;
		if(p.opcode == 5 || p.opcode == 6){
			flush();
			control_stall = true;
		}
		if (p.opcode == 7) {
			flush();
			halt_flag = true;
		}
		

	}
	return 1;
}

int Simulator::dispatch() {
	bool dispatch = true;
	int num_dispatch = 0;
	while (dispatch_buffer.size() > 0 && dispatch && num_dispatch < n_width) {
		dispatch = false;
		if (dispatch_buffer.front().opcode <= 2) {
			if (RS[0].size() < rs_width) {
				dispatch = true;
			}
		}
		else if (dispatch_buffer.front().opcode <= 4) {
			if (RS[1].size() < rs_width) {
				dispatch = true;
			}
		}
		else if (dispatch_buffer.front().opcode <= 6) {
			if (RS[2].size() < rs_width) {
				dispatch = true;
			}
		}
		else {
			dispatch = true;
		}

		if (dispatch) {
			rs_entry rs;
			rob_entry r;
			pipeline_instr p = dispatch_buffer.front();
			int id = generate_id();
			int tag = generate_tag();
			p.id = id;

			r.id = id;
			rs.id = id;
			r.busy = true;

			if (p.opcode <= 2) {
				if (arf_flags[p.op2].valid) {
					p.A = register_file[p.op2];
					rs_f e1;
					e1.valid = true;
					rs.operand.push_back(e1);
				}
				else {
					rs_f e1;
					e1.valid = false;
					e1.tag = arf_flags[p.op2].tag;
					rs.operand.push_back(e1);
				}
				if (p.immediate) {
					p.imm_field = p.op3;
					rs_f e1;
					e1.valid = true;
					rs.operand.push_back(e1);
				}
				else {	
					if (arf_flags[p.op3].valid) {
						p.B = register_file[p.op3];
						rs_f e1;
						e1.valid = true;
						rs.operand.push_back(e1);
					}
					else {
						rs_f e1;
						e1.valid = false;
						e1.tag = arf_flags[p.op3].tag;
						rs.operand.push_back(e1);
					}
				}
				arf_flags[p.op1].valid = false;
				arf_flags[p.op1].tag = tag;
				p.rs_tag = tag;
				rs.instr = p;
				r.instr = p;
				RS[0].push_back(rs);
				rob.push_back(r);
			}
			else if (p.opcode == 3) {
				if (arf_flags[p.op2].valid) {
					p.A = register_file[p.op2];
					rs_f e1;
					e1.valid = true;
					rs.operand.push_back(e1);
				}
				else {
					rs_f e1;
					e1.valid = false;
					e1.tag = arf_flags[p.op2].tag;
					rs.operand.push_back(e1);
				}
				arf_flags[p.op1].valid = false;
				arf_flags[p.op1].tag = tag;
				p.rs_tag = tag;
				rs.instr = p;
				r.instr = p;
				RS[1].push_back(rs);
				rob.push_back(r);
			}
			else if (p.opcode == 4) {
				if (arf_flags[p.op2].valid) {
					p.B = register_file[p.op2];
					rs_f e1;
					e1.valid = true;
					rs.operand.push_back(e1);
				}
				else {
					rs_f e1;
					e1.valid = false;
					e1.tag = arf_flags[p.op2].tag;
					rs.operand.push_back(e1);
				}
				if (arf_flags[p.op1].valid) {
					p.A = register_file[p.op1];
					rs_f e1;
					e1.valid = true;
					rs.operand.push_back(e1);
				}
				else {
					rs_f e1;
					e1.valid = false;
					e1.tag = arf_flags[p.op1].tag;
					rs.operand.push_back(e1);
				}
				p.rs_tag = tag;
				rs.instr = p;
				r.instr = p;
				RS[1].push_back(rs);
				rob.push_back(r);
			}
			else if (p.opcode == 5) {
				p.imm_field = p.op1;
				p.rs_tag = tag;
				rs.instr = p;
				r.instr = p;
				RS[2].push_back(rs);
				rob.push_back(r);
			}
			else if (p.opcode == 6) {
				p.imm_field = p.op2;
				if (arf_flags[p.op1].valid) {
					p.A = register_file[p.op1];
					rs_f e1;
					e1.valid = true;
					rs.operand.push_back(e1);
				}
				else {
					rs_f e1;
					e1.valid = false;
					e1.tag = arf_flags[p.op1].tag;
					rs.operand.push_back(e1);
				}
				p.rs_tag = tag;
				rs.instr = p;
				r.instr = p;
				RS[2].push_back(rs);
				rob.push_back(r);
			}
			else {
				r.finished = true;
				r.instr = p;
				rob.push_back(r);
			}

		    cout<<"Dispatch: "<<p.IR<<endl;	
			num_dispatch++;
			dispatch_buffer.pop();
		}

	}
	return 1;
}

int Simulator::process_rs() {
	tomasulo_update();
	for (int i = 0; i < 3; ++i) {
		if (RS[i].size() > 0) {
			for (int j = 0; j < RS[i].size(); ++j) {
				if (RS[i][j].ready && execute_buffer[i].size() < num_units[i]) {
					execute_buffer[i].push_back(RS[i][j].instr);
					execute_buffer[i][execute_buffer[i].size() - 1].cycle_finish = m_clk + latency[RS[i][j].instr.opcode]; // modify this per ALU opcode for varying latency
					new_issue[i] = true;
					rob[get_rob_entry(RS[i][j].instr.id)].issued = true;
				}
			}
		}
	}
	return 1;
}

int Simulator::multi_execute() {
	for (int i = 0; i < 3; ++i) {
		if (new_issue[i]) {
			for (int j = 0; j < execute_buffer[i].size(); ++j) {
				cout<<"Execute: "<<execute_buffer[i][j].IR<<endl;
				execute (execute_buffer[i][j]);
				new_issue[i] = false;
			}
		}
	}
	return 1;
}

int Simulator::broadcast_cdb(int tag, int val){
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < RS[i].size(); ++j) {
			rs_entry p = RS[i][j];
			if (p.instr.opcode <= 2 || p.instr.opcode == 4) {
				if (p.operand[0].valid == false) {
					if (p.operand[0].tag == tag) {
						p.instr.A = val;
						p.operand[0].valid = true;
					}
				}
				if (p.operand.size() == 2 && p.operand[1].valid == false) {
					if (p.operand[1].tag == tag) {
						p.instr.B = val;
						p.operand[1].valid = true;
					}
				}
			}
			else if (p.instr.opcode == 3 || p.instr.opcode == 6) {
				if (p.operand[0].valid == false) {
					if (p.operand[0].tag == tag) {
						p.instr.A = val;
						p.operand[0].valid = true;
					}
				}
			}
			RS[i][j] = p;
		}
	}
	for (int i = 0; i < NUM_REGISTERS; ++i) {
		if (arf_flags[i].valid == false && arf_flags[i].tag == tag) {
			register_file[i] = val;
			arf_flags[i].valid = true;
		}
	}
	return 1;
}

int Simulator::complete_instr() {
	for (int i = 0; i < 3; ++i) {
		while (execute_buffer[i].size() > 0) {
			if (execute_buffer[i][0].cycle_finish == m_clk) {
				cout << "Complete: "<<execute_buffer[i][0].IR<<endl;
				if (execute_buffer[i][0].opcode <= 3) {
					broadcast_cdb(execute_buffer[i][0].rs_tag, execute_buffer[i][0].broadcast_output);
				}
				rob[get_rob_entry(execute_buffer[i][0].id)].finished = true;
				rob[get_rob_entry(execute_buffer[i][0].id)].instr = execute_buffer[i][0];
				pair<int,int> rs_id = get_rs_entry (execute_buffer[i][0].id);
				if(execute_buffer[i][0].opcode == 5){//JMP
					control_stall = false;
					pc = execute_buffer[i][0].alu_output;
				}
				if(execute_buffer[i][0].opcode == 6){//BEQZ
					control_stall = false;
					if(execute_buffer[i][0].cond)
						pc = execute_buffer[i][0].alu_output;
					else
						pc = execute_buffer[i][0].pc + 2;
				}
				if(execute_buffer[i][0].opcode == 4){
					store_buff s;
					s.id = execute_buffer[i][0].id;
					s.addr = execute_buffer[i][0].alu_output;
					s.val = execute_buffer[i][0].B;
					store_buffer.push_back(s);
					//d_cache[ins_pipeline[ins_index].alu_output] = ins_pipeline[ins_index].B;
				}

				RS[rs_id.first].erase(RS[rs_id.first].begin() + rs_id.second);
				execute_buffer[i].erase(execute_buffer[i].begin() + 0);
			}
		}
	}
	return 1;
}

int Simulator::tomasulo_update() {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < RS[i].size(); ++j) {
			bool ready = true;
			for (int k = 0; k < RS[i][j].operand.size(); ++k) {
				if (RS[i][j].operand[k].valid == false) {
					ready = false;
					break;
				}
			}
			RS[i][j].ready = ready;
		}
	}
	return 1;
}


int Simulator::retire_instr() {
	while (rob.size() > 0) {
		if (rob.front().finished) {
			cout << "Finish: " << rob.front().instr.IR << endl;
			num_ins_executed++;
			if (rob.front().instr.opcode == 7) {
				return 0;
			}
			if (rob.front().instr.opcode == 4) {
				int loc = find_store_buffer_entry(rob.front().instr.id);
				d_cache[store_buffer[loc].addr] = store_buffer[loc].val;
				store_buffer.erase(store_buffer.begin()+loc);
			}
			
			rob.erase(rob.begin());
		}
		else {
			break;
		}
				
	}
	return 1;
}

int Simulator::process_store_buffer() {
	return 1;
}

int Simulator::simulate(){
	load_i_cache();
	print_i_cache();
	int i = 0;
	while(true){
		cout << "CLK: "<<m_clk<<endl;
		multi_fetch();
		multi_decode();
		dispatch();
		multi_execute();
		complete_instr();
		process_rs();
		print_reg_file();
		if (!retire_instr()) {
		   // clean up store buffer
			break;
		}	
		process_store_buffer();
		next_clock_cycle();
		
	}
	print_d_cache();
	printf("clk cycles = %lld\n",m_clk + 1);
	printf("CPI = %f\n",1.0*(m_clk + 1)/num_ins_executed);
	// printf("Stalls = %lld\n",num_stalls - 5);
	// printf("Control Stalls = %lld\n",num_control_stalls);
	// printf("RAW Stalls = %lld\n",num_stalls - 5 - num_control_stalls);
	return 1;
}





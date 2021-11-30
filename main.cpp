#include "processing.cpp"
#include "robot.cpp"
#include "server.cpp"

#include "systemc.h"

#define CLOCK_FREQUENCY 100
#define MAP_SIZE_X 10
#define MAP_SIZE_Y 9
#define GRID_SIZE 2000		//represents 2000 mmm
#define NUM_OF_ROBOTS 4
#define NUM_OF_OBSTACLES 6
#define GRID_SIZE_SCALED GRID_SIZE*CLOCK_FREQUENCY

template<int program_size> class stimulus:public sc_module {
	public:
		//PORTS
		sc_out<bool> clock;

		//CONSTRUCTOR
		SC_HAS_PROCESS(stimulus);

		stimulus(sc_module_name name):sc_module(name) {
			SC_THREAD(main);
		}

		//PROCESS
		void main() {
			clock = 1;
			wait(5, SC_MS);
			for (int i = 0; i < program_size*2; i++) {
				clock = 0;
				wait(5, SC_MS);
				clock = 1;
				wait(5, SC_MS);
			}
		}
};

int sc_main(int argc, char* argv[]) {
    //SIGNALS
    sc_signal<bool> clock;
	sc_signal<bool> tx_ack_s[NUM_OF_ROBOTS];
	sc_signal<bool> tx_flag_s[NUM_OF_ROBOTS];
	sc_signal<sc_uint<16> > tx_data_s[NUM_OF_ROBOTS];
	sc_signal<bool> rx_ack_s[NUM_OF_ROBOTS];
	sc_signal<bool> rx_flag_s[NUM_OF_ROBOTS];
	sc_signal<sc_uint<16> > rx_data_s[NUM_OF_ROBOTS];
	sc_signal<bool> tx_ack_p[NUM_OF_ROBOTS];
	sc_signal<bool> tx_flag_p[NUM_OF_ROBOTS];
	sc_signal<sc_uint<16> > tx_data_p[NUM_OF_ROBOTS];
	sc_signal<bool> rx_ack_p[NUM_OF_ROBOTS];
	sc_signal<bool> rx_flag_p[NUM_OF_ROBOTS];
	sc_signal<sc_uint<16> > rx_data_p[NUM_OF_ROBOTS];
	sc_fifo<int> fifo_data_robot1(80);
	sc_fifo<int> fifo_data_robot2(80);
	sc_fifo<int> fifo_data_robot3(80);
	sc_fifo<int> fifo_data_robot4(80);
	
	//LOCAL VAR
	const int map[MAP_SIZE_Y][MAP_SIZE_X] = 
	{	
		{	1,	2,	3,	4,	5,	6,	7,	8,	9,	10,	},
	 	{	11,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	12	},
	 	{	13,	14,	15,	16,	17,	18,	19,	20,	21,	22	},
		{	23,	-1,	-1,	-1,	-1,	24,	-1,	-1,	-1,	25	},
		{	26,	27,	28,	29,	30,	31,	32,	33,	34,	35	},
		{	36,	-1,	-1,	-1,	-1,	-1,	37,	-1,	-1,	38	},
		{	39,	40,	41,	42,	43,	44,	45,	46,	47,	48	},
		{	49,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	50	},
		{	51,	52,	53,	54,	55,	56,	57,	58,	59,	60	}
	};
	const int robot_path[NUM_OF_ROBOTS][23] =
	{
		{	1,11,13,14,15,16,17,18,24,31,30,29,28,27,26,36,39,49,51,52,53,-1	},
		{	10,12,22,21,20,19,18,24,31,32,33,34,35,25,-1	},
		{	51,49,39,36,26,27,28,29,30,31,32,37,45,46,47,48,38,-1	},
		{	60,50,48,47,46,45,44,43,42,41,40,39,36,26,23,-1	}
	};
	const int obstacle_path[NUM_OF_OBSTACLES][23] =
	{
		{	6,	5,	4,	3,	2,	1,	11,	13,	14,	15, 16,	17,	18,	19,	20,	21,	22,	12,	10,	9,	8,	7,	6	},
		{	18,	17,	16,	15,	14,	13,	23,	26,	27,	28,	29,	30,	31,	24,	18	},
		{	22,	21,	20,	19,	18,	24,	31,	32,	33,	34,	35,	25,	22	},
		{	32,	31,	30,	29,	28,	27,	26,	36,	39,	40,	41,	42,	43,	44,	45,	37,	32	},
		{	35,	34,	33,	32,	37,	45,	46,	47,	48,	38,	35	},
		{	45,	46,	47,	48,	50,	60,	59,	58,	57,	56,	55,	54,	53,	52,	51,	49,	39,	40,	41,	42,	43,	44,	45	}
	};
	

    //MODULES
	sc_trace_file* speed = sc_create_vcd_trace_file("robot_trace");
    processing<MAP_SIZE_X, MAP_SIZE_Y, GRID_SIZE_SCALED, NUM_OF_ROBOTS, NUM_OF_OBSTACLES> processing("processing", (const int*)map, (const int*) obstacle_path, speed);
	processing.clock(clock);
	for (int i = 0; i < NUM_OF_ROBOTS; i++) {
		processing.tx_ack[i](rx_ack_p[i]);
		processing.tx_flag[i](rx_flag_p[i]);
		processing.tx_data[i](rx_data_p[i]);
		processing.rx_ack[i](tx_ack_p[i]);
		processing.rx_flag[i](tx_flag_p[i]);
		processing.rx_data[i](tx_data_p[i]);
	}
	processing.fifo_data[0](fifo_data_robot1);
	processing.fifo_data[1](fifo_data_robot2);
	processing.fifo_data[2](fifo_data_robot3);
	processing.fifo_data[3](fifo_data_robot4);

    server<MAP_SIZE_X, MAP_SIZE_Y, NUM_OF_ROBOTS> server("processing", (const int*)map, (const int*) robot_path);
	server.clock(clock);
	for (int i = 0; i < NUM_OF_ROBOTS; i++) {
		server.tx_ack[i](rx_ack_s[i]);
		server.tx_flag[i](rx_flag_s[i]);
		server.tx_data[i](rx_data_s[i]);
		server.rx_ack[i](tx_ack_s[i]);
		server.rx_flag[i](tx_flag_s[i]);
		server.rx_data[i](tx_data_s[i]);
	}
	server.fifo_data[0](fifo_data_robot1);
	server.fifo_data[1](fifo_data_robot2);
	server.fifo_data[2](fifo_data_robot3);
	server.fifo_data[3](fifo_data_robot4);
	
	robot robots[NUM_OF_ROBOTS] = {{"Robot_1"}, {"Robot_2"}, {"Robot_3"}, {"Robot_4"}};
	for (int i = 0; i < NUM_OF_ROBOTS; i++) {
		robots[i].clock(clock);
		robots[i].tx_ack_p(tx_ack_p[i]);
		robots[i].tx_flag_p(tx_flag_p[i]);
		robots[i].tx_data_p(tx_data_p[i]);
		robots[i].rx_ack_p(rx_ack_p[i]);
		robots[i].rx_flag_p(rx_flag_p[i]);
		robots[i].rx_data_p(rx_data_p[i]);
		robots[i].tx_ack_s(tx_ack_s[i]);
		robots[i].tx_flag_s(tx_flag_s[i]);
		robots[i].tx_data_s(tx_data_s[i]);
		robots[i].rx_ack_s(rx_ack_s[i]);
		robots[i].rx_flag_s(rx_flag_s[i]);
		robots[i].rx_data_s(rx_data_s[i]);
	}
	
	stimulus<2700> stimulus("stim");
	stimulus.clock(clock);

    //TRACES
    sc_trace_file* tf = sc_create_vcd_trace_file("sim_trace");
    sc_trace(tf, clock, "clock");
    for (int i = 0; i < NUM_OF_ROBOTS; i++) {
		char num = i + '0';
    	sc_trace(tf, tx_ack_s[i], "tx_ack_from_server" + std::to_string(i+1));
    	sc_trace(tf, tx_flag_s[i], "tx_flag_to_server" + std::to_string(i+1));
    	sc_trace(tf, tx_data_s[i], "tx_data_to_server" + std::to_string(i+1));
        sc_trace(tf, rx_ack_s[i], "rx_ack_to_server" + std::to_string(i+1));
    	sc_trace(tf, rx_flag_s[i], "rx_flag_from_server" + std::to_string(i+1));
    	sc_trace(tf, rx_data_s[i], "rx_data_from_server" + std::to_string(i+1));
    	sc_trace(tf, tx_ack_p[i], "tx_ack_from_processing" + std::to_string(i+1));
    	sc_trace(tf, tx_flag_p[i], "tx_flag_to_processing" + std::to_string(i+1));
    	sc_trace(tf, tx_data_p[i], "tx_data_to_processing" + std::to_string(i+1));
        sc_trace(tf, rx_ack_p[i], "rx_ack_to_processing" + std::to_string(i+1));
    	sc_trace(tf, rx_flag_p[i], "rx_flag_from_processing" + std::to_string(i+1));
    	sc_trace(tf, rx_data_p[i], "rx_data_from_srocessing" + std::to_string(i+1));
	}
	

    //START SIM
    sc_start(((2700)*2)*10, SC_MS);
    sc_close_vcd_trace_file(tf);
	sc_close_vcd_trace_file(speed);

    return 0;
}

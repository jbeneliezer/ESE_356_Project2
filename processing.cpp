#include "systemc.h"

#define OBSTACLE_SPEED 4000		//4000 mm/s
#define ROBOT_SPEED_MAX 2000	//2000 mm/s

template<int map_size_x, int map_size_y, int grid_size, int num_of_robots, int num_of_obstacles> class processing:public sc_module {
	public:
		//PORTS
		sc_in<bool> clock;
		sc_in<bool> tx_ack[num_of_robots];
		sc_out<bool> tx_flag[num_of_robots];
		sc_out<sc_uint<16> > tx_data[num_of_robots];
		sc_out<bool> rx_ack[num_of_robots];
		sc_in<bool> rx_flag[num_of_robots];
		sc_in<sc_uint<16> > rx_data[num_of_robots];
		sc_fifo_in<int> fifo_data[num_of_robots];
		
		//CONSTRUCTOR
		SC_HAS_PROCESS(processing);
		
		processing(sc_module_name name, const int* map_ptr, const int* obstacle_path_ptr, sc_trace_file* tf_ptr):
		sc_module(name), _map_ptr(map_ptr), _obstacle_path_ptr(obstacle_path_ptr) , tf(tf_ptr){
			SC_METHOD(prc_update);
			sensitive << clock.pos();
			
			SC_THREAD(prc_tx);
			sensitive << tx_ack[0].pos() << tx_ack[1].pos() << tx_ack[2].pos() << tx_ack[3].pos();
			
			SC_THREAD(prc_rx);
			sensitive << rx_flag[0].pos() << rx_flag[1].pos() << rx_flag[2].pos() << rx_flag[3].pos();

			for (int x = 0; x < map_size_x; x++) {
				for (int y = 0; y < map_size_y; y++) {
					_map_data[x][y] = *(_map_ptr + y*map_size_x + x);	//store map data locally
				}
			}
			for (int i = 0; i < num_of_obstacles; i++) {		//initialize all obstacles
				_obstacles[i].status = 0;
				_obstacles[i].position_x = grid_size/2;
				_obstacles[i].position_y = grid_size/2;
				_obstacles[i].speed = OBSTACLE_SPEED;
				for (int o = 0; o < 23; o++) {
					_obstacles[i].path[o] = *(_obstacle_path_ptr + i*23 + o);
				}
				_obstacles[i].current_grid = _obstacles[i].path[0];
				_obstacles[i].next_grid = _obstacles[i].path[1];
				for (int x = 0; x < map_size_x; x++) {
					for (int y = 0; y < map_size_y; y++) {
						//store the map xy coordinate of the grids
						if (_obstacles[i].current_grid == _map_data[x][y]) {
							_obstacles[i].current_grid_map_x = x;
							_obstacles[i].current_grid_map_y = y;
						}
						if (_obstacles[i].next_grid == _map_data[x][y]) {
							_obstacles[i].next_grid_map_x = x;
							_obstacles[i].next_grid_map_y = y;
						}
					}
				}
			}
			for (int i = 0; i < num_of_robots; i++) {			//initialize all robots
				_robot_path[i][0] = -1;
				
				_robots[i].position_x = grid_size/2;			//init robots to center of grid
				_robots[i].position_y = grid_size/2;
				_robots[i].speed = 0;					//init robot speed to 0 (default for phase 2)
				
				_main_table[i].status = 4;						//init main_table
				_main_table[i].prev_status = 0;
				_main_table[i].speed = 0;
				_main_table[i].modified = false;
				_main_table[i].current_grid_map_x = _main_table[i].next_grid_map_x = -1;
				_main_table[i].current_grid_map_y = _main_table[i].next_grid_map_y = -1;
				_main_table[i].current_grid = _main_table[i].next_grid = -1;
				
				_tx_table[i].status = 0;						//init tx_table
				_tx_table[i].modified = false;
				_rx_table[i].status = 0;						//init rx_table
				_rx_table[i].modified = false;
				
				_fifo_data_index[i] = -1;
			}
			_tx_counter = 0;
			_rx_counter = 0;

			sc_trace(tf, _robots[0].speed, "robot_1_speed");
			sc_trace(tf, _robots[1].speed, "robot_2_speed");
			sc_trace(tf, _robots[2].speed, "robot_3_speed");
			sc_trace(tf, _robots[3].speed, "robot_4_speed");
			sc_trace(tf, _main_table[0].current_grid, "robot_1_current_grid");
			sc_trace(tf, _main_table[1].current_grid, "robot_2_current_grid");
			sc_trace(tf, _main_table[2].current_grid, "robot_3_current_grid");
			sc_trace(tf, _main_table[3].current_grid, "robot_4_current_grid");
		}

	private:
		//LOCAL VAR
		typedef struct Robot{
			int position_x;		//positions of robot X
			int position_y;		//positions of robot Y
			int speed;			//speed of robot
		}Robot;
		
		typedef struct Robot_Status {	//NOTE: Used for rx and tx tables
			int status;			//navigation status of robot
			bool modified;		//whether status has been modified
		}Robot_Status;
		
		typedef struct Robot_Main_Status {
			int status;
			int prev_status;
			int current_grid;
			int current_grid_map_x;
			int current_grid_map_y;
			int next_grid;
			int next_grid_map_x;
			int next_grid_map_y;
			int speed;
			bool modified;
		}Robot_Main_Status;

		typedef struct Obstacle{
			int status;
			int position_x;
			int position_y;
			int speed;
			int current_grid;
			int current_grid_map_x;
			int current_grid_map_y;
			int next_grid;
			int next_grid_map_x;
			int next_grid_map_y;
			int path[23];
		}Obstacle;
		
		const int* _map_ptr;						//pointer to map data
		int _map_data[map_size_x][map_size_y];		//locally stored map data
		int _robot_path[num_of_robots][23];			//parameterized robots path (hard-coded for phase 1)
		const int* _obstacle_path_ptr;				//pointer to obstacle path data
		Obstacle _obstacles[num_of_obstacles];		//array of all obstacles
		Robot _robots[num_of_robots];				//array of all robots
		Robot_Main_Status _main_table[num_of_robots];
		
		int _tx_counter;
		int _rx_counter;
		Robot_Status _tx_table[num_of_robots];
		Robot_Status _rx_table[num_of_robots];
		sc_event tx_signal;

		int _clock_count = -1;
		int _fifo_data[num_of_robots][80];
		int _fifo_data_index[num_of_robots];

		sc_trace_file* tf;

		//PROCESS
		void prc_tx() {
			while (1) {
				wait(tx_signal);
				while (_tx_counter > 0) {					//if recieved data
					for (int i = 0; i < num_of_robots; i++) {	//loop through tx table
						if (_tx_table[i].modified) {
							tx_flag[i] = 1;						//set tx flag
							tx_data[i] = _tx_table[i].status;	//write data to tx channel
							wait();								//wait for ack bit from robot
							if (tx_ack[i] == 1) {
								_tx_table[i].modified = false;	//if ack was found,
								_tx_counter--;					//decrement tx_counter
							}
							tx_flag[i] = 0;						//clear tx flag
							wait(SC_ZERO_TIME);
							break;					//note: if ack was not recieved,
						}							//process will keep sending until successful
					}
				}
			}
		}
		
		void prc_rx() {
			while(1) {
				wait();
				for (int i = 0; i < num_of_robots; i++) {
					if (rx_flag[i] == 1) {
						rx_ack[i] = 1;								//send ack bit
						_rx_table[i].status = rx_data[i].read();	//update rx table
						_rx_table[i].modified = 1;
						wait(SC_ZERO_TIME);
						rx_ack[i] = 0;
						_rx_counter++;
					}
				}
			}
		}
		
		void prc_update() {
			while (_rx_counter > 0) {					//if recieved data
				for (int i = 0; i < num_of_robots; i++) {	//loop through rx table
					if (_rx_table[i].modified) {
						int data = -1;
						switch(_rx_table[i].status) {
							case 5:
								_main_table[i].prev_status = _main_table[i].status;
								_main_table[i].status = 1;
								break;
							case 6:
								_main_table[i].prev_status = _main_table[i].status;
								_main_table[i].status = 0;
								break;
							case 7:
							case 8:
								if (_main_table[i].status != 3) {
									_main_table[i].prev_status = _main_table[i].status;
								}
								_main_table[i].status = 3;
								_main_table[i].speed = 0;
								_robots[i].speed = 0;
								break;
							case 9:
								_main_table[i].status = _main_table[i].prev_status;
								break;
							case 10:
								if (_fifo_data_index[i] != -1) {
									for (int o = 0; o < 80; o++) {
										if (_fifo_data[i][o] == -1) {
											fifo_data[i].nb_read(data);
											if (data == -1) {
												break;
											}
											_fifo_data[i][o] = data;
											_fifo_data[i][o+1] = -1;
										}
									}
								}
								else {
									for (int o = 0; o < 80; o++) {
										fifo_data[i].nb_read(data);
										_fifo_data[i][o] = data;
										if (_fifo_data[i][o] == -1) {
											break;
										}
									}
									_fifo_data_index[i] = 0;
								}
								break;
							case 11:
								for (int o = 0; o < 23; o++) {
									fifo_data[i].nb_read(data);
									_robot_path[i][o] = data;	//store robot path data locally
									if (_robot_path[i][o] == -1) {
										break;
									}
								}
								_main_table[i].current_grid = _robot_path[i][0];
								_main_table[i].next_grid = _robot_path[i][1];
								for (int x = 0; x < map_size_x; x++) {
									for (int y = 0; y < map_size_y; y++) {
										//store the map xy coordinate of the grids
										if (_main_table[i].current_grid == _map_data[x][y]) {
											_main_table[i].current_grid_map_x = x;
											_main_table[i].current_grid_map_y = y;
										}
										if (_main_table[i].next_grid == _map_data[x][y]) {
											_main_table[i].next_grid_map_x = x;
											_main_table[i].next_grid_map_y = y;
										}
									}
								}
								_main_table[i].status = 0;
								_main_table[i].prev_status = 3;
								break;
							default:
								break;
						}
						_rx_counter--;
						_rx_table[i].modified = 0;
					}
				}
			}
			
			
			for (int i = 0; i < num_of_obstacles; i++) {
				bool obstacle_moved = obstacle_move(i);
				switch (_obstacles[i].status) {
					case 0:								//STATE: RESUME
						if (obstacle_moved) {
							_obstacles[i].status = 2;	//update status to crossed
						}
						break;
					case 2:								//STATE: CROSSED
						if (obstacle_moved) {
							_obstacles[i].status = 0;	//update status to resume
						}
						break;
					default:
						break;
				}
			}
			for (int i = 0; i < num_of_robots; i++) {
				//SPEED UPDATES
				if (_clock_count % 10 == 0) {			//speed updates every 0.1 s
					if (_fifo_data_index[i] != -1) {	//if there is still speed data from fifo
						if (_fifo_data[i][_fifo_data_index[i]] == 2) {
							_robots[i].speed += 100;	//increase speed by 100 mm/s
							_main_table[i].speed += 50;
							_fifo_data_index[i]++;
						}
						else if (_fifo_data[i][_fifo_data_index[i]] == 1) {
							_robots[i].speed -= 50;		//decrease speed by 50 mm/s
							_main_table[i].speed -= 50;
							_fifo_data_index[i]++;
						}
						else {
							_fifo_data_index[i] = -1;	//end of fifo speed data
						}
						
						if (_fifo_data_index[i] != -1) {
							cout << "TIME: " << sc_time_stamp() << " | "
								 << "Robot_" << (i+1) << " speed is now " << _robots[i].speed << " mm/s" << endl;
						}
					}
				}
				
				//POSITION UPDATES
				bool robot_moved = robot_move(i);
				switch (_main_table[i].status) {
					case 0:								//STATE: RESUME
						if (robot_moved) {
							if (_robots[i].position_x <= grid_size/10 ||
							_robots[i].position_x >= grid_size - (grid_size/10) ||
							_robots[i].position_y <= grid_size/10 ||
							_robots[i].position_y >= grid_size - (grid_size/10)) {
								_tx_table[i].status = 2;
								_tx_table[i].modified = true;
								_tx_counter++;
							}
						}
						else {
							_main_table[i].prev_status = _main_table[i].status;
							_main_table[i].status = 3;		//update status to stopped
							_main_table[i].speed = 0;
							_robots[i].speed = 0;
							_fifo_data_index[i] = -1;
							_tx_table[i].status = 0;
							_tx_table[i].modified = true;
							_tx_counter++;
						}
						break;
					case 1:								//STATE: CROSSING
						if (robot_moved) {
							if (_main_table[i].modified) {
								_main_table[i].prev_status = _main_table[i].status;
								_main_table[i].modified = 0;
								_main_table[i].status = 2;	//update status to crossed
								_tx_table[i].status = 4;
								_tx_table[i].modified = true;
								_tx_counter++;
							}
						}
						else {
							_main_table[i].prev_status = _main_table[i].status;
							_main_table[i].status = 3;		//update status to stopped
							_main_table[i].speed = 0;
							_robots[i].speed = 0;
							_fifo_data_index[i] = -1;
							_tx_table[i].status = 0;
							_tx_table[i].modified = true;
							_tx_counter++;
						}
						break;
					case 2:								//STATE: CROSSED
						if (robot_moved) {
							if (_robots[i].position_x <= (grid_size/2 + ROBOT_SPEED_MAX) &&
								_robots[i].position_x >= (grid_size/2 - ROBOT_SPEED_MAX) &&
								_robots[i].position_y <= (grid_size/2 + ROBOT_SPEED_MAX) &&
								_robots[i].position_y >= (grid_size/2 - ROBOT_SPEED_MAX)) {
									_main_table[i].prev_status = _main_table[i].status;
									_main_table[i].status = 0;	//update status to resume
							}
						}
						else {
							_main_table[i].prev_status = _main_table[i].status;
							_main_table[i].status = 3;			//update status to stopped
							_main_table[i].speed = 0;
							_robots[i].speed = 0;
							_fifo_data_index[i] = -1;
							_tx_table[i].status = 0;
							_tx_table[i].modified = true;
							_tx_counter++;
						}
						break;
					case 3:								//STATE: STOPPED
						if (robot_moved) {
							_tx_table[i].status = 1;
							_tx_table[i].modified = true;
							_tx_counter++;
						}
						else {
							_fifo_data_index[i] = -1;
						}
						break;
					default:
						break;
				}
			}

			if (_tx_counter > 0) {
				tx_signal.notify(SC_ZERO_TIME);
				cout << endl;
				print_stat();

			}

			_clock_count++;
		}
		
		bool robot_move(int robot) {
			int obstacle;
			for (obstacle = 0; obstacle < num_of_obstacles; obstacle++) {
				if (_obstacles[obstacle].current_grid == _main_table[robot].next_grid ||
					_obstacles[obstacle].current_grid == _main_table[robot].current_grid) {
					break;
				}
			}

			if (_main_table[robot].status != 2) {		//if robot is not CROSSED, we need to move towards the middle, regardles of next grid
				//MOVE LEFT
				if (_main_table[robot].next_grid_map_x < _main_table[robot].current_grid_map_x &&
					obstacle == num_of_obstacles) {
					//check if robot is about to cross grids
					if (_robots[robot].position_x - _robots[robot].speed < 0) {
							bool grid_updated = table_update_grid(robot);
							if (grid_updated) {
								_robots[robot].position_x -= _robots[robot].speed;
								_robots[robot].position_x += grid_size;
								return true;								//return true if grid is not occupied and grid updated
							}
							else {
								return false;								//return false if grid not updated (end of path reached)
							}
					}
					else {
						//move the robot if robot is staying inside grid
						_robots[robot].position_x -= _robots[robot].speed;
						return true;
					}
				}
				//MOVE RIGHT
				else if (_main_table[robot].next_grid_map_x > _main_table[robot].current_grid_map_x &&
						 obstacle == num_of_obstacles) {
					//check if robot is about to cross grids
					if (_robots[robot].position_x + _robots[robot].speed > grid_size) {
						bool grid_updated = table_update_grid(robot);
						if (grid_updated) {
							_robots[robot].position_x += _robots[robot].speed;
							_robots[robot].position_x -= grid_size;
							return true;								//return true if grid is not occupied and grid updated
						}
						else {
							return false;								//return false if grid not updated (end of path reached)
						}
					}		
					else {
						//move the robot if robot is staying inside grid
						_robots[robot].position_x += _robots[robot].speed;
						return true;
					}		
				}
				//MOVE DOWN
				else if (_main_table[robot].next_grid_map_y < _main_table[robot].current_grid_map_y &&
						 obstacle == num_of_obstacles) {
					//check if robot is about to cross grids
					if (_robots[robot].position_y - _robots[robot].speed < 0) {
						bool grid_updated = table_update_grid(robot);
						if (grid_updated) {
							_robots[robot].position_y -= _robots[robot].speed;
							_robots[robot].position_y += grid_size;
							return true;								//return true if grid is not occupied and grid updated
						}
						else {
							return false;								//return false if grid not updated (end of path reached)
						}
					}
					else {
						//move the robot if robot is staying inside grid
						_robots[robot].position_y -= _robots[robot].speed;
						return true;
					}
				}
				//MOVE UP
				else if (_main_table[robot].next_grid_map_y > _main_table[robot].current_grid_map_y &&
						 obstacle == num_of_obstacles) {
					//check if robot is about to cross grids
					if (_robots[robot].position_y + _robots[robot].speed > grid_size) {
						bool grid_updated = table_update_grid(robot);
						if (grid_updated) {
							_robots[robot].position_y += _robots[robot].speed;
							_robots[robot].position_y -= grid_size;
							return true;								//return true if grid is not occupied and grid updated
						}
						else {
							return false;								//return false if grid not updated (end of path reached)
						}
					}
					else {
						//move the robot if robot is staying inside grid
						_robots[robot].position_y += _robots[robot].speed;
						return true;
					}
				}
				//DONT MOVE (robot has reached end of path) or (obstacle detected)
				else {
					return false;
				}
			}
			else {
				if (obstacle == num_of_obstacles) {
					if (_robots[robot].position_x < (grid_size/2 - ROBOT_SPEED_MAX)) {
						_robots[robot].position_x += ROBOT_SPEED_MAX;
					}
					else if (_robots[robot].position_x > (grid_size/2 + ROBOT_SPEED_MAX)) {
						_robots[robot].position_x -= ROBOT_SPEED_MAX;
					}
					else if (_robots[robot].position_y < (grid_size/2 - ROBOT_SPEED_MAX)) {
						_robots[robot].position_y += ROBOT_SPEED_MAX;
					}
					else if (_robots[robot].position_y > (grid_size/2 + ROBOT_SPEED_MAX)) {
						_robots[robot].position_y -= ROBOT_SPEED_MAX;
					}
					if (_main_table[robot].next_grid == -1 &&
						_robots[robot].position_x <= (grid_size/2 + ROBOT_SPEED_MAX) &&
						_robots[robot].position_x >= (grid_size/2 - ROBOT_SPEED_MAX) &&
						_robots[robot].position_y <= (grid_size/2 + ROBOT_SPEED_MAX) &&
						_robots[robot].position_y >= (grid_size/2 - ROBOT_SPEED_MAX)) {	//Edge case for when robot reaches last grid in path
						return false;
					}
					else {
						return true;
					}
				}
				else {
					return false;
				}
			}
		}
		
		bool table_update_grid(int robot) {
			int new_next_grid = -1;
			//search for next grid in path
			for (int i = 0; i < 23; i++) {
				if (_robot_path[robot][i] == _main_table[robot].next_grid) {
					new_next_grid = _robot_path[robot][i+1];
					break;
				}
			}
			
			//write new grids into table (only if robot hasn't reached end of path)
			if (new_next_grid != -1) {
				_main_table[robot].current_grid = _main_table[robot].next_grid;
				_main_table[robot].next_grid = new_next_grid;
				for (int x = 0; x < map_size_x; x++) {
					for (int y = 0; y < map_size_y; y++) {
						//store the map xy coordinate of the grids
						if (_main_table[robot].current_grid == _map_data[x][y]) {
							_main_table[robot].current_grid_map_x = x;
							_main_table[robot].current_grid_map_y = y;
						}
						if (_main_table[robot].next_grid == _map_data[x][y]) {
							_main_table[robot].next_grid_map_x = x;
							_main_table[robot].next_grid_map_y = y;
						}
					}
				}
				_main_table[robot].modified = true;
				return true;
			}
			else {
				_main_table[robot].current_grid_map_x = _main_table[robot].next_grid_map_x;
				_main_table[robot].current_grid_map_y = _main_table[robot].next_grid_map_y;
				_main_table[robot].current_grid = _main_table[robot].next_grid;
				_main_table[robot].next_grid = -1;
				if (_main_table[robot].status == 1) {		//Edge case for when robot reaches last grid in path
					_main_table[robot].modified = 1;
					return true;
				}
				else {
					return false;
				}
			}
		}

		bool obstacle_move(int obstacle) {
			if (_obstacles[obstacle].status != 2) {		//if obstacle is not CROSSED, we need to move towards the middle, regardles of next grid
				//MOVE LEFT
				if (_obstacles[obstacle].next_grid_map_x < _obstacles[obstacle].current_grid_map_x) {
					//check if obstacle is about to cross grids
					if (_obstacles[obstacle].position_x - _obstacles[obstacle].speed < 0) {
						//move the obstacle and update the grid if crossing
						obstacle_update_grid(obstacle);
						_obstacles[obstacle].position_x -= _obstacles[obstacle].speed;
						_obstacles[obstacle].position_x += grid_size;
						return true;
					}
					else {
						//move the obstalce if obstacle is staying inside grid
						_obstacles[obstacle].position_x -= _obstacles[obstacle].speed;
						return false;
					}
				}
				//MOVE RIGHT
				else if (_obstacles[obstacle].next_grid_map_x > _obstacles[obstacle].current_grid_map_x) {
					//check if obstacle is about to cross grids
					if (_obstacles[obstacle].position_x + _obstacles[obstacle].speed > grid_size) {
						//move the obstacle and update the grid if crossing
						obstacle_update_grid(obstacle);
						_obstacles[obstacle].position_x += _obstacles[obstacle].speed;
						_obstacles[obstacle].position_x -= grid_size;
						return true;
					}
					else {
						//move the obstalce if obstacle is staying inside grid
						_obstacles[obstacle].position_x += _obstacles[obstacle].speed;
						return false;
					}
				}
				//MOVE DOWN
				else if (_obstacles[obstacle].next_grid_map_y < _obstacles[obstacle].current_grid_map_y) {
					//check if obstacle is about to cross grids
					if (_obstacles[obstacle].position_y - _obstacles[obstacle].speed < 0) {
						//move the obstacle and update the grid if crossing
						obstacle_update_grid(obstacle);
						_obstacles[obstacle].position_y -= _obstacles[obstacle].speed;
						_obstacles[obstacle].position_y += grid_size;
						return true;
					}
					else {
						//move the obstalce if obstacle is staying inside grid
						_obstacles[obstacle].position_y -= _obstacles[obstacle].speed;
						return false;
					}
				}
				//MOVE UP
				else if (_obstacles[obstacle].next_grid_map_y > _obstacles[obstacle].current_grid_map_y) {
					//check if obstacle is about to cross grids
					if (_obstacles[obstacle].position_y + _obstacles[obstacle].speed > grid_size) {
						//move the obstacle and update the grid if crossing
						obstacle_update_grid(obstacle);
						_obstacles[obstacle].position_y += _obstacles[obstacle].speed;
						_obstacles[obstacle].position_y -= grid_size;
						return true;
					}
					else {
						//move the obstalce if obstacle is staying inside grid
						_obstacles[obstacle].position_y += _obstacles[obstacle].speed;
						return false;
					}
				}
				//DONT MOVE
				else {
					return false;
				}
			}
			else {
				if (_obstacles[obstacle].position_x < grid_size/2) {
					_obstacles[obstacle].position_x += _obstacles[obstacle].speed;
				}
				else if (_obstacles[obstacle].position_x > grid_size/2) {
					_obstacles[obstacle].position_x -= _obstacles[obstacle].speed;
				}
				else if (_obstacles[obstacle].position_y < grid_size/2) {
					_obstacles[obstacle].position_y += _obstacles[obstacle].speed;
				}
				else if (_obstacles[obstacle].position_y > grid_size/2) {
					_obstacles[obstacle].position_y -= _obstacles[obstacle].speed;
				}
				if (_obstacles[obstacle].position_x == grid_size/2 &&
					_obstacles[obstacle].position_y == grid_size/2) {
					return true;
				}
				else {
					return false;
				}
			}
		}
		
		void obstacle_update_grid(int obstacle) {
			int new_next_grid = -1;
			//search for next grid in path
			for (int i = 0; i < 23; i++) {
				if (_obstacles[obstacle].path[i] == _obstacles[obstacle].next_grid) {
					new_next_grid = _obstacles[obstacle].path[i+1];
					break;
				}
			}

			_obstacles[obstacle].current_grid = _obstacles[obstacle].next_grid;
			_obstacles[obstacle].next_grid = new_next_grid;
			for (int x = 0; x < map_size_x; x++) {
				for (int y = 0; y < map_size_y; y++) {
					//store the map xy coordinate of the grids
					if (_obstacles[obstacle].current_grid == _map_data[x][y]) {
						_obstacles[obstacle].current_grid_map_x = x;
						_obstacles[obstacle].current_grid_map_y = y;
					}
					if (_obstacles[obstacle].next_grid == _map_data[x][y]) {
						_obstacles[obstacle].next_grid_map_x = x;
						_obstacles[obstacle].next_grid_map_y = y;
					}
				}
			}
		}

		void print_stat() {
			for (int i = 0; i < num_of_robots; i++) {
				cout << "Robot " << i+1 << " Current Grid: " 
						<< _main_table[i].current_grid
						<< " | Next Grid: "
						<< _main_table[i].next_grid
						<< " | Position in grid: ("
						<< _robots[i].position_x << ", "
						<< _robots[i].position_y << ")"
						<< " | Speed: "
						<< _robots[i].speed
						<< endl;
			}
			for (int i = 0; i < num_of_obstacles; i++) {
				cout << "Obstacle " << i+1 << " Current Grid: " 
						<< _obstacles[i].current_grid
						<< " | Next Grid: "
						<< _obstacles[i].next_grid
						<< " | Position in grid: ("
						<< _obstacles[i].position_x << ", "
						<< _obstacles[i].position_y << ")"
						<< endl;
			}
		}

};

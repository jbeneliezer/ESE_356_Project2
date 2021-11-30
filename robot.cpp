#include <systemc.h>

static const char* status_names[] =
	{
		"STOPPED1", "RESTART", "CROSSING", "STOPPED2", "CROSSED",
		"OK1", "OK2", "STOP1", "STOP2", "RESUME", "SPEED", "PATH"
	};

class robot:public sc_module {
	public:
		//PORTS
		sc_in<bool> clock;
		
		sc_in<bool> tx_ack_s;
		sc_out<bool> tx_flag_s;
		sc_out<sc_uint<16> > tx_data_s;
		sc_out<bool> rx_ack_s;
		sc_in<bool> rx_flag_s;
		sc_in<sc_uint<16> > rx_data_s;
		
		sc_in<bool> tx_ack_p;
		sc_out<bool> tx_flag_p;
		sc_out<sc_uint<16> > tx_data_p;
		sc_out<bool> rx_ack_p;
		sc_in<bool> rx_flag_p;
		sc_in<sc_uint<16> > rx_data_p;
		
		//CONSTRUCTOR
		SC_HAS_PROCESS(robot);
		
		robot(sc_module_name name):sc_module(name) {
			SC_METHOD(prc_update);
			sensitive << clock.pos() << rx_flag_s << rx_flag_p;
			
			SC_THREAD(prc_tx_s);
			sensitive << tx_ack_s.pos();
			
			SC_THREAD(prc_rx_s);
			sensitive << rx_flag_s.pos();
			
			SC_THREAD(prc_tx_p);
			sensitive << tx_ack_p.pos();
			
			SC_THREAD(prc_rx_p);
			sensitive << rx_flag_p.pos();

			_tx_table_s.modified = 0;
			_rx_table_s.modified = 0;
			_tx_table_p.modified = 0;
			_rx_table_p.modified = 0;
		}

	private:
		//LOCAL VAR
		typedef struct Robot_Status {	//NOTE: Used for rx and tx tables
			int status;			//navigation status of robot
			bool modified;		//whether status has been modified
		}Robot_Status;
		
		Robot_Status _tx_table_s;
		Robot_Status _rx_table_s;
		Robot_Status _tx_table_p;
		Robot_Status _rx_table_p;
		
		sc_event tx_signal_s;
		sc_event tx_signal_p;
		
		//PROCESS
		void prc_rx_s() {
			while(1) {
				wait();
				rx_ack_s = 1;								//send ack bit
				_rx_table_s.status = rx_data_s.read();		//update rx table
				_rx_table_s.modified = 1;
				cout << "Time " << sc_time_stamp() << " | "
					 <<	name() << " recieved from server: " << status_names[rx_data_s.read()] << endl;
				wait(SC_ZERO_TIME);
				rx_ack_s = 0;
			}
		}
		
		void prc_rx_p() {
			while(1) {
				wait();
				rx_ack_p = 1;								//send ack bit
				_rx_table_p.status = rx_data_p.read();		//update rx table
				_rx_table_p.modified = 1;
				cout << "Time " << sc_time_stamp() << " | "
					 <<	name() << " recieved from processing: " << status_names[rx_data_p.read()] << endl;
				wait(SC_ZERO_TIME);
				rx_ack_p = 0;
			}
		}
		
		void prc_tx_s() {
			while(1) {
				wait(tx_signal_s);
				tx_flag_s = 1;						//set tx flag
				tx_data_s = _tx_table_s.status;		//write data to tx channel
				_tx_table_s.modified = 0;
				wait();								//wait for ack bit from server
				if (tx_ack_s != 1) {				//if not acknowledge from the server
					_tx_table_p.status = 7;			//send STOP1 signal to processing
					_tx_table_p.modified = 1;
					_tx_table_s.status = 3;			//send STOPPED2 signal to server
					_tx_table_s.modified = 0;
				}
				tx_flag_s = 0;						//clear tx flag
				cout << "Time " << sc_time_stamp() << " | "
					 <<	name() << " sent to server: " << status_names[_tx_table_s.status] << endl;
				wait(SC_ZERO_TIME);
			}
		}
		
		void prc_tx_p() {
			while(1) {
				wait(tx_signal_p);
				tx_flag_p = 1;						//set tx flag
				tx_data_p = _tx_table_p.status;		//write data to tx channel
				_tx_table_p.modified = 0;
				wait();								//wait for ack bit from processing
				tx_flag_p = 0;						//clear tx flag
				cout << "Time " << sc_time_stamp() << " | "
					 <<	name() << " sent to processing: " << status_names[_tx_table_p.status] << endl;
				wait(SC_ZERO_TIME);
			}
		}
		
		void prc_update() {
			if (_rx_table_s.modified) {
				_tx_table_p.status = _rx_table_s.status;
				_tx_table_p.modified = 1;
				_rx_table_s.modified = 0;
			}
			if (_rx_table_p.modified) {
				_tx_table_s.status = _rx_table_p.status;
				_tx_table_s.modified = 1;
				_rx_table_p.modified = 0;
			}

			if (_tx_table_s.modified) {
				tx_signal_s.notify(SC_ZERO_TIME);
			}
			if (_tx_table_p.modified) {
				tx_signal_p.notify(SC_ZERO_TIME);
			}
		}
};

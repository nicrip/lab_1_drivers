/******************************************************************************************
* Simple pololu commands via serial to Arduino
* Nick Rypkema
* August 2014
******************************************************************************************/

/******************************************************************************************
* Modifying it to be the driver for the Pololu over LCM - minor other mods.
* Sam Claassens
* July 2015
******************************************************************************************/

#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <lcm/lcm.h>
#include <vector>

// Include the LCM types as well
#include "lcmtypes/njord_servogroup_set_t.h"
#include "lcmtypes/njord_electronics_status_t.h"

using namespace std;
using namespace LibSerial;

/* GLOBALS */
SerialStream serial_port ;
stringstream datawrite;
stringstream read_stream;
std::vector<double> read_vect;


// This function returns queued data on port, returns empty string if there is no data
// does not block
string read(SerialStream& serial_port)
{
    string result;
    while( serial_port.rdbuf()->in_avail() )
    {
        char next_byte;
        serial_port.get(next_byte);
        result += next_byte;
    }
    return result;
}

// This function blocks for upto timeout usec and waits for data
// to be available on the port.
std::string readBlocking(SerialStream& serial_port, int timeout)
{
    while( serial_port.rdbuf()->in_avail() == 0 && timeout > 0 )
    {
        timeout -= 100;
        usleep(100);
    }
    if(timeout < 0)
        return std::string();
    return read(serial_port);
}

void write(SerialStream& serial_port, const std::string& data)
{
    serial_port.write(data.c_str(), data.size());
}

void UpdateAllServos(const int32_t servoValues[])
{
	datawrite << servoValues[0] << ',' << servoValues[1] << ',' << servoValues[2] << ',' << servoValues[3] << ',' << servoValues[4] << ',' << servoValues[5] << '\n';
	write(serial_port, datawrite.str());
	datawrite.str("");
	datawrite.clear();
}


void LCMHandler_servogroup_set_t_MessageReceived(const lcm_recv_buf_t *rbuf, const char *chan, const njord_servogroup_set_t *msg, void *user)
{
	cout << "Received servo group update, setting to [" << msg->servo_setpoints[0] << ", " << msg->servo_setpoints[1] << ", " << msg->servo_setpoints[2] << ", " << msg->servo_setpoints[3] << ", " << msg->servo_setpoints[4] << ", " << msg->servo_setpoints[5] << "]." << endl;
	UpdateAllServos(msg->servo_setpoints);
}


int main(int argc, char** argv)
{
    //
    // Open the serial port.
    //
    serial_port.Open( "/dev/ttymxc3" ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Set the baud rate of the serial port.
    //
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the baud rate." <<  std::endl;
        exit(1) ;
    }
    //
    // Set the number of data bits.
    //
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the character size." << std::endl;
        exit(1) ;
    }
    //
    // Disable parity.
    //
    serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not disable the parity." << std::endl;
        exit(1) ;
    }
    //
    // Set the number of stop bits.
    //
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the number of stop bits." << std::endl;
        exit(1) ;
    }
    //
    // Turn off hardware flow control.
    //
    serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not use hardware flow control." << std::endl;
        exit(1) ;
    }

	//
	// Initialize the LCM components
	//
	lcm_t* lcm = lcm_create(NULL);
	if(!lcm) {
		cerr << "Error: Could not initialize LCM." << endl;
		exit(1);
	}
	// Create the handlers
	njord_servogroup_set_t_subscribe(lcm, "NJORD_V2V_VEH1_SET_SERVOGROUP", &LCMHandler_servogroup_set_t_MessageReceived, NULL);

   cout << "Successfully initialized, going to in the main wait loop..." << endl;
//	unsigned int input;
//	cout << "Input servo value between 500 and 5500, or input 0 to exit" << endl;
//	cin >> input;

  // clear read stream
  read_stream.clear();
  read_stream.str("");

	while (1) {
		lcm_handle(lcm);
   		 // if data available on serial port, read in characters
		while( serial_port.rdbuf()->in_avail() ) {
			char next_byte;
			serial_port.get(next_byte);
			read_stream << next_byte;
			if (next_byte == '\n') {
				while (read_stream.good()) {
					string sub_str;
					getline(read_stream, sub_str, ',');
					read_vect.push_back(atof(sub_str.c_str()));
				}
				if (read_vect.size() == 5) {
					njord_electronics_status_t output_msg;
					output_msg.pressure = read_vect[0];
					output_msg.temperature = read_vect[1];
					output_msg.current_value = (int)read_vect[2];
					output_msg.voltage_value = (int)read_vect[3];
					output_msg.humidity_value = (int)read_vect[4];
					njord_electronics_status_t_publish(lcm, "NJORD_ELECTRONICS_STATUS", &output_msg);
				}
				read_stream.clear();
				read_stream.str("");
				read_vect.clear();
			}
    		}
	}

	lcm_destroy(lcm);
}

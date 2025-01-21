#ifndef MESKO_HARDWARE_ARDUINO_COMMS_HPP
#define MESKO_HARDWARE_ARDUINO_COMMS_HPP

#include <cstring>
#include <sstream>
#include <string>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);
    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }



  std::vector<double> read_joint_values()
    {
        std::string response = send_msg("e\r");
        // std::cout<<response<<std::endl;
        std::vector<double> val;

      std::string delimiter = " ";
      size_t del_pos = 0;
      std::string token;

      // Vector to store all extracted joint values as strings
      std::vector<std::string> tokens;

      // Extract tokens until the end of the string
      while ((del_pos = response.find(delimiter)) != std::string::npos) {
          token = response.substr(0, del_pos);
          tokens.push_back(token);
          response.erase(0, del_pos + delimiter.length());
      }
      tokens.push_back(response);  // Last token after the loop ends

      // Ensure we have exactly 6 joint values
      if (tokens.size() == 6) 
       {
          double val_1 = std::atof(tokens[0].c_str());
          double val_2 = std::atof(tokens[1].c_str());
          double val_3 = std::atof(tokens[2].c_str());
          double val_4 = std::atof(tokens[3].c_str());
          double val_5 = std::atof(tokens[4].c_str());
          double val_6 = std::atof(tokens[5].c_str());
          val.push_back(val_1);
          val.push_back(val_2);
          val.push_back(val_3);
          val.push_back(val_4);
          val.push_back(val_5);
          val.push_back(val_6);
          return val;
        } 

    }

  void set_joints_values(float val_1, float val_2, float val_3, float val_4, float val_5, float val_6)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << " " << val_3 << " " << val_4 << " " << val_5 << " " << val_6 << "\r";
    send_msg(ss.str());
  }


private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // 

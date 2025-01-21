#ifndef MESKO_HARDWARE_ARDUINO_COMMS_HPP
#define MESKO_HARDWARE_ARDUINO_COMMS_HPP

#include <cstring>
#include <sstream>
#include <string>
// #include <cstdlib>
#include <iostream>

#include <string>
#include <vector>
#include <arpa/inet.h> // For socket functions
#include <unistd.h>    // For close()



class ESP32UDPComms
{

public:

  ESP32UDPComms() = default;

  void connect(const std::string &esp32_ip, int udp_port_1,int udp_port_2)
  {  
        esp32_ip_ = esp32_ip;
        udp_port_1 = udp_port_1;
        udp_port_2 = udp_port_2;

        // Create a UDP socket
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0)
        {
            std::cerr << "Error creating socket\n";
            exit(EXIT_FAILURE);
        }

        // Configure the ESP32 address
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(udp_port_1);
        if (inet_pton(AF_INET, esp32_ip.c_str(), &server_addr_.sin_addr) <= 0)
        {
            std::cerr << "Invalid ESP32 IP address\n";
            exit(EXIT_FAILURE);
        }

        memset(&server_addr_1, 0, sizeof(server_addr_1));
        server_addr_1.sin_family = AF_INET;
        server_addr_1.sin_port = htons(udp_port_2);
        if (inet_pton(AF_INET, esp32_ip.c_str(), &server_addr_1.sin_addr) <= 0)
        {
            std::cerr << "Invalid ESP32 IP address\n";
            exit(EXIT_FAILURE);
        }
        
        
        memset(&server_addr_2, 0, sizeof(server_addr_2));
        server_addr_2.sin_family = AF_INET;
        server_addr_2.sin_port = htons(udp_port_2);
        server_addr_2.sin_addr.s_addr = INADDR_ANY; 
        if (bind(sock_, (struct sockaddr *)&server_addr_2, sizeof(server_addr_2)) < 0) {
        std::cerr << "Error binding socket: " << strerror(errno) << "\n";
        close(sock_);
            }
            
        std::cout << "Connected to ESP32 at " << esp32_ip_ << ":" << udp_port_1 << std::endl;
        std::cout << "Connected to ESP32 at " << esp32_ip_ << ":" << udp_port_2 << std::endl;
        return ;
  }

    void disconnect()
    {
        close(sock_);
        std::cout << "Disconnected" << std::endl;
    }

    bool connected() const
    {
        return sock_ >= 0;
    }


    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        // Send the message to the ESP32
        ssize_t sent_bytes = sendto(sock_, msg_to_send.c_str(), msg_to_send.size(), 0,
                                    (struct sockaddr *)&server_addr_, sizeof(server_addr_));
        if (sent_bytes < 0)
        {
            std::cerr << "Error sending message\n";
            return "";
        }

        std::string response="yes";
        return response;
    }


   std::vector<double> read_joint_values(const std::string &msg_to_send, bool print_output = false)
    {
    std::vector<double> val;
    ssize_t sent_bytes = sendto(sock_, msg_to_send.c_str(), msg_to_send.size(), 0,
                                    (struct sockaddr *)&server_addr_1, sizeof(server_addr_1));
    if (sent_bytes < 0)
        {
            std::cerr << "Error sending message\n";
        }            
    char buffer[1024];
    socklen_t addr_len = sizeof(server_addr_2);
    ssize_t recv_bytes = recvfrom(sock_, buffer, sizeof(buffer) - 1, 0,
                                  (struct sockaddr *)&server_addr_2, &addr_len);
    if (recv_bytes < 0) {
            std::cerr << "Error receiving data: " << strerror(errno) << "\n";
    
    } 
    buffer[recv_bytes] = '\0'; // Null-terminate received data
    int num_floats = recv_bytes / sizeof(float);
    float *received_data = reinterpret_cast<float *>(buffer);

    double val_1 = received_data[0];
    double val_2 = received_data[1];
    double val_3 = received_data[2];
    double val_4 = received_data[3];
    double val_5 = received_data[4];
    double val_6 = received_data[5];
    val.push_back(val_1);
    val.push_back(val_2);
    val.push_back(val_3);
    val.push_back(val_4);
    val.push_back(val_5);
    val.push_back(val_6);
    return val;

    }

    void set_joints_values(float val_1, float val_2, float val_3, float val_4, float val_5, float val_6)
    {
      std::stringstream ss;
      ss << "m " << val_1 << " " << val_2 << " " << val_3 << " " << val_4 << " " << val_5 << " " << val_6 << "\r";
      send_msg(ss.str());
    }


private:
    int sock_ = -1;
    struct sockaddr_in server_addr_;
    struct sockaddr_in server_addr_1;
    struct sockaddr_in server_addr_2;
    std::string esp32_ip_;
    int udp_port_1;
    int udp_port_2;

};

#endif // 

#include <vwbot_bringup/VwbotSerialHardware.h>
#include <vwbot_bringup/utils.h>
#include <eigen3/Eigen/Geometry> 
#include <iostream>
#include <vector>

using namespace vwpp;

VwbotSerialHardware::VwbotSerialHardware(std::string model_, std::string port_, int baud_, 
                                           int msg_length_,
                                           boost_serial_base::flow_control::type fc_type_,
                                           boost_serial_base::parity::type pa_type_, 
                                           boost_serial_base::stop_bits::type st_type_):
    model(model_),
    port(port_),
    baud(baud_),
    msg_length(msg_length_),
    fc_type(fc_type_),
    pa_type(pa_type_),
    st_type(st_type_)
{

    uint8_t tmp_msg_start[6] = {0xB5, 0x5B, 0x04, 0x01, 0x05, 0xBB};
    uint8_t tmp_msg_stop[6]  = {0xB5, 0x5B, 0x04, 0x02, 0x06, 0xBB};
    
    this->msg_start = tmp_msg_start;
    this->msg_stop  = tmp_msg_stop;

    this->boost_serial_communicator = new BoostSerialCommunicator(this->port, this->baud);

    if(model == "VWBOT_G1")
    {
        boost_serial_communicator->sendMessage(msg_stop);
        usleep(1000 * 1000);
        boost_serial_communicator->sendMessage(msg_start);
        usleep(1000 * 1000);
    }

    std::cout << "\033[32m" << "Start communicate with vwbot..."
              << "\033[0m" << std::endl;

}


VwbotSerialHardware::~VwbotSerialHardware()
{
    delete   this->boost_serial_communicator;
    delete[] this->msg_start;
    delete[] this->msg_stop;
}


int VwbotSerialHardware::sendMessage(VwbotSerialHardware::Velocity2D vel_)
{
    std::vector<uint8_t> vec_msg;
    vec_msg.push_back(0xB5);
    vec_msg.push_back(0x5B);
    vec_msg.push_back(11);

    int16_t vel_x = round_float(vel_.x*100);
    uint8_t vel_x_hbits = uint8_t(vel_x >> 8);
    uint8_t vel_x_lbits = uint8_t(vel_x & (0x00FF));
    vec_msg.push_back(vel_x_hbits);
    vec_msg.push_back(vel_x_lbits);

    int16_t vel_y = round_float(vel_.y*100);
    uint8_t vel_y_hbits = uint8_t(vel_y >> 8);
    uint8_t vel_y_lbits = uint8_t(vel_y & (0x00FF));
    vec_msg.push_back(vel_y_hbits);
    vec_msg.push_back(vel_y_lbits);

    int16_t vel_yaw = round_float(vel_.yaw*100);
    uint8_t vel_yaw_hbits = uint8_t(vel_yaw >> 8);
    uint8_t vel_yaw_lbits = uint8_t(vel_yaw & (0x00FF));
    vec_msg.push_back(vel_yaw_hbits);
    vec_msg.push_back(vel_yaw_lbits);

    // 校验和,只累加三轴速度值
    uint8_t checksum = (vel_x + vel_y + vel_yaw) % 256;
    vec_msg.push_back(checksum);

    vec_msg.push_back(0xBB);

    uint8_t* msg_buffer = new uint8_t[vec_msg.size()];
    if (!vec_msg.empty())
    {
        memcpy(msg_buffer, &vec_msg[0], vec_msg.size()*sizeof(uint8_t));
    }


    if (this->boost_serial_communicator->sendMessage(msg_buffer) == 1)
    {
        return 1;
    }
    else 
    {
        std::cerr << "\033[31m" << "Send message failed"
                  << "\033[0m"  << std::endl;
        return (-1);
    }

    
}

#include <legged_sirius_hw/rt_usb_interface.h>
#include "iostream"
#include <cstdlib>
#include <thread>
int actual_length_out = 0;
int actual_length_in = 0;

static uint32_t data_checksum(const uint32_t* data_to_check, uint32_t check_length)
{
    uint32_t t = 0;
    for(int i = 0; i < check_length; i++)
    {
        t = t ^ data_to_check[i];
    }

    return t;
}

Sirius_USB2CAN_Board::Sirius_USB2CAN_Board(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin,
                                           uint8_t _motors_epout):
                                           usb_vendor_id(vendor_id), usb_product_id(product_id),
                                           motors_endpoint_in(_motors_epin), motors_endpoint_out(_motors_epout){
    usb_cmd_u = new USB_Cmd_U();
    usb_data_u = new USB_Data_U ();
    control_cmd = new spi_command_t();
    control_data = new spi_data_t();
    usb_in_mutex = new std::mutex();
    usb_out_mutex = new std::mutex();

    time_last_out = std::chrono::steady_clock::now();
    time_now_out = std::chrono::steady_clock::now();
    
    time_last_in = std::chrono::steady_clock::now();
    time_now_in = std::chrono::steady_clock::now();
    out_zero_flag = 0;
    out_zero_count = 0;
    libusb_init(&ctx);
    transfer_out_motors = libusb_alloc_transfer(0);
    transfer_in_motors = libusb_alloc_transfer(0);
    device_handle = libusb_open_device_with_vid_pid(ctx, usb_vendor_id, usb_product_id);
    if(libusb_kernel_driver_active(device_handle, 0x00))
    {
        int success = libusb_detach_kernel_driver(device_handle, 0x00);
        if(success != 0)
        {
            std::cerr << "Detach Driver Failed!" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    int claim_interface = libusb_claim_interface(device_handle, 0x00);
    if(claim_interface != 0)
    {
        std::cerr << "Claim Driver Failed!" << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Claim USB Device OK!\n";
    }
}

Sirius_USB2CAN_Board::~Sirius_USB2CAN_Board() {

}

void Sirius_USB2CAN_Board::destroy_thread() {
    std::cout << "i am out1\n";
    delete  usb_cmd_u;
    delete  usb_data_u;
    delete  control_cmd;
    delete  control_data;
    delete  usb_in_mutex;
    delete  usb_out_mutex;
std::cout << "i am out2\n";
    libusb_free_transfer(transfer_in_motors);
    libusb_free_transfer(transfer_out_motors);
    libusb_release_interface(device_handle, 0);
    libusb_close(device_handle);
    libusb_exit(ctx);
    std::cout << "i am out3\n";
}



void Sirius_USB2CAN_Board::USB2CAN_SetBuffer(spi_command_t* _control_cmd, spi_data_t* _controller_data) {
    control_cmd = _control_cmd;
    control_data = _controller_data;
}

void Sirius_USB2CAN_Board::motor_epin_callback(struct libusb_transfer *_transfer) {
    if(_transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        std::cout << "Motor Ep81 IN Error! Transfer again!\n";

    }
    else if(_transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
        this->Deal_Usb_In_Data();
        time_now_in = std::chrono::steady_clock::now();
        std::chrono::duration<double,std::micro> time_used = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(time_now_in - time_last_in);
        // std::cout << "[Time Interval In]: " << time_used.count() << " us\n\n";
        time_last_in = time_now_in;
        libusb_submit_transfer(_transfer);
    }
}

void Sirius_USB2CAN_Board::motor_epout_callback(struct libusb_transfer *_transfer) {
    if(_transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        std::cout << "Motor Ep01 OUT Error! Transfer again!\n";

    }
    else if(_transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
        this->Deal_Usb_Out_Cmd();
//        time_now_out = std::chrono::steady_clock::now();
//        std::chrono::duration<double,std::micro> time_used = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(time_now_out - time_last_out);
//        std::cout << "[Time Interval Out]: " << time_used.count() << " us\n";
//        time_last_out = time_now_out;

        libusb_submit_transfer(_transfer);
    }
}

float last_one = 1000;

void Sirius_USB2CAN_Board::Deal_Usb_In_Data(){
    uint32_t t = data_checksum((uint32_t*)usb_data_u, usb_motors_in_check_length);
    if(usb_data_u->usb_data.checksum == t)
    {
        this->Unpack_In_Data();
    }
}

void Sirius_USB2CAN_Board::Deal_Usb_Out_Cmd(){
    this->Pack_Out_Cmd();
    usb_cmd_u->usb_cmd.checksum = data_checksum((uint32_t*)usb_cmd_u, usb_motors_out_check_length);
    //std::cout << usb_cmd_u->usb_cmd.leg_cmd[0].flags << " | " << usb_cmd_u->usb_cmd.leg_cmd[1].flags <<
    // " | " << usb_cmd_u->usb_cmd.leg_cmd[2].flags << std::endl;
    out_zero_count++;
    if (out_zero_count>20)
    {
        //out_zero_flag = true;
    }
    
}

void Sirius_USB2CAN_Board::USB2CAN_Start_Transfer_Ans() {
    libusb_fill_interrupt_transfer(transfer_out_motors,device_handle, motors_endpoint_out,
                                           usb_cmd_u->usb_cmd_buff, usb_motors_out_length, usb_motors_out_cbf_wrapper, this, 0);
    libusb_fill_interrupt_transfer(transfer_in_motors, device_handle, motors_endpoint_in,usb_data_u->usb_data_buff,
                                           usb_motors_in_length, usb_motors_in_cbf_wrapper, this, 0);
    libusb_submit_transfer(transfer_in_motors);
    libusb_submit_transfer(transfer_out_motors);
    if((!transfer_out_motors->status) & (!transfer_in_motors->status))
    {
        std::cout << "[Good] All endpoints start transfering!\n";
    }
    else
    {
        std::cout << "[Bad] Some endpoints not work\n";
        exit(EXIT_FAILURE);
    }
}

void Sirius_USB2CAN_Board::lock_in_mutex(){
    usb_in_mutex->lock();
}

void Sirius_USB2CAN_Board::lock_out_mutex() {
    usb_out_mutex->lock();
}

void Sirius_USB2CAN_Board::unlock_out_mutex() {
    usb_out_mutex->unlock();
}

void Sirius_USB2CAN_Board::unlock_in_mutex() {
    usb_in_mutex->unlock();
}

void Sirius_USB2CAN_Board::Unpack_In_Data() {
    /// convert ints to floats ///
    for(int i = 0; i < NUMBER_CHIPS; i++) // 0 1 2
    {
        for(int j = 0; j < NUMBER_CHIP_CMD; j++)
        {
            int motor_index = i*9+j; // 0~26
            int p_int = (usb_data_u->usb_data.chip_data[i].motor_data[j].packed_data[0] << 8) | (usb_data_u->usb_data.chip_data[i].motor_data[j].packed_data[1]);

            int v_int = (usb_data_u->usb_data.chip_data[i].motor_data[j].packed_data[2] << 4) | (usb_data_u->usb_data.chip_data[i].motor_data[j].packed_data[3] >> 4);
            int t_int = ((usb_data_u->usb_data.chip_data[i].motor_data[j].packed_data[3] & 0xF) << 8) | (usb_data_u->usb_data.chip_data[i].motor_data[j].packed_data[4]);
            float p_temp = uint_to_float(p_int, P_MIN, P_MAX, 16);
            float v_temp = uint_to_float(v_int, V_MIN, V_MAX, 12);
            float i_temp = uint_to_float(t_int, KI_MIN, KI_MAX, 12);
            int switch_index = j % 3; // (abad | hip | knee) * 9
            // 000 111 222 333 444 555 666 777 888
            int cd_index = motor_index / 3; // 0 ~ 26 -> 0 ~ 8
            switch(switch_index)
            {
                case 0:
                    control_data->q_abad[cd_index] = p_temp;
                    control_data->qd_abad[cd_index] = v_temp;
                    control_data->tau_abad[cd_index] = i_temp;
                    break;
                case 1:
                    control_data->q_hip[cd_index] = p_temp;
                    control_data->qd_hip[cd_index] = v_temp;
                    control_data->tau_hip[cd_index] = i_temp;
                    break;
                case 2:
                    control_data->q_knee[cd_index] = p_temp;
                    control_data->qd_knee[cd_index] = v_temp;
                    control_data->tau_knee[cd_index] = i_temp;
                    break;
                default:
                    break;
            }
            // std::cout << p_temp << std::endl;
        }
        // unpack data done, copy flg
        control_data->flags[i] = usb_data_u->usb_data.chip_data[i].flg_data;
    }

}

void Sirius_USB2CAN_Board::Pack_Out_Cmd() {
    time_last_out = std::chrono::steady_clock::now();
    // 10us mmkiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
    float p_des, v_des, t_des, kp_des, kd_des;
    for(int i = 0 ; i < NUMBER_CHIPS; i++)
    {
        for(int j = 0; j < NUMBER_CHIP_CMD; j++)
        {
            int motor_index = i*9+j; // 0~26
            int switch_index = j % 3; // (abad | hip | knee) * 9
            // 000 111 222 333 444 555 666 777 888
            int cd_index = motor_index / 3; // 0 ~ 26 -> 0 ~ 8
            switch(switch_index)
            {
                case 0:
                    p_des = control_cmd->q_des_abad[cd_index];
                    v_des = control_cmd->qd_des_abad[cd_index];
                    t_des = control_cmd->tau_abad_ff[cd_index];
                    kp_des = control_cmd->kp_abad[cd_index];
                    kd_des = control_cmd->kd_abad[cd_index];
                    break;
                case 1:
                    p_des = control_cmd->q_des_hip[cd_index];
                    v_des = control_cmd->qd_des_hip[cd_index];
                    t_des = control_cmd->tau_hip_ff[cd_index];
                    kp_des = control_cmd->kp_hip[cd_index];
                    kd_des = control_cmd->kd_hip[cd_index];
                    break;
                case 2:
                    p_des = control_cmd->q_des_knee[cd_index];
                    v_des = control_cmd->qd_des_knee[cd_index];
                    t_des = control_cmd->tau_knee_ff[cd_index];
                    kp_des = control_cmd->kp_knee[cd_index];
                    kd_des = control_cmd->kd_knee[cd_index];
                    break;
                default:
                    break;
            }
            p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
            v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
            kp_des = fminf(fmaxf(KP_MIN, kp_des), KP_MAX);
            kd_des = fminf(fmaxf(KD_MIN, kd_des), KD_MAX);
            t_des = fminf(fmaxf(KI_MIN, t_des), KI_MAX);
            /// convert floats to unsigned ints ///
            int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
            int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
            int kp_int = float_to_uint(kp_des, KP_MIN, KP_MAX, 12);
            int kd_int = float_to_uint(kd_des, KD_MIN, KD_MAX, 12);
            int t_int = float_to_uint(t_des, KI_MIN, KI_MAX, 12);
            usb_cmd_u->usb_cmd.chip_cmd[i].motor_cmd[j].packed_cmd[0] = p_int >> 8;
            usb_cmd_u->usb_cmd.chip_cmd[i].motor_cmd[j].packed_cmd[1] = p_int & 0xFF;
            usb_cmd_u->usb_cmd.chip_cmd[i].motor_cmd[j].packed_cmd[2] = v_int >> 4;
            usb_cmd_u->usb_cmd.chip_cmd[i].motor_cmd[j].packed_cmd[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
            usb_cmd_u->usb_cmd.chip_cmd[i].motor_cmd[j].packed_cmd[4] = kp_int & 0xFF;
            usb_cmd_u->usb_cmd.chip_cmd[i].motor_cmd[j].packed_cmd[5] = kd_int >> 4;
            usb_cmd_u->usb_cmd.chip_cmd[i].motor_cmd[j].packed_cmd[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
            usb_cmd_u->usb_cmd.chip_cmd[i].motor_cmd[j].packed_cmd[7] = t_int & 0xff;
        }
        usb_cmd_u->usb_cmd.chip_cmd[i].flg_cmd = control_cmd->flags[i];
    }
    time_now_out = std::chrono::steady_clock::now();
    std::chrono::duration<double,std::micro> time_used = std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(time_now_out - time_last_out);
    // std::cout << "[Time Interval Out]: " << time_used.count() << " us\n";
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void usb_motors_in_cbf_wrapper(struct libusb_transfer* _transfer)
{
    auto *temp = reinterpret_cast<Sirius_USB2CAN_Board*>(_transfer->user_data);
    temp->motor_epin_callback(_transfer);
}

void usb_motors_out_cbf_wrapper(struct libusb_transfer* _transfer)
{
    auto *temp = reinterpret_cast<Sirius_USB2CAN_Board*>(_transfer->user_data);
    temp->motor_epout_callback(_transfer);
}

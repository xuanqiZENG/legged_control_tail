/*
 * This file contains data structs of motors
 */

#ifndef USB_INTERFACE_H_
#define USB_INTERFACE_H_

#include <cstdint>
#include "libusb-1.0/libusb.h"
#include <chrono>
#include <mutex>
#include <cmath>

const uint8_t NUMBER_CHIPS = 3;
const uint16_t NUMBER_MOTORS = 27;
const uint16_t NUMBER_CHIP_CMD = 9;
const uint16_t usb_motors_in_length = 232;
const uint16_t usb_motors_out_length = 232;
const uint16_t usb_motors_in_check_length = usb_motors_in_length/4 - 1;
const uint16_t usb_motors_out_check_length = usb_motors_out_length/4 - 1;


const uint16_t sirius_usb2can_vendor_id = 0x1111;
const uint16_t sirius_usb2can_product_id = 0x2222;
const uint8_t  motors_ep_in = 0x81;
const uint8_t  motors_ep_out = 0x01;
//todo: Check the size of remote controllers

#define K_KNEE_OFFSET_POS 4.35f
#define P_MIN (-12.5f)
#define P_MAX 12.5f
#define V_MIN (-45.0f)
#define V_MAX 45.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define KI_MIN (-150.0f)
#define KI_MAX 150.0f

const float max_torque[3] = {24.f, 24.f, 26.f};
const float wimp_torque[3] = {6.f, 6.f, 6.f};
const float disabled_torque[3] = {0.f, 0.f, 0.f};
// only used for actual robot
const float abad_side_sign[4] = {1.f, 1.f, 1.f, 1.f};
const float hip_side_sign[4] = {-1.f, 1.f, 1.f, 1.f};
const float knee_side_sign[4] = {1.f, 1.f, 1.f, 1.f};
// only used for actual robot
const float abad_offset[4] = {-1.88849f, 0.f, 0.f, 0.f}; //

const float hip_offset[4] = {5.54456f, 0.f, 0.f, 0.f};
const float knee_offset[4] = {5.48505f, 0.f, 0.f, 0.f};
                              
typedef struct {
    float q_des_abad[9];
    float q_des_hip[9];
    float q_des_knee[9];
    float qd_des_abad[9];
    float qd_des_hip[9];
    float qd_des_knee[9];
    float kp_abad[9];
    float kp_hip[9];
    float kp_knee[9];
    float kd_abad[9];
    float kd_hip[9];
    float kd_knee[9];
    float tau_abad_ff[9];
    float tau_hip_ff[9];
    float tau_knee_ff[9];
    uint32_t flags[3];
} spi_command_t;

// 9 * 4 = 36
typedef struct {
    float q_abad[9];
    float q_hip[9];
    float q_knee[9];
    float qd_abad[9];
    float qd_hip[9];
    float qd_knee[9];
    float tau_abad[9];
    float tau_hip[9];
    float tau_knee[9];
    uint32_t flags[3];
} spi_data_t;

// each motor cmd 8 bytes
typedef struct Motor_Cmd{
    uint8_t packed_cmd[8];
} Motor_Cmd_T;

// each chip cmd 8*9+4 = 76;
typedef struct Chip_Cmd
{
    Motor_Cmd_T motor_cmd[NUMBER_CHIP_CMD];
    uint32_t flg_cmd;
} Chip_Cmd_T;

// 8 bytes
typedef struct Motor_Data{
    uint8_t packed_data[8];
} Motor_Data_T;

// 45 + 4 = 76;
typedef struct Chip_Data
{
    Motor_Data_T motor_data[NUMBER_CHIP_CMD];
    uint32_t flg_data;
} Chip_Data_T;

// 76 * 3 + 4 = 232 bytes
typedef struct USB_Cmd{
    Chip_Cmd_T chip_cmd[NUMBER_CHIPS];
    uint32_t checksum;
} USB_Cmd_T;

typedef union USB_CMD{
    USB_Cmd_T usb_cmd;
    uint8_t   usb_cmd_buff[usb_motors_out_length];
} USB_Cmd_U;

// 76 * 3 + 4 = 232 bytes
typedef struct USB_Data{
    Chip_Data_T chip_data[NUMBER_CHIPS];
    uint32_t checksum;
} USB_Data_T;

typedef union USB_DATA{
    USB_Data_T usb_data;
    uint8_t    usb_data_buff[usb_motors_in_length];
} USB_Data_U;

class Sirius_USB2CAN_Board{
public:
    Sirius_USB2CAN_Board(uint16_t vendor_id, uint16_t product_id, uint8_t _motors_epin, uint8_t _motors_epout);
    ~Sirius_USB2CAN_Board();
    // data union of this class is a temp buff, data checkok, memcpy to controll databuff.
    void USB2CAN_SetBuffer(spi_command_t* _control_cmd, spi_data_t* _controller_data);
    void USB2CAN_Start_Transfer_Ans();
    void motor_epin_callback(struct libusb_transfer* _transfer);
    void motor_epout_callback(struct libusb_transfer* _transfer);
    libusb_context*          ctx;
    void lock_in_mutex();
    void lock_out_mutex();
    void unlock_out_mutex();
    void unlock_in_mutex();
    void destroy_thread();
    bool                     out_zero_flag;
private:

    // for controller data protocals
    uint16_t                 usb_vendor_id;
    uint16_t                 usb_product_id;

    libusb_transfer*         transfer_in_motors{};
    libusb_transfer*         transfer_out_motors{};

    std::mutex*              usb_in_mutex;
    std::mutex*              usb_out_mutex;

    uint8_t                  motors_endpoint_in;
    uint8_t                  motors_endpoint_out;
    
    uint8_t                  out_zero_count;
    USB_Data_U*              usb_data_u;
    USB_Cmd_U*               usb_cmd_u;
    spi_command_t *          control_cmd;
    spi_data_t*              control_data;
    libusb_device_handle*    device_handle{};
    std::chrono::steady_clock::time_point time_last_in;
    std::chrono::steady_clock::time_point time_now_in;
    std::chrono::steady_clock::time_point time_last_out;
    std::chrono::steady_clock::time_point time_now_out;
    void Deal_Usb_In_Data();
    void Unpack_In_Data();
    void Deal_Usb_Out_Cmd();
    void Pack_Out_Cmd();
};

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void usb_motors_in_cbf_wrapper(struct libusb_transfer* _transfer);
void usb_motors_out_cbf_wrapper(struct libusb_transfer* _transfer);
void fresh_usb_cmd_from_controller();
#endif

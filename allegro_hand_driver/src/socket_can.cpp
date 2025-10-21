/*======================*/
/*       Includes       */
/*======================*/
// System headers
#include <stdio.h>
#include <errno.h>
#ifndef _WIN32
  #include <inttypes.h>
  #include <pthread.h>
  #include <syslog.h>
  #include <unistd.h>
  #include <fcntl.h>
#else
  #include <windows.h>
#endif
#include <malloc.h>
#include <assert.h>
#include <cstring>

typedef unsigned int DWORD;
typedef unsigned short WORD;
typedef char BYTE;
typedef void* LPSTR;

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <linux/net_tstamp.h>

#include "allegro_hand_driver/candrv/candef.h"
#include "allegro_hand_driver/candrv/candrv.h"
// #include <rclcpp/rclcpp.hpp> // for RCLCPP_ERROR, RCLCPP_INFO

CANAPI_BEGIN

/*=====================*/
/*       Defines       */
/*=====================*/
// Macros
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )

// Constants
#define NUM_OF_FINGERS          4 // number of fingers
#define NUM_OF_TEMP_SENSORS     4 // number of temperature sensors

/*=========================================*/
/*       Global file-scope variables       */
/*=========================================*/
unsigned char CAN_ID = 0;

/*==========================================*/
/*       Private functions prototypes       */
/*==========================================*/
int canReadMsg(void* /*ch*/, uint64_t *timestamp_us, int *id, int *len, unsigned char *data, int blocking, int timeout_usec);
int canSendMsg(void* /*ch*/, int id, char len, unsigned char *data, int blocking, int timeout_usec);
int canSentRTR(void* /*ch*/, int id, int blocking, int timeout_usec);

/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
int socket_;

void printMsg(const can_frame &msg)
{
    printf("msg: {%X}, %c", msg.can_id, msg.can_dlc);
}

int canInit(const char *device_id_)
{
    // Removed unused variable 'err'
    int i;
    printf("CAN: Initializing device");
    sockaddr_can addr;
    ifreq ifr;
    if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1)
    {
        printf("Failed to open CAN socket errno:%d", errno);
        return -1;
    }
    strcpy(ifr.ifr_name, device_id_);
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) == -1)
    {
        printf("Trouble finding CAN bus %s: %s", device_id_, strerror(errno));
        printf("Failed to initialize CAN interface");
        return -1;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    printf("%s at index %d", device_id_, ifr.ifr_ifindex);
    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) == -1)
    {
        printf("Error binding CAN socket: %s", strerror(errno));
        return -1;
    }
    // Set socket non-blocking
    int flags = fcntl(socket_, F_GETFL);
    if (flags == -1)
    {
        printf("Error getting CAN socket flags: %s", strerror(errno));
        return -1;
    }
    if (fcntl(socket_, F_SETFL, flags | O_NONBLOCK) != 0)
    {
        printf("Error setting CAN socket non-blocking: %s", strerror(errno));
        return -1;
    }
    // =========================================================================
    int timestamping_flags = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
    if (setsockopt(socket_, SOL_SOCKET, SO_TIMESTAMPING, &timestamping_flags, sizeof(timestamping_flags)) < 0)
    {
        printf("Error setting SO_TIMESTAMPING for socket: %s", strerror(errno));
        return -1;
    }
    printf("CAN: Timestamping enabled on socket.");
    // =========================================================================
    printf("CAN: Clearing the CAN buffer");
    can_frame dummy_msg;
    for (i = 0; i < 100; i++)
    {
        read(socket_, &dummy_msg, sizeof(can_frame));
    }
    return 0;
}

int canReadMsg(void* /*ch*/, uint64_t *timestamp_us, int *id, int *len, unsigned char *data, int blocking, int timeout_usec)
{
    if (blocking || timeout_usec < 0)
    {
        printf("SOCKET CAN does not support blocking read, proceeding with nonblocking");
    }

    struct msghdr msg_hdr;
    struct iovec iov;
    struct can_frame msg;

    // Buffer pour les données de contrôle (là où se trouvera le timestamp)
    char ctrl_buf[CMSG_SPACE(sizeof(struct timeval))];

    // Préparer la structure iovec pour recevoir les données principales (le can_frame)
    iov.iov_base = &msg;
    iov.iov_len = sizeof(msg);

    // Préparer l'en-tête du message pour recvmsg
    msg_hdr.msg_name = NULL;
    msg_hdr.msg_namelen = 0;
    msg_hdr.msg_iov = &iov;
    msg_hdr.msg_iovlen = 1;
    msg_hdr.msg_control = ctrl_buf;
    msg_hdr.msg_controllen = sizeof(ctrl_buf);
    msg_hdr.msg_flags = 0;

    // Utiliser recvmsg au lieu de read
    ssize_t result = recvmsg(socket_, &msg_hdr, 0);

    if (result < 0)
    {
        if (errno != EAGAIN) // EAGAIN signifie "pas de message disponible pour l'instant"
        {
            printf("Error reading CAN socket with recvmsg: %s", strerror(errno));
        }
        return -1; // -1 pour "pas de message" ou "erreur"
    }

    if (result < static_cast<ssize_t>(sizeof(can_frame)))
    {
        printf("Read incomplete CAN frame.");
        return -1;
    }

    *timestamp_us = 0; 
    for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg_hdr); cmsg != NULL; cmsg = CMSG_NXTHDR(&msg_hdr, cmsg))
    {
        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPING)
        {
            struct timeval *tv = (struct timeval *)CMSG_DATA(cmsg);
            *timestamp_us = (uint64_t)tv->tv_sec * 1000000 + (uint64_t)tv->tv_usec/1000;
            break; 
        }
    }
    
    if (*timestamp_us == 0) {
        printf("Could not retrieve timestamp for CAN message.");
    }
    
    if (msg.can_id & CAN_ERR_FLAG)
    {
        printMsg(msg);
        printf("Received CAN error.");
        return -1;
    }

    *id = (msg.can_id >> 2);
    *len = msg.can_dlc;
    memcpy(data, msg.data, msg.can_dlc);

    return 0; 
}

int canSendMsg(void* /*ch*/, int id, char len, unsigned char *data, int blocking, int timeout_usec)
{
    can_frame msg;
    memcpy(msg.data, data, len);
    msg.can_dlc = len;
    msg.can_id = (id << 2);
    int result = write(socket_, &msg, sizeof(can_frame));
    if (result != sizeof(can_frame))
    {
        printf("Failed to send CAN message: %s", strerror(errno));
        return -1;
    }
    if (blocking || timeout_usec < 0)
    {
        // RCLCPP_WARN_ONCE(rclcpp::get_logger(__FILE__),
        //                  "Socket CAN does not support blocking send, proceed with nonblocking");
    }
    return 0;
}

int canSentRTR(void* /*ch*/, int id, int blocking, int timeout_usec)
{
    can_frame msg;
    msg.can_dlc = 0;
    msg.can_id = (id << 2);
    msg.can_id |= CAN_RTR_FLAG;
    int result = write(socket_, &msg, sizeof(can_frame));
    if (result != sizeof(can_frame))
    {
        printf("Failed to send CAN message: %s", strerror(errno));
        return -1;
    }
    if (blocking || timeout_usec < 0)
    {
        // RCLCPP_WARN_ONCE(rclcpp::get_logger(__FILE__),
        //                  "Socket CAN does not support blocking send, proceed with nonblocking");
    }
    return 0;
}

/*========================================*/
/*             CAN API                  */
/*========================================*/
int command_can_open_with_name(void*& /*ch*/, const char* dev_name)
{
    // RCLCPP_INFO(rclcpp::get_logger(__FILE__),
    //             "CAN: Opening device on channel [%s]", dev_name);
    return canInit(dev_name);
}

int command_can_open(void* /*ch*/)
{
    // RCLCPP_ERROR(rclcpp::get_logger(__FILE__),
    //              "CAN: Error! Unsupported function call, can_open(void*&)");
    return -1;
}

int command_can_open_ex(void* /*ch*/, int /*type*/, int /*index*/)
{
    // RCLCPP_ERROR(rclcpp::get_logger(__FILE__),
    //              "CAN: Error! Unsupported function call, can_open(void*&, int, int)");
    return -1;
}

int command_can_flush(void* /*ch*/)
{
    can_frame msg;
    for (int i = 0; i < 100; i++)
    {
        read(socket_, &msg, sizeof(can_frame));
    }
    return 0;
}

int command_can_reset(void* /*ch*/)
{
    return -1;
}

int command_can_close(void* /*ch*/)
{
    int err = close(socket_);
    if (err)
    {
        printf("CAN: Error in CAN_Close()");
        return -1;
    }
    return 0;
}

int command_can_set_id(void* /*ch*/, unsigned char can_id)
{
    CAN_ID = can_id;
    return 0;
}

int command_servo_on(void* /*ch*/)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_SYSTEM_ON;
    ret = canSendMsg(nullptr, Txid, 0, data, TRUE, 0);
    return ret;
}

int command_servo_off(void* /*ch*/)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_SYSTEM_OFF;
    ret = canSendMsg(nullptr, Txid, 0, data, TRUE, 0);
    return ret;
}

int command_set_torque(void* /*ch*/, int findex, short* pwm)
{
    assert(findex >= 0 && findex < NUM_OF_FINGERS);
    long Txid;
    unsigned char data[8];
    int ret;

    data[0] = (unsigned char)((pwm[0]) & 0x00ff);
    data[1] = (unsigned char)((pwm[0] >> 8) & 0x00ff);
    data[2] = (unsigned char)((pwm[1]) & 0x00ff);
    data[3] = (unsigned char)((pwm[1] >> 8) & 0x00ff);
    data[4] = (unsigned char)((pwm[2]) & 0x00ff);
    data[5] = (unsigned char)((pwm[2] >> 8) & 0x00ff);
    data[6] = (unsigned char)((pwm[3]) & 0x00ff);
    data[7] = (unsigned char)((pwm[3] >> 8) & 0x00ff);

    Txid = ID_CMD_SET_TORQUE_1 + findex;
    ret = canSendMsg(nullptr, Txid, 8, data, TRUE, 0);
    return ret;
}

int command_set_pose(void* /*ch*/, int findex, short* jposition)
{
    assert(findex >= 0 && findex < NUM_OF_FINGERS);
    long Txid;
    unsigned char data[8];
    int ret;

    data[0] = (unsigned char)((jposition[0]) & 0x00ff);
    data[1] = (unsigned char)((jposition[0] >> 8) & 0x00ff);
    data[2] = (unsigned char)((jposition[1]) & 0x00ff);
    data[3] = (unsigned char)((jposition[1] >> 8) & 0x00ff);
    data[4] = (unsigned char)((jposition[2]) & 0x00ff);
    data[5] = (unsigned char)((jposition[2] >> 8) & 0x00ff);
    data[6] = (unsigned char)((jposition[3]) & 0x00ff);
    data[7] = (unsigned char)((jposition[3] >> 8) & 0x00ff);

    Txid = ID_CMD_SET_POSE_1 + findex;
    ret = canSendMsg(nullptr, Txid, 8, data, TRUE, 0);
    return ret;
}

int command_set_period(void* /*ch*/, short* period)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_SET_PERIOD;
    if (period != 0)
    {
        data[0] = (unsigned char)((period[0]) & 0x00ff);
        data[1] = (unsigned char)((period[0] >> 8) & 0x00ff);
        data[2] = (unsigned char)((period[1]) & 0x00ff);
        data[3] = (unsigned char)((period[1] >> 8) & 0x00ff);
        data[4] = (unsigned char)((period[2]) & 0x00ff);
        data[5] = (unsigned char)((period[2] >> 8) & 0x00ff);
    }
    else
    {
        data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 0x0;
    }
    ret = canSendMsg(nullptr, Txid, 6, data, TRUE, 0);
    return ret;
}

int command_set_device_id(void* /*ch*/, unsigned char did)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_CONFIG;
    data[0] = did | 0x80;
    data[1] = 0x0;
    data[5] = 0x0;
    ret = canSendMsg(nullptr, Txid, 6, data, TRUE, 0);
    return ret;
}

int command_set_rs485_baudrate(void* /*ch*/, unsigned int baudrate)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_CONFIG;
    data[0] = 0x0;
    data[1] = (unsigned char)((baudrate) & 0x000000ff);
    data[2] = (unsigned char)((baudrate >> 8) & 0x000000ff);
    data[3] = (unsigned char)((baudrate >> 16) & 0x000000ff);
    data[4] = (unsigned char)(((baudrate >> 24) & 0x000000ff) | 0x80);
    data[5] = 0x0;
    ret = canSendMsg(nullptr, Txid, 6, data, TRUE, 0);
    return ret;
}

int request_hand_information(void* /*ch*/)
{
    long Txid = ID_RTR_HAND_INFO;
    int ret = canSentRTR(nullptr, Txid, TRUE, 0);
    return ret;
}

int request_hand_serial(void* /*ch*/)
{
    long Txid = ID_RTR_SERIAL;
    int ret = canSentRTR(nullptr, Txid, TRUE, 0);
    return ret;
}

int request_finger_pose(void* /*ch*/, int findex)
{
    assert(findex >= 0 && findex < NUM_OF_FINGERS);
    long Txid = ID_RTR_FINGER_POSE + findex;
    int ret = canSentRTR(nullptr, Txid, TRUE, 0);
    return ret;
}

int request_imu_data(void* /*ch*/)
{
    long Txid = ID_RTR_IMU_DATA;
    int ret = canSentRTR(nullptr, Txid, TRUE, 0);
    return ret;
}

int request_temperature(void* /*ch*/, int sindex)
{
    assert(sindex >= 0 && sindex < NUM_OF_TEMP_SENSORS);
    long Txid = ID_RTR_TEMPERATURE + sindex;
    int ret = canSentRTR(nullptr, Txid, TRUE, 0);
    return ret;
}

int can_write_message(void* /*ch*/, int id, int len, unsigned char* data, int blocking, int timeout_usec)
{
    int err = canSendMsg(nullptr, id, len, data, blocking, timeout_usec);
    return err;
}

int can_read_message(void* /*ch*/, uint64_t *timestamp_us, int* id, int* len, unsigned char* data, int blocking, int timeout_usec)
{
    int err = canReadMsg(nullptr, timestamp_us, id, len, data, blocking, timeout_usec);
    return err;
}

CANAPI_END

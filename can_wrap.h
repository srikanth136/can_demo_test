#ifndef CAN_WRAP_H
#define CAN_WRAP_H

#include <linux/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdbool.h>
#include <string.h>

/**
 * @brief Opens socket for CAN bus communication via socketcan. 
 * Returns socket identifier which is used as an argument for 
 * future can::read(), can::write() and can::close() calls.
 * 
 * 
 * @param fd Open as normal CAN or CANFD.
 * @param channel Network channel to use, e.g. can0 or vcan0.
 * @return const int Socket identifier.
 * @return -1 If socket creation failed.
 * @return -2 If could not enable CANFD.
 * @return -3 If could not bind socket.
 */
const int can_connect( const char* channel, const bool fd  )
{
    const int canSocket = socket( PF_CAN, SOCK_RAW, CAN_RAW );
    if( canSocket < 0 )
        return -1;

    struct ifreq ifr;
    strncpy( ifr.ifr_name, channel, sizeof(ifr.ifr_name)-1 );
    ioctl( canSocket, SIOCGIFINDEX, &ifr );

    struct sockaddr_can addr;
    memset( &addr, 0, sizeof(addr) );
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int enableCanFd = 1;
    if( fd && setsockopt( canSocket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
            &enableCanFd, sizeof(enableCanFd) ) )
        return -2;

    if( bind( canSocket, (struct sockaddr*)&addr, sizeof(addr) ) < 0 )
        return -3;

    return canSocket;
}

/**
 * @brief Close specified socketcan socket.
 * 
 * @param socket Socket identifer as returned from can::connect().
 * 
 * @return false If socket close failed
 */
bool can_close( const int socket )
{
    return close( socket ) >= 0;
}

/**
 * @brief Passes call to can::_read<can_frame>() to read standard CAN frame.
 * 
 * @pre must call connect() successfully first.
 * @param socket Socket identifer as returned from can_connect().
 * @param frame CAN frame variable to place read data.
 * @return Success.
 */
bool can_read( const int socket, struct can_frame* frame )
{
    return read( socket, frame, sizeof(struct can_frame) ) >= 0;
}

/**
 * @brief Passes call to can::_read<canfd_frame>() to read CANFD frame.
 * 
 * @pre must call connectfd() successfully first.
 * @param socket Socket identifer as returned from can_connect().
 * @param frame CAN frame variable to place read data.
 * @return Success.
 */
bool can_readfd( const int socket, struct canfd_frame* frame )
{
    return read( socket, frame, sizeof(struct canfd_frame) ) >= 0;
}

/**
 * @brief Passes call to can::_write<can_frame>() to write standard CAN frame.
 * 
 * @param socket Socket identifer as returned from can_connect().
 * @param frame CAN frame to be sent.
 * @return Success.
 */
bool can_write( const int socket, struct can_frame* frame )
{
    return write( socket, frame, sizeof(struct can_frame) ) == sizeof(struct can_frame);
}

/**
 * @brief Passes call to can::_write<canfd_frame>() to write CANFD frame.
 * 
 * @param socket Socket identifer as returned from can::connect().
 * @param frame CANFD frame to be sent.
 * @return Success.
 */
bool can_writefd( const int socket, struct canfd_frame* frame )
{
    return write( socket, frame, sizeof(struct canfd_frame) ) == sizeof(struct canfd_frame);
}

#endif 
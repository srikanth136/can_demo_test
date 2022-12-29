#ifndef CAN_WRAP_HPP
#define CAN_WRAP_HPP

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace can
{
    /**
     * @brief Opens socket for CAN bus communication via socketcan. 
     * Returns socket identifier which is used as an argument for 
     * future can::read(), can::write() and can::close() calls.
     * 
     * @throws std::runtime_error If socket creation failed.
     * @throws std::runtime_error If could not enable CANFD.
     * @throws std::runtime_error If could not bind socket.
     * 
     * @param fd Open as normal CAN or CANFD.
     * @param channel Network channel to use, e.g. can0 or vcan0.
     * @return const int Socket identifier.
     */
    const int _connect( const bool fd, const std::string& channel )
    {
        const int canSocket = socket( PF_CAN, SOCK_RAW, CAN_RAW );
        if( canSocket < 0 )
            throw std::runtime_error( "Create socket failed" );

        struct ifreq ifr;
        std::strncpy( ifr.ifr_name, channel.c_str(), sizeof(ifr.ifr_name)-1 );
        ioctl( canSocket, SIOCGIFINDEX, &ifr );

        struct sockaddr_can addr;
        std::memset( &addr, 0, sizeof(addr) );
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        int enableCanFd = 1;
        if( fd && setsockopt( canSocket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                &enableCanFd, sizeof(enableCanFd) ) )
            throw std::runtime_error( "Enable CAN FD failed" );

        if( bind( canSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr) ) < 0 )
            throw std::runtime_error( "Bind socket failed" );

        return canSocket;
    }

    /**
     * @brief Passes call to can::_connect with fd argument as false.
     */
    inline const int connect( const std::string channel="vcan0" )
    {
        return _connect( false, channel );
    }

    /**
     * @brief Passes call to can::_connect with fd argument as true.
     */
    inline const int connectfd( const std::string channel="vcan0" )
    {
        return  _connect( true, channel );
    }

    /**
     * @brief Close specified socketcan socket.
     * 
     * @throws std::runtime_error If close failed.
     * 
     * @param socket Socket identifer as returned from can::connect().
     */
    void close( const int socket )
    {
        if( ::close( socket ) < 0 )
            throw std::runtime_error( "Socket close failed" );
    }

    /**
     * @brief Read next frame from CAN bus.
     * 
     * @warning This function will block until a frame is available.
     * 
     * @throws std::runtime_error If read failed.
     * 
     * @param socket Socket identifer as returned from can::connect().
     * @return const can_frame 
     */
    template<typename FRAME>
    const FRAME _read( const int socket )
    {
        FRAME frame;

        if( read( socket, &frame, sizeof(frame) ) < 0 ) 
            throw std::runtime_error( "Read failed" );

        return frame;
    }

    /**
     * @brief Passes call to can::_read<can_frame>() to read standard CAN frame.
     * 
     * @pre must call connect() successfully first.
     * @param socket Socket identifer as returned from can::connect().
     * @return const can_frame 
     */
    inline const can_frame read( const int socket )
    {
        return _read<can_frame>( socket );
    }

    /**
     * @brief Passes call to can::_read<canfd_frame>() to read CANFD frame.
     * 
     * @pre must call connectfd() successfully first.
     * @param socket Socket identifer as returned from can::connect().
     * @return const canfd_frame 
     */
    inline const canfd_frame readfd( const int socket )
    {
        return _read<canfd_frame>( socket );
    }

    /**
     * @brief Send CAN frame.
     * 
     * @throws std::runtime_error If write failed. 
     * 
     * @tparam FRAME can_frame or canfd_frame
     * @param socket Socket identifer as returned from can::connect().
     * @param frame CAN or CANFD frame to be sent.
     * @param numBytes Number of bytes to be written.
     */
    template<typename FRAME>
    void _write( const int socket, const FRAME& frame, const uint8_t numBytes )
    {
        if( write( socket, &frame, numBytes ) != numBytes )
            std::runtime_error( "Write failed" );
    }

    /**
     * @brief Passes call to can::_write<can_frame>() to write standard CAN frame.
     * 
     * @param socket Socket identifer as returned from can::connect().
     * @param frame CAN frame to be sent.
     */
    void write( const int socket, const can_frame& frame )
    {
        // 8 bytes for the frame header and the rest for the actual data.
        _write( socket, frame, sizeof(can_frame) );
    }

    /**
     * @brief Passes call to can::_write<canfd_frame>() to write CANFD frame.
     * 
     * @param socket Socket identifer as returned from can::connect().
     * @param frame CANFD frame to be sent.
     */
    void write( const int socket, const canfd_frame& frame )
    {
        // 8 bytes for the frame header and the rest for the actual data.
        _write( socket, frame, sizeof(canfd_frame) );
    }
    
    
    std::ostream& operator<<( std::ostream &os, const can_frame& frame )
    {
        os << "0x" << std::setw(3) << std::hex << frame.can_id << 
            " [" << static_cast<int>(frame.can_dlc) << ']';
        for( int i=0; i<frame.can_dlc; ++i )
            os << ' ' << std::uppercase << std::setfill('0') << std::setw(2)
               << static_cast<int>(frame.data[i]);

        return os;
    }

    std::ostream& operator<<( std::ostream &os, const canfd_frame& frame )
    {
        os << "0x" << std::setw(3) << std::hex << frame.can_id << 
            " [" << static_cast<int>(frame.len) << ']';
        os << std::uppercase << std::setfill('0') << std::setw(2);
        for( int i=0; i<frame.len; ++i )
            os << ' ' << std::uppercase << std::setfill('0') << std::setw(2)
               << static_cast<int>(frame.data[i]);

        return os;
    }
}

#endif

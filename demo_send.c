#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_wrap.h"

int main( int argc, char* argv[] )
{
    const char* canChannel = "vcan0";
    const int canSocket = can_connect( canChannel, false );

    for( int c=0; c<100; ++c )
    {
        struct can_frame frame;
        memset( &frame, 0, sizeof(frame) );

        frame.can_id = 0x188;
        frame.can_dlc = 4;

        switch( c%4 )
        {
            case 0:  frame.data[0] = 0b00000000; break;
            case 1:  frame.data[0] = 0b00000001; break;
            case 2:  frame.data[0] = 0b00000010; break;
            default: frame.data[0] = 0b00000011; break;
        }

        printf("0x%X [%X]", frame.can_id, frame.can_dlc );
        for( int i=0; i<frame.can_dlc; ++i )
            printf( " %02X", frame.data[i] );
        printf("\n");

        if( !can_write( canSocket, &frame ) )
            printf( "Error\n" );

        sleep( 1 );
    }

	can_close( canSocket );

	return 0;
}
#ifndef TRIA_MESSAGE_H_
#define TRIA_MESSAGE_H_

#include "tria_protocol.pb.h"
#include <nanopb/pb.h>
#include <nanopb/pb_common.h>
#include <nanopb/pb_encode.h>
#include <nanopb/pb_decode.h>

tria_CommandPacket decode_command_packet(uint8_t* buffer, int buffer_size)
{
    tria_CommandPacket command = tria_CommandPacket_init_default;

    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);

    if(pb_decode(&stream, tria_CommandPacket_fields, &command))
    {
        return command;
    }
    else
    {
        tria_CommandPacket zero_command = tria_CommandPacket_init_zero;
        return zero_command;
    }
}

int encode_sensor_packet(uint8_t* buffer, tria_SensorPacket sensor)
{
    pb_ostream_t out_stream = pb_ostream_from_buffer(buffer, 256);

    if (pb_encode(&out_stream, tria_SensorPacket_fields, &sensor))
    {
        return out_stream.bytes_written;
    }
    else
    {
        return -1;
    }
}

#endif
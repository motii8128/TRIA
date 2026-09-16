add_library(tria_message STATIC
    ${CMAKE_CURRENT_LIST_DIR}
    tria_protocol.pb.c
    nanopb/pb_encode.c
    nanopb/pb_decode.c
    nanopb/pb_common.c
)

target_include_directories(tria_message
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/
    ${CMAKE_CURRENT_LIST_DIR}/nanopb/
)

target_link_libraries(tria_message
    PUBLIC
)
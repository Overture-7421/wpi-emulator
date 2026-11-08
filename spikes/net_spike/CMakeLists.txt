# Temporary spike: proves ENet6 and Monocypher build and work. Deleted once net/ consumes them.
add_executable(net_spike main.cpp)
wpiemu_target_defaults(net_spike)
target_link_libraries(net_spike PRIVATE wpiemu_third_party_enet wpiemu_third_party_monocypher)
add_test(NAME net.spike COMMAND net_spike)

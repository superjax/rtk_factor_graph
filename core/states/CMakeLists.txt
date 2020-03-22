library(states states.cxx)
target_link_libraries(states utctime)

add_gtest(test_states test_states.cxx)
target_link_libraries(test_states states)

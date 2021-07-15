add_opp_run(fcw)
add_opp_test(fcw SUFFIX inet SIMTIME_LIMIT 60s)
add_opp_test(fcw SUFFIX veins CONFIG veins SIMTIME_LIMIT 60s)
add_opp_test(fcw SUFFIX repeat CONFIG repeat)

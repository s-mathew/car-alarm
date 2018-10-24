
 add_fsm_encoding \
       {fuel_pump_fsm.state} \
       { }  \
       {{000 00} {001 01} {010 10} {011 11} }

 add_fsm_encoding \
       {anti_theft_fsm.state} \
       { }  \
       {{000 000} {001 001} {010 010} {011 011} {100 100} {101 101} {110 110} {111 111} }

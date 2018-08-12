# Convert the pdflatex-generated PDF to nice medium-resolution textures
# for use on URDF models with Collada meshes...
#
# phase-1: generate PNGs for each page of the pdf
# pdftoppm tiago_command_cards.pdf outputname -png -rx 150 -ry 150 
#
# phase-2: use netpbm to crop the cards..
#
# upper-left card: (72,86) .. (616, 867)
# upper-right card: (623,86) .. (1168, 867)
#
# lower-left card: (72,902) .. (616,1683)
# lower-right-card: (623,902) .. (1168, 1683)
# 
pngtopnm outputname-01.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_emergency_stop.png
pngtopnm outputname-01.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_emergency_stop_atag_402.png
pngtopnm outputname-01.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_emergency_stop2.png
pngtopnm outputname-01.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_emergency_stop2_atag_402.png
#
pngtopnm outputname-02.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_pause.png
pngtopnm outputname-02.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_pause_atag_403.png
pngtopnm outputname-02.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_continue.png
pngtopnm outputname-02.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_continue_atag_404.png
#
pngtopnm outputname-03.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_cancel.png
pngtopnm outputname-03.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_cancel_atag_405.png
pngtopnm outputname-03.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_cancel_all.png
pngtopnm outputname-03.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_cancel_all_atag_406.png
#
pngtopnm outputname-04.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_look_straight.png
pngtopnm outputname-04.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_look_straight_atag_407.png
pngtopnm outputname-04.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_look_at_me.png
pngtopnm outputname-04.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_look_at_me_atag_408.png
#
pngtopnm outputname-05.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_tuck_arm.png
pngtopnm outputname-05.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_tuck_arm_atag_409.png
pngtopnm outputname-05.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_move_arm_to_home.png
pngtopnm outputname-05.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_move_arm_to_home_atag_410.png
#
pngtopnm outputname-06.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_move_torso_down.png
pngtopnm outputname-06.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_move_torso_down.png
pngtopnm outputname-06.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_move_torso_up.png
pngtopnm outputname-06.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_move_torso_up_atag_412.png
#
pngtopnm outputname-07.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_move_torso.png
pngtopnm outputname-07.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_move_torso_atag_413.png
pngtopnm outputname-07.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_open_gripper.png
pngtopnm outputname-07.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_open_gripper_atag_414.png
#
pngtopnm outputname-08.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_close_gripper.png
pngtopnm outputname-08.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_close_gripper_atag_415.png
pngtopnm outputname-08.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_move_arm.png
pngtopnm outputname-08.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_move_arm_atag_416.png
#
pngtopnm outputname-09.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_drive_to_docking_station.png
pngtopnm outputname-09.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_drive_to_docking_station_atag_420.png
pngtopnm outputname-09.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_drive_home.png
pngtopnm outputname-09.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_drive_home_atag_421.png
#
pngtopnm outputname-10.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_turn_left.png
pngtopnm outputname-10.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_turn_left_atag_422.png
pngtopnm outputname-10.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_turn_right.png
pngtopnm outputname-10.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_turn_right_atag_423.png
#
pngtopnm outputname-11.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_drive_reverse.png
pngtopnm outputname-11.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_drive_reverse_atag_424.png
pngtopnm outputname-11.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_drive_forward.png
pngtopnm outputname-11.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_drive_forward_atag_425.png
#
pngtopnm outputname-12.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_come_here.png
pngtopnm outputname-12.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_come_here_atag_426.png
pngtopnm outputname-12.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_follow_me.png
pngtopnm outputname-12.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_follow_me_atag_427.png
#
pngtopnm outputname-13.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_order_still_water.png
pngtopnm outputname-13.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_order_still_water_atag_430.png
pngtopnm outputname-13.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_order_sparkling_water.png
pngtopnm outputname-13.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_order_sparkling_water_atag_431.png
#
pngtopnm outputname-14.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_order_tonic_water.png
pngtopnm outputname-14.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_order_tonic_water_atag_433.png
pngtopnm outputname-14.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_order_bitter_lemon.png
pngtopnm outputname-14.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_order_bitter_lemon_atag_434.png
#
pngtopnm outputname-15.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_order_bitter_orange.png
pngtopnm outputname-15.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_order_bitter_orange_atag_435.png
pngtopnm outputname-15.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_order_coca_cola.png
pngtopnm outputname-15.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_order_coca_cola_atag_436.png
#
pngtopnm outputname-16.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_order_fanta.png
pngtopnm outputname-16.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_order_fanta_atag_437.png
pngtopnm outputname-16.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_order_apple_juice.png
pngtopnm outputname-16.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_order_apple_juice_atag_440.png
#
pngtopnm outputname-17.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_order_orange_juice.png
pngtopnm outputname-17.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_order_orange_juice_atag_441.png
pngtopnm outputname-17.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_order_pineapple_juice.png
pngtopnm outputname-17.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_order_pineapple_juice_atag_442.png
#
#
pngtopnm outputname-18.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_order_white_wine.png
pngtopnm outputname-18.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_order_white_wine_atag_450.png
pngtopnm outputname-18.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_order_red_wine.png
pngtopnm outputname-18.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_order_red_wine_atag_451.png
#
pngtopnm outputname-19.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_order_sangria.png
pngtopnm outputname-19.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_order_sangria_atag_452.png
pngtopnm outputname-19.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_order_cuba_libre.png
pngtopnm outputname-19.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_order_cuba_libre_atag_460.png
#
pngtopnm outputname-20.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_order_tequila_pure.png
pngtopnm outputname-20.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_order_tequila_pure_atag_460.png
pngtopnm outputname-20.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_order_tequila_sunrise.png
pngtopnm outputname-20.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_order_tequila_sunrise_atag_461.png
#
pngtopnm outputname-18.png | pnmcut -left  72 -right  616 -top  86 -bottom  867 | pnmtopng > cc_order_white_wine.png
pngtopnm outputname-18.png | pnmcut -left 623 -right 1168 -top  86 -bottom  867 | pnmtopng > cc_order_white_wine_atag_450.png
pngtopnm outputname-18.png | pnmcut -left  72 -right  616 -top 902 -bottom 1683 | pnmtopng > cc_order_red_wine.png
pngtopnm outputname-18.png | pnmcut -left 623 -right 1168 -top 902 -bottom 1683 | pnmtopng > cc_order_red_wine_atag_451.png
#


#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#


add_controller("ObserverbasedImpedanceSampleController" "mc_ObserverbasedImpedance_sample_controller.cpp" "mc_ObserverbasedImpedance_sample_controller.h")
configure_file(etc/ObserverbasedImpedanceSampleController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedImpedanceSampleController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedImpedanceSampleController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

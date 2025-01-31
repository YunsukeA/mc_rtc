#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#


add_controller("ObserverbasedImpedance" "ObserverbasedImpedance_controller.cpp" "ObserverbasedImpedance_controller.h")
configure_file(etc/ObserverbasedImpedance.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedImpedance.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedImpedance.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

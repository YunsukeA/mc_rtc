mc_rtc_set_prefix(STATES mc_controller/fsm/states)
mc_rtc_set_prefix(ObserverbasedAdmittance_STATES mc_controller/ObserverbasedAdmittanceSample/states)

configure_file(
  etc/ObserverbasedAdmittanceSample.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedAdmittanceSample.yaml
)
configure_file(
  etc/ObserverbasedAdmittanceSample.in.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedAdmittanceSample_TVM.yaml
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedAdmittanceSample.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/etc/ObserverbasedAdmittanceSample_TVM.yaml
  DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/
)

add_subdirectory(src)

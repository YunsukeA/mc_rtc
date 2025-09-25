cmake_minimum_required(VERSION 3.1)

project(observer_tasks_plugin LANGUAGES CXX VERSION 1.0.0)

find_package(mc_rtc REQUIRED)

# Create the plugin library
add_library(observer_tasks_plugin SHARED
  src/ObserverbasedImpedanceTask.cpp
  src/ObserverbasedAdmittanceTask.cpp
  src/plugin.cpp
)

target_include_directories(observer_tasks_plugin PRIVATE include)
target_link_libraries(observer_tasks_plugin PUBLIC mc_rtc::mc_rtc)
target_compile_features(observer_tasks_plugin PUBLIC cxx_std_17)

# Install the plugin
install(TARGETS observer_tasks_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers (optional, if you want others to use your tasks)
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

# Set plugin installation directory
if(DEFINED ENV{MC_RTC_INSTALL_PREFIX})
  set(MC_RTC_USER_DIR "$ENV{MC_RTC_INSTALL_PREFIX}")
else()
  set(MC_RTC_USER_DIR "$ENV{HOME}/.config")
endif()

# Install plugin configuration
install(TARGETS observer_tasks_plugin
  LIBRARY DESTINATION "${MC_RTC_USER_DIR}/mc_rtc/plugins"
)
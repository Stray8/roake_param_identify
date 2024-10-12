#-------------------------------------------------------------------------------
# RT mode example executables setup

# Here puts required example source files

set(XDZ_EXAMPLES
    # cart_position_control
    )


set(inc "/home/robot/robot/roake_param_identify/include/robotiq")
add_library(RQSensorLinux STATIC ${inc}/rq_sensor_com.c ${inc}/rq_sensor_state.c)
set_target_properties(RQSensorLinux PROPERTIES OUTPUT_NAME "libRQSensorLinux")
add_executable(driverSensor /home/robot/robot/roake_param_identify/example/cpp/xdz/robotiq.cpp ${xCoreSDK-examples_SOURCE_DIR}/cpp/print_helper.hpp) 
target_include_directories(driverSensor PUBLIC ${SHARED_INCLUDEDIR})
target_link_libraries(driverSensor RQSensorLinux Rokae::Rokae eigen Threads::Threads)



foreach(example ${XDZ_EXAMPLES})
    add_executable(${example} ${example}.cpp ${xCoreSDK-examples_SOURCE_DIR}/cpp/print_helper.hpp)
    target_include_directories(${example} PUBLIC
        ${SHARED_INCLUDEDIR})
    target_link_libraries(${example} Rokae::Rokae eigen Threads::Threads)
    if(MSVC)
        add_compile_definitions(_USE_MATH_DEFINES)
    endif() # for M_PI in cmath
endforeach()

# check for xMateModel library option
if(XCORE_USE_XMATE_MODEL)
    set(EXAMPLES_USE_XMATEMODEL
        # cart_control
        )
    foreach(example ${EXAMPLES_USE_XMATEMODEL})
        add_executable(${example} ${example}.cpp ${xCoreSDK-examples_SOURCE_DIR}/cpp/print_helper.hpp)
        target_include_directories(${example} PUBLIC
            ${SHARED_INCLUDEDIR})
        target_link_libraries(${example} Rokae::Rokae eigen Threads::Threads)
        target_compile_definitions(${example} PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endforeach()
endif()


#-------------------------------------------------------------------------------
# Installation
install(TARGETS ${XDZ_EXAMPLES}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

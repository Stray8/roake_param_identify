#-------------------------------------------------------------------------------
# LY  example executables setup

# Here puts required example source files

set(LY_EXAMPLES
    # test_collect
    ly
    )

include_directories(/home/robot/robot/roake_param_identify/include/rokae/)
include_directories(/home/robot/.local/lib/python3.8/site-packages/numpy/core/include/numpy/)
include_directories(/usr/include/x86_64-linux-gnu/python3.8)

foreach(example ${LY_EXAMPLES})
    add_executable(${example} ${example}.cpp ${xCoreSDK-examples_SOURCE_DIR}/cpp/print_helper.hpp)
    target_include_directories(${example} PUBLIC
        ${SHARED_INCLUDEDIR})
    target_link_libraries(${example} Rokae::Rokae eigen Threads::Threads)
    target_link_libraries(${example} /usr/lib/x86_64-linux-gnu/libpython3.8.so)
    if(MSVC)
        add_compile_definitions(_USE_MATH_DEFINES)
    endif() # for M_PI in cmath
endforeach()

# check for xMateModel library option
if(XCORE_USE_XMATE_MODEL)
    set(EXAMPLES_USE_XMATEMODEL
        test_collect
        SGPR_robot
        PD
        # vic
        )
    foreach(example ${EXAMPLES_USE_XMATEMODEL})
        add_executable(${example} ${example}.cpp ${xCoreSDK-examples_SOURCE_DIR}/cpp/print_helper.hpp)
        target_include_directories(${example} PUBLIC ${SHARED_INCLUDEDIR})
        target_link_libraries(${example} Rokae::Rokae eigen Threads::Threads )
        target_link_libraries(${example} /usr/lib/x86_64-linux-gnu/libpython3.8.so)
        target_compile_definitions(${example} PUBLIC XMATEMODEL_LIB_SUPPORTED)
    endforeach()
endif()

#-------------------------------------------------------------------------------
# Installation
install(TARGETS ${LY_EXAMPLES}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

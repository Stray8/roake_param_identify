# Add shared/static libraries
add_library(${PROJECT_NAME}_lib SHARED IMPORTED GLOBAL)
add_library(${PROJECT_NAME}_static STATIC IMPORTED GLOBAL)
add_library(xmatemodel_lib STATIC IMPORTED)

if(WIN32)
    if(${CMAKE_SIZEOF_VOID_P} EQUAL 8) #64bit
        set(_ARCH_NAME "64bit")
    elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 4) #32bit
        set(_ARCH_NAME "32bit")
    endif()

    set_target_properties(xmatemodel_lib PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Release/64bit/xMateModel.lib)
    set_target_properties(${PROJECT_NAME}_lib PROPERTIES
        IMPORTED_IMPLIB_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Release/${_ARCH_NAME}/${PROJECT_NAME}.lib
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Release/${_ARCH_NAME}/${PROJECT_NAME}.dll
        IMPORTED_IMPLIB_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Debug/${_ARCH_NAME}/${PROJECT_NAME}.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Debug/${_ARCH_NAME}/${PROJECT_NAME}.dll
        )
    set_target_properties(${PROJECT_NAME}_static PROPERTIES
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Release/${_ARCH_NAME}/${PROJECT_NAME}_static.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/cpp/Debug/${_ARCH_NAME}/${PROJECT_NAME}_static.lib
        INTERFACE_LINK_LIBRARIES xmatemodel_lib # xMateModel模型库目前只支持Release类型编译
        )

elseif(UNIX)
        set(XCORESDK_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Linux/cpp/x86_64)
        set_target_properties(xmatemodel_lib PROPERTIES
            IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxMateModel.a)
        if(XCORE_USE_XMATE_MODEL)
            set_target_properties(${PROJECT_NAME}_lib PROPERTIES
                IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/lib${PROJECT_NAME}.so.${CMAKE_PROJECT_VERSION}
                INTERFACE_LINK_LIBRARIES xmatemodel_lib)
        else()
            set_target_properties(${PROJECT_NAME}_lib PROPERTIES
                IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/NoModel/lib${PROJECT_NAME}.so.${CMAKE_PROJECT_VERSION})
        endif()
        set_target_properties(${PROJECT_NAME}_static PROPERTIES
            IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/lib${PROJECT_NAME}.a
            INTERFACE_LINK_LIBRARIES xmatemodel_lib)
endif()

if(XCORE_LINK_SHARED_LIBS)
    add_library(Rokae::Rokae ALIAS ${CMAKE_PROJECT_NAME}_lib)
else(XCORE_LINK_SHARED_LIBS)
    add_library(Rokae::Rokae ALIAS ${CMAKE_PROJECT_NAME}_static)
endif(XCORE_LINK_SHARED_LIBS)
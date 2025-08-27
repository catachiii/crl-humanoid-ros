# -----------------------------------------------------------------------------
# macro and functions
# -----------------------------------------------------------------------------
include(GNUInstallDirs)

# create static library named LIB_NAME
function(
        create_crl_library #
        LIB_NAME #
        SOURCE #
        DEPENDENCY #
        INCLUDE_DIRS #
        LINK_LIBS #
        COMPILE_DEFINITIONS #
)
    add_library(${LIB_NAME} STATIC ${SOURCE})
    add_library(crl::${LIB_NAME} ALIAS ${LIB_NAME})
    if (DEPENDENCY)
        add_dependencies(${LIB_NAME} ${DEPENDENCY})
    endif ()
    target_include_directories(${LIB_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} PUBLIC ${LINK_LIBS})

    if (COMPILE_DEFINITIONS)
        # scope is PUBLIC so these definitions are propagated
        target_compile_definitions(${LIB_NAME} PUBLIC ${COMPILE_DEFINITIONS})
    endif ()

    # install library headers
    install(
            DIRECTORY ${INCLUDE_DIRS}
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # install library binaries
    install(
            TARGETS ${LIB_NAME}
            EXPORT crl-loco-targets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # for solution explorer in visual studios
    set_property(TARGET ${LIB_NAME} PROPERTY FOLDER "Libs")
endfunction()

# create executable named APP_NAME
function(
        create_crl_app #
        APP_NAME #
        SOURCE #
        DEPENDENCY #
        INCLUDE_DIRS #
        LINK_LIBS #
        COMPILE_DEFINITIONS #
)

    add_executable(${APP_NAME} ${SOURCE})
    add_dependencies(${APP_NAME} ${DEPENDENCY})
    target_include_directories(${APP_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${APP_NAME} PUBLIC ${LINK_LIBS})

    if (COMPILE_DEFINITIONS)
        target_compile_definitions(${APP_NAME} PUBLIC ${COMPILE_DEFINITIONS})
    endif ()

    # for solution explorer in visual studios
    set_property(TARGET ${APP_NAME} PROPERTY FOLDER "Apps")
endfunction()


add_subdirectory(crl-basic)


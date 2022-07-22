find_package(PythonInterp 3)

if (PYTHON_EXECUTABLE)
    # Determine installation path
    execute_process(COMMAND
        ${PYTHON_EXECUTABLE} -c "import sys; print([x for x in sys.path if x.startswith('${CMAKE_INSTALL_PREFIX}')][0]) "
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)

    install(DIRECTORY manus DESTINATION ${PYTHON_SITE_PACKAGES})
	install(DIRECTORY ${CMAKE_BINARY_DIR}/python/manus DESTINATION ${PYTHON_SITE_PACKAGES})
endif()

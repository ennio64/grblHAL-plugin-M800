add_library (keyway INTERFACE)

target_sources (keyway INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/keyway.c
)

target_include_directories (keyway INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

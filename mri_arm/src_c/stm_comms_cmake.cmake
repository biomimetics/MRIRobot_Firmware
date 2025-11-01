# cmake/comms-lib.cmake

# Create a static library from comms.c
#add_library(comms STATIC
#    ${CMAKE_SOURCE_DIR}/stm_comms.c
#)

# Include the directory where comms.h lives
#target_include_directories(comms PUBLIC
#    ${CMAKE_SOURCE_DIR}
#)

# Link the library to the LF main target
# LF defines a variable for the main target name
# (normally something like "Main" or the name of your LF app)
#target_link_libraries(${LF_MAIN_TARGET} PUBLIC comms)


target_sources(${LF_MAIN_TARGET} PRIVATE stm_comms.c stm_comms.h)
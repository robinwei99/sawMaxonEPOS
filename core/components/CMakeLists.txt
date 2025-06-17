#
# (C) Copyright 2016-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawMaxonController VERSION 0.1.0)

# So we can use Findgclib.cmake
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes)

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message( FATAL_ERROR "sawMaxonController requires cisst to be built with JSON" )
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()


  # Find maxon EposCmdLib but don't fail if not found
  find_package (EposCmdLib)

  if (EposCmdLib_FOUND)

    include_directories (${EposCmdLib_INCLUDE_DIR})
    link_directories (${EposCmdLib_LIBRARY_DIR})

    set (sawMaxonController_INCLUDE_DIR
      "${sawMaxonController_SOURCE_DIR}/include"
      "${sawMaxonController_BINARY_DIR}/include")
    set (sawMaxonController_HEADER_DIR "${sawMaxonController_SOURCE_DIR}/include/sawMaxonController")
    set (sawMaxonController_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}" "${EposCmdLib_LIBRARY_DIR}")
    set (sawMaxonController_LIBRARIES sawMaxonController)

    include_directories (BEFORE ${sawMaxonController_INCLUDE_DIR})

    # add all config files for this component
    cisst_add_config_files (sawMaxonController)

    set (sawMaxonController_HEADER_FILES
      "${sawMaxonController_HEADER_DIR}/mtsMaxonController.h"
      "${sawMaxonController_HEADER_DIR}/sawMaxonControllerExport.h"
      ${sawMaxonController_CISST_DG_HDRS})

    set (sawMaxonController_SOURCE_FILES
      code/mtsMaxonController.cpp)

    add_library (
      sawMaxonController
      ${IS_SHARED}
      ${sawMaxonController_HEADER_FILES}
      ${sawMaxonController_SOURCE_FILES})

    set_target_properties (
      sawMaxonController PROPERTIES
      VERSION ${sawMaxonController_VERSION}
      FOLDER "sawMaxonController")

    target_link_libraries (
      sawMaxonController
      ${gclib_LIBRARIES})

    cisst_target_link_libraries (
      sawMaxonController
      ${REQUIRED_CISST_LIBRARIES})

    # Install target for headers and library
    install (
      DIRECTORY
      "${sawMaxonController_SOURCE_DIR}/include/sawMaxonController"
      "${sawMaxonController_BINARY_DIR}/include/sawMaxonController"
      DESTINATION include
      COMPONENT sawMaxonController-dev)

    install (
      TARGETS sawMaxonController
      COMPONENT sawMaxonController
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib)

  else (EposCmdLib_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires the Maxon SDK")
  endif (EposCmdLib_FOUND)
else (cisst_FOUND_AS_REQUIRED)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)

# Try to find TypeIRML Lib

find_path( TypeIRML_INCLUDE_DIR TypeIRML.h
  $ENV{FRIL_PATH}/include
  $ENV{HOME}/FRILibrary/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../FRILibrary/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../FRILibrary/include
)

find_library( TypeIRML_LIBRARY_DEBUG
  LIBRARY_NAMES
    TypeIRML
  PATHS
  $ENV{FRIL_PATH}/Linux/x64/debug/lib
  $ENV{HOME}/FRILibrary/Linux/x64/debug/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../FRILibrary/Linux/x64/debug/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../FRILibrary/Linux/x64/debug/lib
)

find_library( TypeIRML_LIBRARY_RELEASE
  LIBRARY_NAMES
    TypeIRML
  PATHS
  $ENV{FRIL_PATH}/Linux/x64/release/lib
  $ENV{HOME}/FRILibrary/Linux/x64/release/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../FRILibrary/Linux/x64/release/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../FRILibrary/Linux/x64/release/lib
)

if(TypeIRML_INCLUDE_DIR AND
   TypeIRML_LIBRARY_DEBUG AND
   TypeIRML_LIBRARY_RELEASE)

  set( TypeIRML_FOUND true )
  set( TypeIRML_LIBRARIES
       ${TypeIRML_LIBRARY_DEBUG}
       ${TypeIRML_LIBRARY_RELEASE})

endif(TypeIRML_INCLUDE_DIR AND
      TypeIRML_LIBRARY_DEBUG AND
      TypeIRML_LIBRARY_RELEASE)

set(TypeIRML_LIBRARIES
  ${TypeIRML_LIBRARY_DEBUG}
  ${TypeIRML_LIBRARY_RELEASE})

IF(TypeIRML_FOUND)
  MESSAGE(STATUS "Found TypeIRML Library: ${TypeIRML_LIBRARIES}")
ELSE(TypeIRML_FOUND)
  IF(TypeIRML_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find TypeIRML library")
  ENDIF(TypeIRML_FIND_REQUIRED)
ENDIF(TypeIRML_FOUND)

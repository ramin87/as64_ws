# Try to find FastResearchInterfaceLibrary Lib

find_path( FastResearchInterfaceLibrary_INCLUDE_DIR FastResearchInterface.h
  $ENV{FRIL_PATH}/include
  $ENV{HOME}/FRILibrary/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../FRILibrary/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../FRILibrary/include
)

find_library( FastResearchInterfaceLibrary_LIBRARY_DEBUG
  LIBRARY_NAMES
    FastResearchInterfaceLibrary
  PATHS
  $ENV{FRIL_PATH}/Linux/x64/debug/lib
  $ENV{HOME}/FRILibrary/Linux/x64/debug/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../FRILibrary/Linux/x64/debug/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../FRILibrary/Linux/x64/debug/lib
)

find_library( FastResearchInterfaceLibrary_LIBRARY_RELEASE
  LIBRARY_NAMES
    FastResearchInterfaceLibrary
  PATHS
  $ENV{FRIL_PATH}/Linux/x64/release/lib
  $ENV{HOME}/FRILibrary/Linux/x64/release/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../FRILibrary/Linux/x64/release/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../../FRILibrary/Linux/x64/release/lib
)

if(FastResearchInterfaceLibrary_INCLUDE_DIR AND
   FastResearchInterfaceLibrary_LIBRARY_DEBUG AND
   FastResearchInterfaceLibrary_LIBRARY_RELEASE)

  set( FastResearchInterfaceLibrary_FOUND true )
  set( FastResearchInterfaceLibrary_LIBRARIES
       ${FastResearchInterfaceLibrary_LIBRARY_DEBUG}
       ${FastResearchInterfaceLibrary_LIBRARY_RELEASE})

endif(FastResearchInterfaceLibrary_INCLUDE_DIR AND
      FastResearchInterfaceLibrary_LIBRARY_DEBUG AND
      FastResearchInterfaceLibrary_LIBRARY_RELEASE)

set(FastResearchInterfaceLibrary_LIBRARIES
  ${FastResearchInterfaceLibrary_LIBRARY_DEBUG}
  ${FastResearchInterfaceLibrary_LIBRARY_RELEASE})

IF(FastResearchInterfaceLibrary_FOUND)
  MESSAGE(STATUS "Found FRI Library: ${FastResearchInterfaceLibrary_LIBRARIES}")
ELSE(FastResearchInterfaceLibrary_FOUND)
  IF(FastResearchInterfaceLibrary_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find FastResearchInterfaceLibrary library")
  ENDIF(FastResearchInterfaceLibrary_FIND_REQUIRED)
ENDIF(FastResearchInterfaceLibrary_FOUND)

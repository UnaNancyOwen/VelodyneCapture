cmake_minimum_required( VERSION 3.6 )

# Require C++11
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Create Project
project( sample )
add_executable( simple VelodyneCapture.h main.cpp )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "simple" )

# Find Packages
# Find Package Thread
find_package( Threads REQUIRED )

# Find Package Boost
set( BOOST_ROOT "C:/Program Files/boost" )
set( Boost_USE_STATIC_LIBS ON ) # Static Link Libraries ( libboost_* )
set( Boost_USE_MULTITHREADED ON ) # Multi Thread Libraries ( *-mt-* )
set( Boost_USE_STATIC_RUNTIME OFF ) # Static Runtime Libraries ( *-s* )
find_package( Boost COMPONENTS system date_time regex )

# If Direct Capture from Sensors, VelodyneCapture are required Boost.Asio.
# Please Define HAVE_BOOST in Preprocessor.
set( HAVE_BOOST )
if( Boost_FOUND )
  set( HAVE_BOOST "-DHAVE_BOOST" )
endif()

# Find Package PCAP
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
set( PCAP_DIR "C:/Program Files/WpdPack" )
find_package( PCAP )

# If Capture from PCAP Files, VelodyneCapture are required PCAP.
# Please Define HAVE_PCAP in Preprocessor.
set( HAVE_PCAP )
if( PCAP_FOUND )
  set( HAVE_PCAP "-DHAVE_PCAP" )
endif()

# Use GPS timestamps insead of Unix epoch
OPTION( HAVE_GPSTIME "Use GPS timestamps instead of Unix " OFF )
if( HAVE_GPSTIME )
  add_definitions( -DHAVE_GPSTIME )
endif()

# Set Properties
if( PCAP_FOUND OR Boost_FOUND )
  # Additional Include Directories
  include_directories( ${Boost_INCLUDE_DIRS} )
  include_directories( ${PCAP_INCLUDE_DIRS} )

  # Add Definitions
  add_definitions( ${Boost_LIB_DIAGNOSTIC_DEFINITIONS} )
  add_definitions( ${HAVE_BOOST} )
  add_definitions( ${HAVE_PCAP} )

  # Additional Library Directories
  link_directories( ${Boost_LIBRARY_DIRS} )
  link_directories( ${PCAP_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( simple ${CMAKE_THREAD_LIBS_INIT} )
  target_link_libraries( simple ${Boost_LIBRARIES} )
  target_link_libraries( simple ${PCAP_LIBRARIES} )
else()
  message( WARNING "VelodyneCapture need at least either Boost or PCAP." )
endif()

#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>

// Include VelodyneCapture Header
#include "VelodyneCapture.h"

int main( int argc, char* argv[] )
{
    // Open VelodyneCapture that retrieve from Sensor
    const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.21" );
    const unsigned short port = 2368;
    velodyne::VLP16Capture capture( address, port );
    //velodyne::HDL32ECapture capture( address, port );
    const std::string filename = "/home/e/Downloads/vlp16mrtt-000.pcap";
    velodyne::VLP16Capture capture( filename );

    /*
    // Open VelodyneCapture that retrieve from PCAP
    //velodyne::HDL32ECapture capture( filename );
    */

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    while( capture.isRun() ){
        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;
        if( lasers.empty() ){
            continue;
        }

        /*
        // Sort Laser Data ( 0 degree -> 359 degree, 0 id -> n id )
        std::sort( lasers.begin(), lasers.end() );
        */

        // Access to Laser Data
        for( const velodyne::Laser& laser : lasers ){
            // Laser Azimuth ( degree )
            const double azimuth = laser.azimuth;
            std::cout << "azimuth : " << azimuth << "\n";

            // Laser Vertical ( degree )
            const double vertical = laser.vertical;
            std::cout << "vertical : " << vertical << "\n";

            // Laser Distance ( centimeter )
            const unsigned short distance = laser.distance;
            std::cout << "distance : " << distance << "\n";

            // Laser Intensity
            const unsigned int intensity = static_cast<unsigned int>( laser.intensity );
            std::cout << "intensity : " << intensity << "\n";

            // Laser ID ( VLP-16 : 0 - 15, HDL-32E : 0 - 31 )
            const unsigned int id = static_cast<unsigned int>( laser.id );
            std::cout << "id : " << id << "\n";

            // Laser TimeStamp ( microseconds )
            const long long timestamp = laser.time;
            std::cout << "timestamp : " << timestamp << "\n";

            /*
            // Laser TimeStamp ( MM/DD/YY hh:mm:ss.ffffff )
            std::time_t time = static_cast<int>( laser.time / 1000000 );
            const tm* localtime = std::localtime( &time );
            std::cout << "localtime : " << std::put_time( localtime, "%c" ) << "." << static_cast<int>( laser.time % 1000000 ) << "\n";
            */

            std::cout << std::endl;
        }
    }

    return 0;
}

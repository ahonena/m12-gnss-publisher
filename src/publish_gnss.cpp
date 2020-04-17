/* Looked example from:
http://www.microhowto.info/howto/listen_for_and_receive_udp_datagrams_in_c.html
*/


#include <iostream>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <bitset>
#include <array>


#include "GSOF_M12.hpp"
#include "GSOF_M12Plugin.hpp"

#include "GSOF_attitude.hpp"
#include "GSOF_attitudePlugin.hpp"
#include "GSOF_dop.hpp"
#include "GSOF_dopPlugin.hpp"
#include "GSOF_llh.hpp"
#include "GSOF_llhPlugin.hpp"
#include "GSOF_positionsigma.hpp"
#include "GSOF_positionsigmaPlugin.hpp"
#include "GSOF_positiontime.hpp"
#include "GSOF_positiontimePlugin.hpp"
#include "GSOF_positionvcv.hpp"
#include "GSOF_positionvcvPlugin.hpp"
#include "GSOF_velocity.hpp"
#include "GSOF_velocityPlugin.hpp"
#include <dds/domain/ddsdomain.hpp>
#include <dds/core/ddscore.hpp>
#include <dds/topic/ddstopic.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>

//#define PRINT_STUFF

int main(){
  std::cout << "publish_gnss stub" << std::endl;


  const char* hostname=0; /* wildcard */
  const char* portname="28002";
  struct addrinfo hints;
  memset(&hints,0,sizeof(hints));
  hints.ai_family=AF_INET;
  hints.ai_socktype=SOCK_DGRAM;
  hints.ai_protocol=IPPROTO_UDP;
  hints.ai_flags=AI_PASSIVE;
  struct addrinfo* res=0;
  int err=getaddrinfo(hostname,portname,&hints,&res);
  if (err!=0) {
    fprintf(stderr, "failed to resolve local socket address (err=%d)\n",err);
    return -1;
  }

  int fd=socket(res->ai_family,res->ai_socktype,res->ai_protocol);
  if (fd==-1) {
    fprintf(stderr, "socket error: %s\n",strerror(errno));
    return -1;
  }

  if (bind(fd,res->ai_addr,res->ai_addrlen)==-1) {
    fprintf(stderr, "bind error: %s\n",strerror(errno));
    return -1;
  }
  freeaddrinfo(res);

  int dds_domain = 0;
  dds::domain::DomainParticipant participant_m12(dds_domain);
  dds::pub::Publisher publisher_m12(participant_m12);

  dds::topic::Topic<GSOF_M12> GSOF_M12_topic(participant_m12, "M12_GNSS");
  /*
  dds::topic::Topic<GSOF_attitude> GSOF_attitude_topic(participant_m12, "GNSS_Attitude");
  dds::topic::Topic<GSOF_dop> GSOF_dop_topic(participant_m12, "GNSS_DOP");
  dds::topic::Topic<GSOF_llh> GSOF_llh_topic(participant_m12, "GNSS_LLH");
  dds::topic::Topic<GSOF_positionsigma> GSOF_positionsigma_topic(participant_m12, "GNSS_sigma");
  dds::topic::Topic<GSOF_positiontime> GSOF_positiontime_topic(participant_m12, "GNSS_time");
  dds::topic::Topic<GSOF_positionvcv> GSOF_positionvcv_topic(participant_m12, "GNSS_VCV");
  dds::topic::Topic<GSOF_velocity> GSOF_velocity_topic(participant_m12, "GNSS_velocity");
*/
  dds::pub::DataWriter<GSOF_M12> writer_GSOF_M12_topic(publisher_m12, GSOF_M12_topic);
  /*
  dds::pub::DataWriter<GSOF_attitude> writer_GSOF_attitude_topic(publisher_m12, GSOF_attitude_topic);
  dds::pub::DataWriter<GSOF_dop> writer_GSOF_dop_topic(publisher_m12, GSOF_dop_topic);
  dds::pub::DataWriter<GSOF_llh> writer_GSOF_llh_topic(publisher_m12, GSOF_llh_topic);
  dds::pub::DataWriter<GSOF_positionsigma> writer_GSOF_positionsigma_topic(publisher_m12, GSOF_positionsigma_topic);
  dds::pub::DataWriter<GSOF_positiontime> writer_GSOF_positiontime_topic(publisher_m12, GSOF_positiontime_topic);
  dds::pub::DataWriter<GSOF_positionvcv> writer_GSOF_positionvcv_topic(publisher_m12, GSOF_positionvcv_topic);
  dds::pub::DataWriter<GSOF_velocity> writer_GSOF_velocity_topic(publisher_m12, GSOF_velocity_topic);
*/
  GSOF_M12 GSOF_M12_sample;
  /*
  GSOF_attitude GSOF_attitude_sample;
  GSOF_dop GSOF_dop_sample;
  GSOF_llh GSOF_llh_sample;
  GSOF_positionsigma GSOF_positionsigma_sample;
  GSOF_positiontime GSOF_positiontime_sample;
  GSOF_positionvcv GSOF_positionvcv_sample;
  GSOF_velocity GSOF_velocity_sample;
*/


  // https://www.trimble.com/OEM_ReceiverHelp/v5.11/en/GSOFmessages_GSOF.html
  while(1){

    char buffer[1024];
    struct sockaddr_storage src_addr;
    socklen_t src_addr_len=sizeof(src_addr);
    ssize_t count=recvfrom(fd,buffer,sizeof(buffer),0,(struct sockaddr*)&src_addr,&src_addr_len);
    if (count==-1) {
      fprintf(stderr, "recvfrom error: %s\n",strerror(errno));
      return -1;
    }
    else if (count==sizeof(buffer)) {
      fprintf(stderr, "datagram too large for buffer: truncated\n");
    }
    else{
      #ifdef PRINT_STUFF
      std::cout << std::endl;
      printf("GNSS message received, length %d \n", (int) count);
      #endif

      int GNSS_msg_size = count;
      uint8_t report_packet_type = buffer[2];
      //std::cout << "REcord packet type: ";
      //std::cout << std::hex << int(report_packet_type) << std::endl;
      if(report_packet_type != 0x40){
        std::cout << "Received report packet was not of type 0x40" << std::endl;
      }
      else{
        //std::cout << "The message is" << std::endl << std::endl;

        // First 6 bytes are just report package info, not GNSS data
        int packet_offset = 7;
        // The last two bytes are checksum and EOM
        while(packet_offset < GNSS_msg_size - 2){
          uint8_t gsof_type = buffer[packet_offset];
          uint8_t gsof_msg_length = buffer[packet_offset+1];

          // TIME

          uint8_t posflags2 = 0x00;

          bool differential_position = false;

          // LLH
          uint8_t latitude_raw[8];
          double latitude = 0;

          uint8_t longitude_raw[8];
          double longitude = 0;

          uint8_t height_raw[8];
          double height = 0;

          // velocity
          uint8_t velocity_flags;

          uint8_t horizontal_speed_raw[4];
          float horizontal_speed;

          uint8_t heading_consecutive_raw[4];
          float heading_consecutive;

          uint8_t vertical_speed_raw[4];
          float vertical_speed;


          // Sigma
          // 2-5 float position RMS
          // 6-9 float sigma east
          // 10-13 float sigma north
          // 14-17 float cov_EN
          // 18-21 float sigma up
          // 22-25 float semimajoraxis
          // 26-29 float semiminoraxis
          // 30-33 float orientation
          // 34-37 float univar
          // 38-39 short epochs

          uint8_t sigma_east_raw[4];
          float sigma_east;

          uint8_t sigma_north_raw[4];
          float sigma_north;

          uint8_t sigma_up_raw[4];
          float sigma_up;



          // Attitude:

          // 2-5 long GPS time
          // 6 flags
          // 7 number of satellites
          // 8 calcmode
          // 9 unused
          // 10-17 double pitch
          // 18-25 double yaw
          // 26-33 double roll
          // 34-41 double masterslaverange
          // 42-43 short PDOP
          // 44-47 float pitchvar
          // 48-51 float yawvar
          // 52-55 float rollvar
          // 56-59 float pitchyawcov
          // 60-63 float pitchrollcov
          // 64-67 float yawrollcov
          // 68-71 float masterslaverangecov

          uint8_t attitude_flags;

          uint8_t pitch_raw[8];
          double pitch;

          uint8_t yaw_raw[8];
          double yaw;

          uint8_t roll_raw[8];
          double roll;


          #ifdef PRINT_STUFF
          std::cout << "packet_offset: " << packet_offset << std::endl;
          #endif

          switch(gsof_type){
//------------------------------------------------------------------------------


            case 0x01 :
            #ifdef PRINT_STUFF
            std::cout << "GSOF message: TIME" << std::endl;
            #endif

            posflags2 = buffer[packet_offset + 10];
            differential_position = (bool) posflags2 & 0x01;
            GSOF_M12_sample.differential_position(differential_position);
            //GSOF_positiontime_sample.position_flags_1(posflags2);
            // 11 init number
            //writer_GSOF_positiontime_topic.write(GSOF_positiontime_sample);
            //writer_GSOF_M12_topic.write(GSOF_M12_sample);
            packet_offset += gsof_msg_length + 2;
            break;
//------------------------------------------------------------------------------


            case 0x02 :
            #ifdef PRINT_STUFF
            std::cout << "GSOF message: LLH" << std::endl;
            #endif

            // 2-9 double, WGS-84 Latitude
            // 10-17  double, WGS-84 Longitude
            // 18-25 double, WGS-84 Height

            for(int i = 0; i < 8; i++){
              latitude_raw[7-i] = buffer[packet_offset + 2 + i];
              longitude_raw[7-i] = buffer[packet_offset + 10 + i];
              height_raw[7-i] = buffer[packet_offset + 18 + i];
            }

            memcpy(&latitude, latitude_raw, 8);
            memcpy(&longitude, longitude_raw, 8);
            memcpy(&height, height_raw, 8);

            /*
            GSOF_llh_sample.latitude(latitude);
            GSOF_llh_sample.longitude(longitude);
            GSOF_llh_sample.height(height);
*/
            GSOF_M12_sample.latitude(latitude);
            GSOF_M12_sample.longitude(longitude);
            GSOF_M12_sample.height(height);
            #ifdef PRINT_STUFF
            std::cout << "Latitude: " << latitude << std::endl;
            std::cout << "Longitude: " << longitude << std::endl;
            std::cout << "Height: " << height << std::endl;
            #endif
            //writer_GSOF_llh_topic.write(GSOF_llh_sample);
            packet_offset += gsof_msg_length + 2;
            break;
//------------------------------------------------------------------------------


            case 0x08 :
            #ifdef PRINT_STUFF
            std::cout << "GSOF message: Velocity" << std::endl;
            #endif
            //std::cout << "Velocity msg length: " << (int) gsof_msg_length << std::endl;
            // 2 velocity flags
            velocity_flags = buffer[packet_offset + 2];
            //GSOF_velocity_sample.velocity_flags(velocity_flags);
            // 3-6 float horizontal speed
            // 7-10 float heading consecutive
            // 11-14 float vertical speed
            for(int i = 0; i < 4; i++){
              horizontal_speed_raw[3-i] = buffer[packet_offset + 3 + i];
              heading_consecutive_raw[3-i] = buffer[packet_offset + 7 + i];
              vertical_speed_raw[3-i] = buffer[packet_offset + 11 + i];

            }
            memcpy(&horizontal_speed, horizontal_speed_raw, 4);
            memcpy(&heading_consecutive, heading_consecutive_raw, 4);
            memcpy(&vertical_speed, vertical_speed_raw, 4);

            /*
            GSOF_velocity_sample.horizontal_speed(horizontal_speed);
            GSOF_velocity_sample.heading_consecutive(heading_consecutive);
            GSOF_velocity_sample.vertical_speed(vertical_speed);
            */

            GSOF_M12_sample.speed(horizontal_speed);
            GSOF_M12_sample.heading(heading_consecutive);

            #ifdef PRINT_STUFF
            std::cout << "horizontal_speed: " << horizontal_speed << std::endl;
            std::cout << "heading_consecutive: " << heading_consecutive << std::endl;
            std::cout << "vertical_speed: " << vertical_speed << std::endl;
            #endif

            //writer_GSOF_velocity_topic.write(GSOF_velocity_sample);
            packet_offset += gsof_msg_length + 2;
            break;
//------------------------------------------------------------------------------


            case 0x09 :
            #ifdef PRINT_STUFF
            std::cout << "GSOF message: DOP" << std::endl;
            #endif
            packet_offset += gsof_msg_length + 2;
            break;
//------------------------------------------------------------------------------


            case 0x0C :
            #ifdef PRINT_STUFF
            std::cout << "GSOF message: SIGMA" << std::endl;
            #endif
            // 2-5 float position RMS
            // 6-9 float sigma east
            // 10-13 float sigma north
            // 14-17 float cov_EN
            // 18-21 float sigma up
            // 22-25 float semimajoraxis
            // 26-29 float semiminoraxis
            // 30-33 float orientation
            // 34-37 float univar

            for(int i = 0; i < 4; i++){

              sigma_east_raw[3-i] = buffer[packet_offset + 6 + i];
              sigma_north_raw[3-i] = buffer[packet_offset + 10 + i];
              sigma_up_raw[3-i] = buffer[packet_offset + 18 + i];
              /*
              position_RMS_raw[3-i] = buffer[packet_offset + 2 + i];
              cov_EN_raw[3-i] = buffer[packet_offset + 14 + i];
              semimajoraxis_raw[3-i] = buffer[packet_offset + 22 + i];
              semiminoraxis_raw[3-i] = buffer[packet_offset + 26 + i];
              orientation_raw[3-i] = buffer[packet_offset + 30 + i];
              univar_raw[3-i] = buffer[packet_offset + 34 + i];
              */
            }

            //memcpy(&position_RMS, position_RMS_raw,4);
            memcpy(&sigma_east, sigma_east_raw,4);
            memcpy(&sigma_north, sigma_north_raw,4);
            //memcpy(&cov_EN, cov_EN_raw,4);
            memcpy(&sigma_up, sigma_up_raw,4);
            //memcpy(&semimajoraxis, semimajoraxis_raw,4);
            //memcpy(&semiminoraxis, semiminoraxis_raw,4);
            //memcpy(&orientation, orientation_raw,4);
            //memcpy(&univar, univar_raw,4);

            GSOF_M12_sample.sigma_east(sigma_east);
            GSOF_M12_sample.sigma_north(sigma_north);
            GSOF_M12_sample.sigma_up(sigma_up);
            /*
            GSOF_positionsigma_sample.position_rms(position_RMS);
            GSOF_positionsigma_sample.sigma_east(sigma_east);
            GSOF_positionsigma_sample.sigma_north(sigma_north);
            GSOF_positionsigma_sample.E_N_covar(cov_EN);
            GSOF_positionsigma_sample.sigma_up(sigma_up);
            GSOF_positionsigma_sample.semi_major_axis(semimajoraxis);
            GSOF_positionsigma_sample.semi_minor_axis(semiminoraxis);
            GSOF_positionsigma_sample.orientation(orientation);
            GSOF_positionsigma_sample.unit_variance(univar);
            */
            // 38-39 short epochs

//            epochs_raw[1] = buffer[packet_offset + 38];
//            epochs_raw[0] = buffer[packet_offset + 39];

//            memcpy(&epochs, epochs_raw,2);
    /*
            GSOF_positionsigma_sample.epochs(epochs);
*/
            #ifdef PRINT_STUFF
            std::cout << "position_RMS: " << position_RMS << std::endl;
            std::cout << "sigma_east: " << sigma_east << std::endl;
            std::cout << "sigma_north: " << sigma_north << std::endl;
            std::cout << "cov_EN: " << cov_EN << std::endl;
            std::cout << "sigma_up: " << sigma_up << std::endl;
            std::cout << "semimajoraxis: " << semimajoraxis << std::endl;
            std::cout << "semiminoraxis: " << semiminoraxis << std::endl;
            std::cout << "orientation: " << orientation << std::endl;
            std::cout << "univar: " << univar << std::endl;

            #endif

            //writer_GSOF_positionsigma_topic.write(GSOF_positionsigma_sample);

            packet_offset += gsof_msg_length + 2;
            break;
//------------------------------------------------------------------------------


            case 0x1B :
            #ifdef PRINT_STUFF
            std::cout << "GSOF message: Attitude" << std::endl;
            #endif
            // 2-5 long GPS time
            // 6 flags
            //attitude_flags = buffer[packet_offset + 6];
            //GSOF_attitude_sample.flags(attitude_flags);
            // 7 number of satellites
            // 8 calcmode
            // 9 unused
            // 10-17 double pitch
            // 18-25 double yaw
            // 26-33 double roll
            for(int i = 0; i < 8; i++){
              pitch_raw[7-i] = buffer[packet_offset + 10 + i];
              yaw_raw[7-i] = buffer[packet_offset + 18 + i];
              roll_raw[7-i] = buffer[packet_offset + 26 + i];
            }
            memcpy(&pitch, pitch_raw, 8);
            memcpy(&yaw, yaw_raw, 8);
            memcpy(&roll, roll_raw, 8);
            /*
            GSOF_attitude_sample.pitch(pitch);
            GSOF_attitude_sample.yaw(yaw);
            GSOF_attitude_sample.roll(roll);
            */
            GSOF_M12_sample.pitch(pitch);
            GSOF_M12_sample.yaw(yaw);
            GSOF_M12_sample.roll(roll);

            #ifdef PRINT_STUFF
            std::cout << "pitch: " << pitch << std::endl;
            std::cout << "yaw: " << yaw << std::endl;
            std::cout << "roll: " << roll << std::endl;
            #endif

            // 68-71 float masterslaverangecov
            packet_offset += gsof_msg_length + 2;
            break;
//------------------------------------------------------------------------------


            case 0x0B :
            #ifdef PRINT_STUFF
            std::cout << "GSOF message: Position VCV" << std::endl;
            #endif
            packet_offset += gsof_msg_length + 2;
            break;
//------------------------------------------------------------------------------


            default:
            #ifdef PRINT_STUFF
            std::cout << "Unknown gsof message" << std::endl;
            #endif
            packet_offset += gsof_msg_length + 2;
            break;
          }
          writer_GSOF_M12_topic.write(GSOF_M12_sample);
        }
        #ifdef PRINT_STUFF
        std::cout << "FINAL PACKET OFFSET: " << packet_offset << std::endl;
        #endif
      }
      /*

      if(just_once){
      for(int i = 0; i < GNSS_msg_size; i++){
      raw_data.at(GNSS_msg_size - i - 1) = buffer[i];
      std::cout << std::hex << (uint16_t) raw_data.at(GNSS_msg_size - i - 1) << std::endl;
    }
  }
  just_once = false;
  std::cout << std::endl << std::endl;
  */
  //handle_datagram(buffer,count);
}







}
}

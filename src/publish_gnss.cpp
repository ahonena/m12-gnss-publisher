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

#include <array>
//#define GNSS_PORT 28002;

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



  bool just_once = true;
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
      std::cout << std::endl;
      printf("GNSS message received, length %d \n", (int) count);
      int GNSS_msg_size = count;
      uint8_t report_packet_type = buffer[2];
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
          std::cout << "packet_offset: " << packet_offset << std::endl;
          switch(gsof_type){
            case 0x01 :
            std::cout << "GSOF message: TIME" << std::endl;





            packet_offset += gsof_msg_length + 2;
            break;

            case 0x02 :
            std::cout << "GSOF message: LLH" << std::endl;
            packet_offset += gsof_msg_length + 2;
            break;

            case 0x08 :
            std::cout << "GSOF message: Velocity" << std::endl;
            packet_offset += gsof_msg_length + 2;
            break;

            case 0x09 :
            std::cout << "GSOF message: DOP" << std::endl;
            packet_offset += gsof_msg_length + 2;
            break;

            case 0x0C :
            std::cout << "GSOF message: SIGMA" << std::endl;
            packet_offset += gsof_msg_length + 2;
            break;

            case 0x1B :
            std::cout << "GSOF message: Attitude" << std::endl;
            packet_offset += gsof_msg_length + 2;
            break;

            case 0x0B :
            std::cout << "GSOF message: Position VCV" << std::endl;
            packet_offset += gsof_msg_length + 2;
            break;

            default:
            std::cout << "Unknown gsof message" << std::endl;
            packet_offset += gsof_msg_length + 2;
            break;
          }
        }
        std::cout << "FINAL PACKET OFFSET: " << packet_offset << std::endl;
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

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



  while(1){

    char buffer[1024];
    struct sockaddr_storage src_addr;
    socklen_t src_addr_len=sizeof(src_addr);
    ssize_t count=recvfrom(fd,buffer,sizeof(buffer),0,(struct sockaddr*)&src_addr,&src_addr_len);
    if (count==-1) {
        fprintf(stderr, "recvfrom error: %s\n",strerror(errno));
        return -1;
    } else if (count==sizeof(buffer)) {
        fprintf(stderr, "datagram too large for buffer: truncated\n");
    } else {
      printf("UDP message received, length %d \n", count);
        //handle_datagram(buffer,count);
    }






  }
}

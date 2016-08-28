#ifndef DEF_UDP_CLIENT
#define DEF_UDP_CLIENT

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define SERVERPORT "4950"    // the port users will be connecting to


class Udp_client{
    public:
    Udp_client(char* hostname);
    Udp_client(char* hostname,char* serverport);
    ~Udp_client();
    int send_msg(const char* message); 
  
    private:
    int m_sockfd;
    int m_rv;
    int m_numbytes;
    struct addrinfo m_hints;
    struct addrinfo *m_servinfo;
    struct addrinfo *m_p;   
};



#endif

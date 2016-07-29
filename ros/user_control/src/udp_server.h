#ifndef DEF_UDP_SERVER
#define DEF_UDP_SERVER

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
#include <sys/time.h>
#include <sys/select.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>


#define MAXBUFLEN 100
#define MYPORT "4950" //the port users will be connecting to

class Udp_server{
    public:
    Udp_server();
    Udp_server(char* myport);
    ~Udp_server();
    void *get_in_addr(struct sockaddr *sa);
    const char* get_addr_name();
    int msg_box(std::string &command);
 
    private:
    int m_sockfd;
    int m_rv;
    int m_numbytes;
    char m_buf[MAXBUFLEN];
    char m_s[INET6_ADDRSTRLEN];
    socklen_t m_addr_len;
    struct addrinfo m_hints;
    struct addrinfo *m_servinfo;
    struct addrinfo *m_p;
    struct sockaddr_storage m_their_addr;

};

#endif

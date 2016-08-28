#include "udp_client.h"

Udp_client::Udp_client(char* hostname){
	memset(&(this->m_hints), 0, sizeof m_hints);
    this->m_hints.ai_family = AF_UNSPEC;
    this->m_hints.ai_socktype = SOCK_DGRAM;

    if ((this->m_rv = getaddrinfo(hostname, SERVERPORT, &this->m_hints, &this->m_servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(this->m_rv));
    }

    // loop through all the results and make a socket
    for(this->m_p = this->m_servinfo; this->m_p != NULL; this->m_p = this->m_p->ai_next) {
        if ((this->m_sockfd = socket(this->m_p->ai_family, this->m_p->ai_socktype, this->m_p->ai_protocol)) == -1) {
            perror("talker: socket");
            continue;
        }

        break;
    }

    if (this->m_p == NULL) {
        fprintf(stderr, "talker: failed to bind socket\n");
    }

}

Udp_client::Udp_client(char* hostname,char* serverport){
   memset(&(this->m_hints), 0, sizeof m_hints);
   this->m_hints.ai_family = AF_UNSPEC;
   this->m_hints.ai_socktype = SOCK_DGRAM;

   if ((this->m_rv = getaddrinfo(hostname, serverport, &this->m_hints, &this->m_servinfo)) != 0) {
       fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(this->m_rv));
   }

   // loop through all the results and make a socket
   for(this->m_p = this->m_servinfo; this->m_p != NULL; this->m_p = this->m_p->ai_next) {
       if ((this->m_sockfd = socket(this->m_p->ai_family, this->m_p->ai_socktype, this->m_p->ai_protocol)) == -1) {
           perror("talker: socket");
           continue;
       }

       break;
   }

   if (this->m_p == NULL) {
       fprintf(stderr, "talker: failed to bind socket\n");
   }

}

int Udp_client::send_msg(const char* message){
    if ((this->m_numbytes = sendto(this->m_sockfd, message, strlen(message), 0, (struct sockaddr*) this->m_p->ai_addr, this->m_p->ai_addrlen)) == -1) {
       return 0;
    }
    
    return 1;
}

Udp_client::~Udp_client(){
	freeaddrinfo(this->m_servinfo);
	close(this->m_sockfd);
}

#include "udp_server.h"
using namespace std;

Udp_server::Udp_server(){

memset(&(this->m_hints),0,sizeof (this->m_hints));

   this->m_hints.ai_family = AF_UNSPEC;
   this->m_hints.ai_socktype = SOCK_DGRAM;
   this->m_hints.ai_flags = AI_PASSIVE;

   //create socket   
  if((this->m_rv=getaddrinfo(NULL, MYPORT, &this->m_hints, &this->m_servinfo)) != 0){
      fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(this->m_rv));
    }

  for(this->m_p = this->m_servinfo;this->m_p != NULL; this->m_p = this->m_p->ai_next) {
        if ((this->m_sockfd = socket(this->m_p->ai_family, this->m_p->ai_socktype,
                this->m_p->ai_protocol)) == -1) {
            perror("listener: socket");
            continue;
        }

        if (bind(this->m_sockfd, this->m_p->ai_addr, this->m_p->ai_addrlen) == -1){
            close(this->m_sockfd);
            perror("listener: bind");
            continue;
        }
        break;
    }

    if (this->m_p == NULL) {
        fprintf(stderr, "listener: failed to bind socket\n");
    }

    freeaddrinfo(this->m_servinfo);

}


Udp_server::Udp_server(char* myport){

	memset(&(this->m_hints),0,sizeof (this->m_hints));
	
	this->m_hints.ai_family = AF_UNSPEC;
	this->m_hints.ai_socktype = SOCK_DGRAM;
	this->m_hints.ai_flags = AI_PASSIVE;
	
	  
	if((this->m_rv=getaddrinfo(NULL, myport, &this->m_hints, &this->m_servinfo)) != 0){
	  fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(this->m_rv));
	}
	
	//create socket 
	for(this->m_p = this->m_servinfo;this->m_p != NULL; this->m_p = this->m_p->ai_next) {
		if ((this->m_sockfd = socket(this->m_p->ai_family, this->m_p->ai_socktype,
				this->m_p->ai_protocol)) == -1) {
			perror("listener: socket");
			continue;
		}
	
		if (bind(this->m_sockfd, this->m_p->ai_addr, 
			this->m_p->ai_addrlen) == -1){
			close(this->m_sockfd);
			perror("listener: bind");
			continue;
		}
		break;
	}
	
	if (this->m_p == NULL) {
		fprintf(stderr, "listener: failed to bind socket\n");
	}
	
	freeaddrinfo(this->m_servinfo);
}

/*
int Udp_server::wait_for_stdin(int seconds){
	
    fd_set set;
    struct timeval timeout = { seconds, 0 };
 
    FD_ZERO(&set);
    FD_SET(0, &set);
    return select(1, &set, NULL, NULL, &timeout) == 1;
}*/

void* Udp_server::get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

const char* Udp_server::get_addr_name()
{
	return inet_ntop(this->m_their_addr.ss_family,
		         get_in_addr((struct sockaddr *)&(this->m_their_addr)),
		         this->m_s, sizeof (this->m_s));
}

int Udp_server::msg_box(std::string &command){
    
    int rv;
	fd_set readfds;
	struct timeval tv;

    command.clear(); 
    ROS_DEBUG("listener: waiting to recvfrom...\n");
	
	// clear the set ahead of time
	FD_ZERO(&readfds);
	
	// add our descriptors to the set
	FD_SET(this->m_sockfd, &readfds);
	
	// wait until socket has data ready to be recv()d (timeout 3.0 secs)
	tv.tv_sec = 0;
	tv.tv_usec = 100000;
	rv = select(this->m_sockfd+1, &readfds, NULL, NULL, &tv);
	if(rv>0){
		this->m_addr_len = sizeof (this->m_their_addr);
		if ((this->m_numbytes = recvfrom(this->m_sockfd, this->m_buf,
			MAXBUFLEN-1 , 0, (struct sockaddr *)&(this->m_their_addr), 
			&(this->m_addr_len))) == -1) {
				perror("recvfrom");
				exit(1);
			}
		
		ROS_DEBUG("listener: got packet from %s\n",
		inet_ntop(this->m_their_addr.ss_family,
				  get_in_addr((struct sockaddr *)&(this->m_their_addr)),
				  this->m_s, sizeof (this->m_s)));
		ROS_DEBUG("listener: packet is %d bytes long\n",this->m_numbytes);
		m_buf[this->m_numbytes] = '\0';
		ROS_DEBUG("listener: packet contains \"%s\"\n", this->m_buf);     
		command.replace(0, this->m_numbytes+1, m_buf);
		
		return 1;
	}else if(rv==-1){
		return 0;
	}else if(rv==0){
		//timeout occurs
		return 2;
	}
   return 0;
}

Udp_server::~Udp_server(){
     close(this->m_sockfd);
}

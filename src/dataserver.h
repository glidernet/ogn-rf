#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

#include <vector>

#include "socket.h"

// ======================================================================================================

class TCP_DataServer
{ public:
   int Server;
   std::vector<int> Client;

  public:
   TCP_DataServer() { BlockSIGPIPE(); Server=(-1); }
  ~TCP_DataServer() { Close(); }

   void static BlockSIGPIPE(void)
   { sigset_t set;                             // disable SIGPIPE signal for the current thread when writing to a closed socket
     sigemptyset (&set);                       // this is required for the DataServer to function properly.
     sigaddset (&set, SIGPIPE);
     pthread_sigmask(SIG_BLOCK, &set, 0); }

   int Listen(int Port, int MaxCons=16)
   { Close();
     Server = socket(AF_INET, SOCK_STREAM, 0); if(Server<0) return -1;

     int Set=1;
     setsockopt(Server, SOL_SOCKET, SO_REUSEADDR, &Set, sizeof(Set));

     int Flags = fcntl(Server, F_GETFL, 0);
     Flags |=  O_NONBLOCK;
     fcntl(Server, F_SETFL, Flags);

     struct sockaddr_in ListenAddress;
     bzero((char *) &ListenAddress, sizeof(ListenAddress));
     ListenAddress.sin_family      = AF_INET;
     ListenAddress.sin_addr.s_addr = htonl(INADDR_ANY);
     ListenAddress.sin_port        = htons(Port);
     if(bind(Server, (struct sockaddr *) &ListenAddress, sizeof(ListenAddress))<0) { Close(); return -1; }

     if(listen(Server, MaxCons)<0) { Close(); return -1; }

     return 0; }

   int Clients(void) const { return Client.size(); }

   int Accept(void)
   { int Count=0;
     for( ; ; )
     { struct sockaddr Addr; socklen_t Len;
       int New=accept(Server, &Addr, &Len); if(New<0) break;
       // int Flag = 1;
       // setsockopt(New, SOL_SOCKET, SO_NOSIGPIPE, &Flag, sizeof(Flag));
       printf("Accepted a new client: %d\n", New);
       Client.push_back(New); Count++; }
     return Count; }

   void Close(size_t Idx)
   { if(Idx>=Client.size()) return;
     close(Client[Idx]); Client[Idx]=(-1); }

   int RemoveClosed(void)
   { int Count=0;
     for(size_t Idx=0; Idx<Client.size(); )
     { if(Client[Idx]<0)
       { size_t NewSize=Client.size()-1;
         Client[Idx]=Client[NewSize];
         Client.resize(NewSize);
         Count++; }
       Idx++; }
     return Count; }

   void Close(void)
   { if(Server<0) return;
     for(size_t Idx=0; Idx<Client.size(); Idx++)
     { if(Client[Idx]>=0) close(Client[Idx]); }
     Client.resize(0);
     close(Server); Server=(-1);
   }

   int isListenning(void) const { return Server>=0; }

} ;

// ======================================================================================================


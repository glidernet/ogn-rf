/*
    OGN - Open Glider Network - http://glidernet.org/
    Copyright (c) 2015 The OGN Project

    A detailed list of copyright holders can be found in the file "AUTHORS".

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>

// =========================================================================================

#ifndef __SOCKET_H__
#define __SOCKET_H__

#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
// #include <netinet/ip_icmp.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/sendfile.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#ifdef __MACH__
#define MSG_NOSIGNAL 0
#endif


class SocketAddress                    // IP address and port
{ public:

   struct sockaddr_in Address;

  private:

   static const int TmpStringLen = 64;
   char TmpString[TmpStringLen];

  public:

   SocketAddress()
     { Init(); }

   void Init(void)
     { Address.sin_family = AF_INET; setIP(); }

   // set IP and port from an ASCII string: "IP:port", IP can be numeric or a host name
   int set(const char *HostColonPort)
     { const char *Colon=strchr(HostColonPort,':');
       if(Colon==0) return setIP(HostColonPort);
       int Port; if(sscanf(Colon+1,"%d",&Port)!=1) return -1;
       if((Port<0)||(Port>0xFFFF)) return -1;
       setPort((unsigned short)Port);
       int Len=Colon-HostColonPort; if(Len>=TmpStringLen) return -1;
       memcpy(TmpString, HostColonPort, Len); TmpString[Len]='\0';
       int Error = setIP(TmpString);
       return Error; }

   // set only the IP (includes DNS name resolve)
   int setIP(const char *AsciiHost)
     { in_addr_t IP=inet_addr(AsciiHost);
       if(IP != (in_addr_t)(-1)) { Address.sin_addr.s_addr=IP; return 0; }
       struct hostent *Host = gethostbyname(AsciiHost); if(Host==0) return -1;
       char *AddrPtr=Host->h_addr_list[0];              if(AddrPtr==0) return -1;
       memcpy(&Address.sin_addr, Host->h_addr_list[0],  Host->h_length);
       return 0; }

   // set the IP from a 32-bit integer
   int setIP(unsigned long IP=INADDR_ANY)
     { Address.sin_addr.s_addr = htonl(IP); return 0; }

   // get IP as a 32-bit integer
   unsigned long getIP(void) const
     { return ntohl(Address.sin_addr.s_addr); }

   // is the address defined already ?
   bool isSetup(void) const
     { return getIP()!=INADDR_ANY; }

   int setBroadcast(void)
     { Address.sin_addr.s_addr = htonl(INADDR_BROADCAST); return 0; }

   // set the port
   int setPort(unsigned short Port)
     { Address.sin_port = htons(Port); return 0; }

   // get the port
   unsigned short getPort(void) const
     { return ntohs(Address.sin_port); }

   // get IP as an ASCII string (to print)
   char *getAsciiIP(void) const
     { return inet_ntoa(Address.sin_addr); }

   // get my own host name
   char *getHostName(void) { return getHostName(TmpString, TmpStringLen); }

   static char *getHostName(char *Name, int NameLen)
     { if(gethostname(Name, NameLen)<0) return 0;
       return Name; }

   // get the "IP:port" ASCII string, IP will be numeric
   char *getIPColonPort(void)
     { char *IP = getAsciiIP(); if(IP==0) return IP;
       if(strlen(IP)>(TmpStringLen-8)) return 0;
       unsigned short Port=getPort();
       sprintf(TmpString, "%s:%d", IP, Port);
       return TmpString; }

} ;

class SocketBuffer             // data buffer for IP sockets
{ public:

   char *Data;                 //         data storage
   size_t Allocated;           // [bytes] allocated
   size_t Len;                 // [bytes] filled with data
   size_t Done;                // [bytes] processed
   static const size_t AllocUnit = 4096; // allocation step

  public:

   SocketBuffer()
     { Data=0; Allocated=0; Len=0; Done=0; }

   ~SocketBuffer()
     { Free(); }

   void Free(void)
     { if(Data) { free(Data); Data=0; }
       Allocated=0; Len=0; Done=0; }

   size_t Relocate(size_t Size)
     { if(Size<=Allocated) return Allocated;
       // printf("Relocate(%d)",Size);
       size_t Units=(Size+AllocUnit-1)/AllocUnit; Size=Units*AllocUnit;
       // printf(" => Units=%d, Size=%d\n", Units, Size);
       Data=(char *)realloc(Data, Size); if(Data==0) Free(); else Allocated=Size;
       return Allocated; }

   int NullTerm(void)        // put null byte at the end, thus it can be treated as a null-terminated string
     { if(Relocate(Len+1)<=0) return 0;
       Data[Len]=0;
       return Len; }

   void Clear(void)
     { Len=0; Done=0; }

   bool isDone(void) const   // is all data processed ?
     { return Done==Len; }

   int Delete(size_t Ofs, size_t DelLen) // delete some part of the data (involves memory move)
     { if(Ofs>=Len) return 0;
       if((Ofs+DelLen)>Len) DelLen=Len-Ofs;
       memcpy(Data+Ofs, Data+Ofs+DelLen, Len-DelLen); Len-=DelLen;
       Data[Len]=0; return DelLen; }

   int SearchLineTerm(int StartIdx=0) // search for a line terminator: \r or \n
     { size_t Idx; char Term=0;
       for( Idx=StartIdx; Idx<Len; Idx++)
       { Term=Data[Idx];
         if((Term=='\r')||(Term=='\n')) break; }
       if(Idx>=Len) return -1;        // return -1 if terminator not found
       return Idx-StartIdx; }         // return the line length (not including the terminator)

   int ReadFromFile(char *FileName)
     { FILE *File = fopen(FileName,"r"); if(File==0) return -1;
       int Total=0;
       for( ; ; )
       { if(Relocate(Len+AllocUnit)<0) { fclose(File); return -1; }
         int ToRead = Allocated-Len;
         int Read = fread(Data+Len, 1, ToRead, File);
         if(Read<0) { fclose(File); return -1; }
         Len+=Read; Total+=Read; if(Read!=ToRead) break;
       }
       fclose(File);
       return Total; }

   int WriteToFile(FILE *File=stdout) const
     { int ToWrite = Len-Done; if(ToWrite<0) ToWrite=0;
       int Written = fwrite(Data+Done, 1, ToWrite, File);
       if(Written<0) return Written;
       return Written==ToWrite ? Written:-1; }

   int WriteToFile(const char *FileName) const
     { FILE *File = fopen(FileName,"w"); if(File==0) return -1;
       int ToWrite = Len-Done; if(ToWrite<0) ToWrite=0;
       int Written = fwrite(Data+Done, 1, ToWrite, File);
       fclose(File);
       if(Written<0) return Written;
       return Written==ToWrite ? Written:-1; }

// -----------------------------------------------------------------------------

   int LineLen(size_t Ofs=0, size_t MaxLen=256) const
   { size_t Idx=Ofs;
     for( ; Idx<Len; Idx++)
     { char Byte=Data[Idx]; if( (Byte=='\r') || (Byte=='\n') ) break; }
     return Idx-Ofs; }

   int EOL(size_t Ofs) const
   { if(Ofs>=Len) return 0;
     char Byte1=Data[Ofs];
     if( (Byte1!='\r') && (Byte1!='\n') ) return 0;
     Ofs++;
     if(Ofs>=Len) return 1;
     char Byte2=Data[Ofs];
     if( (Byte2!='\r') && (Byte2!='\n') ) return 1;
     if(Byte2==Byte1) return 1;
     return 2; }

   int getStatus(void) const
   { char Protocol[16]; int Status=0;
     int FirstLineLen=LineLen(0, 128);
     if(FirstLineLen<=8) return -1;
     if(FirstLineLen>=128) return -1;
     if(sscanf(Data, "%s %d", Protocol, &Status)!=2) return -1;
     return Status; }

   int getHeaderLen(void) const
   { size_t Idx=0;
     for( ; ; )
     { int Len=LineLen(Idx);
       int TermLen=EOL(Idx+Len);
       Idx+=Len+TermLen;
       if(Len==0) break; }
     return Idx; }

   int FindTag(const char *Tag, size_t Ofs=0)
   { size_t TagLen=strlen(Tag);
     for(size_t Idx=Ofs; Idx<Len; Idx++)
     { char Byte=Data[Idx]; if(Byte!='<') continue;
       if((Idx+1+TagLen)>=Len) return -1;
       if(memcmp(Tag, Data+Idx+1, TagLen)==0) return Idx-Ofs; }
     return -1; }

   int TagLen(size_t Ofs=0) const
   { for(size_t Idx=Ofs+1; Idx<Len; Idx++)
     { char Byte=Data[Idx]; if(Byte=='>') return Idx-Ofs+1; }
     return -1; }

// -----------------------------------------------------------------------------

} ;

class Socket                   // IP socket
{ public:

   int SocketFile;
   // unsigned long BytesSent, BytesReceived;

  public:

   Socket()
     { SocketFile=(-1); }

   ~Socket()
     { Close(); }

   // create a socket
   int Create(int Type=SOCK_STREAM, int Protocol=IPPROTO_TCP)
     { Close();
       SocketFile=socket(PF_INET, Type, Protocol);
       return SocketFile; }
   int Create_STREAM(void) { return Create(SOCK_STREAM, IPPROTO_TCP); }
   int Create_DGRAM(void) { return Create(SOCK_DGRAM, 0); }

   int Copy(int NewSocketFile)
     { Close();
       return SocketFile=NewSocketFile; }

   // set connect/read/write to be blocking or not
   int setBlocking(int Block=1)
     { int Flags = fcntl(SocketFile,F_GETFL,0);
       if(Block) Flags &= ~O_NONBLOCK;
            else Flags |=  O_NONBLOCK;
       return fcntl(SocketFile,F_SETFL,Flags); }

   int setNonBlocking(void)
     { return setBlocking(0); }

   // avoids waiting (in certain cases) till the socket closes completely after the previous server exits
   int setReuseAddress(int Set=1)
     { return setsockopt(SocketFile, SOL_SOCKET, SO_REUSEADDR, &Set, sizeof(Set)); }

   int setKeepAlive(int KeepAlive=1) // keep checking if connection alive while no data is transmitted
     { return setsockopt(SocketFile, SOL_SOCKET, SO_KEEPALIVE, &KeepAlive, sizeof(KeepAlive)); }

   int setLinger(int ON, int Seconds) // gracefull behavior on socket close
     { struct linger Linger; Linger.l_onoff=ON; Linger.l_linger=Seconds;
       return setsockopt(SocketFile, SOL_SOCKET, SO_LINGER, &Linger, sizeof(Linger)); }

   int setNoDelay(int ON=1)
   { return setsockopt(SocketFile, IPPROTO_TCP, TCP_NODELAY, &ON, sizeof(ON)); }

   int setSendBufferSize(int Bytes)
     { return setsockopt(SocketFile, SOL_SOCKET, SO_SNDBUF, &Bytes, sizeof(Bytes)); }

   int getSendBufferSize(void)
     { int Bytes=0; socklen_t Size;
       int Error=getsockopt(SocketFile, SOL_SOCKET, SO_SNDBUF, &Bytes, &Size);
       return Error<0 ? -1:Bytes; }

   int setReceiveBufferSize(int Bytes)
     { return setsockopt(SocketFile, SOL_SOCKET, SO_RCVBUF, &Bytes, sizeof(Bytes)); }

   int getReceiveBufferSize(void)
     { int Bytes=0; socklen_t Size;
       int Error=getsockopt(SocketFile, SOL_SOCKET, SO_RCVBUF, &Bytes, &Size);
       return Error<0 ? -1:Bytes; }

/* on Cygwin send and receive timeouts seem to have no effect ...
#ifdef __WINDOWS__
   int setReceiveTimeout(double Seconds) // a blocking receive() will not wait forever
   { long Time = (long)floor(1000*Seconds+0.5);
     return setsockopt(SocketFile, SOL_SOCKET, SO_RCVTIMEO, &Time, sizeof(Time)); }

   int setSendTimeout(double Seconds)   // a blocking send() will not wait forever
   { long Time = (long)floor(1000*Seconds+0.5);
      return setsockopt(SocketFile, SOL_SOCKET, SO_SNDTIMEO, &Time, sizeof(Time)); }
#endif
*/

#ifdef __CYGWIN__  // dummy routine for Cygwin, only to satify the compiler
   int setReceiveTimeout(double Seconds) { return -1; }
   int setSendTimeout(double Seconds) { return -1; }
#else
   int setReceiveTimeout(double Seconds) // a blocking receive() will not wait forever
     { struct timeval Time;
       Time.tv_sec  = (long)floor(Seconds);
       Time.tv_usec = (long)floor(1000000*(Seconds-Time.tv_sec)+0.5);
       return setsockopt(SocketFile, SOL_SOCKET, SO_RCVTIMEO, &Time, sizeof(Time)); }

   int setSendTimeout(double Seconds)   // a blocking send() will not wait forever
     { struct timeval Time;
       Time.tv_sec  = (long)floor(Seconds);
       Time.tv_usec = (long)floor(1000000*(Seconds-Time.tv_sec)+0.5);
       return setsockopt(SocketFile, SOL_SOCKET, SO_SNDTIMEO, &Time, sizeof(Time)); }
#endif

#if defined(__MACH__) || defined(__CYGWIN__)
#else
   int getMTU(void)
     { int Bytes;
       if(ioctl(SocketFile, SIOCGIFMTU, &Bytes)<0) return -1;
       return Bytes; }
#endif

   int getReceiveQueue(void)
     { int Bytes;
       if(ioctl(SocketFile, FIONREAD, &Bytes)<0) return -1;
       return Bytes; }

   int getError(void)
     { int ErrorCode=0;
       socklen_t Size=sizeof(ErrorCode);
       int Error=getsockopt(SocketFile, SOL_SOCKET, SO_ERROR, &ErrorCode, &Size);
       return Error<0 ? -1:ErrorCode; }

   int isListenning(void)
     { int Yes=0;
       socklen_t Size=sizeof(Yes);
       int Error=getsockopt(SocketFile, SOL_SOCKET, SO_ACCEPTCONN, &Yes, &Size);
       return Error<0 ? -1:Yes; }

   // listen for incoming UDP connections (become a UDP server)
   int Listen_DGRAM(unsigned short ListenPort)
     { if(SocketFile<0) { if(Create_DGRAM()<0) return -1; }

       setReuseAddress(1);

       struct sockaddr_in ListenAddress;
       ListenAddress.sin_family      = AF_INET;
       ListenAddress.sin_addr.s_addr = htonl(INADDR_ANY);
       ListenAddress.sin_port        = htons(ListenPort);

       if(bind(SocketFile, (struct sockaddr *) &ListenAddress, sizeof(ListenAddress))<0)
       { Close(); return -1; }

       return 0; }

   // listen for incoming TCP connections (become a TCP server)
   int Listen(unsigned short ListenPort, int MaxConnections=8)
     { if(SocketFile<0) { if(Create()<0) return -1; }

       setReuseAddress(1);

       struct sockaddr_in ListenAddress;
       ListenAddress.sin_family      = AF_INET;
       ListenAddress.sin_addr.s_addr = htonl(INADDR_ANY);
       ListenAddress.sin_port        = htons(ListenPort);

       if(bind(SocketFile, (struct sockaddr *) &ListenAddress, sizeof(ListenAddress))<0)
       { Close(); return -1; }

       if(listen(SocketFile, MaxConnections)<0)
       { Close(); return -1; }

       return 0; }

   // accept a new client (when being a server)
   int Accept(Socket &ClientSocket, SocketAddress &ClientAddress)
     { ClientSocket.Close();
       socklen_t ClientAddressLength=sizeof(ClientAddress.Address);
       return ClientSocket.SocketFile=accept(SocketFile, (struct sockaddr *) &(ClientAddress.Address), &ClientAddressLength); }

   // connect to a remote server
   int Connect(SocketAddress &ServerAddress)
     { if(SocketFile<0) { if(Create_STREAM()<0) return -1; } // if no socket yet, create a STREAM-type one.
       socklen_t ServerAddressLength=sizeof(ServerAddress.Address);
       return connect(SocketFile, (struct sockaddr *) &(ServerAddress.Address), ServerAddressLength); }

   // send data (on a connected socket)
   int Send(void *Message, int Bytes, int Flags=MSG_NOSIGNAL)
     { return send(SocketFile, Message, Bytes, Flags); }

   int Send(const char *Message)
     { return Send((void *)Message, strlen(Message)); }

   int Send(SocketBuffer &Buffer, int Flags=MSG_NOSIGNAL)
     { size_t Bytes = Buffer.Len-Buffer.Done; // if(Bytes>4096) Bytes=4096;
       int SentBytes=Send(Buffer.Data+Buffer.Done, Bytes, Flags);
       if(SentBytes>0) Buffer.Done+=SentBytes;
       return SentBytes; }

   int SendFile(const char *FileName)
   { int File=open(FileName, O_RDONLY); if(File<0) return File;
     struct stat Stat; fstat(File, &Stat); int Size=Stat.st_size;
     int Ret=sendfile(SocketFile, File, 0, Size);
     close(File);
     return Ret; }

   // send data (on a non-connected socket)
   int SendTo(const void *Message, int Bytes, SocketAddress Address, int Flags=MSG_NOSIGNAL)
     { socklen_t AddressLength=sizeof(Address.Address);
       return sendto(SocketFile, Message, Bytes, Flags, (struct sockaddr *) &(Address.Address), AddressLength); }

   int SendTo(const void *Message, int Bytes, int Flags=MSG_NOSIGNAL)
     { return sendto(SocketFile, Message, Bytes, Flags, 0, 0); }

   // say: I won't send any more data on this connection
   int SendShutdown(void)
     { return shutdown(SocketFile, SHUT_WR); }

#ifndef __CYGWIN__ // Cygwin C++ does not know abour TIOCOUTQ ?
   int getSendQueue(void)
     { int Bytes;
       ioctl(SocketFile, TIOCOUTQ, &Bytes);
       return Bytes; }
#endif

   // receive data (on a stream socket)
   int Receive(void *Message, int MaxBytes, int Flags=MSG_NOSIGNAL)
     { int Len=recv(SocketFile, Message, MaxBytes, Flags);
       if(Len>=0) return Len;
       return errno==EWOULDBLOCK ? 0:Len; }

   // receive (stream) data into a buffer
   int Receive(SocketBuffer &Buffer, int Flags=MSG_NOSIGNAL)
     { size_t NewSize=Buffer.Len+Buffer.AllocUnit/2;
       size_t Allocated=Buffer.Relocate(NewSize);
       int MaxBytes=Allocated-Buffer.Len-1;
       int ReceiveBytes=Receive(Buffer.Data+Buffer.Len, MaxBytes, Flags);
       // printf("Allocated = %d, Receive(%d) => %d\n", Allocated, MaxBytes, ReceiveBytes);
       if(ReceiveBytes>0) { Buffer.Len+=ReceiveBytes; Buffer.Data[Buffer.Len]=0; }
       return ReceiveBytes; }

   // receive data (on a non-connected socket)
   int ReceiveFrom(void *Message, int MaxBytes, SocketAddress &Address, int Flags=MSG_NOSIGNAL)
     { socklen_t AddressLength=sizeof(Address.Address);
       return recvfrom(SocketFile, Message, MaxBytes, Flags, (struct sockaddr *) &(Address.Address), &AddressLength); }

   // tell if socket is open
   int isOpen(void)
     { return SocketFile>=0; }

   // close the socket
   int Close(void)
     { if(SocketFile>=0) close(SocketFile);
       SocketFile=(-1); return 0; }

   // get the local IP and port
   int getLocalAddress(SocketAddress &Address)
     { socklen_t AddressLength=sizeof(Address.Address);
       return getsockname(SocketFile, (struct sockaddr *) &(Address.Address), &AddressLength); }

   // get the remote IP and port
   int getRemoteAddress(SocketAddress &Address)
     { socklen_t AddressLength=sizeof(Address.Address);
       return getpeername(SocketFile, (struct sockaddr *) &(Address.Address), &AddressLength); }

   static void CopyNetToHost(uint32_t *Dst, uint32_t *Src, int Words)
   { for( ; Words; Words--) (*Dst++) = ntohl(*Src++); }

   static void CopyHostoNet(uint32_t *Dst, uint32_t *Src, int Words)
   { for( ; Words; Words--) (*Dst++) = htonl(*Src++); }

} ;

class UDP_Sender
{ public:
   Socket Sock;
   const static int MaxDest = 4;
   SocketAddress Dest[MaxDest];

  public:
   void ClearDest(void)                                                     // clear the list of destination IP's
   { for(int Idx=0; Idx<MaxDest; Idx++)
     { Dest[Idx].setIP((long unsigned int)0); }
   }

   int Open(void)           { ClearDest(); return Sock.Create_DGRAM(); }
   int Close(void)          { return Sock.Close(); }
   int setNonBlocking(void) { return Sock.setNonBlocking(); }

   int addDest(const char *Addr)
   { for(int Idx=0; Idx<MaxDest; Idx++)
     { if(Dest[Idx].getIP()==0)
       { if(Dest[Idx].set(Addr)<0) return -1;
         return Idx; }
     }
     return -1; }
/*
   int addBroadcast(void)
   { int Idx;
     for( Idx=0; Idx<MaxDest; Idx++)
     { if(Dest[Idx].getIP()==0)
       { if(Dest[Idx].SetIP(INADDR_BROADCAST)<0) return -1;
         return Idx; }
     }
     return -1; }
*/
   void PrintDest(void)
   { printf("Dest[] =");
     for(int Idx=0; Idx<MaxDest; Idx++)
     { if(Dest[Idx].getIP()==0) continue;
       printf(" %s", Dest[Idx].getIPColonPort()); }
     printf("\n");
   }

   int Send(uint32_t *Msg, int Words)
   { return Send((void *)Msg, Words*sizeof(uint32_t)); }

   int Send(void *Msg, int Bytes)
   { int Count=0;
     for( int Idx=0; Idx<MaxDest; Idx++)
     { if(Dest[Idx].getIP()==0) continue;
       if(Sock.SendTo(Msg, Bytes, Dest[Idx])<0) continue;
       Count++; }
     return Count; }

   int Receive(void *Msg, int MaxBytes, SocketAddress &Source)
   { return Sock.ReceiveFrom(Msg, MaxBytes, Source); }

} ;

class UDP_Receiver
{ public:
   Socket Sock;

  public:
   int Open(int Port)       { return Sock.Listen_DGRAM(Port); }
   int Close(void)          { return Sock.Close(); }
   int setNonBlocking(void) { return Sock.setNonBlocking(); }
   // int getPort(void) const  { return Sock.}

   int Receive(void *Msg, int MaxBytes, SocketAddress &Source)
   { return Sock.ReceiveFrom(Msg, MaxBytes, Source); }

   int Receive(uint32_t *Msg, int MaxWords, SocketAddress &Source)
   { int Words=Sock.ReceiveFrom(Msg, MaxWords*sizeof(uint32_t), Source);
     return Words<0 ? Words:Words/sizeof(uint32_t); }

} ;

#endif // of __SOCKET_H__

// =========================================================================================


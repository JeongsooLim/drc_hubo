#ifndef MYLIB_SOCKUT_H
#define MYLIB_SOCKUT_H

#include "mylib/base/exceptiondef.h"
#include "string"

#ifdef WIN32
    #include <winsock2.h>
#else
    #include <sys/socket.h>

    #define SOCKET_ERROR -1
    #define INVALID_SOCKET -1
    typedef int SOCKET;
    typedef struct sockaddr SOCKADDR;
#endif

#define DEFAULT_PORT 2924
#define MAX_BUFFER_SIZE 4096


inline int sockrecv(SOCKET s, char* buf, int len, int flags){return recv(s,buf,len,flags);}
inline int socksend(SOCKET s, const char* buf, int len, int flags){return send(s,buf,len,flags);}
inline int socklisten(SOCKET s,int backlog){return listen(s,backlog);}
inline int sockconnect(SOCKET s, const sockaddr* name, int namelen){return connect(s,name, namelen);}

#ifndef WIN32
    inline int closesocket(SOCKET s){return close(s);}
#endif

class SocketFailException : std::exception{
private:
    std::string msg;

public:
    SocketFailException():std::exception(),msg(""){}
    SocketFailException(const char* msg):std::exception(),msg(msg){}
    virtual ~SocketFailException() throw() {}
    virtual const char* what() const throw() {return msg.c_str();}
};

class ClientHandle{
private:
    SOCKET as;
public:
    ClientHandle():as(SOCKET_ERROR){}
    ClientHandle(SOCKET as):as(as){}
    virtual ~ClientHandle(){this->close();}
    int recv(char* buf, int n){n = sockrecv(this->as, buf, n, 0); return n;}
    int send(char* buf, int n){return socksend(this->as, buf, n, 0);}
    void close()              {if(this->as != SOCKET_ERROR){closesocket(as);as = SOCKET_ERROR;}}
};
class Server{
private:
    SOCKET ls;
    sockaddr_in service;
    int port;
    int state;
public:
    Server(int port = DEFAULT_PORT){this->port = port;}
    virtual ~Server()              {this->close();}
    void startup(){
        this-> ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if(ls == INVALID_SOCKET)
            throw SocketFailException("Socket generation fail");

        memset(&this->service, 0, sizeof(this->service));
        this->service.sin_family = AF_INET;
        this->service.sin_addr.s_addr = INADDR_ANY;
        this->service.sin_port = htons(this->port);


        if (bind(this->ls, (SOCKADDR*) &this->service, sizeof(this->service)) == SOCKET_ERROR)
            throw SocketFailException("Bind fail");

        if (socklisten(this->ls, 1) == SOCKET_ERROR)
            throw SocketFailException("Listen fail");

        this->state = 1;
    }
    ClientHandle listen(){
        SOCKET as;
        while(1){
            as = accept(ls, NULL, NULL);
            if (as != SOCKET_ERROR)
                return ClientHandle(as);
        }
    }
    void close(){
        if(this->state==1){
            closesocket(ls);
        }
    }

};

class Client{
private:
    std::string ip;
    int port;
    SOCKET cs;
    sockaddr_in client;
public:
    Client()                               :ip("127.0.0.1"),port(DEFAULT_PORT){startup();}
    Client(int port)                       :ip("127.0.0.1"),port(port){startup();}
    Client(const std::string& ip)          :ip(ip),port(DEFAULT_PORT){startup();}
    Client(const std::string& ip, int port):ip(ip),port(port){startup();}

    void startup(){
        this->cs = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (cs == INVALID_SOCKET)
            throw SocketFailException("Socket generation fail");


        memset(&this->client, 0, sizeof(this->client));

        this->client.sin_family = AF_INET;
        this->client.sin_addr.s_addr = inet_addr(this->ip.c_str());
        this->client.sin_port = htons(this->port);


    }
    void connect(){
        if (sockconnect(this->cs, (SOCKADDR *)&this->client, sizeof(this->client)) == SOCKET_ERROR)
            throw SocketFailException("Connection fail");
    }
    int recv(char* buf, int n){n = sockrecv(this->cs, buf, n, 0); return n;}
    int send(char* buf, int n){return socksend(this->cs, buf, n, 0);}
    void close(){closesocket(this->cs);}
};


#endif

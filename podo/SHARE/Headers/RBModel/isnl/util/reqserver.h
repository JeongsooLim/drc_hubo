#ifndef MYLIB_REQSERVER_H
#define MYLIB_REQSERVER_H

#include "isnl/base/exceptiondef.h"
#include "isnl/base/array.h"
#include "isnl/util/console.h"
#include "string"
#include "tinythread.h"

namespace isnl{

#ifdef WIN32
    #include <winsock2.h>
	typedef int socklen_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>

    #define SOCKET_ERROR -1
    #define INVALID_SOCKET -1
    typedef int SOCKET;
    typedef struct sockaddr SOCKADDR;
#endif

#define DEFAULT_PORT 2924
#define MAX_BUFFER_SIZE 4096

#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(push)
#pragma warning(disable:4996)
#endif

class PacketIter;

class Packet{
    char *buf;
	int  *sz;
    int len;
public:
	Packet()          :len(MAX_BUFFER_SIZE){
		buf = new char[MAX_BUFFER_SIZE];
		sz = (int*)buf;
		*sz = 0;
	}
    Packet(int len)   :len(len){
		buf = new char[len]; 
		sz = (int*)buf;
		*sz = sizeof(int);
	}	
	virtual ~Packet() {}
	void clear() {
		*sz = sizeof(int);
	}
	void del()       {
		if(buf!=NULL){
			delete[] buf; 
			buf = NULL;
			sz = NULL;
		}
	}
    PacketIter begin();
	      char* getBuf()       {return buf;}
	const char* getBuf() const {return buf;}
	int size()   const {return *sz;}
	int length() const {return len;}
};
class PacketIter{
	char *p;
	int  *sz;
    int len;
public:
    PacketIter(char *buf, int* sz, int len){
        this->p   = buf;
        this->len = len;
		this->sz  = sz;
    }
    PacketIter& operator << (char    val){
        int l = sizeof(char);
        assert(len>l);
        memcpy(p, &val, l);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (bool    val){
        int l = sizeof(bool);
        assert(len>l);
        memcpy(p, &val, l);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (int     val){
        int l = sizeof(int);
        assert(len>l);
        memcpy(p, &val, l);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (long    val){
        int l = sizeof(long);
        assert(len>l);
        memcpy(p, &val, l);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (float   val){
        int l = sizeof(float);
        assert(len>l);
        memcpy(p, &val, l);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (double  val){
        int l = sizeof(double);
        assert(len>l);
        memcpy(p, &val, l);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (const ints& arr){
        typedef std::vector<int>::const_iterator iter1;
        typedef int* iter2;
        int n = arr.size();
        int l = n*sizeof(int);
        assert(len>l);
        *this << n;
        std::copy<iter1, iter2>(arr.begin(), arr.end(), (int*)p);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (const longs& arr){
        int n = arr.size();
        int l = n*sizeof(long);
        assert(len>l);
        *this << n;
        std::copy(arr.begin(), arr.end(), (long*)p);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (const floats& arr){
        int n = arr.size();
        int l = n*sizeof(float);
        assert(len>l);
        *this << n;
        std::copy(arr.begin(), arr.end(), (float*)p);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (const doubles& arr){
        int n = arr.size();
        int l = n*sizeof(double);
        assert(len>l);
        *this << n;
        std::copy(arr.begin(), arr.end(), (double*)p);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (const int2& arr){
        int n = arr.size(0), m = arr.size(1);
        int l = n*m*sizeof(int);
        assert(len>l);
        *this << n << m;
        std::copy(arr.tbegin(), arr.tend(), (int*)p);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (const long2& arr){
        int n = arr.size(0), m = arr.size(1);
        int l = n*m*sizeof(long);
        assert(len>l);
        *this << n << m;
        std::copy(arr.tbegin(), arr.tend(), (long*)p);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (const float2& arr){
        int n = arr.size(0), m = arr.size(1);
        int l = n*m*sizeof(float);
        assert(len>l);
        *this << n << m;
        std::copy(arr.tbegin(), arr.tend(), (float*)p);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator << (const double2& arr){
        int n = arr.size(0), m = arr.size(1);
        int l = n*m*sizeof(double);
        assert(len>l);
        *this << n << m;
        std::copy(arr.tbegin(), arr.tend(), (double*)p);
        p+=l; len-=l; *sz+=l;
        return *this;
    }
    PacketIter& operator >> (char   &val){
        int l = sizeof(char);
        assert(len>l);
        memcpy(&val, p, l);
        p+= l; len-=l;
        return *this;
    }
    PacketIter& operator >> (bool   &val){
        int l = sizeof(bool);
        assert(len>l);
        memcpy(&val, p, l);
        p+= l; len-=l;
        return *this;
    }
    PacketIter& operator >> (int    &val){
        int l = sizeof(int);
        assert(len>l);
        memcpy(&val, p, l);
        p+= l; len-=l;
        return *this;
    }
    PacketIter& operator >> (long   &val){
        int l = sizeof(long);
        assert(len>l);
        memcpy(&val, p, l);
        p+= l; len-=l;
        return *this;
    }
    PacketIter& operator >> (float  &val){
        int l = sizeof(float);
        assert(len>l);
        memcpy(&val, p, l);
        p+= l; len-=l;
        return *this;
    }
    PacketIter& operator >> (double &val){
        int l = sizeof(double);
        assert(len>l);
        memcpy(&val, p, l);
        p+= l; len-=l;
        return *this;
    }
    PacketIter& operator >> (ints& arr){
        int n; *this >> n;
        int l = sizeof(int)*n;
        assert(len>l);
        arr.resize(n);
        std::copy((int*)p, ((int*)p)+n, arr.begin());
        p+=l; len-=l;
        return *this;
    }
    PacketIter& operator >> (longs& arr){
        int n; *this >> n;
        int l = sizeof(long)*n;
        assert(len>l);
        arr.resize(n);
        std::copy((long*)p, ((long*)p)+n, arr.begin());
        p+=l; len-=l;
        return *this;
    }
    PacketIter& operator >> (floats& arr){
        int n; *this >> n;
        int l = sizeof(float)*n;
        assert(len>l);
        arr.resize(n);
        std::copy((float*)p, ((float*)p)+n, arr.begin());
        p+=l; len-=l;
        return *this;
    }
    PacketIter& operator >> (doubles& arr){
        int n; *this >> n;
        int l = sizeof(double)*n;
        assert(len>l);
        arr.resize(n);
        std::copy((double*)p, ((double*)p)+n, arr.begin());
        p+=l; len-=l;
        return *this;
    }
    PacketIter& operator >> (int2& arr){
        int n,m; *this >> n >> m;
        int l = sizeof(int)*n*m;
        assert(len>l);
        arr.resize(n,m);
        std::copy((int*)p, ((int*)p)+n*m, arr.tbegin());
        p+=l; len-=l;
        return *this;
    }
    PacketIter& operator >> (long2& arr){
        int n,m; *this >> n >> m;
        int l = sizeof(long)*n*m;
        assert(len>l);
        arr.resize(n,m);
        std::copy((long*)p, ((long*)p)+n*m, arr.tbegin());
        p+=l; len-=l;
        return *this;
    }
    PacketIter& operator >> (float2& arr){
		int n,m; *this >> n >> m;
		int l = sizeof(float)*n*m;
		assert(len>l);
		arr.resize(n,m);
		std::copy((float*)p, ((float*)p)+n*m, arr.tbegin());
		p+=l; len-=l;
		return *this;
    }
    PacketIter& operator >> (double2& arr){
        int n,m; *this >> n >> m;
        int l = sizeof(double)*n*m;
        assert(len>l);
        arr.resize(n,m);
        std::copy((double*)p, ((double*)p)+n*m, arr.tbegin());
        p+=l; len-=l;
        return *this;
    }

};

inline PacketIter Packet::begin(){return PacketIter(buf+sizeof(int), sz, len);}

#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(pop)
#endif


inline bool initializeSocket(){
#ifdef WIN32
    WSADATA WsaData;
    return WSAStartup( MAKEWORD(2,2), &WsaData) == NO_ERROR;
#else
    return true;
#endif
}
inline void ShutdownSocket(){
#ifdef WIN32
    WSACleanup();
#endif
}
inline int sockrecv(SOCKET s, char* buf, int len, int flags){return recv(s,buf,len,flags);}
inline int socksend(SOCKET s, const char* buf, int len, int flags){return send(s,buf,len,flags);}
inline int socklisten(SOCKET s,int backlog){return listen(s,backlog);}
inline int sockconnect(SOCKET s, const sockaddr* name, int namelen){return connect(s,name, namelen);}

#ifndef WIN32
    inline int closesocket(SOCKET s){return close(s);}
#endif

class Message{
public:
	virtual void encode(Packet& packet)=0;
	virtual void decode(Packet& packet)=0;
};

class MsgClientHandle{
private:
	SOCKET as;
	Packet pak;  // packet buffer
public:
	MsgClientHandle():as(SOCKET_ERROR){}
	MsgClientHandle(SOCKET as):as(as){}
	virtual ~MsgClientHandle(){}
	void recv(Packet& pak){
		int n = sockrecv(this->as, pak.getBuf(), pak.length(), 0);
		if(n <= 0) throw SocketFailException("Connection Lost");
        //printf("recv : [%d,%d]\n",n, pak.size());
	}
	void send(Packet& pak){
		int n = socksend(this->as, pak.getBuf(), pak.size(), 0);
		if(n <= 0) throw SocketFailException("Connection Lost");
        //printf("send : [%d,%d]\n",n, pak.size());
	}
	void recv(Message& msg){
		int n = sockrecv(this->as, pak.getBuf(), pak.length(), 0);
		if(n <= 0) throw SocketFailException("Connection Lost");
		//printf("recv : [%d,%d]\n",n, pak.size());
		msg.decode(pak);
	}
	void send(Message& msg){
		pak.clear();
		msg.encode(pak);
		int n = socksend(this->as, pak.getBuf(), pak.size(), 0);
		if(n <= 0) throw SocketFailException("Connection Lost");
		//printf("send : [%d,%d]\n",n, pak.size());
	}

	void close()              {
		if(this->as != SOCKET_ERROR){
			//closesocket(as);
			shutdown(as,SHUT_RDWR);
			as = SOCKET_ERROR;
		}
	}
};
class MsgServer{
private:
	SOCKET ls;
    struct sockaddr_in service;
	int port;
	int state;
public:
	MsgServer()                       :port(DEFAULT_PORT),state(0){}
	MsgServer(int port)               :port(port)        ,state(0){}
	virtual ~MsgServer()              {this->stop();}
	void setPort(int port)            {this->port = port;}
	void start(){
		if(this->state==1) return;
		this->ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

		// set receive & send buffer size;
		int so_reuse = 1;
		int so_bufsize = MAX_BUFFER_SIZE;
		setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &so_reuse, sizeof(so_reuse));
		setsockopt(ls, SOL_SOCKET, SO_RCVBUF,    &so_bufsize, sizeof(so_bufsize));
		setsockopt(ls, SOL_SOCKET, SO_SNDBUF,    &so_bufsize, sizeof(so_bufsize));

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
	MsgClientHandle listen(){
		SOCKET as;
		while(1){
			as = accept(ls, NULL, NULL);
			if (as != SOCKET_ERROR)
				return MsgClientHandle(as);
		}
	}
	void stop(){
		if(state==1){
			this->state=0;

			//closesocket(ls);
			shutdown(ls,SHUT_RDWR);
		}
	}
	bool isrun(){
		return (state==1);
	}

};
class MsgClient{
private:
	std::string ip;
	int port;
	SOCKET cs;
	sockaddr_in client;
	Packet pak;
    bool connected;
public:
	MsgClient()                               :ip("127.0.0.1"),port(DEFAULT_PORT){init();}
	MsgClient(int port)                       :ip("127.0.0.1"),port(port)        {init();}
	MsgClient(const std::string& ip)          :ip(ip),         port(DEFAULT_PORT){init();}
	MsgClient(const std::string& ip, int port):ip(ip),         port(port)        {init();}
	void init(){connected = false;}
	void setIP(const std::string& ip)         {this->ip = ip;}
	void setPort(int port)                    {this->port = port;}
	void connect(){
		this->cs = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (cs == INVALID_SOCKET)
			throw SocketFailException("Socket generation fail");

		memset(&this->client, 0, sizeof(this->client));

		this->client.sin_family = AF_INET;
		this->client.sin_addr.s_addr = inet_addr(this->ip.c_str());
		this->client.sin_port = htons(this->port);

		if (sockconnect(this->cs, (SOCKADDR *)&this->client, sizeof(this->client)) == SOCKET_ERROR){
			printf("Connection fail");
			throw SocketFailException("Connection fail");
		}
		connected = true;
	}
	void close(){
		if (connected){
			closesocket(cs);
		}
		connected = false;
	}
    bool isconnected(){return connected;}
	void recv(Packet& pak){
		int n = sockrecv(this->cs, pak.getBuf(), pak.length(), 0);
		if(n <= 0) throw SocketFailException("Connection Lost");
		//printf("recv : [%d,%d]\n",n, pak.size());
	}
	void send(Packet& pak){
		int n = socksend(this->cs, pak.getBuf(), pak.size(), 0);
		if(n <= 0) throw SocketFailException("Connection Lost");
		//printf("send : [%d,%d]\n",n, pak.size());
	}
	void recv(Message& msg){
		int n = sockrecv(this->cs, pak.getBuf(), pak.length(), 0);
		if(n <= 0) throw SocketFailException("Connection Lost");
		//printf("recv : [%d,%d]\n",n, pak.size());
		msg.decode(pak);
	}
	void send(Message& msg){
		pak.clear();
		msg.encode(pak);
		int n = socksend(this->cs, pak.getBuf(), pak.size(), 0);
		if(n <= 0) throw SocketFailException("Connection Lost");
		//printf("send : [%d,%d]\n",n, pak.size());
	}
};




class Request : public Message{
protected:
    int id;
public:
    Request(int id=0):id(id){}
};
class Response : public Message{
protected:
    int id;
public:
    Response(int id=0):id(id){}

};

//
void rsockListener(void *data);
void rsockRequestHandler(void *data);
class PacketHandler{
public:
	virtual void operator() (MsgClientHandle& ch, Packet& pak)=0; // message handler : handle request packet
	virtual PacketHandler* initialize(){return this;}             // handler copy initializer : implement if there is local variable
	virtual void           finalize(){}                           // handler copy finalizer
};
class ReqServer{
	friend void rsockListener(void *data);
	friend void rsockRequestHandler(void *data);
private:
	SOCKET ls;
	sockaddr_in service;
	int port;
	int state;
	PacketHandler *handler;
	tthread::thread *tlistner;
	std::vector<tthread::thread*> handle;
public:
	ReqServer()                                  :port(DEFAULT_PORT),handler(   NULL){}
    ReqServer(int port)                          :port(        port),handler(   NULL){}
	ReqServer(PacketHandler * handler)           :port(DEFAULT_PORT),handler(handler){}
	ReqServer(int port, PacketHandler * handler) :port(        port),handler(handler){}
	virtual ~ReqServer()              {this->stop();}
	void setPort(int port)            {this->port = port;}
	void setPacketHandler(PacketHandler * handler){this->handler = handler;}
    void start(){
        this->ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

		// set receive & send buffer size;
        int so_reuse = 1;
        int so_bufsize = MAX_BUFFER_SIZE;
        setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &so_reuse, sizeof(so_reuse));
        setsockopt(ls, SOL_SOCKET, SO_RCVBUF,    &so_bufsize, sizeof(so_bufsize));
        setsockopt(ls, SOL_SOCKET, SO_SNDBUF,    &so_bufsize, sizeof(so_bufsize));

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

		tlistner = new tthread::thread(rsockListener, this);
		this->state = 1;
	}
	void stop(){
		if(this->state==1){
			this->state=0;

            //closesocket(ls);
            shutdown(ls,SHUT_RDWR);
            tlistner->join();
			delete tlistner;
			tlistner = NULL;

		}
	}

};

}
#endif

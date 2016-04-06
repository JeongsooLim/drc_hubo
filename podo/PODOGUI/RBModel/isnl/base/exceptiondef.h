#ifndef _MYLIB_EXCEPTIONDEF_H_
#define _MYLIB_EXCEPTIONDEF_H_
#include <exception>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#define MAX_EX_BUF_SIZE 256


namespace isnl{


#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(push)
#pragma warning(disable:4996)
#endif

class Exception : public std::exception{
protected:
	char msg[MAX_EX_BUF_SIZE];
public:
	Exception(){this->msg[0] = '\0';}
	Exception(const Exception& e){
		strncpy(this->msg, e.msg, MAX_EX_BUF_SIZE);
	}
	Exception(const char* name, const char* msg){
		sprintf(this->msg, "%s : %s", name, msg);
	}
	virtual const char* what(){return msg;}
};

#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(pop)
#endif

class IOException : public Exception{
public:
	IOException();
	IOException(const char* msg):Exception("IOException",msg){}
};

class SocketFailException : public Exception{
public:
    SocketFailException():Exception(){}
    SocketFailException(const char* msg):Exception("SocketFailException",msg){}
    virtual ~SocketFailException() throw() {}
};

}
#endif

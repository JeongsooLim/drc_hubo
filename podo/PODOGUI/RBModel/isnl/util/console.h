#ifndef MyLib_CONSOLE_H
#define MyLib_CONSOLE_H
#include <stdio.h>
#include <iostream>
#include "isnl/base/array.h"
	//////////////////////////////////////////////////////////////////////////
	// clrscr
	//
	// Clear console screen
    std::ostream & gotoxy(int x, int y);
	inline void clrscr(){
	#ifdef WIN32
		CONSOLE_SCREEN_BUFFER_INFO csbi;
		HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
		COORD coord = {0, 0};
		DWORD count;

		GetConsoleScreenBufferInfo(hStdOut, &csbi);
		FillConsoleOutputCharacter(hStdOut, ' ', csbi.dwSize.X * csbi.dwSize.Y, coord, &count);
		SetConsoleCursorPosition(hStdOut, coord);
	#else
		int i;

		for (i = 0; i < 100; i++)
			// Insert new lines to create a blank screen
			putchar('\n');
		gotoxy(0,0);
	#endif
	}
	//////////////////////////////////////////////////////////////////////////
	// gotoxy
	//
	// Sets the cursor position at the specified console position
	//
	// Input
	//	 x	: New horizontal cursor position
	//   y	: New vertical cursor position
	inline std::ostream & gotoxy(int x, int y){
	#ifdef WIN32
		COORD coord;
		coord.X = x;
		coord.Y = y;
		SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
		return std::cout;
	#else
		char essq[100];		// String variable to hold the escape sequence
		char xstr[100];		// Strings to hold the x and y coordinates
		char ystr[100];		// Escape sequences must be built with characters

		/*
		** Convert the screen coordinates to strings
		*/
		sprintf(xstr, "%d", x);
		sprintf(ystr, "%d", y);

		/*
		** Build the escape sequence (vertical move)
		*/
		essq[0] = '\0';
		strcat(essq, "\033[");
		strcat(essq, ystr);

		/*
		** Described in man terminfo as vpa=\E[%p1%dd
		** Vertical position absolute
		*/
		strcat(essq, "d");

		/*
		** Horizontal move
		** Horizontal position absolute
		*/
		strcat(essq, "\033[");
		strcat(essq, xstr);
		// Described in man terminfo as hpa=\E[%p1%dG
		strcat(essq, "G");

		/*
		** Execute the escape sequence
		** This will move the cursor to x, y
		*/
		printf("%s", essq);
        return std::cout;
	#endif
	}
	template <class T>
	void printArrayFormat(const std::vector<T>& arr, const char* format){
		printf("vec[");
		int n = arr.size();
		for(int i = 0; i < n; ++i){
			printf(format,arr[i]);
			if(i<n-1) printf(",");
		}
		printf("]");
	}
	template <class T>
	void printArrayFormatLn(const isnl::Array2<T>& arr, const char* format){
		int n = arr.size(0), m = arr.size(1);
		printf("mat<%d,%d>\n",n,m);
		for(int i = 0; i < n; ++i){
			printf("[");
			for(int j = 0; j < m; ++j){
				printf(format,arr(i,j));
				if(j<m-1) printf(",");
			}
			printf("]\n");
		}
	}

	inline void print(char val)             {printf("%c",val);}
	inline void print(char* val)            {printf("%s",val);}
	inline void print(short val)            {printf("%d",val);}
	inline void print(int val)              {printf("%d",val);}
	inline void print(long val)             {printf("%ld",val);}
	inline void print(float val)            {printf("%f",val);}
	inline void print(double val)           {printf("%f",val);}
	inline void print(const std::string & val){printf("%s",val.c_str());}
	inline void print(unsigned char val)    {printf("%c",val);}
	inline void print(unsigned short val)   {printf("%d",val);}
	inline void print(unsigned int val)     {printf("%d",val);}
	inline void print(unsigned long val)    {printf("%ld",val);}

	inline void println()                    {printf("\n");}
	inline void println(char val)            {printf("%c\n",val);}
	inline void println(char* val)           {printf("%s\n",val);}
	inline void println(short val)           {printf("%d\n",val);}
	inline void println(int val)             {printf("%d\n",val);}
	inline void println(long val)            {printf("%ld\n",val);}
	inline void println(float val)           {printf("%f\n",val);}
	inline void println(double val)          {printf("%f\n",val);}
	inline void println(const std::string & val){printf("%s\n",val.c_str());}
	inline void println(unsigned char val)   {printf("%c\n",val);}
	inline void println(unsigned short val)  {printf("%d\n",val);}
	inline void println(unsigned int val)    {printf("%d\n",val);}
	inline void println(unsigned long val)   {printf("%ld\n",val);}

	inline void print(const ints& arr)     {printArrayFormat(arr,"%d");}
	inline void print(const longs& arr)    {printArrayFormat(arr,"%ld");}
	inline void print(const floats& arr)   {printArrayFormat(arr,"%f");}
	inline void print(const doubles& arr)  {printArrayFormat(arr,"%f");}
	inline void println(const ints& arr)   {printArrayFormat(arr,"%d");printf("\n");}
	inline void println(const longs& arr)  {printArrayFormat(arr,"%ld");printf("\n");}
	inline void println(const floats& arr) {printArrayFormat(arr,"%f");printf("\n");}
	inline void println(const doubles& arr){printArrayFormat(arr,"%f");printf("\n");}
	inline void println(const int2& arr)   {printArrayFormatLn(arr,"%d");}
	inline void println(const long2& arr)  {printArrayFormatLn(arr,"%ld");}
	inline void println(const float2& arr) {printArrayFormatLn(arr,"%f");}
	inline void println(const double2& arr){printArrayFormatLn(arr,"%f");}
#endif

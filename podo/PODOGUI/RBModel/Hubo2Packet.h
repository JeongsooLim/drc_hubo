#ifndef HUBO2PACKET_H
#define HUBO2PACKET_H
#include "../Share/include/Hubo2Def.h"


class HuboPacketHelper{
protected:
	static int paksize;
public:
	inline static int  packetsize();
	inline static int  encode(const HUBO2* sharedMemory, char* buf);
	inline static void decode(const char* buf, HUBO2* sharedMemory);
};

class PacketHandler{
protected:
	char *buf, *p;
	int sz;
public:
	inline PacketHandler(char* buf):buf(buf),p(buf),sz(0){}

	inline PacketHandler& operator << (char    val){*(char*)p   = val; p += sizeof(char);   sz += sizeof(char);   return *this;}
	inline PacketHandler& operator << (short   val){*(short*)p  = val; p += sizeof(short);  sz += sizeof(short);  return *this;}
	inline PacketHandler& operator << (int     val){*(int*)p    = val; p += sizeof(int);    sz += sizeof(int);    return *this;}
	inline PacketHandler& operator << (long    val){*(long*)p   = val; p += sizeof(long);   sz += sizeof(long);   return *this;}
	inline PacketHandler& operator << (float   val){*(float*)p  = val; p += sizeof(float);  sz += sizeof(float);  return *this;}
	inline PacketHandler& operator << (double  val){*(double*)p = val; p += sizeof(double); sz += sizeof(double); return *this;}

	inline PacketHandler& operator >> (char   &val){val = *(char*)p;   p += sizeof(char);   return *this;}
	inline PacketHandler& operator >> (short  &val){val = *(short*)p;  p += sizeof(short);  return *this;}
	inline PacketHandler& operator >> (int    &val){val = *(int*)p;    p += sizeof(int);    return *this;}
	inline PacketHandler& operator >> (long   &val){val = *(long*)p;   p += sizeof(long);   return *this;}
	inline PacketHandler& operator >> (float  &val){val = *(float*)p;  p += sizeof(float);  return *this;}
	inline PacketHandler& operator >> (double &val){val = *(double*)p; p += sizeof(double); return *this;}

	inline char   getChar()  {char   val = *(char*)p;     p += sizeof(char);   return val;}
	inline short  getShort() {short  val = *(short*)p;    p += sizeof(short);  return val;}
	inline int    getInt()   {int    val = *(int*)p;      p += sizeof(int);    return val;}
	inline long   getLong()  {long   val = *(long*)p;     p += sizeof(long);   return val;}
	inline float  getFloat() {float  val = *(float*)p;    p += sizeof(float);  return val;}
	inline double getDouble(){double val = *(double*)p;   p += sizeof(double); return val;}

	int size(){return sz;}
	void reset(){p = buf;}
};

#define crNull *(char*)0
#define irNull *(int*)0

inline char boolEncode(char b0=0, char b1=0, char b2=0, char b3=0, char b4=0, char b5=0, char b6=0, char b7=0);
inline char boolEncode(int  b0=0, int  b1=0, int  b2=0, int  b3=0, int  b4=0, int  b5=0, int  b6=0, int  b7=0);
inline void boolDecode(char pak, char &b0 = crNull, char &b1 = crNull, char &b2 = crNull, char &b3 = crNull,
								 char &b4 = crNull, char &b5 = crNull, char &b6 = crNull, char &b7 = crNull);
inline void boolDecode(char pak, int  &b0 = irNull, int  &b1 = irNull, int  &b2 = irNull, int  &b3 = irNull,
								 int  &b4 = irNull, int  &b5 = irNull, int  &b6 = irNull, int  &b7 = irNull);
inline char boolDecode(char pak, int i);
char boolEncode(char b0, char b1, char b2, char b3, char b4, char b5, char b6, char b7){
	char ret = 0;
	if(b0) ret |= 0x01;
	if(b1) ret |= 0x02;
	if(b2) ret |= 0x04;
	if(b3) ret |= 0x08;
	if(b4) ret |= 0x10;
	if(b5) ret |= 0x20;
	if(b6) ret |= 0x40;
	if(b7) ret |= 0x80;
	return ret;
}
char boolEncode(int b0, int b1, int b2, int b3, int b4, int b5, int b6, int b7){
	char ret = 0;
	if(b0) ret |= 0x01;
	if(b1) ret |= 0x02;
	if(b2) ret |= 0x04;
	if(b3) ret |= 0x08;
	if(b4) ret |= 0x10;
	if(b5) ret |= 0x20;
	if(b6) ret |= 0x40;
	if(b7) ret |= 0x80;
	return ret;
}
void boolDecode(char pak, char &b0, char &b1, char &b2, char &b3, char &b4, char &b5, char &b6, char &b7){
	if(&b0 != NULL) b0 = (pak & 0x01) == 0x01;
	if(&b1 != NULL) b1 = (pak & 0x02) == 0x02;
	if(&b2 != NULL) b2 = (pak & 0x04) == 0x04;
	if(&b3 != NULL) b3 = (pak & 0x08) == 0x08;
	if(&b4 != NULL) b4 = (pak & 0x10) == 0x10;
	if(&b5 != NULL) b5 = (pak & 0x20) == 0x20;
	if(&b6 != NULL) b6 = (pak & 0x40) == 0x40;
	if(&b7 != NULL) b7 = (pak & 0x80) == 0x80;
}
void boolDecode(char pak, int &b0, int &b1, int &b2, int &b3, int &b4, int &b5, int &b6, int &b7){
	if(&b0 != NULL) b0 = (pak & 0x01) == 0x01;
	if(&b1 != NULL) b1 = (pak & 0x02) == 0x02;
	if(&b2 != NULL) b2 = (pak & 0x04) == 0x04;
	if(&b3 != NULL) b3 = (pak & 0x08) == 0x08;
	if(&b4 != NULL) b4 = (pak & 0x10) == 0x10;
	if(&b5 != NULL) b5 = (pak & 0x20) == 0x20;
	if(&b6 != NULL) b6 = (pak & 0x40) == 0x40;
	if(&b7 != NULL) b7 = (pak & 0x80) == 0x80;
}
char boolDecode(char pak, int i){
	switch(i){
	case 0:  return (pak & 0x01) == 0x01;
	case 1:  return (pak & 0x02) == 0x02;
	case 2:  return (pak & 0x04) == 0x04;
	case 3:  return (pak & 0x08) == 0x08;
	case 4:  return (pak & 0x10) == 0x10;
	case 5:  return (pak & 0x20) == 0x20;
	case 6:  return (pak & 0x40) == 0x40;
	case 7:  return (pak & 0x80) == 0x80;
	default: return false;
	}
}

inline short floatEncode(float val, float mn, float mx){
	short smx = 0x7FFF;
	float a = smx*(2.f*(val - mn)/(mx - mn) -1.f); // change scale into -smn ~ smx range
	return (short) a;
}
inline float floatDecode(short val, float mn, float mx){
	short smx = 0x7FFF;
	return (mx - mn)*(float(val)/float(smx) +1.f)/2.f +mn;
}
inline short floatEncodeDeg(float val){
	return floatEncode(val, -360.f, 360.f);
}
inline float floatDecodeDeg(short val){
	return floatDecode(val, -360.f, 360.f);
}
inline short floatEncodeRad(float val){
	return floatEncode(val, -6.2832, 6.2832);
}
inline float floatDecodeRad(short val){
	return floatDecode(val, -6.2832, 6.2832);
}

int HuboPacketHelper::packetsize(){
	static int paksize = 0;
	if(paksize == 0){
		HUBO2 sm;
		char buf[sizeof(HUBO2)];
		paksize = HuboPacketHelper::encode(&sm, &buf[0]);
	}
	return paksize;
}
int  HuboPacketHelper::encode(const HUBO2* sm, char* buf){
	PacketHandler ph(buf);


	//
	ph << sm->Command;
	ph << boolEncode(char(sm->SersorEnabled), sm->JointSpaceFlag);
	//ph << sm->Command << sm->SersorEnabled << sm->JointSpaceFlag

	// joint reference & encoder
	for(int i = 0; i < NO_OF_JOINTS; ++i){
		ph << floatEncodeDeg(sm->Joint[i].RefPos) << floatEncodeDeg(sm->Joint[i].CurrentPosition);
	}
	ph << sm->LimitedJoint << sm->LimitType;

	// task space
	ph << sm->CurTaskPos.mass_RightHand << sm->CurTaskPos.mass_LeftHand;
	ph << sm->CurTaskPos.Q_34x1[idX] << sm->CurTaskPos.Q_34x1[idY] << sm->CurTaskPos.Q_34x1[idZ]
		<< sm->CurTaskPos.Q_34x1[idQ0] << sm->CurTaskPos.Q_34x1[idQ1] << sm->CurTaskPos.Q_34x1[idQ2] << sm->CurTaskPos.Q_34x1[idQ3];

	// sensor
	ph << sm->IMUSensor[CIMU].Roll  << sm->IMUSensor[CIMU].RollVel  << sm->IMUSensor[CIMU].RollOffset;
	ph << sm->IMUSensor[CIMU].Pitch << sm->IMUSensor[CIMU].PitchVel << sm->IMUSensor[CIMU].PitchOffset;
	ph << sm->IMUSensor[CIMU].Yaw   << sm->IMUSensor[CIMU].YawVel   << sm->IMUSensor[CIMU].YawOffset;
	for(int i = 0; i < NO_OF_FTSENSOR; ++i)
		ph << sm->FTSensor[i].Mx << sm->FTSensor[i].My << sm->FTSensor[i].Fz
		 << sm->FTSensor[i].VelRoll << sm->FTSensor[i].VelPitch;
	for(int i = 0; i < 6; ++i)
		ph << sm->ZMP[i];

	return ph.size();
}
void HuboPacketHelper::decode(const char* buf, HUBO2* sm){
	PacketHandler ph((char*)buf);
	char temp;

	//
	ph >> sm->Command >> temp;
	sm->SersorEnabled  = boolDecode(temp, 0);
	sm->JointSpaceFlag = boolDecode(temp, 1);

	// joint reference
	for(int i = 0; i < NO_OF_JOINTS; ++i){
		sm->Joint[i].RefPos = floatDecodeDeg(ph.getShort());
		sm->Joint[i].CurrentPosition = floatDecodeDeg(ph.getShort());
	}
	ph >> sm->LimitedJoint >> sm->LimitType;


	// task space
	ph >> sm->CurTaskPos.mass_RightHand >> sm->CurTaskPos.mass_LeftHand;
	ph >> sm->CurTaskPos.Q_34x1[idX] >> sm->CurTaskPos.Q_34x1[idY] >> sm->CurTaskPos.Q_34x1[idZ]
		>> sm->CurTaskPos.Q_34x1[idQ0] >> sm->CurTaskPos.Q_34x1[idQ1] >> sm->CurTaskPos.Q_34x1[idQ2] >> sm->CurTaskPos.Q_34x1[idQ3];

	// task space
	ph >> sm->IMUSensor[CIMU].Roll  >> sm->IMUSensor[CIMU].RollVel  >> sm->IMUSensor[CIMU].RollOffset;
	ph >> sm->IMUSensor[CIMU].Pitch >> sm->IMUSensor[CIMU].PitchVel >> sm->IMUSensor[CIMU].PitchOffset;
	ph >> sm->IMUSensor[CIMU].Yaw   >> sm->IMUSensor[CIMU].YawVel   >> sm->IMUSensor[CIMU].YawOffset;
	for(int i = 0; i < NO_OF_FTSENSOR; ++i)
		ph >> sm->FTSensor[i].Mx >> sm->FTSensor[i].My >> sm->FTSensor[i].Fz
		 >> sm->FTSensor[i].VelRoll >> sm->FTSensor[i].VelPitch;
	for(int i = 0; i < 6; ++i)
		ph >> sm->ZMP[i];
}


#endif

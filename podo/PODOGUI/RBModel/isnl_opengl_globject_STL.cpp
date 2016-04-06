#include <isnl/opengl/globject.h>
#include "cstdio"

using namespace isnl;
using namespace std;
void GLSTL::ReduceToUnit(float vector[3]) const
{
float length;

// Calculate the length of the vector
length = (float)sqrt((vector[0]*vector[0]) +
(vector[1]*vector[1]) +
(vector[2]*vector[2]));

// Keep the program from blowing up by providing an exceptable
// value for vectors that may calculated too close to zero.
if(length == 0.0f)
length = 1.0f;

// Dividing each element by the length will result in a
// unit normal vector.
vector[0] /= length;
vector[1] /= length;
vector[2] /= length;
}

// Points p1, p2, & p3 specified in counter clock-wise order
void GLSTL::calcNormal(group facej, float out[3])  const
{
	float v1[3],v2[3];
	static const int x = 0;
	static const int y = 1;
	static const int z = 2;

	// Calculate two vectors from the three points
	v1[x] = (facej).vertex1[x] - (facej).vertex2[x];
	v1[y] = (facej).vertex1[y] - (facej).vertex2[y];
	v1[z] = (facej).vertex1[z] - (facej).vertex2[z];

	v2[x] = (facej).vertex2[x] - (facej).vertex3[x];
	v2[y] = (facej).vertex2[y] - (facej).vertex3[y];
	v2[z] = (facej).vertex2[z] - (facej).vertex3[z];

	// Take the cross product of the two vectors to get
	// the normal vector which will be stored in out
	out[x] = v1[y]*v2[z] - v1[z]*v2[y];
	out[y] = v1[z]*v2[x] - v1[x]*v2[z];
	out[z] = v1[x]*v2[y] - v1[y]*v2[x];

	// Normalize the vector (shorten length to one)
	ReduceToUnit(out);
}


GLSTL::GLSTL(){

}
GLSTL::GLSTL(std::string filename){
	FILE *binary;
	if( (binary =fopen(filename.c_str(),"rb"))==NULL )////stl file location
	{
	printf(" \n the is failed to accessed\n");
	return;
	}
	char ff[80];
	int numfacet[2];
	fread(ff,sizeof(char),80,binary) ;
	fread(numfacet,sizeof(int),1,binary);
	int num=numfacet[0];
	facet.assign(num,group());
    //printf("%s facecount %d \n",filename.c_str(),num);
	for (int j=0;j<num;j++)
	{
	fread(facet[j].normal,sizeof(float),3,binary);
	fread(facet[j].vertex1,sizeof(float),3,binary);
	fread(facet[j].vertex2,sizeof(float),3,binary);
	fread(facet[j].vertex3,sizeof(float),3,binary);
	fread(facet[j].unuse,sizeof(char),2,binary);
	}

}
void GLSTL::initialize() const {}
void GLSTL::render() const{
	//if(!visible)return;

	float reflectance[4] = {baseColor.x, baseColor.y, baseColor.z, 1.0f};


	glPushMatrix();
	glMultMatrixf(m.v);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);

	//do---------
	float normal[3];

	glBegin(GL_TRIANGLES);
	{

		for(int j=0;j<facet.size();j++)
		{
			facet[j].vertex1;
			facet[j].vertex2;
			facet[j].vertex3;
			calcNormal(facet[j],normal);
			glNormal3fv(normal);
			glVertex3fv(facet[j].vertex1);
			glVertex3fv(facet[j].vertex2);
			glVertex3fv(facet[j].vertex3);
		}
	}glEnd();

	glPopMatrix();
}

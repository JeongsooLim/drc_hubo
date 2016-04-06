#ifndef GLOBJECT_H
#define GLOBJECT_H

#include <qgl.h>
#include <isnl/math/geometry.h>

class GLObject{
protected:
	isnl::mat4 m; // transpose of transformation matrix
	isnl::vec3 baseColor;
	mutable GLuint listid;
	bool visible;
public:
	GLObject();
	virtual ~GLObject();
	isnl::pos getPosition();
	void setPosition(const isnl::pos& p);
	virtual void initialize() const=0; // make vertex list of the object
	virtual void render() const; // render the object
	void setBaseColor(float r, float g, float b);
	void setBaseColor(const isnl::vec3& color);
	void setVisible(bool b = true);
	bool isVisible();
};

class GLComplex : public GLObject{
protected:
	std::vector<GLObject*> objs;
public:
	GLComplex();
	~GLComplex();
	virtual void initialize() const;
	virtual void render() const;

	void add(GLComplex* obj);
	void add(GLObject* obj);
	GLComplex& operator << (GLComplex* obj);
	GLComplex& operator << (GLObject* obj);
		  GLObject* operator[](int index);
	const GLObject* operator[](int index) const;
	int size();
};
class GLBox : public GLObject{
protected:
	isnl::vec3 size;
public:
	GLBox();
	GLBox(float sx, float sy, float sz);
	GLBox(const isnl::vec3& size);
	GLBox(const isnl::pos& p, float sx, float sy, float sz);
	GLBox(const isnl::pos& p, const isnl::vec3& size);
	GLBox(const isnl::pos& p, float sx, float sy, float sz, const isnl::vec3& color);
	GLBox(const isnl::pos& p, const isnl::vec3& size, const isnl::vec3& color);
	void initialize() const;
	void render() const;
};
class GLCylinder : public GLObject{
protected:
	float height, topRadius, botRadius;
	int slice;
public:
	GLCylinder();
	GLCylinder(float height, float radius, int slice=20);
	GLCylinder(float height, float topRadius, float bottomRadius, int slice=20);
	void initialize() const;
	void render() const;
};
class GLGear : public GLObject{
protected:
	float innerRadius;
	float outerRadius;
	float thickness;
	float toothSize;
	GLint toothCount;
public:
	GLGear();
	GLGear(double innerRadius, double outerRadius, double thickness, double toothSize, int toothCount);
	void initialize() const;
};
class GLSphere : public GLObject{
protected:
	float radius;
	int slice;
public:
	GLSphere(float radius);
	GLSphere(float radius, int slice);
	void initialize() const;
};
class GLPolygon : public GLObject{};

class GLGridPlane : public GLObject{
protected:
	float dx, dy;
	int   nx, ny;
	isnl::vec3 color1, color2;
public:
	GLGridPlane();
	GLGridPlane(float dx, float dy, int nx, int ny);
	GLGridPlane(float dx, float dy, int nx, int ny, const isnl::vec3& color1, const isnl::vec3& color2);
	void initialize() const;
};
class GLArrow : public GLObject{
protected:
	float radius, tipradius, length, tiplength;
	mutable GLuint polelistid, tiplistid;
	int slice;
public:
	// length : total length
	GLArrow();
	GLArrow(float radius, float length);
	GLArrow(float radius, float tipRadius, float length);
	GLArrow(float radius, float tipRadius, float length, float tipLength);
	void initialize() const;
	void render() const;
	void setLength(float length);
	void setDirection(const isnl::vec3& dir);
	void setDirection(const isnl::vec3& p, const isnl::vec3& dir);
};

class GLSTL : public GLObject{
	struct group{

	float normal[3];

	float vertex1[3];
	float vertex2[3];
	float vertex3[3];

	char unuse [2];

	} ;
protected:
	std::vector<group> facet;
public:
	// length : total length
	GLSTL();
	GLSTL(std::string filename);
	void initialize() const;
	void render() const;
private:
	void calcNormal(group facej, float out[3])  const;
	void ReduceToUnit(float vector[3])  const;

};
//class GLCamera{
//protected:
//	mat4 m;
//public:
//	GLCamera();

//	isnl::pos getPosition();
//	void setPosition(const isnl::pos& p);


//	void on();
//};
#endif // GLOBJECT_H

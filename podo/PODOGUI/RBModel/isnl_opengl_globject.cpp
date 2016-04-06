#include <isnl/opengl/globject.h>

using namespace isnl;
using namespace std;
GLObject::GLObject(){
	this->setBaseColor(1.f, 1.f, 1.f);
	this->listid = 0;
	this->visible = true;
	this->m = mat4::eye();
}
GLObject::~GLObject(){
	if(listid)
		glDeleteLists(listid, 1);
}
pos GLObject::getPosition(){
	return pos(~m);
}
void GLObject::setPosition(const pos& p){this->m = ~mat4(p);}
void GLObject::render() const {
	if(visible){
		glPushMatrix();
		glMultMatrixf(m.v);
		glCallList(listid);
		glPopMatrix();
	}
}
void GLObject::setBaseColor(float r, float g, float b){
	this->baseColor = vec3(r,g,b);
}
void GLObject::setBaseColor(const isnl::vec3& color){
	this->baseColor = color;
}
void GLObject::setVisible(bool b){
	this->visible = b;
}
bool GLObject::isVisible(){return visible;}

GLComplex::GLComplex(){}
GLComplex::~GLComplex(){
	//~GLObject();
}
void GLComplex::initialize() const {
	int n = objs.size();
	for(int i = 0; i < n; ++i)
		objs[i]->initialize();
}
void GLComplex::render() const {
	int n = objs.size();
	for(int i = 0; i < n; ++i)
		objs[i]->render();
}
void GLComplex::add(GLObject* obj){
	objs.push_back(obj);
}
void GLComplex::add(GLComplex* obj){
	objs.insert(objs.end(), obj->objs.begin(), obj->objs.end());
}
GLComplex& GLComplex::operator << (GLObject* obj){
	objs.push_back(obj); return *this;
}
GLComplex& GLComplex::operator << (GLComplex* obj){
	objs.insert(objs.end(), obj->objs.begin(), obj->objs.end()); return *this;
}
GLObject* GLComplex::operator[](int index){
	return objs[index];
}
const GLObject* GLComplex::operator[](int index) const{
	return objs[index];
}
int GLComplex::size(){return objs.size();}

GLBox::GLBox(){
	this->size = vec3(1.0f, 1.0f, 1.0f);
}
GLBox::GLBox(float sx, float sy, float sz){
	this->size = vec3(sx, sy, sz);
}
GLBox::GLBox(const isnl::vec3& size){
	this->size = size;
}
GLBox::GLBox(const isnl::pos& p, float sx, float sy, float sz){
	this->setPosition(p);
	this->size = vec3(sx, sy, sz);
}
GLBox::GLBox(const isnl::pos& p, const isnl::vec3& size){
	this->setPosition(p);
	this->size = size;
}
GLBox::GLBox(const isnl::pos& p, float sx, float sy, float sz, const isnl::vec3& color){
	this->setPosition(p);
	this->size = vec3(sx, sy, sz);
	setBaseColor(color);
}
GLBox::GLBox(const isnl::pos& p, const isnl::vec3& size, const isnl::vec3& color){
	this->setPosition(p);
	this->size = size;
	setBaseColor(color);
}
void GLBox::initialize() const {}
void GLBox::render() const{
	if(!visible)return;

	float reflectance[4] = {baseColor.x, baseColor.y, baseColor.z, 1.0f};

	//listid = glGenLists(1);
	//glNewList(listid, GL_COMPILE);
	glPushMatrix();
	glMultMatrixf(m.v);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);
	float x = size.x/2.f, y = size.y/2.f, z = size.z/2.f;

	glBegin(GL_QUADS);
	glNormal3f( 1.0f, 0.0f, 0.0f);
	glVertex3f( x, y, z);
	glVertex3f( x,-y, z);
	glVertex3f( x,-y,-z);
	glVertex3f( x, y,-z);
	glEnd();
	glBegin(GL_QUADS);
	glNormal3f( 0.0f, 1.0f, 0.0f);
	glVertex3f( x, y, z);
	glVertex3f( x, y,-z);
	glVertex3f(-x, y,-z);
	glVertex3f(-x, y, z);
	glEnd();
	glBegin(GL_QUADS);
	glNormal3f( 0.0f, 0.0f, 1.0f);
	glVertex3f( x, y, z);
	glVertex3f(-x, y, z);
	glVertex3f(-x,-y, z);
	glVertex3f( x,-y, z);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-x, y, z);
	glVertex3f(-x,-y, z);
	glVertex3f(-x,-y,-z);
	glVertex3f(-x, y,-z);
	glEnd();
	glBegin(GL_QUADS);
	glNormal3f( 0.0f,-1.0f, 0.0f);
	glVertex3f( x,-y, z);
	glVertex3f( x,-y,-z);
	glVertex3f(-x,-y,-z);
	glVertex3f(-x,-y, z);
	glEnd();
	glBegin(GL_QUADS);
	glNormal3f( 0.0f, 0.0f,-1.0f);
	glVertex3f( x, y,-z);
	glVertex3f(-x, y,-z);
	glVertex3f(-x,-y,-z);
	glVertex3f( x,-y,-z);
	glEnd();

	//glEndList();
	glPopMatrix();
}

GLCylinder::GLCylinder(){
	this->height    = 1.f;
	this->topRadius = 1.f;
	this->botRadius = 1.f;
	this->slice     = 20;
}
GLCylinder::GLCylinder(float height, float radius, int slice){
	this->height    = height;
	this->topRadius = radius;
	this->botRadius = radius;
	this->slice     = slice;
}
GLCylinder::GLCylinder(float height, float topRadius, float bottomRadius, int slice){
	this->height    = height;
	this->topRadius = topRadius;
	this->botRadius = bottomRadius;
	this->slice     = slice;
}
void GLCylinder::initialize() const {}
void GLCylinder::render() const {
	if(!visible)return;
	float pi = 3.1415926535897932384626433832795f;

	float reflectance[4] = {baseColor.x, baseColor.y, baseColor.z, 1.0f};

	//listid = glGenLists(1);
	//glNewList(listid, GL_COMPILE);
	glPushMatrix();
	glMultMatrixf(m.v);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);
	float h = height/2.f, t = topRadius, b = botRadius;
	float d = 2*pi/slice;

	for(int i = 0;i < slice; ++i){
		int j = i+1;
		glBegin(GL_QUADS);
		isnl::vec3 p1( t*cos(d*i), t*sin(d*i), h);
		isnl::vec3 p2( t*cos(d*j), t*sin(d*j), h);
		isnl::vec3 p3( b*cos(d*i), b*sin(d*i),-h);
		isnl::vec3 n  = cross(p1 - p3, p1 - p2);
		glNormal3f(n.x, n.y, n.z);
		glVertex3f( t*cos(d*i), t*sin(d*i), h);
		glVertex3f( t*cos(d*j), t*sin(d*j), h);
		glVertex3f( b*cos(d*j), b*sin(d*j),-h);
		glVertex3f( b*cos(d*i), b*sin(d*i),-h);
		glEnd();
	}


	// draw top
	glBegin(GL_POLYGON);
	for(int i = 0;i < slice; ++i){
		glNormal3f( 0.0f, 0.0f, 1.0f);
		glVertex3f( t*cos(d*i), t*sin(d*i), h);
	}
	glEnd();

	// draw bottom
	glBegin(GL_POLYGON);
	for(int i = 0;i < slice; ++i){
		glNormal3f( 0.0f, 0.0f,-1.0f);
		glVertex3f( b*cos(d*i), b*sin(d*i),-h);
	}
	glEnd();

	//glEndList();
	glPopMatrix();

}


GLSphere::GLSphere(float radius){
	this->radius = radius;
	this->slice  = 20;
}
GLSphere::GLSphere(float radius, int slice){
	this->radius = radius;
	this->slice  = slice;
}
void GLSphere::initialize() const {
	float pi = 3.1415926535897932384626433832795f;

	float r = radius;
	float d = 2*pi/slice;
	float nx = 2*int(slice/2); // slice shoud be even
	float nz = slice/2;

	float reflectance[4] = {baseColor.x, baseColor.y, baseColor.z, 1.0f};

	if(listid != 0)
		glDeleteLists(listid,1);

	listid = glGenLists(1);
	glNewList(listid, GL_COMPILE);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);
	for(int i = 0; i < nz-1; ++i){
		for(int j = 0; j < nx; ++j){
			int k = i+1, l = j+1;
			vec3 p1(sin(d*i)*cos(d*j), sin(d*i)*sin(d*j), cos(d*i));
			vec3 p2(sin(d*k)*cos(d*j), sin(d*k)*sin(d*j), cos(d*k));
			vec3 p3(sin(d*k)*cos(d*l), sin(d*k)*sin(d*l), cos(d*k));
			vec3 p4(sin(d*i)*cos(d*l), sin(d*i)*sin(d*l), cos(d*i));
			glBegin(GL_QUADS);
			glNormal3f(p1.x, p1.y, p1.z); glVertex3f(r*p1.x, r*p1.y, r*p1.z);
			glNormal3f(p2.x, p2.y, p2.z); glVertex3f(r*p2.x, r*p2.y, r*p2.z);
			glNormal3f(p3.x, p3.y, p3.z); glVertex3f(r*p3.x, r*p3.y, r*p3.z);
			glNormal3f(p4.x, p4.y, p4.z); glVertex3f(r*p4.x, r*p4.y, r*p4.z);
			glEnd();
		}
	}
	glEndList();
}

GLGear::GLGear(double innerRadius, double outerRadius, double thickness, double toothSize, int toothCount){
	this->innerRadius = innerRadius;
	this->outerRadius = outerRadius;
	this->thickness   = thickness;
	this->toothSize   = toothSize;
	this->toothCount  = toothCount;
	this->listid = 0;
}
void GLGear::initialize() const{
	const double Pi = 3.14159265358979323846;

	float reflectance[4] = {baseColor.x, baseColor.y, baseColor.z, 1.0f};

	listid = glGenLists(1);
	glNewList(listid, GL_COMPILE);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);

	float r0 = innerRadius;
	float r1 = outerRadius - toothSize / 2.0;
	float r2 = outerRadius + toothSize / 2.0;
	float delta = (2.0 * Pi / toothCount) / 4.0;
	float z = thickness / 2.0;
	int i, j;

	glShadeModel(GL_FLAT);

	for (i = 0; i < 2; ++i) {
		float sign = (i == 0) ? +1.0 : -1.0;

		glNormal3f(0.0f, 0.0f, sign);

		glBegin(GL_QUAD_STRIP);
		for (j = 0; j <= toothCount; ++j) {
			float angle = 2.0 * Pi * j / toothCount;
			glVertex3f(r0 * cos(angle), r0 * sin(angle), sign * z);
			glVertex3f(r1 * cos(angle), r1 * sin(angle), sign * z);
			glVertex3f(r0 * cos(angle), r0 * sin(angle), sign * z);
			glVertex3f(r1 * cos(angle + 3 * delta), r1 * sin(angle + 3 * delta),
					   sign * z);
		}
		glEnd();

		glBegin(GL_QUADS);
		for (j = 0; j < toothCount; ++j) {
			float angle = 2.0 * Pi * j / toothCount;
			glVertex3f(r1 * cos(angle), r1 * sin(angle), sign * z);
			glVertex3f(r2 * cos(angle + delta), r2 * sin(angle + delta),
					   sign * z);
			glVertex3f(r2 * cos(angle + 2 * delta), r2 * sin(angle + 2 * delta),
					   sign * z);
			glVertex3f(r1 * cos(angle + 3 * delta), r1 * sin(angle + 3 * delta),
					   sign * z);
		}
		glEnd();
	}

	glBegin(GL_QUAD_STRIP);
	for (i = 0; i < toothCount; ++i) {
		for (j = 0; j < 2; ++j) {
			float angle = 2.0 * Pi * (i + (j / 2.0)) / toothCount;
			float s1 = r1;
			float s2 = r2;
			if (j == 1)
				qSwap(s1, s2);

			glNormal3f(cos(angle), sin(angle), 0.0);
			glVertex3f(s1 * cos(angle), s1 * sin(angle), +z);
			glVertex3f(s1 * cos(angle), s1 * sin(angle), -z);

			glNormal3f(s2 * sin(angle + delta) - s1 * sin(angle),
					   s1 * cos(angle) - s2 * cos(angle + delta), 0.0);
			glVertex3f(s2 * cos(angle + delta), s2 * sin(angle + delta), +z);
			glVertex3f(s2 * cos(angle + delta), s2 * sin(angle + delta), -z);
		}
	}
	glVertex3f(r1, 0.0, +z);
	glVertex3f(r1, 0.0, -z);
	glEnd();

	glShadeModel(GL_SMOOTH);

	glBegin(GL_QUAD_STRIP);
	for (i = 0; i <= toothCount; ++i) {
		float angle = i * 2.0 * Pi / toothCount;
		glNormal3f(-cos(angle), -sin(angle), 0.0);
		glVertex3f(r0 * cos(angle), r0 * sin(angle), +z);
		glVertex3f(r0 * cos(angle), r0 * sin(angle), -z);
	}
	glEnd();

	glEndList();
}

GLGridPlane::GLGridPlane(){
	this->dx = 0.1f;
	this->dy = 0.1f;
	this->nx = 10;
	this->ny = 10;
	this->color1 = vec3(0.8f, 0.5f, 0.3f);
	this->color2 = vec3(1.0f, 1.0f, 0.0f);
}
GLGridPlane::GLGridPlane(float dx, float dy, int nx, int ny){
	this->dx = dx;
	this->dy = dy;
	this->nx = nx;
	this->ny = ny;
	this->color1 = vec3(0.8f, 0.5f, 0.3f);
	this->color2 = vec3(1.0f, 1.0f, 0.0f);
}
GLGridPlane::GLGridPlane(float dx, float dy, int nx, int ny, const isnl::vec3& color1, const isnl::vec3& color2){
	this->dx = dx;
	this->dy = dy;
	this->nx = nx;
	this->ny = ny;
	this->color1 = color1;
	this->color2 = color2;
}
void GLGridPlane::initialize() const {

	float reflectance1[4] = {color1.x, color1.y, color1.z, 1.0f};
	float reflectance2[4] = {color2.x, color2.y, color2.z, 1.0f};

	float x0 = -(dx*nx)/2.f, y0 = -(dy*ny)/2.f;

	if(listid != 0)
		glDeleteLists(listid,1);

	listid = glGenLists(1);
	glNewList(listid, GL_COMPILE);

	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance1);
	for(int i = 0; i < nx; ++i){
		for(int j = 0; j < ny; ++j){
			if((i+j)%2 == 0){
				int k = i+1, l = j+1;
				glBegin(GL_QUADS);
				glNormal3f( 1.0f, 0.0f, 0.0f);
				glVertex3f( x0+dx*i, y0+dy*j, 0);
				glVertex3f( x0+dx*i, y0+dy*l, 0);
				glVertex3f( x0+dx*k, y0+dy*l, 0);
				glVertex3f( x0+dx*k, y0+dy*j, 0);
				glEnd();
			}
		}
	}
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance2);
	for(int i = 0; i < nx; ++i){
		for(int j = 0; j < ny; ++j){
			if((i+j)%2 == 1){
				int k = i+1, l = j+1;
				glBegin(GL_QUADS);
				glNormal3f( 1.0f, 0.0f, 0.0f);
				glVertex3f( x0+dx*i, y0+dy*j, 0);
				glVertex3f( x0+dx*i, y0+dy*l, 0);
				glVertex3f( x0+dx*k, y0+dy*l, 0);
				glVertex3f( x0+dx*k, y0+dy*j, 0);
				glEnd();
			}
		}
	}


	glEndList();
}

GLArrow::GLArrow(){
	this->radius    = 0.2f;
	this->tipradius = 0.3f;
	this->length    = 2.0f;
	this->tiplength = 1.0f;
	this->slice     = 20;
}
GLArrow::GLArrow(float radius, float length){
	this->radius    = radius;
	this->tipradius = 1.5f*radius;
	this->length    = length;
	this->tiplength = 3.0f*tipradius;
	this->slice     = 20;
}
GLArrow::GLArrow(float radius, float tipRadius, float length){
	this->radius    = radius;
	this->tipradius = tipRadius;
	this->length    = length;
	this->tiplength = 3.0f*tipradius;
	this->slice     = 20;
}
GLArrow::GLArrow(float radius, float tipRadius, float length, float tipLength){
	this->radius    = radius;
	this->tipradius = tipRadius;
	this->length    = length;
	this->tiplength = tipLength;
	this->slice     = 20;
}
void GLArrow::setDirection(const vec3& dir){
	this->length = dir.norm();
	pos pp = this->getPosition();
	vec3 d = dir.unit();
	vec3 z(0,0,1);
	vec3 axis = cross(d,z);
	float s = axis.norm(), c = dot(d, z);
	pp.q = quat(axis, atan2(s, c));
	this->setPosition(pp);
}
void GLArrow::setDirection(const vec3& p, const vec3& dir){
	this->length = dir.norm();
	pos pp = p;
	vec3 d = dir.unit();
	vec3 z(0,0,1);
	vec3 axis = cross(d,z);
	float s = axis.norm(), c = dot(d, z);
	pp.q = quat(axis, atan2(s, c));
	this->setPosition(pp);
}
void GLArrow::setLength(float length){
	this->length = length;
}
void GLArrow::initialize() const {
	float pi = 3.1415926535897932384626433832795f;

	float reflectance[4] = {baseColor.x, baseColor.y, baseColor.z, 1.0f};
	float d = 2*pi/slice;

	// draw arrow tip
	tiplistid = glGenLists(1);
	glNewList(tiplistid, GL_COMPILE);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);

	glBegin(GL_TRIANGLE_FAN);
	glNormal3f( 0.f, 0.f, 1.f);
	glVertex3d( 0.f, 0.f, 1.f);
	for(int i = 0;i < slice+1; ++i){
		float x = cos(d*i), y = sin(d*i);
		glNormal3f(x, y, 1.f);
		glVertex3f(x, y, 0.f);
	}
	glEnd();

	// draw bottom
	glBegin(GL_POLYGON);
	for(int i = 0;i < slice; ++i){
		glNormal3f( 0.0f, 0.0f,-1.0f);
		glVertex3f( cos(d*i), sin(d*i), 0.f);
	}
	glEnd();

	glEndList();


	// draw arrow pole
	polelistid = glGenLists(1);
	glNewList(polelistid, GL_COMPILE);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);

	for(int i = 0;i < slice; ++i){
		int j = i+1;
		glBegin(GL_QUADS);
		isnl::vec3 p1( cos(d*i), sin(d*i), 1.f);
		isnl::vec3 p2( cos(d*j), sin(d*j), 1.f);
		isnl::vec3 p3( cos(d*i), sin(d*i), 0.f);
		isnl::vec3 n  = cross(p1 - p3, p1 - p2);
		glNormal3f(n.x, n.y, n.z);
		glVertex3f( cos(d*i), sin(d*i), 1.f);
		glVertex3f( cos(d*j), sin(d*j), 1.f);
		glVertex3f( cos(d*j), sin(d*j), 0.f);
		glVertex3f( cos(d*i), sin(d*i), 0.f);
		glEnd();
	}

	// draw bottom
	glBegin(GL_POLYGON);
	glNormal3f( 0.0f, 0.0f,-1.0f);
	for(int i = 0;i < slice; ++i){
		glVertex3d( cos(d*i), sin(d*i), 0.f);
	}
	glEnd();
	glEndList();
}
void GLArrow::render() const {
	// if total length < 2*tiplength, resize tiplength as length/2
	float s = (length > 0 ? 1 : -1);
	float r1 = radius, r2 = tipradius, d1 = fabs(length);
	float d2 = (d1 > 2.f*tiplength ? tiplength : d1/2.f);

	if(visible){
		glPushMatrix();
		glMultMatrixf(m.v);

		// draw pole
		glPushMatrix();
		glScalef(r1, r1, s*(d1-d2));
		glCallList(polelistid);
		glPopMatrix();

		// draw tip
		glPushMatrix();
		glTranslatef(0.f, 0.f, s*(d1-d2));
		glScalef(r2, r2, s*d2);
		glCallList(tiplistid);
		glPopMatrix();
		glPopMatrix();
	}
}

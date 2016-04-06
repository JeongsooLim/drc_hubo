#include <isnl/util/skeleton.h>

using namespace isnl;
using namespace std;

Skeleton::Skeleton(Bones& bones, Joints& joints){
	this->root = bones[0];
	this->jointnum = joints.size();
	this->bonenum  = bones.size();

	this->comps.reserve(jointnum+bonenum);
	for(int i = 0; i < jointnum; ++i)
		comps.push_back(joints[i]);
	for(int i = 0; i < bonenum; ++i)
		comps.push_back(bones[i]);

	int n = comps.size();
	for(int i = 0; i < n; ++i){
		comps[i]->id = i;
	}
}
int   Skeleton::ncomp() const {
	return comps.size();
}
int   Skeleton::njoint() const {
	return jointnum;
}
int   Skeleton::nbone() const {
	return bonenum;
}
pos   Skeleton::getPosition(const floats& ref, int id) const {
	return comps[id]->getPosition(ref);
}
pos_s Skeleton::getPosition(const floats& ref, const ints& id) const {
	int n = id.size();
	pos_s ret(n);
	for(int i = 0; i < n; ++i)
		ret[i] = comps[i]->getPosition(ref);
	return ret;
}
int   Skeleton::name2id(const string& name) const {
	typedef std::map<std::string, int>::const_iterator iter;
	iter it = compmap.find(name);
	return (it != compmap.end() ? it->second : -1);
}
ints  Skeleton::name2id(const strings& name) const {
	int n = name.size();
	ints ret(n);
	for(int i = 0; i < n; ++i)
		ret[i] = name2id(name[i]);
	return ret;
}

Component::Component(const std::string& name){
	this->id = 0;
	this->name = name;
	this->model = NULL;
}
Component::Component(const std::string& name, GLObject* model){
	this->name = name;
	this->model = model;
}
void Component::setModel(GLObject *model){
	this->model = model;
}
GLObject* Component::getModel(){
	return model;
}
int Component::nchild() const {
	return children.size();
}
Component* Component::child(int i){
	return children[i];
}
const Component* Component::child(int i) const{
	return children[i];
}
const GLObject* Component::getModel() const{
	return model;
}
pos Component::getPosition(const floats& ref) const{
	mat4 m = mat4::eye();
    int i=0;

    for(const Component* curr= this; curr != 0; curr = curr->parent){
        i++;
		m =curr->transform(ref)*m;
	}
	m = mat4(pos(ref[0], ref[1], ref[2], ref[3], ref[4], ref[5], ref[6])) * m;
	return m;
}
Component& Component::operator <<(Component* child){
	child->parent = this;
	this->children.push_back(child);
	return *this;
}
Component& Component::operator + (Component* child){
	child->parent = this;
	this->children.push_back(child);
	return *child;
}


Joint::Joint(const std::string& name):Component(name){}

RevoluteJoint::RevoluteJoint(const string& name, const vec3& axis):Joint(name){
	this->axis = axis;
	this->offset = 0.f;
}
mat4 RevoluteJoint::transform(const floats& ref) const {
	return mat4::rotate(ref[id+7]+offset, axis);
}
void RevoluteJoint::setOffset(float offset){
	this->offset = offset;
}
RevoluteXJoint::RevoluteXJoint(const std::string& name):RevoluteJoint(name, vec3(1,0,0)){}
mat4 RevoluteXJoint::transform(const floats& ref) const {
	return mat4::rotateX(ref[id+7]+offset);
}
RevoluteYJoint::RevoluteYJoint(const std::string& name):RevoluteJoint(name, vec3(1,0,0)){}
mat4 RevoluteYJoint::transform(const floats& ref) const {
	return mat4::rotateY(ref[id+7]+offset);
}
RevoluteZJoint::RevoluteZJoint(const std::string& name):RevoluteJoint(name, vec3(1,0,0)){}
mat4 RevoluteZJoint::transform(const floats& ref) const {
	return mat4::rotateZ(ref[id+7]+offset);
}
SpheroidJoint::SpheroidJoint(const std::string& name, int refidx):Joint(name){
	this->refidx = refidx;
}
mat4 SpheroidJoint::transform(const floats& ref) const {
	return mat4(quat(ref[refidx], ref[refidx+1], ref[refidx+2], ref[refidx+3]));
}


PrismaticJoint::PrismaticJoint(const string& name, const vec3& dir):Joint(name){
	this->dir  = dir;
}
isnl::mat4 PrismaticJoint::transform(const floats& ref) const{
	return mat4::translate(ref[id+7]*dir);
}

Bone::Bone(const string& name, const pos& offset, GLObject* model):Component(name){
	this->offset = offset;
	this->model  = model;
}
isnl::mat4 Bone::transform(const floats& ) const {
	return offset;
}



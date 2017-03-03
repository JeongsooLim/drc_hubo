#ifndef SKELETON_H
#define SKELETON_H

#include <isnl/base/array.h>
#include <isnl/math/geometry.h>
#include <map>

class Skeleton;
class Component;
class Joint;
class Bone;
class GLObject;

typedef std::vector<Component*> Components;
typedef std::vector<Joint*>     Joints;
typedef std::vector<Bone*>     Bones;


class Skeleton{
protected:
	Components comps;
	std::map<std::string, int>       compmap;
	int jointnum;
	int bonenum;
	int rootid;
protected:
	Component* root;
public:
	Skeleton(Bones& bones, Joints& joints);

	int ncomp() const;
	int njoint() const;
	int nbone() const;

	// Forward Kinematics
	isnl::pos   getPosition(const floats& ref, int id) const;
	isnl::pos_s getPosition(const floats& ref, const ints& id) const;

	//
	int  name2id(const std::string& name) const;
	ints name2id(const strings& name) const;
};
class Component{
	friend class Skeleton;
protected:
	std::string name;
	int id;           // component id
	Component *parent;
	Components children;
	GLObject *model;

public:
	Component(const std::string& name);
	Component(const std::string& name, GLObject* model);
	void setModel(GLObject *model);
		  GLObject* getModel();
	const GLObject* getModel() const;
	int nchild() const;
		  Component* child(int i);
	const Component* child(int i) const;

	virtual isnl::mat4 transform(const floats& ref) const=0;
	isnl::pos getPosition(const floats& ref) const;

	Component& operator << (Component* child);
	Component& operator + (Component* child);

    void setParent(Component *_parent){parent = _parent;}
};
class Joint : public Component{
protected:
public:
	Joint(const std::string& name);
};
class RevoluteJoint : public Joint{
protected:
	isnl::vec3 axis; // axis of rotation
	float offset;
public:
	RevoluteJoint(const std::string& name, const isnl::vec3& axis);
	void setOffset(float offset);
	isnl::mat4 transform(const floats& ref) const;
};
class RevoluteXJoint : public RevoluteJoint{
public:
	RevoluteXJoint(const std::string& name);
	isnl::mat4 transform(const floats& ref) const;
};
class RevoluteYJoint : public RevoluteJoint{
public:
	RevoluteYJoint(const std::string& name);
	isnl::mat4 transform(const floats& ref) const;
};
class RevoluteZJoint : public RevoluteJoint{
public:
	RevoluteZJoint(const std::string& name);
	isnl::mat4 transform(const floats& ref) const;
};
class PrismaticJoint : public Joint{
protected:
	isnl::vec3 dir; // direction of translation
public:
	PrismaticJoint(const std::string& name, const isnl::vec3& dir);
	isnl::mat4 transform(const floats& ref) const;
};


class SpheroidJoint : public Joint{
protected:
	int refidx;
public:
	SpheroidJoint(const std::string& name, int refidx);
	isnl::mat4 transform(const floats& ref) const;
};


class Bone : public Component{
protected:
	isnl::mat4 offset;
public:
	Bone(const std::string& name, const isnl::pos& offset, GLObject* model=NULL);
	isnl::mat4 transform(const floats& ref) const;
};



#endif // SKELETON_H

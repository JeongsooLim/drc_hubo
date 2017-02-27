#ifndef ISNL_GLSKELETON_H
#define ISNL_GLSKELETON_H


#include <isnl/util/skeleton.h>
#include <isnl/opengl/globject.h>

class GLSkeleton : public Skeleton, public GLObject{
public:
	floats ref;
public:
	GLSkeleton(Bones& bones, Joints& joints);

	// Forward Kinematics
	isnl::pos   getPosition(int id) const;
	isnl::pos_s getPosition(const ints& id) const;
	isnl::pos   getPosition(const floats& ref, int id) const;
	isnl::pos_s getPosition(const floats& ref, const ints& id) const;

	void initialize() const;
	void render() const;
};

#endif

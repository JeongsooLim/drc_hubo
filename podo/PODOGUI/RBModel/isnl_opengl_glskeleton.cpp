#include <isnl/opengl/glskeleton.h>

using namespace isnl;
using namespace std;

void renderSeq(const Component* comp, const floats& ref);


GLSkeleton::GLSkeleton(Bones& bones, Joints& joints):Skeleton(bones, joints){}
void GLSkeleton::initialize() const {
	int n = this->ncomp();
	for(int i = 0; i < n; ++i)
		if(comps[i]->getModel())
			comps[i]->getModel()->initialize();
}
void GLSkeleton::render() const {
	if(visible){
		glPushMatrix();
		mat4 m(pos(ref[0], ref[1], ref[2], ref[3], ref[4], ref[5], ref[6])); m = ~m;
		glMultMatrixf(m.v);
		renderSeq(this->root, ref);
		glPopMatrix();
	}
}

pos   GLSkeleton::getPosition(int id) const {
	return Skeleton::getPosition(ref, id);
}
pos_s GLSkeleton::getPosition(const ints& id) const {
	return Skeleton::getPosition(ref, id);
}
pos   GLSkeleton::getPosition(const floats& ref, int id) const {
	return Skeleton::getPosition(ref, id);
}
pos_s GLSkeleton::getPosition(const floats& ref, const ints& id) const {
	return Skeleton::getPosition(ref, id);
}

void renderSeq(const Component* comp, const floats& ref){
	glPushMatrix();
	// Rendering current component
	mat4 m = comp->transform(ref); m = ~m;
	glMultMatrixf(m.v);

	const GLObject *model = comp->getModel();
	if(model) model->render();


	// Rendering children
	int n = comp->nchild();
	for(int i = 0; i < n; ++i)
		renderSeq(comp->child(i), ref);
	glPopMatrix();
}

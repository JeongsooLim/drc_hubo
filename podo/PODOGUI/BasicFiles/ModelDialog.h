#ifndef MODELDIALOG_H
#define MODELDIALOG_H

#include <QDialog>
#include <QGLWidget>
#include <QtOpenGL>


#include "isnl/opengl/glskeleton.h"
#include "CommonHeader.h"

static isnl::vec3 basecolor = isnl::vec3(1.0f, 1.0f, 1.0f);
GLSTL*  newStl(float x, float y, float z, std::string filename);

class QLabel;
class QMenu;
class QScrollArea;
class QSlider;
class GLWidget;


namespace Ui {
class ModelDialog;
}

class ModelDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ModelDialog(QWidget *parent = 0);
    ~ModelDialog();

private:
    Ui::ModelDialog *ui;
    QTimer		*displayTimer;

    GLWidget	*glWidget;
    QLabel		*pixmapLabel;
    QSlider		*xSlider;
    QSlider		*ySlider;
    QSlider		*zSlider;

    QAction		*exitAct;
    QAction		*aboutAct;
    QAction		*aboutQtAct;

    QSlider *createSlider(const char *changedSignal, const char *setterSlot);

private slots:
    void DisplayUpdate();
    void on_BT_CamLeft_clicked();
    void on_BT_CamRight_clicked();
    void on_BT_CamFront_clicked();

};



//==========================================
class HUBOModel : public GLSkeleton
{
protected:
    std::vector<GLComplex*> objects;
public:
    HUBOModel(Bones& bones, Joints& joints, std::vector<GLComplex*> objects) : GLSkeleton(bones, joints){
        this->objects = objects;
    }

    void setColor(int id, const isnl::vec3& color){
        GLComplex& temp = *objects[id];
        for(int i = 0; i < temp.size(); ++i){
            temp[i]->setBaseColor(color);
        }
    }
};
inline GLSTL*     newStl(float x, float y, float z, std::string filename){
    GLSTL *ret = new GLSTL(filename);
    ret->setPosition(isnl::pos(x,y,z));
    ret->setBaseColor(basecolor);
    return ret;
}
inline GLBox*      newBox(float x, float y, float z, float sx, float sy, float sz){
    return new GLBox(isnl::pos(x,y,z), sx, sy, sz, basecolor);
}
//===========================================





class GLWidget : public QGLWidget
{
    Q_OBJECT
    friend class ModelDialog;

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();

    int xRotation() const { return xRot; }
    int yRotation() const { return yRot; }
    int zRotation() const { return zRot; }

public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);

signals:
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

private:
    void normalizeAngle(int *angle);

    GLComplex		globjs;
    HUBOModel       *model;
    GLGridPlane 	*floor;
    GLArrow         *rwforce, *lwforce, *rfforce, *lfforce;
    GLArrow         *rwheel, *lwheel;

    float           rwh_speed, lwh_speed;

    int xRot;
    int yRot;
    int zRot;
    int gear1Rot;

    QPoint currPos, lastPos;

    HUBOModel*     newModel();
};


#endif // MODELDIALOG_H

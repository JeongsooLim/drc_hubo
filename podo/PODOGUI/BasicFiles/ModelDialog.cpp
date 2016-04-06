#include "ModelDialog.h"
#include "ui_ModelDialog.h"


using namespace isnl;


enum JointSequentialNumber_Finger{
    RF_a = NO_OF_JOINTS, RF_b, RF_c, LF_a, LF_b, LF_c, NO_OF_MODEL_JOINTS
};


enum BONE_ID{
    B_WST,B_TORSO,B_CAMERA,
    B_LSP,B_LSR,B_LSY,B_LEB,B_LWY,B_LWP,B_LWFT,B_LF_a,B_LF_b,B_LF_c,
    B_RSP,B_RSR,B_RSY,B_REB,B_RWY,B_RWP,B_RWFT,B_RF_a,B_RF_b,B_RF_c,
    B_LHY,B_LHR,B_LHP,B_LKN,B_LAP,B_LAR,B_LWH,
    B_RHY,B_RHR,B_RHP,B_RKN,B_RAP,B_RAR,B_RWH,
    NBONE
};
enum COMP_ID{
    JRHY, JRHR, JRHP, JRKN, JRAP, JRAR,
    JLHY, JLHR, JLHP, JLKN, JLAP, JLAR,
    JRSP, JRSR, JRSY, JREB, JRWY, JRWP,
    JLSP, JLSR, JLSY, JLEB, JLWY, JLWP,
    JWST,
    JRWY2, JRHAND, JLWY2, JLHAND,
    JRWH, JLWH,
    JRF_a, JRF_b, JRF_c, JLF_a, JLF_b, JLF_c,
    BWST,BTORSO,BCAMERA,
    BLSP,BLSR,BLSY,BLEB,BLWY,BLWP,BLWFT,BLF_a,BLF_b,BLF_c,
    BRSP,BRSR,BRSY,BREB,BRWY,BRWP,BRWFT,BRF_a,BRF_b,BRF_c,
    BLHY,BLHR,BLHP,BLKN,BLAP,BLAR,BLWH,
    BRHY,BRHR,BRHP,BRKN,BRAP,BRAR,BRWH,
    NCOMP
};
static const char COMP_NAME[NCOMP+1][10] = {
    "RHY", "RHR", "RHP", "RKN", "RAP", "RAR",
    "LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP",
    "WST",
    "RWY2", "RHAND", "LWY2", "LHAND",
    "JRF_a", "JRF_b", "JRF_c", "JLF_a", "JLF_b", "JLF_c",
    "RWH", "LWH",
    "BWST","BTORSO","BCAMERA",
    "BLSP","BLSR","BLSY","BLEB","BLWY","BLWP","BLWFT","BLF_a","BLF_b","BLF_c",
    "BRSP","BRSR","BRSY","BREB","BRWY","BRWP","BRWFT","BRF_a","BRF_b","BRF_c",
    "BLHY","BLHR","BLHP","BLKN","BLAP","BLAR","BLWH",
    "BRHY","BRHR","BRHP","BRKN","BRAP","BRAR","BRWH",
    "Null"
};


std::string STLPath = "../SHARE/GUI/stl/";

ModelDialog::ModelDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ModelDialog)
{
    ui->setupUi(this);

    glWidget = new GLWidget(this);

    QScrollArea *glWidgetArea = new QScrollArea;
    glWidgetArea->setWidget(glWidget);
    glWidgetArea->setWidgetResizable(true);
    glWidgetArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    glWidgetArea->setMinimumSize(50, 50);

    xSlider = createSlider(SIGNAL(xRotationChanged(int)), SLOT(setXRotation(int)));
    ySlider = createSlider(SIGNAL(yRotationChanged(int)), SLOT(setYRotation(int)));
    zSlider = createSlider(SIGNAL(zRotationChanged(int)), SLOT(setZRotation(int)));

    ui->LAYOUT_MODEL->addWidget(glWidgetArea, 0, 0);
    ui->LAYOUT_SLIDER->addWidget(xSlider,0,0);
    ui->LAYOUT_SLIDER->addWidget(ySlider,1,0);
    ui->LAYOUT_SLIDER->addWidget(zSlider,2,0);

    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue(180 * 16);

    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(100);
    //connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(DisplayUpdate()));
}

ModelDialog::~ModelDialog()
{
    delete ui;
}


void ModelDialog::DisplayUpdate(){
    if(ui->CB_USE_ENCODER->isChecked()){
        for(int i = 0; i < NO_OF_JOINTS; ++i)
            glWidget->model->ref[i+7] = JointEncoder(i) * D2Rf;

        glWidget->model->ref[3] = 1.0;
        glWidget->model->ref[4] = 0.0;
        glWidget->model->ref[5] = 0.0;
        glWidget->model->ref[6] = 0.0;
    }else{
        for(int i = 0; i < NO_OF_JOINTS; ++i)
            glWidget->model->ref[i+7]= JointReference(i) * D2Rf;
        glWidget->model->ref[3] = 1.0f;
        glWidget->model->ref[4] = 0.0f;
        glWidget->model->ref[5] = 0.0f;
        glWidget->model->ref[6] = 0.0f;
    }

    // Fingers always use encoder
    glWidget->model->ref[RF_a+7] = ( 100/36.2)*(36.2-JointEncoder(RHAND))*D2Rf;
    glWidget->model->ref[RF_b+7] = ( 100/36.2)*(36.2-JointEncoder(RHAND))*D2Rf;
    glWidget->model->ref[RF_c+7] = (-100/36.2)*(36.2-JointEncoder(RHAND))*D2Rf;

    glWidget->model->ref[LF_a+7] = ( 100/36.2)*(36.2-JointEncoder(LHAND))*D2Rf;
    glWidget->model->ref[LF_b+7] = ( 100/36.2)*(36.2-JointEncoder(LHAND))*D2Rf;
    glWidget->model->ref[LF_c+7] = (-100/36.2)*(36.2-JointEncoder(LHAND))*D2Rf;

    // Wheel Speed
    glWidget->rwh_speed = JointReference(RWH)/2.0;
    glWidget->lwh_speed = JointReference(LWH)/2.0;

    glWidget->floor->setVisible(ui->CB_SHOW_FLOOR->isChecked());
    glWidget->rfforce->setVisible(ui->CB_SHOW_FT->isChecked());
    glWidget->rwforce->setVisible(ui->CB_SHOW_FT->isChecked());
    glWidget->lfforce->setVisible(ui->CB_SHOW_FT->isChecked());
    glWidget->lwforce->setVisible(ui->CB_SHOW_FT->isChecked());
    glWidget->rwheel->setVisible(ui->CB_SHOW_WHEEL->isChecked());
    glWidget->lwheel->setVisible(ui->CB_SHOW_WHEEL->isChecked());

    glWidget->updateGL();
}


QSlider* ModelDialog::createSlider(const char *changedSignal, const char *setterSlot){
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    connect(slider, SIGNAL(valueChanged(int)), glWidget, setterSlot);
    connect(glWidget, changedSignal, slider, SLOT(setValue(int)));
    return slider;
}

void ModelDialog::on_BT_CamFront_clicked(){
    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue(180 * 16);
}
void ModelDialog::on_BT_CamRight_clicked(){
    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue(270 * 16);
}
void ModelDialog::on_BT_CamLeft_clicked(){
    xSlider->setValue(180 * 16);
    ySlider->setValue(180 * 16);
    zSlider->setValue( 90 * 16);
}


GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    xRot = 0;
    yRot = 0;
    zRot = 0;


    floor = new GLGridPlane(0.5f, 0.5f, 20, 20);
    floor->setPosition(isnl::pos(0.f, 0.f, -1.f));

    model = newModel();
    model->ref.resize(NO_OF_MODEL_JOINTS+7);//NO_OF_JOINTS+7);
    model->ref[3] = 1.f;

    globjs << model;
    globjs << floor;

    rwforce = new GLArrow(0.01f ,0.5f); rwforce->setBaseColor(1, 0, 0);
    lwforce = new GLArrow(0.01f ,0.5f); lwforce->setBaseColor(1, 0, 0);
    rfforce = new GLArrow(0.01f ,0.5f); rfforce->setBaseColor(1, 0, 0);
    lfforce = new GLArrow(0.01f ,0.5f); lfforce->setBaseColor(1, 0, 0);
    rwheel  = new GLArrow(0.018f,0.5f); rwheel->setBaseColor(1, 0.674, 0.173);
    lwheel  = new GLArrow(0.018f,0.5f); lwheel->setBaseColor(1, 0.674, 0.173);
    globjs << rwforce << lwforce << rfforce << lfforce;
    globjs << rwheel << lwheel;


}
GLWidget::~GLWidget()
{
    makeCurrent();
}

void GLWidget::setXRotation(int angle){
    normalizeAngle(&angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
    }
}
void GLWidget::setYRotation(int angle){
    normalizeAngle(&angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
    }
}
void GLWidget::setZRotation(int angle){
    normalizeAngle(&angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
    }
}
void GLWidget::initializeGL()
{
    GLfloat lightColor[] = {1.0f, 1.0f, 1.0f, 1.0f};

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, lightColor);
    glEnable(GL_DEPTH_TEST);

    globjs.initialize();

    glEnable(GL_NORMALIZE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}
void GLWidget::paintGL()
{
    GLfloat lightPos0[4] = { 10.0f,-10.0f, 7.0f, 1.0f };
    GLfloat lightPos1[4] = {-10.0f, 10.0f, 7.0f, 1.0f };
    GLfloat lightPos2[4] = {-10.0f,-10.0f, 7.0f, 1.0f };

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    glRotated(xRot / 16.0 -180, 1.0, 0.0, 0.0);
    glRotated(yRot / 16.0 -180, 0.0, 1.0, 0.0);
    glRotated(zRot / 16.0 -180, 0.0, 0.0, 1.0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);
    glLightfv(GL_LIGHT2, GL_POSITION, lightPos2);


    // ft ----
    rwforce->setPosition(model->getPosition(BRWFT)* isnl::pos(-0.05f,0.0f,0.0f));
    lwforce->setPosition(model->getPosition(BLWFT)* isnl::pos(-0.05f,0.0f,0.0f));
    rfforce->setPosition(model->getPosition(BRAR) * isnl::pos(-0.15f,0.0f,0.0f));
    lfforce->setPosition(model->getPosition(BLAR) * isnl::pos(-0.15f,0.0f,0.0f));
    rfforce->setLength(PODO_DATA.CoreSEN.FT[0].Fz/800.0);
    lfforce->setLength(PODO_DATA.CoreSEN.FT[1].Fz/800.0);
    rwforce->setLength(PODO_DATA.CoreSEN.FT[2].Fz/50.0);
    lwforce->setLength(PODO_DATA.CoreSEN.FT[3].Fz/50.0);

    // wheel----
    rwheel->setLength(rwh_speed);
    isnl::pos rwheelPos = model->getPosition(BRKN);
    isnl::quat rwheelQuat = isnl::quat::rotateY(90.0*D2Rf);
    rwheelPos.qw = rwheelQuat.w;
    rwheelPos.qx = rwheelQuat.x;
    rwheelPos.qy = rwheelQuat.y;
    rwheelPos.qz = rwheelQuat.z;
    rwheelPos.y -= 0.15;
    rwheel->setPosition(rwheelPos);

    lwheel->setLength(lwh_speed);
    isnl::pos lwheelPos = model->getPosition(BLKN);
    isnl::quat lwheelQuat = isnl::quat::rotateY(90.0*D2Rf);
    lwheelPos.qw = lwheelQuat.w;
    lwheelPos.qx = lwheelQuat.x;
    lwheelPos.qy = lwheelQuat.y;
    lwheelPos.qz = lwheelQuat.z;
    lwheelPos.y += 0.15;
    lwheel->setPosition(lwheelPos);


    globjs.render();
    glPopMatrix();
}

void GLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-0.1, +0.1, -0.1, 0.1, 0.1, 60.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0.0, 0.0, -1.0);
    // change camera coord to world coord :
    glRotatef(2.0944*R2Df, -0.5774, -0.5774, -0.5774);
}
void GLWidget::mousePressEvent(QMouseEvent *event){
    lastPos = event->pos();
}
void GLWidget::mouseMoveEvent(QMouseEvent *event){
    currPos = event->pos();
    int dx = currPos.x() - lastPos.x();
    int dy = currPos.y() - lastPos.y();

    if(event->buttons() & Qt::RightButton){
        setYRotation(yRot + 2 * dy);
        setZRotation(zRot + 2 * dx);
    }else if (event->buttons() & Qt::LeftButton){
//        setXRotation(xRot + 8 * dy);
//        setZRotation(zRot + 8 * dx);
        glTranslatef(0, dx/500.0, -dy/500.0);
    }
    lastPos = event->pos();
}
void GLWidget::wheelEvent(QWheelEvent *event){
    //if(event->modifiers().testFlag(Qt::ControlModifier)){
        glTranslatef(-event->delta()/1000.0, 0, 0);
    //}
}

void GLWidget::normalizeAngle(int *angle){
    while (*angle < 0)
        *angle += 360 * 16;
    while (*angle > 360 * 16)
        *angle -= 360 * 16;
}


// ===========================
// ===========================


HUBOModel* GLWidget::newModel(){

    Joints joints(NO_OF_MODEL_JOINTS);
    Bones  bones(NBONE);
    Bones tempbone(21);
    std::vector<GLComplex*> objs(1);
    objs[0] = new GLComplex();

    // Lower body joints
    joints[RHY]     = new RevoluteZJoint("RHY");
    joints[RHR]     = new RevoluteXJoint("RHR");
    joints[RHP]     = new RevoluteYJoint("RHP");
    joints[RKN]     = new RevoluteYJoint("RKN");
    joints[RAP]     = new RevoluteYJoint("RAP");
    joints[RAR]     = new RevoluteXJoint("RAR");
    joints[LHY]     = new RevoluteZJoint("LHY");
    joints[LHR]     = new RevoluteXJoint("LHR");
    joints[LHP]     = new RevoluteYJoint("LHP");
    joints[LKN]     = new RevoluteYJoint("LKN");
    joints[LAP]     = new RevoluteYJoint("LAP");
    joints[LAR]     = new RevoluteXJoint("LAR");
    // Upper body joints
    joints[RSP]     = new RevoluteYJoint("RSP");
    joints[RSR]     = new RevoluteXJoint("RSR");
    joints[RSY]     = new RevoluteZJoint("RSY");
    joints[REB]     = new RevoluteYJoint("REB");
    joints[RWY]     = new RevoluteZJoint("RWY");
    joints[RWP]     = new RevoluteYJoint("RWP");
    joints[LSP]     = new RevoluteYJoint("LSP");
    joints[LSR]     = new RevoluteXJoint("LSR");
    joints[LSY]     = new RevoluteZJoint("LSY");
    joints[LEB]     = new RevoluteYJoint("LEB");
    joints[LWY]     = new RevoluteZJoint("LWY");
    joints[LWP]     = new RevoluteYJoint("LWP");
    // Waist, Fingers
    joints[WST]     = new RevoluteZJoint("WST");
    joints[RWY2]    = new RevoluteZJoint("RWY2");
    joints[RHAND]   = new RevoluteZJoint("RHAND");
    joints[LWY2]    = new RevoluteZJoint("LWY2");
    joints[LHAND]   = new RevoluteZJoint("LHAND");
    joints[RWH]     = new RevoluteZJoint("RWH");
    joints[LWH]     = new RevoluteZJoint("LWH");
    joints[RF_a] = new RevoluteYJoint("RF_a");
    joints[RF_b] = new RevoluteYJoint("RF_b");
    joints[RF_c] = new RevoluteYJoint("RF_c");
    joints[LF_a] = new RevoluteYJoint("LF_a");
    joints[LF_b] = new RevoluteYJoint("LF_b");
    joints[LF_c] = new RevoluteYJoint("LF_c");

    ((RevoluteJoint*)joints[RSR])->setOffset(-15.f*D2Rf);
    ((RevoluteJoint*)joints[LSR])->setOffset( 15.f*D2Rf);
    ((RevoluteJoint*)joints[REB])->setOffset(-20.f*D2Rf);
    ((RevoluteJoint*)joints[LEB])->setOffset(-20.f*D2Rf);


    GLObject *body_camera  = newBox(0.0, 0.0, 0.58, 0.05, 0.05, 0.05);
    GLObject *body_torso   = newStl(0.0, 0.0, 0.0, STLPath+"Body_TORSO_col.STL");
    GLObject *body_wst	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_WST_col.STL");

    //LARM
    GLObject *body_lsp	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LSP_col.STL");
    GLObject *body_lsr     = newStl(0.0, 0.0, 0.0, STLPath+"Body_LSR_col.STL");
    GLObject *body_lsy     = newStl(0.0, 0.0, 0.0, STLPath+"Body_LSY_col.STL");
    GLObject *body_leb     = newStl(0.0, 0.0, 0.0, STLPath+"Body_LEB_col.STL");
    GLObject *body_lwy     = newStl(0.0, 0.0, 0.0, STLPath+"Body_LWY_col.STL");
    GLObject *body_lwp     = newStl(0.0, 0.0, 0.0, STLPath+"Body_LWP_col.STL");
    GLObject *body_lwft    = newStl(0.0, 0.0, 0.0, STLPath+"Body_LF1_col.STL");
    //LFinger
    GLObject *body_LF_a    = newStl(0.0, 0.0, 0.0, STLPath+"Body_LF2_a_col.STL");
    GLObject *body_LF_b    = newStl(0.0, 0.0, 0.0, STLPath+"Body_LF2_b_col.STL");
    GLObject *body_LF_c    = newStl(0.0, 0.0, 0.0, STLPath+"Body_LF2_c_col.STL");

    //RARM
    GLObject *body_rsp	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RSP_col.STL");
    GLObject *body_rsr     = newStl(0.0, 0.0, 0.0, STLPath+"Body_RSR_col.STL");
    GLObject *body_rsy     = newStl(0.0, 0.0, 0.0, STLPath+"Body_RSY_col.STL");
    GLObject *body_reb     = newStl(0.0, 0.0, 0.0, STLPath+"Body_REB_col.STL");
    GLObject *body_rwy     = newStl(0.0, 0.0, 0.0, STLPath+"Body_RWY_col.STL");
    GLObject *body_rwp     = newStl(0.0, 0.0, 0.0, STLPath+"Body_RWP_col.STL");
    GLObject *body_rwft    = newStl(0.0, 0.0, 0.0, STLPath+"Body_RF1_col.STL");
    //RFinger
    GLObject *body_RF_a    = newStl(0.0, 0.0, 0.0, STLPath+"Body_RF2_a_col.STL");
    GLObject *body_RF_b    = newStl(0.0, 0.0, 0.0, STLPath+"Body_RF2_b_col.STL");
    GLObject *body_RF_c    = newStl(0.0, 0.0, 0.0, STLPath+"Body_RF2_c_col.STL");

    //LLEG
    GLObject *body_lhy	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LHY_col.STL");
    GLObject *body_lhr	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LHR_col.STL");
    GLObject *body_lhp	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LHP_col.STL");
    GLObject *body_lkn	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LKN_col.STL");
    GLObject *body_lap	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LAP_col.STL");
    GLObject *body_lar	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LAR_col.STL");
    GLObject *body_lwh	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_LWH_col.STL");

    //RLEG
    GLObject *body_rhy	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RHY_col.STL");
    GLObject *body_rhr	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RHR_col.STL");
    GLObject *body_rhp	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RHP_col.STL");
    GLObject *body_rkn	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RKN_col.STL");
    GLObject *body_rap	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RAP_col.STL");
    GLObject *body_rar	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RAR_col.STL");
    GLObject *body_rwh	   = newStl(0.0, 0.0, 0.0, STLPath+"Body_RWH_col.STL");


    bones[B_TORSO]  = new Bone(COMP_NAME[BTORSO],  isnl::pos(0.0, 0.0, 0.0), body_torso);
    bones[B_CAMERA] = new Bone(COMP_NAME[BCAMERA], isnl::pos(0.0, 0.0, 0.0), body_camera);
    bones[B_WST]    = new Bone(COMP_NAME[BWST],    isnl::pos(0.0, 0.0, 0.0), body_wst);

    bones[B_LSP]    = new Bone(COMP_NAME[BLSP],    isnl::pos(0.0, 0.0, 0.0), body_lsp);
    bones[B_LSR]    = new Bone(COMP_NAME[BLSR],    isnl::pos(0.0, 0.0, 0.0), body_lsr);
    bones[B_LSY]    = new Bone(COMP_NAME[BLSY],    isnl::pos(0.0, 0.0, 0.0), body_lsy);
    bones[B_LEB]    = new Bone(COMP_NAME[BLEB],    isnl::pos(0.0, 0.0, 0.0), body_leb);
    bones[B_LWY]    = new Bone(COMP_NAME[BLWY],    isnl::pos(0.0, 0.0, 0.0), body_lwy);
    bones[B_LWP]    = new Bone(COMP_NAME[BLWP],    isnl::pos(0.0, 0.0, 0.0), body_lwp);
    bones[B_LWFT]   = new Bone(COMP_NAME[BLWFT],   isnl::pos(0.0, 0.0, 0.0), body_lwft);

    bones[B_LF_a]   = new Bone(COMP_NAME[BLF_a],   isnl::pos(0.0, 0.0, 0.0), body_LF_a);
    bones[B_LF_b]   = new Bone(COMP_NAME[BLF_b],   isnl::pos(0.0, 0.0, 0.0), body_LF_b);
    bones[B_LF_c]   = new Bone(COMP_NAME[BLF_c],   isnl::pos(0.0, 0.0, 0.0), body_LF_c);

    bones[B_RSP]    = new Bone(COMP_NAME[BRSP],    isnl::pos(0.0, 0.0, 0.0), body_rsp);
    bones[B_RSR]    = new Bone(COMP_NAME[BRSR],    isnl::pos(0.0, 0.0, 0.0), body_rsr);
    bones[B_RSY]    = new Bone(COMP_NAME[BRSY],    isnl::pos(0.0, 0.0, 0.0), body_rsy);
    bones[B_REB]    = new Bone(COMP_NAME[BREB],    isnl::pos(0.0, 0.0, 0.0), body_reb);
    bones[B_RWY]    = new Bone(COMP_NAME[BRWY],    isnl::pos(0.0, 0.0, 0.0), body_rwy);
    bones[B_RWP]    = new Bone(COMP_NAME[BRWP],    isnl::pos(0.0, 0.0, 0.0), body_rwp);
    bones[B_RWFT]   = new Bone(COMP_NAME[BRWFT],   isnl::pos(0.0, 0.0, 0.0), body_rwft);

    bones[B_RF_a]   = new Bone(COMP_NAME[BRF_a],   isnl::pos(0.0, 0.0, 0.0), body_RF_a);
    bones[B_RF_b]   = new Bone(COMP_NAME[BRF_b],   isnl::pos(0.0, 0.0, 0.0), body_RF_b);
    bones[B_RF_c]   = new Bone(COMP_NAME[BRF_c],   isnl::pos(0.0, 0.0, 0.0), body_RF_c);

    bones[B_LHY]    = new Bone(COMP_NAME[BLHY],    isnl::pos(0.0, 0.0, 0.0), body_lhy);
    bones[B_LHR]    = new Bone(COMP_NAME[BLHR],    isnl::pos(0.0, 0.0, 0.0), body_lhr);
    bones[B_LHP]    = new Bone(COMP_NAME[BLHP],    isnl::pos(0.0, 0.0, 0.0), body_lhp);
    bones[B_LKN]    = new Bone(COMP_NAME[BLKN],    isnl::pos(0.0, 0.0, 0.0), body_lkn);
    bones[B_LAP]    = new Bone(COMP_NAME[BLAP],    isnl::pos(0.0, 0.0, 0.0), body_lap);
    bones[B_LAR]    = new Bone(COMP_NAME[BLAR],    isnl::pos(0.0, 0.0, 0.0), body_lar);
    bones[B_LWH]    = new Bone(COMP_NAME[BLWH],    isnl::pos(0.0, 0.0, 0.0), body_lwh);

    bones[B_RHY]    = new Bone(COMP_NAME[BRHY],    isnl::pos(0.0, 0.0, 0.0), body_rhy);
    bones[B_RHR]    = new Bone(COMP_NAME[BRHR],    isnl::pos(0.0, 0.0, 0.0), body_rhr);
    bones[B_RHP]    = new Bone(COMP_NAME[BRHP],    isnl::pos(0.0, 0.0, 0.0), body_rhp);
    bones[B_RKN]    = new Bone(COMP_NAME[BRKN],    isnl::pos(0.0, 0.0, 0.0), body_rkn);
    bones[B_RAP]    = new Bone(COMP_NAME[BRAP],    isnl::pos(0.0, 0.0, 0.0), body_rap);
    bones[B_RAR]    = new Bone(COMP_NAME[BRAR],    isnl::pos(0.0, 0.0, 0.0), body_rar);
    bones[B_RWH]    = new Bone(COMP_NAME[BRWH],    isnl::pos(0.0, 0.0, 0.0), body_rwh);


    //tree
    bones[B_WST]->setParent(NULL);
    *bones[B_WST] + joints[WST] + bones[B_TORSO] + bones[B_CAMERA];


    tempbone[4]  = new Bone("T_RS",       isnl::pos(0,-0.249,0.456));
    tempbone[5]  = new Bone("RS_REB",     isnl::pos(0.035,0,-0.35));
    tempbone[6]  = new Bone("REB_RWP",    isnl::pos(-0.035,0,-0.34));
    tempbone[15] = new Bone("RWFT_RF2_a", isnl::pos(0.0250,-0.02325,-0.14655));
    tempbone[16] = new Bone("RWFT_RF2_b", isnl::pos(0.0250, 0.02325,-0.14655));
    tempbone[17] = new Bone("RWFT_RF2_c", isnl::pos(-0.0250, 0.0000,-0.14655));
    *bones[B_TORSO]  +tempbone[4]+ joints[RSP]+ bones[B_RSP] + joints[RSR] + bones[B_RSR]  + joints[RSY] + bones[B_RSY]
            +tempbone[5]+ joints[REB] + bones[B_REB]
            +tempbone[6]+ joints[RWY] + bones[B_RWY] + joints[RWP] + bones[B_RWP]
            +joints[RWY2] + bones[B_RWFT] ;
    *bones[B_RWFT] + tempbone[15] + joints[RF_a] + bones[B_RF_a];
    *bones[B_RWFT] + tempbone[16] + joints[RF_b] + bones[B_RF_b];
    *bones[B_RWFT] + tempbone[17] + joints[RF_c] + bones[B_RF_c];


    tempbone[0]  = new Bone("T_LS",       isnl::pos(0,0.249,0.456));
    tempbone[1]  = new Bone("LS_LEB",     isnl::pos(0.035,0,-0.35));
    tempbone[2]  = new Bone("LEB_LWP",    isnl::pos(-0.035,0,-0.34));
    tempbone[18] = new Bone("LWFT_LF2_a", isnl::pos(0.0250,-0.02325,-0.14655));
    tempbone[19] = new Bone("LWFT_LF2_b", isnl::pos(0.0250, 0.02325,-0.14655));
    tempbone[20] = new Bone("LWFT_LF2_c", isnl::pos(-0.0250, 0.0000,-0.14655));
    *bones[B_TORSO]+ tempbone[0] + joints[LSP]+ bones[B_LSP] + joints[LSR] + bones[B_LSR]  + joints[LSY] + bones[B_LSY]
            +tempbone[1]+ joints[LEB] + bones[B_LEB]
            +tempbone[2]+ joints[LWY] + bones[B_LWY] + joints[LWP] + bones[B_LWP]
            +joints[LWY2] + bones[B_LWFT] ;
    *bones[B_LWFT] + tempbone[18] + joints[LF_a] + bones[B_LF_a];
    *bones[B_LWFT] + tempbone[19] + joints[LF_b] + bones[B_LF_b];
    *bones[B_LWFT] + tempbone[20] + joints[LF_c] + bones[B_LF_c];

    tempbone[7]  = new Bone("W_RH",   isnl::pos(0,-0.105,0));
    tempbone[8]  = new Bone("RH_RKN", isnl::pos(0.,0,-0.40));
    tempbone[9]  = new Bone("RKN_RA", isnl::pos(0,0,-0.38));
    tempbone[13] = new Bone("RA_RA2", isnl::pos(0,0,-0.025));
    *bones[B_WST] +tempbone[7]+ joints[RHY] + bones[B_RHY]+joints[RHR] + bones[B_RHR]+ joints[RHP] + bones[B_RHP]
            +tempbone[8]+ joints[RKN] + bones[B_RKN]
            +tempbone[9]+joints[RAP] + bones[B_RAP]+ tempbone[13]+joints[RAR] + bones[B_RAR];

    tempbone[10] = new Bone("W_LH",   isnl::pos(0,0.105,0));
    tempbone[11] = new Bone("LH_LKN", isnl::pos(0,0,-0.40));
    tempbone[12] = new Bone("LKN_LA", isnl::pos(0,0,-0.38));
    tempbone[14] = new Bone("LA_LA2", isnl::pos(0,0,-0.025));
    *bones[B_WST] + tempbone[10]+joints[LHY] + bones[B_LHY]+joints[LHR] + bones[B_LHR]+ joints[LHP] + bones[B_LHP]
            +tempbone[11]+ joints[LKN] + bones[B_LKN]
            +tempbone[12]+joints[LAP] + bones[B_LAP]+  tempbone[14]+joints[LAR] + bones[B_LAR];
    //tree

    return new HUBOModel(bones, joints, objs);
}

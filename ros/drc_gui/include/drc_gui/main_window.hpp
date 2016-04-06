/**
 * @file /include/drc_gui/main_window.hpp
 *
 * @brief Qt based gui for drc_gui.
 *
 * @date November 2010
 **/
#ifndef drc_gui_MAIN_WINDOW_H
#define drc_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace drc_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:
    void on_BTN_SEND_POS();
    void on_BTN_GOTO_GOAL();
    void on_BTN_GOTO_ORIGIN();
    void slot_new_object_tf();


private:
    Ui::MainWindowDesign ui;
    QNode qnode;
};

}  // namespace drc_gui

#endif // drc_gui_MAIN_WINDOW_H

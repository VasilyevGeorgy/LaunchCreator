#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>

#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPixmapItem>

#include "moveitem.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_browse_clicked();
    void on_browse1_clicked();

    void on_checkBox_stateChanged(int arg1);
    void on_buttonBox_2_accepted();

    void on_buttonBox_2_rejected();
    void on_checkBox_8_stateChanged(int arg1);

    void on_checkBox_7_stateChanged(int arg1);

    void on_checkBox_10_stateChanged(int arg1);

    void on_comboBox_currentIndexChanged(int index);

    void on_browse_2_clicked();

    void on_checkBox_9_stateChanged(int arg1);

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;

    QString world;
    QString lfname;
    QString lfolder_path;
    QString lfile_path;
    QString home_name;
    QString node;
    QString robot;

    QStringList final_launch;

    bool def_wrld_set;
    bool paused;
    bool use_sim_time;
    bool gui;
    bool respawn_gazebo;
    bool debug;
    bool default_params;
    bool in_gazebo_ros;
    bool empty_world;
    bool turtlebot_world;
    bool use_map;

    QString boolToString(bool a);
    void thru_empty(QString launch_name, bool is_default_params);
    void thru_empty(QString launch_name, bool is_default_params, QString node_name, QString package_name);
    void add_args(QStringList& list_name);
    void add_params(QStringList& list_name);
    void add_node(QStringList& list_name);
    void write_file(QString folder_path, QString file_path, QStringList& list_name);
    bool spawn_robot(QStringList& list_name);
    bool world_check();
    bool lfname_check();
    bool lfpath_check();
    bool nname_check();
    void choose_robot();
    bool robot_check();

    bool *is_pressed;

    MoveItem *robot_pos;

};

#endif // MAINWINDOW_H

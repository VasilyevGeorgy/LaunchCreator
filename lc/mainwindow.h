#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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


private:
    Ui::MainWindow *ui;

    QString world;
    QString lfname;
    QString lfolder_path;
    QString lfile_path;
    QString home_name;

    QStringList arg_list;
    QStringList final_launch;

    bool def_wrld_set;
    bool paused;
    bool use_sim_time;
    bool gui;
    bool headless;
    bool debug;

    bool arg_check;

    QString boolToString(bool a);
    bool lfname_check();
    bool world_check();
};

#endif // MAINWINDOW_H

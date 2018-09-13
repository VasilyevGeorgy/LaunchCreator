#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qfiledialog.h>
#include <qmessagebox.h>
#include <QProcess>
#include <QFile>
#include <QTextStream>
#include <QColor>
#include <QColorDialog>
#include <QDir>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //world = new QString;
    //cr_path = new QString;

    ui->checkBox->setChecked(true);
    ui->checkBox_2->setChecked(false);
    ui->checkBox_3->setChecked(true);
    ui->checkBox_4->setChecked(true);
    ui->checkBox_5->setChecked(false);
    ui->checkBox_6->setChecked(false);
    ui->groupBox->setEnabled(false);

    final_launch << "<launch>";

    arg_check = false;

    QStringList env = QProcess::systemEnvironment();
    home_name = env.at(env.indexOf(QRegExp("^HOME=.+")));
    home_name.remove(0,5);

}

MainWindow::~MainWindow()
{
    delete ui;
    //delete world;
}

QString MainWindow::boolToString(bool a){
    QString result;
    if (a) result = "true";
    else result = "false";
    return result;
}

bool MainWindow::lfname_check(){

    QString launch_name = ui->lineEdit_2->text();
    QRegExp lfn_check("[a-zA-Z0-9]*.launch");
    lfn_check.setPatternSyntax(QRegExp::Wildcard);
    if (launch_name.isEmpty() || !lfn_check.exactMatch(launch_name)){

        ui->lineEdit_2->setText("Enter name of launch-file!");
        //ui->lineEdit_2->setStyleSheet("color: red;");
        return false;
    }
    //lfname = launch_name;
    return true;
}

bool MainWindow::lfpath_check(){
    QString launch_path = ui->lineEdit_3->text();
    QRegExp lfp_check("[/][a-zA-Z0-9_/]*"); // [^/]*
    //lfp_check.setPatternSyntax(QRegExp::Wildcard);
    if (launch_path.isEmpty() || !lfp_check.exactMatch(launch_path)){

        ui->lineEdit_3->setText("Enter path of launch-file!");
        //ui->lineEdit_2->setStyleSheet("color: red;");
        return false;
    }
    //lfolder_path = launch_path;
    return true;

}

bool MainWindow::world_check(){

    QString world_name = ui->lineEdit->text();
    QRegExp wn_check("*.launch");
    wn_check.setPatternSyntax(QRegExp::Wildcard);

    if(world_name.isEmpty() || !wn_check.exactMatch(world_name)){
        ui->lineEdit->setText("Enter name of launch-file!");
        return false;
    }
    //world = world_name;
    return true;
}

void MainWindow::on_browse_clicked()
{
    QString world_name = QFileDialog::getOpenFileName(
                this,
                tr("Choose World"),
                "/opt/ros/kinetic/share/gazebo_ros/launch",
                "launch files (*.launch)"
                );
    //world = "/opt/ros/kinetic/share/gazebo_ros/launch" + world_name;
    ui->lineEdit->setText("");
    ui->lineEdit->setText("/opt/ros/kinetic/share/gazebo_ros/launch/" + world_name);
}

void MainWindow::on_browse1_clicked()
{

    if(!lfname_check())
        return;

        QString create_in = QFileDialog::getExistingDirectory(
                    this,
                    tr("Choose package"),
                    home_name,
                    QFileDialog::ShowDirsOnly
                    );
        //lfolder_path = create_in;
        //lfile_path = lfolder_path +"/"+lfname;
        ui->lineEdit_3->setText(create_in);

}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    ui->groupBox->setEnabled(true);

    if (ui->checkBox->isChecked()){
           ui->checkBox_2->setChecked(false);
           ui->checkBox_3->setChecked(true);
           ui->checkBox_4->setChecked(true);
           ui->checkBox_5->setChecked(false);
           ui->checkBox_6->setChecked(false);
           ui->groupBox->setEnabled(false);
    }
}

void MainWindow::on_buttonBox_2_accepted()
{
    if(!world_check() || !lfname_check() || !lfpath_check() || (ui->lineEdit_3->text().isEmpty())){
        return;
    }

    lfolder_path = ui->lineEdit_3->text() + "/launch";
    lfile_path = lfolder_path + "/" + ui->lineEdit_2->text();

    ui->lineEdit_4->setText(lfile_path);

    paused = ui->checkBox_2->isChecked();
    use_sim_time = ui->checkBox_3->isChecked();
    gui = ui->checkBox_4->isChecked();
    headless = ui->checkBox_5->isChecked();
    debug = ui->checkBox_6->isChecked();

    QString args =  "  < arg name=\"paused\" default=\"" + boolToString(paused) + "\" />";
    arg_list << args;
    args =  "  < arg name=\"use_sim_time\" default=\"" + boolToString(use_sim_time) + "\" />";
    arg_list << args;
    args =  "  < arg name=\"gui\" default=\"" + boolToString(gui) + "\" />";
    arg_list << args;
    args =  "  < arg name=\"headless\" default=\"" + boolToString(headless) + "\" />";
    arg_list << args;
    args =  "  < arg name=\"debug\" default=\"" + boolToString(debug) + "\" />";
    arg_list << args;

    if (!QDir(lfolder_path).exists())
        QDir().mkdir(lfolder_path);

    QFile file_out(lfile_path);
    if (file_out.open(QIODevice::ReadWrite))
    {
        QTextStream out(&file_out);
        for (QStringList::Iterator it = arg_list.begin();
                it != arg_list.end(); ++it)
            out << *it << "\n";

        //out << "something" << endl;
    }
    file_out.close();
    file_out.setPermissions(QFile::ReadOwner | QFile::ReadUser | QFile::ReadGroup);

}

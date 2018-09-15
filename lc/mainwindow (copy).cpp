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
#include <QList>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Launch Creator v0.1");

    ui->checkBox->setChecked(true);
    ui->checkBox_2->setChecked(false);
    ui->checkBox_3->setChecked(true);
    ui->checkBox_4->setChecked(true);
    ui->checkBox_5->setChecked(false);
    ui->checkBox_6->setChecked(false);
    ui->groupBox->setEnabled(false);

    default_params = true;
    in_gazebo_folder = false;

    QStringList env = QProcess::systemEnvironment();
    home_name = env.at(env.indexOf(QRegExp("^HOME=.+")));
    home_name.remove(0,5);

}

MainWindow::~MainWindow()
{
    delete ui;
}

QString MainWindow::boolToString(bool a){
    QString result;
    if (a) result = "true";
    else result = "false";
    return result;
}

void MainWindow::on_checkBox_8_stateChanged(int arg1)
{
    // Empty world checkbox
    if (ui->checkBox_8->isChecked()){
        ui->lineEdit->setEnabled(false);
        ui->browse->setEnabled(false);
        ui->lineEdit->setText("");
        empty_world = true;
    }
    else{
        ui->lineEdit->setEnabled(true);
        ui->browse->setEnabled(true);
        empty_world = false;
    }
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
    QRegExp wn_check1("*.world");
    wn_check1.setPatternSyntax(QRegExp::Wildcard);

    if(world_name.isEmpty() || (!wn_check.exactMatch(world_name) && !wn_check1.exactMatch(world_name))){
        ui->lineEdit->setText("Enter name of world or launch file!");
        //ui->lineEdit_4->setText(boolToString(wn_check1.exactMatch(world_name)));
        return false;
    }
    //world = world_name;
    return true;
}

bool MainWindow::nname_check(){
    QString node_name = ui->lineEdit_5->text();
    QRegExp nn_check("(\\w+)");
    //nn_check.setPatternSyntax(QRegExp::Wildcard);

    if(node_name.isEmpty() || !nn_check.exactMatch(node_name)){
        ui->lineEdit_5->setText("Enter node name!");
        ui->lineEdit_4->setText(boolToString(nn_check.exactMatch(node_name)));
        return false;
    }
    return true;
}

void MainWindow::on_browse_clicked()
{
    QString world_name = QFileDialog::getOpenFileName(
                this,
                tr("Choose World"),
                "/opt/ros/kinetic/share/gazebo_ros/launch",
                "launch files (*.launch);; world files (*.world)"
                );
    //world = "/opt/ros/kinetic/share/gazebo_ros/launch" + world_name;
    ui->lineEdit->setText("");
    ui->lineEdit->setText(world_name);

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
    default_params = false;

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
    //Check empty fields
    if(!world_check() || !lfname_check() || !lfpath_check()
            || (ui->lineEdit_3->text().isEmpty()) || !nname_check()){
        return;
    }
    //Empty && TurtleBot check
    turtlebot_world = ui->checkBox_9->isChecked();

    //Get file name and directory path
    world = ui->lineEdit->text();
    lfolder_path = ui->lineEdit_3->text() + "/launch";
    lfile_path = lfolder_path + "/" + ui->lineEdit_2->text();

    //Get args
    paused = ui->checkBox_2->isChecked();
    use_sim_time = ui->checkBox_3->isChecked();
    gui = ui->checkBox_4->isChecked();
    headless = ui->checkBox_5->isChecked();
    debug = ui->checkBox_6->isChecked();

    //.launch or .world check
    QRegExp type_check("/opt/ros/kinetic/share/gazebo_ros/launch/[a-zA-Z0-9_]*.launch");
    type_check.setPatternSyntax(QRegExp::Wildcard);
    in_gazebo_folder = type_check.exactMatch(world);

    //Get node
    node = ui->lineEdit_5->text();

    //Get package
    QStringList p_list = (ui->lineEdit_3->text()).split('/',QString::SkipEmptyParts);
    QString package = p_list.last();
    //ui->lineEdit_4->setText(package);

    //Compose launch-list
    final_launch << "<launch>";

    if (!default_params){
        final_launch << "  <!-- these are the arguments you can pass this launch file -->";
        final_launch << "  <arg name=\"paused\" default=\"" + boolToString(paused) + "\" />";
        final_launch << "  <arg name=\"use_sim_time\" default=\"" + boolToString(use_sim_time) + "\" />";
        final_launch << "  <arg name=\"gui\" default=\"" + boolToString(gui) + "\" />";
        final_launch << "  <arg name=\"headless\" default=\"" + boolToString(headless) + "\" />";
        final_launch << "  <arg name=\"debug\" default=\"" + boolToString(debug) + "\" />";
        final_launch << "";
    }

    /// (in_gazebo_folder && default_params) && (in_gazebo_folder && !default_params)
    if (in_gazebo_folder){
        //Get world name
        QStringList list = world.split('/',QString::SkipEmptyParts);
        QString world_name = list.at(list.indexOf(QRegExp("[a-zA-Z0-9_]*.launch")));
        //ui->lineEdit_4->setText(world_name);
        if (default_params){
            final_launch << "  <!-- Launch the world -->";
            final_launch << "  <include file=\"$(find gazebo_ros)/launch/" + world_name + "\" />";
        }
        else{
            final_launch << "  <!-- Launch the world with parameters -->";
            final_launch << "  <include file=\"$(find gazebo_ros)/launch/" + world_name + "\">";
            final_launch << "    <arg name=\"paused\" value=\"$(arg paused)\" />";
            final_launch << "    <arg name=\"use_sim_time\" value=\"$(arg use_sim_time)\" />";
            final_launch << "    <arg name=\"gui\" value=\"$(arg gui)\" />";
            final_launch << "    <arg name=\"headless\" value=\"$(arg headless)\" />";
            final_launch << "    <arg name=\"debug\" value=\"$(arg debug)\" />";
            final_launch << "  </include>";
        }
    }

    /// (!in_gazebo_folder && default_params) && (i!n_gazebo_folder && !default_params)
    if (!in_gazebo_folder){
        final_launch << "  <!-- Run own world thru empty_world.launch params changing -->";
        final_launch << "  <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">";
        if (default_params){
            final_launch << "    <arg name=\"world_name\" value=\"" + world + "\" />";
            final_launch << "  </include>";
        }
        else{
            final_launch << "    <arg name=\"paused\" value=\"$(arg paused)\" />";
            final_launch << "    <arg name=\"use_sim_time\" value=\"$(arg use_sim_time)\" />";
            final_launch << "    <arg name=\"gui\" value=\"$(arg gui)\" />";
            final_launch << "    <arg name=\"headless\" value=\"$(arg headless)\" />";
            final_launch << "    <arg name=\"debug\" value=\"$(arg debug)\" />";
            final_launch << "    <arg name=\"world_name\" value=\"" + lfile_path + "\" />";
            final_launch << "  </include>";
        }

    }

    final_launch << "";
    final_launch << "  <!-- Launch node -->";
    final_launch << "  <node name =\""+node+"\" pkg=\""+package+"\" type=\""+node+"\" output=\"screen\" />";

    //<node name="spawn_urdf"pkg="gazebo_ros" type="spawn_model" respawn="false" args="-param robot_description -urdf -model hobo" />


    final_launch << "</launch>";



    if (!QDir(lfolder_path).exists())
        QDir().mkdir(lfolder_path);

    QFile file_out(lfile_path);
    if (file_out.open(QIODevice::ReadWrite))
    {
        QTextStream out(&file_out);
        for (QStringList::Iterator it = final_launch.begin(); //final_launch
                it != final_launch.end(); ++it)
            out << *it << "\n";

        //out << "something" << endl;
    }
    file_out.close();
    file_out.setPermissions(QFile::ReadOwner | QFile::ReadUser | QFile::ReadGroup);

    if(QFile(lfile_path).exists())
        QMessageBox::information(0, "LaunchCreatorMessage", "Launch file is created!");

}

void MainWindow::on_buttonBox_2_rejected()
{
    QApplication::quit();
}



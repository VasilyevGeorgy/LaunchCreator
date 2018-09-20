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
#include <QPixmap>

#include "moveitem.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    this->resize(660,500);
    this->setFixedSize(660,500);


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
    in_gazebo_ros = false;
    empty_world = false;

    QStringList env = QProcess::systemEnvironment();
    home_name = env.at(env.indexOf(QRegExp("^HOME=.+")));
    home_name.remove(0,5);

    //is_pressed = new bool();

    ui->browse_2->setEnabled(false);
    ui->lineEdit_4->setEnabled(false);

    scene = new QGraphicsScene(-108,-108,216,216,this);
    ui->graphicsView->resize(220,220);
    QPixmap pim("/home/gera/seproject/launch_creator/lc/map_2.jpg");
    scene->setBackgroundBrush(pim.scaled(10.2,10.2,Qt::IgnoreAspectRatio,Qt::SmoothTransformation));

    ui->graphicsView->setScene(scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);
    ui->graphicsView->setCacheMode(QGraphicsView::CacheBackground);
    ui->graphicsView->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    //scene->addRect(scene->sceneRect());

    robot_pos = new MoveItem(this,ui->lineEdit_6, ui->lineEdit_8);
    robot_pos->setPos(0,0);
    scene->addItem(robot_pos);

    ui->graphicsView->setEnabled(false);

    ui->lineEdit_6->setEnabled(false);
    ui->lineEdit_8->setEnabled(false);

    //ui->lineEdit_4->setText(pos_x);

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
    QRegExp nn_check("(\\w*)");
    //nn_check.setPatternSyntax(QRegExp::Wildcard);

    if(!nn_check.exactMatch(node_name)){
        ui->lineEdit_5->setText("Enter node name properly!");
        //ui->lineEdit_4->setText(boolToString(nn_check.exactMatch(node_name)));
        return false;
    }
    return true;
}

void MainWindow::add_params(QStringList &list_name){
    list_name << "  <!-- these are the arguments you can pass this launch file -->";
    list_name << "  <arg name=\"paused\" default=\"" + boolToString(paused) + "\" />";
    list_name << "  <arg name=\"use_sim_time\" default=\"" + boolToString(use_sim_time) + "\" />";
    list_name << "  <arg name=\"gui\" default=\"" + boolToString(gui) + "\" />";
    list_name << "  <arg name=\"respawn_gazebo\" default=\"" + boolToString(respawn_gazebo) + "\" />";
    list_name << "  <arg name=\"debug\" default=\"" + boolToString(debug) + "\" />";
    list_name << "";
}

void MainWindow::add_args(QStringList& list_name){
    list_name << "    <arg name=\"paused\" value=\"$(arg paused)\" />";
    list_name << "    <arg name=\"use_sim_time\" value=\"$(arg use_sim_time)\" />";
    list_name << "    <arg name=\"gui\" value=\"$(arg gui)\" />";
    list_name << "    <arg name=\"respawn_gazebo\" value=\"$(arg respawn_gazebo)\" />";
    list_name << "    <arg name=\"debug\" value=\"$(arg debug)\" />";
}

void MainWindow::write_file(QString folder_path, QString file_path, QStringList& list_name){

    if (!QDir(folder_path).exists())
        QDir().mkdir(folder_path);

    QFile file_out(file_path);
    if (file_out.open(QIODevice::ReadWrite))
    {
        QTextStream out(&file_out);
        for (QStringList::Iterator it = list_name.begin();
                it != list_name.end(); ++it)
            out << *it << "\n";
    }
    file_out.close();
    file_out.setPermissions(QFile::ReadOwner | QFile::ReadUser | QFile::ReadGroup);
}


//Launch file generation thru empty_world.launch
void MainWindow::thru_empty(QString launch_name, bool is_default_params){

    QStringList launch;
    launch << "<launch>";
    QStringList& flref = launch;

    if (!is_default_params){
        add_params(flref);
        launch << "  <!-- Launch empty_world with parameters -->";
        launch << "  <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">";
    }
    else{
        launch << "  <!-- Launch empty_world with -->";
        launch << "  <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">";
    }
    if (in_gazebo_ros){
        //Get world name
        QStringList list = world.split('/',QString::SkipEmptyParts);
        QString world_name = list.at(list.indexOf(QRegExp("[a-zA-Z0-9_]*.launch")));
        world_name = world_name.left(world_name.lastIndexOf(QChar('.')));
        world_name = world_name.replace('_','.');

        launch << "    <arg name=\"world_name\" value=\"worlds/" + world_name + "\" />";
    }
    else{
        //Filename extension check
        QStringList list = world.split('.',QString::SkipEmptyParts);
        QString world_extension = list.at(1);
        if (world_extension == "launch"){
            ui->lineEdit->setText("Choose it properly!");
            return;
        }
        launch << "    <arg name=\"world_name\" value=\"" + world + "\" />";
    }
    if (default_params)
        launch << "  </include>";
    else{
        add_args(flref);
        launch << "  </include>";
    }
    launch << "</launch>";

    write_file(lfolder_path, lfolder_path + "/" + launch_name, launch);
}

//Overloading
void MainWindow::thru_empty(QString launch_name, bool is_default_params, QString node_name, QString package_name){

    QStringList launch;
    launch << "<launch>";
    QStringList& flref1 = launch;

    if (!is_default_params){
        add_params(flref1);
        launch << "  <!-- Launch empty_world with parameters -->";
        launch << "  <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">";
    }
    else{
        launch << "  <!-- Launch empty_world with -->";
        launch << "  <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">";
    }
    if (in_gazebo_ros){
        //Get world name
        QStringList list = world.split('/',QString::SkipEmptyParts);
        QString world_name = list.at(list.indexOf(QRegExp("[a-zA-Z0-9_]*.launch")));
        world_name = world_name.left(world_name.lastIndexOf(QChar('.')));
        world_name = world_name.replace('_','.');

        launch << "    <arg name=\"world_name\" value=\"worlds/" + world_name + "\" />";
    }
    else{
        //Filename extension check
        QStringList list = world.split('.',QString::SkipEmptyParts);
        QString world_extension = list.at(1);
        if (world_extension == "launch"){
            ui->lineEdit->setText("Choose it properly!");
            return;
        }
        launch << "    <arg name=\"world_name\" value=\"" + world + "\" />";
    }
    if (default_params)
        launch << "  </include>";
    else{
        add_args(flref1);
        launch << "  </include>";
    }
    launch << "";
    launch << "  <node name =\""+node_name+"\" pkg=\""+package_name+"\" type=\""+node_name+"\" output=\"screen\" />";
    launch << "</launch>";

    write_file(lfolder_path, lfolder_path + "/" + launch_name, launch);

    //QStringList name_list = launch_name.split('.',QString::SkipEmptyParts);
    //launch_name = name_list.at(0) + "_1.launch";

}

void MainWindow::spawn_robot(QStringList &list_name){

    list_name << "";
    QString desc_type = robot;
    //QString desc_type = robot.right(robot.lastIndexOf(QChar('.')));


    desc_type = (desc_type.split('.')).last();
    //node_name = node_name.left(node_name.indexOf(QChar(' ')));



    ui->lineEdit_5->setText(desc_type);
    if (desc_type == "urdf"){
        list_name << "  <!-- Spawn a robot into Gazebo -->";
        if (!ui->checkBox_10->isChecked())
            list_name << "    <node name=\"spawn_urdf\" pkg=\"gazebo_ros\" type=\"spawn_model\" args=\"-file " + robot + " -urdf -model robot\" />";
        else
            list_name << "    <node name=\"spawn_urdf\" pkg=\"gazebo_ros\" type=\"spawn_model\" args=\"-file "
                            + robot + " -urdf -x " + ui->lineEdit_6->text() + " -y " + ui->lineEdit_8->text() + "-model robot\" />";

    }
    if (desc_type == "xacro"){
        list_name << "<!-- Convert an xacro and put on parameter server -->";
        list_name << "<param name=\"robot_description\" command=\"$(find xacro)/xacro.py " + robot + "\" />";
        list_name << "";
        list_name << "<!-- Spawn a robot into Gazebo -->";
        if (!ui->checkBox_10->isChecked())
            list_name << "    <node name=\"spawn_urdf\" pkg=\"gazebo_ros\" type=\"spawn_model\" args=\"-param robot_description -urdf -model robot\" />";
        else
            list_name << "    <node name=\"spawn_urdf\" pkg=\"gazebo_ros\" type=\"spawn_model\" args=\"-param robot_description -urdf -x "
                            + ui->lineEdit_6->text() + " -y " + ui->lineEdit_8->text() + "-model robot\" />";

    }

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

    if(ui->checkBox_7->isChecked())
        ui->checkBox_7->setChecked(false);

    QString package_path;
    if (QDir(home_name + "/catkin_ws/src").exists())
        package_path = home_name + "/catkin_ws/src";
    else
        package_path = home_name;
    QString& ppath_ref = package_path;

    QString create_in = QFileDialog::getExistingDirectory(
                this,
                tr("Choose package"),
                ppath_ref,
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
    if( !((!world_check() && empty_world) || (world_check() && !empty_world))
            || !lfname_check() || !lfpath_check()
            || (ui->lineEdit_3->text().isEmpty()) || !nname_check()   || ((ui->comboBox->currentIndex()==3) && (!robot_check()))  ){

        if (empty_world)
            ui->lineEdit->setText("");
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
    respawn_gazebo = ui->checkBox_5->isChecked();
    debug = ui->checkBox_6->isChecked();

    //gazebo_ros check
    QRegExp type_check("/opt/ros/kinetic/share/gazebo_ros/launch/[a-zA-Z0-9_]*.launch");
    type_check.setPatternSyntax(QRegExp::Wildcard);
    in_gazebo_ros = type_check.exactMatch(world);

    //Get node
    node = ui->lineEdit_5->text();

    //Get package
    QStringList p_list = (ui->lineEdit_3->text()).split('/',QString::SkipEmptyParts);
    QString package = p_list.last();
    //ui->lineEdit_4->setText(package);

    //Get robot
    robot = ui->lineEdit_4->text();

    //Compose launch-list

    QStringList& flref = final_launch;

    /// (in_gazebo_ros && default_params) && (in_gazebo_ros && !default_params)
    if (empty_world && turtlebot_world){
        final_launch << "<launch>";
        if (default_params){
            final_launch << "  <!-- Launch turtlebot_world -->";
            final_launch << "  <include file=\"$(find turtlebot_gazebo)/launch/turtlebot_world.launch\" />";
        }
        else{
            add_params(flref);
            final_launch << "  <!-- Launch turtlebot_world with parameters -->";
            final_launch << "  <include file=\"$(find turtlebot_gazebo)/launch/turtlebot_world.launch\" >";
            add_args(flref);
            final_launch << "  </include>";
        }

        final_launch << "";
        if (!node.isEmpty()){
            final_launch << "  <!-- Launch node -->";
            final_launch << "  <node name =\""+node+"\" pkg=\""+package+"\" type=\""+node+"\" output=\"screen\" />";
        }

        final_launch << "</launch>";

        write_file(lfolder_path, lfile_path, flref);
    }

    if (empty_world && !turtlebot_world){
        final_launch << "<launch>";
        if (default_params){
            final_launch << "  <!-- launch empty_world -->";
            final_launch << "  <include file=\"$(find gazebo_ros)/launch/empty_world.launch\" />";
        }
        else{
            add_params(flref);
            final_launch << "  <!-- launch empty_world with parameters -->";
            final_launch << "  <include file=\"$(find gazebo_ros)/launch/empty_world.launch\" >";
            add_args(flref);
            final_launch << "  </include>";

         }

        if (true) // ui->comboBox->currentIndex()!=0
            spawn_robot(flref);

        final_launch << "";
        if (!node.isEmpty()){
            final_launch << "  <!-- Launch node -->";
            final_launch << "  <node name =\""+node+"\" pkg=\""+package+"\" type=\""+node+"\" output=\"screen\" />";
        }

        final_launch << "</launch>";

        write_file(lfolder_path, lfile_path, flref);
    }

    if (!empty_world && !turtlebot_world){

        if (node.isEmpty())
            thru_empty(ui->lineEdit_2->text(),default_params);
        else
            thru_empty(ui->lineEdit_2->text(), default_params, node, package);
    }

    if (!empty_world && turtlebot_world){

        QStringList name_list = ui->lineEdit_2->text().split('.',QString::SkipEmptyParts);

        //turtlebot launch file
        QStringList tb_launch;
        QStringList& tbref = tb_launch;
        tb_launch << "<launch>";
        tb_launch << "  <arg name=\"world_file\" default=\"$(env TURTLEBOT_GAZEBO_WORLD_FILE)\"/>";
        tb_launch << "  <arg name=\"base\" value=\"$(optenv TURTLEBOT_BASE kobuki)\"/>";
        tb_launch << "  <arg name=\"battery\" value=\"$(optenv TURTLEBOT_BATTERY /proc/acpi/battery/BAT0)\"/>";
        tb_launch << "  <arg name=\"stacks\" value=\"$(optenv TURTLEBOT_STACKS hexagons)\"/>";
        tb_launch << "  <arg name=\"3d_sensor\" value=\"$(optenv TURTLEBOT_3D_SENSOR kinect)\"/>";
        tb_launch << "";
        if(!default_params)
            add_params(tbref);
        ///tb_launch << "  <param name=\"robot_description\" command=\"$(find xacro)/xacro --inorder $(arg model)\" />";
        tb_launch << "  <include file=\"$(find gazebo_ros)/launch/empty_world.launch\">";
        tb_launch << "    <arg name=\"world_name\" value=\"$(arg world_file)\"/>";
        if(!default_params)
            add_args(tbref);
        tb_launch << "  </include>";
        tb_launch << "";
        if (!ui->checkBox_10->isChecked())
            tb_launch << "  <include file=\"$(find turtlebot_gazebo)/launch/includes/$(arg base).launch.xml\">";
        else
            tb_launch << "  <include file=\""+lfolder_path+"/"+name_list.at(0)+"_2.launch.xml"+"\">";
        tb_launch << "    <arg name=\"base\" value=\"$(arg base)\"/>";
        tb_launch << "    <arg name=\"stacks\" value=\"$(arg stacks)\"/>";
        tb_launch << "    <arg name=\"3d_sensor\" value=\"$(arg 3d_sensor)\"/>";
        tb_launch << "  </include>";
        tb_launch << "";
        tb_launch << "  <node pkg=\"robot_state_publisher\" type=\"robot_state_publisher\" name=\"robot_state_publisher\">";
        tb_launch << "    <param name=\"publish_frequency\" type=\"double\" value=\"30.0\" />";
        tb_launch << "  </node>";
        tb_launch << "";
        tb_launch << "  <!-- Fake laser -->";
        tb_launch << "  <node pkg=\"nodelet\" type=\"nodelet\" name=\"laserscan_nodelet_manager\" args=\"manager\"/>";
        tb_launch << "  <node pkg=\"nodelet\" type=\"nodelet\" name=\"depthimage_to_laserscan\"";
        tb_launch << "        args=\"load depthimage_to_laserscan/DepthImageToLaserScanNodelet laserscan_nodelet_manager\">";
        tb_launch << "    <param name=\"scan_height\" value=\"10\"/>";
        tb_launch << "    <param name=\"output_frame_id\" value=\"/camera_depth_frame\"/>";
        tb_launch << "    <param name=\"range_min\" value=\"0.45\"/>";
        tb_launch << "    <remap from=\"image\" to=\"/camera/depth/image_raw\"/>";
        tb_launch << "    <remap from=\"scan\" to=\"/scan\"/>";
        tb_launch << "  </node>";
        tb_launch << "</launch>";

        QString tb_name = lfolder_path+"/"+name_list.at(0)+"_1.launch";
        write_file(lfolder_path, tb_name, tbref);

        if (ui->checkBox_10->isChecked()){
            QStringList kobuki_list;

            kobuki_list << "<launch>";
            kobuki_list << "  <arg name=\"base\"/>";
            kobuki_list << "  <arg name=\"stacks\"/>";
            kobuki_list << "  <arg name=\"3d_sensor\"/>";
            kobuki_list << "";
            kobuki_list << "  <arg name=\"urdf_file\" default=\"$(find xacro)/xacro.py \'$(find turtlebot_description)/robots/$(arg base)_$(arg stacks)_$(arg 3d_sensor).urdf.xacro\'\" />";
            kobuki_list << "  <param name=\"robot_description\" command=\"$(arg urdf_file)\" />";
            kobuki_list << "";
            kobuki_list << "  <!-- Gazebo model spawner -->";
            kobuki_list << "  <node name=\"spawn_turtlebot_model\" pkg=\"gazebo_ros\" type=\"spawn_model\"";
            kobuki_list << "        args=\"-x " + ui->lineEdit_6->text() + " -y " + ui->lineEdit_8->text() + " -unpause -urdf -param robot_description -model mobile_base\"/>";
            kobuki_list << "";
            kobuki_list << "  <!-- Velocity muxer -->";
            kobuki_list << "  <node pkg=\"nodelet\" type=\"nodelet\" name=\"mobile_base_nodelet_manager\" args=\"manager\"/>";
            kobuki_list << "  <node pkg=\"nodelet\" type=\"nodelet\" name=\"cmd_vel_mux\"";
            kobuki_list << "        args=\"load yocs_cmd_vel_mux/CmdVelMuxNodelet mobile_base_nodelet_manager\">";
            kobuki_list << "    <param name=\"yaml_cfg_file\" value=\"$(find turtlebot_bringup)/param/mux.yaml\" />";
            kobuki_list << "    <remap from=\"cmd_vel_mux/output\" to=\"mobile_base/commands/velocity\"/>";
            kobuki_list << "  </node>";
            kobuki_list << "";
            kobuki_list << "  <!-- Bumper/cliff to pointcloud (not working, as it needs sensors/core messages) -->";
            kobuki_list << "  <include file=\"$(find turtlebot_bringup)/launch/includes/kobuki/bumper2pc.launch.xml\"/>";
            kobuki_list << "</launch>";

            QStringList &kl_ref = kobuki_list;
            write_file(lfolder_path, lfolder_path+"/"+name_list.at(0)+"_2.launch.xml", kl_ref);
        }

        //executable file
        final_launch << "<launch>";

        //if (!default_params){
        //    add_params(flref);
        //    final_launch << "  <!-- Launch turtlebot_world with parameters -->";
        //}
        //else{
        //    final_launch << "  <!-- Launch turtlebot_world -->";
        //}

        final_launch << "  <!-- Launch turtlebot_world -->";

        final_launch << "  <include file=\"" + tb_name + "\">";
        if (in_gazebo_ros){
            //Get world name
            QStringList list = world.split('/',QString::SkipEmptyParts);
            QString world_name = list.at(list.indexOf(QRegExp("[a-zA-Z0-9_]*.launch")));
            world_name = world_name.left(world_name.lastIndexOf(QChar('.')));
            world_name = world_name.replace('_','.');

            final_launch << "    <arg name=\"world_file\" value=\"worlds/" + world_name + "\" />";
        }
        else{
            //Filename extension check
            QStringList list = world.split('.',QString::SkipEmptyParts);
            QString world_extension = list.at(1);
            if (world_extension == "launch"){
                ui->lineEdit->setText("Choose it properly!");
                return;
            }
            final_launch << "    <arg name=\"world_name\" value=\"" + world + "\" />";
        }
        //if (default_params)
        //    final_launch << "  </include>";
        //else{
        //    add_args(flref);
        //    final_launch << "  </include>";
        //}

        final_launch << "  </include>";

        final_launch << "";
        if (!node.isEmpty()){
            final_launch << "  <!-- Launch node -->";
            final_launch << "  <node name =\""+node+"\" pkg=\""+package+"\" type=\""+node+"\" output=\"screen\" />";
        }
        final_launch << "</launch>";

        write_file(lfolder_path, lfile_path, flref);
    }

    //<node name="spawn_urdf"pkg="gazebo_ros" type="spawn_model" respawn="false" args="-param robot_description -urdf -model hobo" />

    if(QFile(lfile_path).exists()){
        QMessageBox::information(0, "LaunchCreatorMessage", "Launch file is created!");
        QApplication::quit();
    }
}

void MainWindow::on_buttonBox_2_rejected()
{
    QApplication::quit();
}


void MainWindow::on_checkBox_7_stateChanged(int arg1)
{
    ui->lineEdit_5->setText("");


    //Get node's name
    QString node_name;
    QRegExp nn_rx("^add_executable.*"); // "^add_executable.*"

    if(QFile(ui->lineEdit_3->text() + "/CMakeLists.txt").exists()){

        QFile cmake_file(ui->lineEdit_3->text() + "/CMakeLists.txt");

        if (cmake_file.open(QIODevice::ReadOnly | QIODevice::Text)){

            QTextStream stream(&cmake_file);
            while (!stream.atEnd()){
                QString cur_str = stream.readLine();
                if (nn_rx.exactMatch(cur_str)){
                    node_name.append(cur_str);
                    break;
                }
            }
        }

        cmake_file.close();

        if(ui->checkBox_7->isChecked()){
            node_name = (node_name.split('(')).at(1);
            node_name = node_name.left(node_name.indexOf(QChar(' ')));
            ui->lineEdit_5->setText(node_name);
        }
        else
            ui->lineEdit_5->setText("");
    }


}

void MainWindow::on_checkBox_10_stateChanged(int arg1)
{
    if (ui->checkBox_10->isChecked()){

        ui->graphicsView->setEnabled(true);
        ui->lineEdit_6->setEnabled(true);
        ui->lineEdit_8->setEnabled(true);
    }
    else{
        ui->graphicsView->setEnabled(false);
        ui->lineEdit_6->setText("");
        ui->lineEdit_8->setText("");
        ui->lineEdit_6->setEnabled(false);
        ui->lineEdit_8->setEnabled(false);

    }
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    ui->lineEdit_4->setText("");

    choose_robot();

    //if (index==3){
    //    ui->browse_2->setEnabled(true);
    //}
    //    //QMessageBox::information(0, "LaunchCreatorMessage", "test!");

    Q_UNUSED (index);

}

void MainWindow::choose_robot(){

    QString folder_path = "/opt/ros/kinetic/share/";

    if (ui->comboBox->currentIndex() == 0){
        ui->lineEdit_4->setEnabled(false);
    }
    if(ui->comboBox->currentIndex() == 1){
        ui->lineEdit_4->setEnabled(true);
        folder_path = folder_path + "pr2_description";
        if (!QDir(folder_path).exists())
            ui->lineEdit_4->setText("PR2 isn't found!");
        else
            ui->lineEdit_4->setText(folder_path + "/robots/pr2.urdf.xacro");
    }

    if(ui->comboBox->currentIndex() == 2){
        ui->lineEdit_4->setEnabled(true);
        folder_path = folder_path + "baxter_description";
        if (!QDir(folder_path).exists())
            ui->lineEdit_4->setText("Baxter isn't found!");
        else
            ui->lineEdit_4->setText(folder_path + "/urdf/baxter.urdf");
    }

    if(ui->comboBox->currentIndex() == 3){
        ui->lineEdit_4->setReadOnly(false);
        ui->browse_2->setEnabled(true);
        ui->lineEdit_4->setEnabled(true);
    }
    else{
        ui->browse_2->setEnabled(false);
        ui->lineEdit_4->setReadOnly(true);
    }

}

void MainWindow::on_browse_2_clicked()
{
    ui->checkBox_4->setText("");

    QString description = QFileDialog::getOpenFileName(
                this,
                tr("Choose robot description file"),
                "/opt/ros/kinetic/share/",
                "urdf files (*.urdf);; xacro files (*.xacro)"
                );

    ui->lineEdit_4->setText("");
    ui->lineEdit_4->setText(description);
}

bool MainWindow::robot_check(){
    QRegExp rx("[/][a-zA-Z0-9_/-]*.xacro");
    QRegExp rx1("[/][a-zA-Z0-9_/-]*.urdf");

    rx.setPatternSyntax(QRegExp::Wildcard);
    if (ui->lineEdit_4->text().isEmpty() || (!rx1.exactMatch(ui->lineEdit_4->text()) && !rx1.exactMatch(ui->lineEdit_4->text()) ) ){
        ui->lineEdit_4->setText("Choose it properly!");
        //ui->lineEdit_2->setStyleSheet("color: red;");
        return false;
    }
    return true;
}



#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    QIcon icoFwd(":/images/btnUp");
    QIcon icoBwd(":/images/btnDown");
    QIcon icoLeft(":/images/btnLeft");
    QIcon icoRight(":/images/btnRight");
    ui->btnFwd->setIcon(icoFwd);
    ui->btnBwd->setIcon(icoBwd);
    ui->btnLeft->setIcon(icoLeft);
    ui->btnRight->setIcon(icoRight);

    QObject::connect(ui->btnFwd, SIGNAL(pressed()), this, SLOT(btnFwdPressed()));
    QObject::connect(ui->btnFwd, SIGNAL(released()), this, SLOT(btnFwdReleased()));
    QObject::connect(ui->btnBwd, SIGNAL(pressed()), this, SLOT(btnBwdPressed()));
    QObject::connect(ui->btnBwd, SIGNAL(released()), this, SLOT(btnBwdReleased()));
    QObject::connect(ui->btnLeft, SIGNAL(pressed()), this, SLOT(btnLeftPressed()));
    QObject::connect(ui->btnLeft, SIGNAL(released()), this, SLOT(btnLeftReleased()));
    QObject::connect(ui->btnRight, SIGNAL(pressed()), this, SLOT(btnRightPressed()));
    QObject::connect(ui->btnRight, SIGNAL(released()), this, SLOT(btnRightReleased()));
    //QObject::connect(ui->btnCmdVel, SIGNAL(pressed()), this, SLOT(btnCmdVelPressed()));
    //QObject::connect(ui->btnCmdVel, SIGNAL(released()), this, SLOT(btnCmdVelReleased()));

    QObject::connect(ui->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
    //QObject::connect(ui->navBtnExecPath, SIGNAL(clicked()), this, SLOT(navBtnExecPath_pressed()));

    QObject::connect(ui->navTxtInflation  , SIGNAL(returnPressed()), this, SLOT(txtSmoothingReturnPressed()));
    //QObject::connect(ui->navTxtNearness   , SIGNAL(returnPressed()), this, SLOT(txtSmoothingReturnPressed()));
    //QObject::connect(ui->navTxtSmoothAlpha, SIGNAL(returnPressed()), this, SLOT(txtSmoothingReturnPressed()));
    //QObject::connect(ui->navTxtSmoothBeta , SIGNAL(returnPressed()), this, SLOT(txtSmoothingReturnPressed()));

    //QObject::connect(ui->torTxtPos, SIGNAL(valueChanged(double)), this, SLOT(torSbPosValueChanged(double)));
    QObject::connect(ui->laTxtAngles1, SIGNAL(valueChanged(double)), this, SLOT(laSbAnglesValueChanged(double)));
    QObject::connect(ui->laTxtAngles2, SIGNAL(valueChanged(double)), this, SLOT(laSbAnglesValueChanged(double)));
    QObject::connect(ui->laTxtAngles3, SIGNAL(valueChanged(double)), this, SLOT(laSbAnglesValueChanged(double)));
    QObject::connect(ui->laTxtAngles4, SIGNAL(valueChanged(double)), this, SLOT(laSbAnglesValueChanged(double)));
    QObject::connect(ui->laTxtAngles5, SIGNAL(valueChanged(double)), this, SLOT(laSbAnglesValueChanged(double)));
    QObject::connect(ui->laTxtAngles6, SIGNAL(valueChanged(double)), this, SLOT(laSbAnglesValueChanged(double)));
    QObject::connect(ui->laTxtAngles7, SIGNAL(valueChanged(double)), this, SLOT(laSbAnglesValueChanged(double)));
    QObject::connect(ui->raTxtAngles1, SIGNAL(valueChanged(double)), this, SLOT(raSbAnglesValueChanged(double)));
    QObject::connect(ui->raTxtAngles2, SIGNAL(valueChanged(double)), this, SLOT(raSbAnglesValueChanged(double)));
    QObject::connect(ui->raTxtAngles3, SIGNAL(valueChanged(double)), this, SLOT(raSbAnglesValueChanged(double)));
    QObject::connect(ui->raTxtAngles4, SIGNAL(valueChanged(double)), this, SLOT(raSbAnglesValueChanged(double)));
    QObject::connect(ui->raTxtAngles5, SIGNAL(valueChanged(double)), this, SLOT(raSbAnglesValueChanged(double)));
    QObject::connect(ui->raTxtAngles6, SIGNAL(valueChanged(double)), this, SLOT(raSbAnglesValueChanged(double)));
    QObject::connect(ui->raTxtAngles7, SIGNAL(valueChanged(double)), this, SLOT(raSbAnglesValueChanged(double)));
    QObject::connect(ui->laTxtAnglesG, SIGNAL(valueChanged(double)), this, SLOT(laSbGripperValueChanged(double)));
    QObject::connect(ui->raTxtAnglesG, SIGNAL(valueChanged(double)), this, SLOT(raSbGripperValueChanged(double)));
    QObject::connect(ui->laTxtPredefined, SIGNAL(returnPressed()), this, SLOT(laTxtPredefinedReturnPressed()));
    QObject::connect(ui->raTxtPredefined, SIGNAL(returnPressed()), this, SLOT(raTxtPredefinedReturnPressed()));
    QObject::connect(ui->hdTxtPan, SIGNAL(valueChanged(double)), this, SLOT(hdSbHeadValueChanged(double)));
    QObject::connect(ui->hdTxtTilt, SIGNAL(valueChanged(double)), this, SLOT(hdSbHeadValueChanged(double)));

    QObject::connect(ui->vsTxtFindObject, SIGNAL(returnPressed()), this, SLOT(vsTxtFindObjectReturnPressed()));

    QObject::connect(ui->spgTxtSay, SIGNAL(returnPressed()), this, SLOT(spgTxtSayReturnPressed()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode)
{
    this->qtRosNode = qtRosNode;

    //Connect signals from QtRosNode to MainWindow
    //For example, when ros finishes or when a rostopic is received
    QObject::connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    QObject::connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsReceived()));
    txtSmoothingReturnPressed();
    qtRosNode->publish_head_angles(ui->hdTxtPan->value(), ui->hdTxtTilt->value());
}

void MainWindow::setYamlParser(YamlParser* yamlParser)
{
    this->yamlParser = yamlParser;
}

//
//SLOTS FOR SIGNALS EMITTED IN THE MAINWINDOW
//
void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    //event->accept();
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    //pmCamera.loadFromData(qtRosNode->imgCompressed.data(), qtRosNode->imgCompressed.size(), "JPG");
    //giCamera->setPixmap(pmCamera);
    boost::algorithm::to_lower(qtRosNode->str_recognized_speech);
    this->ui->sprLblRecognized->setText(qtRosNode->str_recognized_speech.c_str());
}

void MainWindow::btnFwdPressed()
{
    qtRosNode->start_publishing_cmd_vel(0.3, 0, 0);
}

void MainWindow::btnFwdReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::btnBwdPressed()
{
    qtRosNode->start_publishing_cmd_vel(-0.3, 0, 0);
}

void MainWindow::btnBwdReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::btnLeftPressed()
{
    qtRosNode->start_publishing_cmd_vel(0, 0, 0.5);
}

void MainWindow::btnLeftReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::btnRightPressed()
{
    qtRosNode->start_publishing_cmd_vel(0, 0, -0.5);
}

void MainWindow::btnRightReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::btnCmdVelPressed()
{
    std::stringstream ssLinearX(ui->txtLinearX->text().toStdString());
    //std::stringstream ssLinearY(ui->txtLinearY->text().toStdString());
    std::stringstream ssAngular(ui->txtAngular->text().toStdString());
    float linearX = 0;
    float linearY = 0;
    float angular = 0;
    bool correct_format = true;
    if(!(ssLinearX >> linearX))
    {
        ui->txtLinearX->setText("Invalid format");
        correct_format = false;
    }
    /*if(!(ssLinearY >> linearY))
    {
        ui->txtLinearY->setText("Invalid format");
        correct_format = false;
        }*/
    if(!(ssAngular >> angular))
    {
        ui->txtAngular->setText("Invalid format");
        correct_format = false;
    }
    if(correct_format)
        //qtRosNode->start_publishing_cmd_vel(linearX, linearY, angular);
        qtRosNode->start_publishing_cmd_vel(linearX, 0, angular);
}

void MainWindow::btnCmdVelReleased()
{
    qtRosNode->stop_publishing_cmd_vel();
}

void MainWindow::navBtnCalcPath_pressed()
{
    float startX = 0;
    float startY = 0;
    float startA = 0;
    float goalX = 0;
    float goalY = 0;
    float goalA = 0;
    std::vector<std::string> parts;
    
    std::string str = this->ui->navTxtStartPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    
    if(str.compare("") == 0 || str.compare("robot") == 0) //take robot pose as start position
    {
        this->ui->navTxtStartPose->setText("Robot");
        qtRosNode->get_robot_pose(startX, startY, startA);
    }
    else if(parts.size() >= 2) //Given data correspond to numbers
    {
        std::stringstream ssStartX(parts[0]);
        std::stringstream ssStartY(parts[1]);
        if(!(ssStartX >> startX) || !(ssStartY >> startY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    else
    {
	this->ui->navTxtStartPose->setText("Invalid format");
	return;
    }

    str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtGoalPose->setText("Invalid format");
            return;
        }
    }
    else
    {
	this->ui->navTxtGoalPose->setText("Invalid format");
	return;
    }
    nav_msgs::Path path, smooth_path;
    if(!qtRosNode->call_a_star_search(startX, startY, goalX, goalY, path))
        return;
    qtRosNode->call_smooth_path(path, smooth_path);
}

void MainWindow::navBtnExecPath_pressed()
{
    float goalX = 0;
    float goalY = 0;
    float goalA = 0;
    std::vector<std::string> parts;
    std::string str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtGoalPose->setText("Invalid format");
            return;
        }
	if(parts.size() >= 3)
	{
	    std::stringstream ssGoalA(parts[2]);
	    if(!(ssGoalA >> goalA))
	    {
		this->ui->navTxtGoalPose->setText("Invalid Format");
		return;
	    }
	}
    }
    else
    {
	this->ui->navTxtGoalPose->setText("Invalid format");
	return;
    }
    qtRosNode->publish_goto_xya(goalX, goalY, goalA);
}

void MainWindow::txtSmoothingReturnPressed()
{
    std::stringstream ssInflation  (this->ui->navTxtInflation  ->text().toStdString());
    std::stringstream ssNearness   (this->ui->navTxtNearness   ->text().toStdString());
    std::stringstream ssSmoothAlpha(this->ui->navTxtSmoothAlpha->text().toStdString());
    std::stringstream ssSmoothBeta (this->ui->navTxtSmoothBeta ->text().toStdString());
    float smoothing_alpha;
    float smoothing_beta;
    float inflation_radius;
    float nearness_radius;
 
    if(!(ssInflation >> inflation_radius))
	this->ui->navTxtInflation->setText("Invalid");
    else
	qtRosNode->set_param_inflation_radius(inflation_radius);
    if(!(ssNearness >> nearness_radius))
	this->ui->navTxtNearness->setText("Invalid");
    else
	qtRosNode->set_param_cost_radius(nearness_radius);
    if(!(ssSmoothAlpha >> smoothing_alpha))
	this->ui->navTxtSmoothAlpha->setText("Invalid");
    else
	qtRosNode->set_param_smoothing_alpha(smoothing_alpha);
    if(!(ssSmoothBeta >> smoothing_beta))
	this->ui->navTxtSmoothBeta->setText("Invalid");
    else
	qtRosNode->set_param_smoothing_beta(smoothing_beta);
}

void MainWindow::torSbPosValueChanged(double d)
{
    //qtRosNode->publish_torso_position(ui->torTxtPos->value());
}

void MainWindow::laSbAnglesValueChanged(double d)
{
    qtRosNode->publish_la_goal_angles(ui->laTxtAngles1->value(), ui->laTxtAngles2->value(), ui->laTxtAngles3->value(),
                                      ui->laTxtAngles4->value(), ui->laTxtAngles5->value(), ui->laTxtAngles6->value(),
                                      ui->laTxtAngles7->value());
}

void MainWindow::raSbAnglesValueChanged(double d)
{
    qtRosNode->publish_ra_goal_angles(ui->raTxtAngles1->value(), ui->raTxtAngles2->value(), ui->raTxtAngles3->value(),
                                      ui->raTxtAngles4->value(), ui->raTxtAngles5->value(), ui->raTxtAngles6->value(),
                                      ui->raTxtAngles7->value());
}

void MainWindow::laTxtPredefinedReturnPressed()
{
    std::vector<std::string> parts;
    std::string str = this->ui->laTxtPredefined->text().toStdString();
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    std::vector<float> cartesian;
    cartesian.resize(parts.size());
    bool is_cartesian = true;
    for(size_t i=0; i<parts.size() && is_cartesian; i++)
    {
        std::stringstream ss(parts[i]);
        if(!(ss >> cartesian[i]))
            is_cartesian = false;
    }
    if(is_cartesian && cartesian.size() == 6)
    {
        std::vector<float> articular;
        if(!qtRosNode->call_la_inverse_kinematics(cartesian, articular))
            return;
        qtRosNode->publish_la_goal_angles(articular[0], articular[1], articular[2], articular[3], articular[4], articular[5], articular[6]);
        return;
    }
    
    YAML::Node n = yamlParser->nodeLaPredefined[ui->laTxtPredefined->text().toStdString()];
    if(!n)
    {
        ui->laTxtPredefined->setText("Invalid pose");
        return;
    }
    qtRosNode->publish_la_goal_angles(n[0].as<float>(), n[1].as<float>(), n[2].as<float>(), n[3].as<float>(),
                                      n[4].as<float>(), n[5].as<float>(), n[6].as<float>());
}

void MainWindow::raTxtPredefinedReturnPressed()
{
    std::vector<std::string> parts;
    std::string str = this->ui->raTxtPredefined->text().toStdString();
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    std::vector<float> cartesian;
    cartesian.resize(parts.size());
    bool is_cartesian = true;
    for(size_t i=0; i<parts.size() && is_cartesian; i++)
    {
        std::stringstream ss(parts[i]);
        if(!(ss >> cartesian[i]))
            is_cartesian = false;
    }
    if(is_cartesian && cartesian.size() == 6)
    {
        std::vector<float> articular;
        if(!qtRosNode->call_ra_inverse_kinematics(cartesian, articular))
            return;
        qtRosNode->publish_ra_goal_angles(articular[0], articular[1], articular[2], articular[3], articular[4], articular[5], articular[6]);
        return;
    }
    
    YAML::Node n = yamlParser->nodeRaPredefined[ui->raTxtPredefined->text().toStdString()];
    if(!n)
    {
        ui->raTxtPredefined->setText("Invalid pose");
        return;
    }
    qtRosNode->publish_ra_goal_angles(n[0].as<float>(), n[1].as<float>(), n[2].as<float>(), n[3].as<float>(),
                                      n[4].as<float>(), n[5].as<float>(), n[6].as<float>());
}

void MainWindow::laSbGripperValueChanged(double d)
{
    qtRosNode->publish_la_grip_angles(ui->laTxtAnglesG->value(), ui->laTxtAnglesG->value()); 
}

void MainWindow::raSbGripperValueChanged(double d)
{
    qtRosNode->publish_ra_grip_angles(ui->raTxtAnglesG->value(), ui->raTxtAnglesG->value()); 
}

void MainWindow::hdSbHeadValueChanged(double d)
{
    qtRosNode->publish_head_angles(ui->hdTxtPan->value(), ui->hdTxtTilt->value());
}

void MainWindow::vsTxtFindObjectReturnPressed()
{
    std::string obj = this->ui->vsTxtFindObject->text().toStdString();
    qtRosNode->call_find_object(obj);
}

void MainWindow::spgTxtSayReturnPressed()
{
    std::string txt = this->ui->spgTxtSay->text().toStdString();
    qtRosNode->publish_text_to_say(txt);
}

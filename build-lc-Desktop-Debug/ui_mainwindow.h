/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *label;
    QPushButton *browse;
    QLineEdit *lineEdit;
    QLabel *label1_3;
    QLineEdit *lineEdit_3;
    QPushButton *browse1;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QLabel *label_3;
    QGroupBox *groupBox;
    QCheckBox *checkBox_6;
    QCheckBox *checkBox_3;
    QCheckBox *checkBox_5;
    QCheckBox *checkBox_4;
    QCheckBox *checkBox_2;
    QLabel *label_5;
    QCheckBox *checkBox;
    QDialogButtonBox *buttonBox_2;
    QLabel *label_7;
    QLineEdit *lineEdit_5;
    QCheckBox *checkBox_7;
    QCheckBox *checkBox_8;
    QCheckBox *checkBox_9;
    QGraphicsView *graphicsView;
    QCheckBox *checkBox_10;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label_6;
    QLineEdit *lineEdit_6;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_8;
    QLineEdit *lineEdit_8;
    QComboBox *comboBox;
    QPushButton *browse_2;
    QLineEdit *lineEdit_4;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(660, 500);
        MainWindow->setMouseTracking(false);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 10, 131, 17));
        browse = new QPushButton(centralWidget);
        browse->setObjectName(QStringLiteral("browse"));
        browse->setGeometry(QRect(180, 60, 89, 25));
        lineEdit = new QLineEdit(centralWidget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setGeometry(QRect(10, 30, 261, 25));
        label1_3 = new QLabel(centralWidget);
        label1_3->setObjectName(QStringLiteral("label1_3"));
        label1_3->setGeometry(QRect(10, 290, 251, 17));
        lineEdit_3 = new QLineEdit(centralWidget);
        lineEdit_3->setObjectName(QStringLiteral("lineEdit_3"));
        lineEdit_3->setGeometry(QRect(10, 310, 261, 25));
        browse1 = new QPushButton(centralWidget);
        browse1->setObjectName(QStringLiteral("browse1"));
        browse1->setGeometry(QRect(180, 340, 89, 25));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 230, 131, 17));
        lineEdit_2 = new QLineEdit(centralWidget);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));
        lineEdit_2->setGeometry(QRect(10, 250, 261, 25));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setEnabled(true);
        label_3->setGeometry(QRect(350, 10, 131, 17));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 140, 261, 81));
        checkBox_6 = new QCheckBox(groupBox);
        checkBox_6->setObjectName(QStringLiteral("checkBox_6"));
        checkBox_6->setGeometry(QRect(180, 50, 92, 23));
        checkBox_3 = new QCheckBox(groupBox);
        checkBox_3->setObjectName(QStringLiteral("checkBox_3"));
        checkBox_3->setGeometry(QRect(90, 30, 121, 23));
        checkBox_5 = new QCheckBox(groupBox);
        checkBox_5->setObjectName(QStringLiteral("checkBox_5"));
        checkBox_5->setGeometry(QRect(90, 50, 92, 23));
        checkBox_4 = new QCheckBox(groupBox);
        checkBox_4->setObjectName(QStringLiteral("checkBox_4"));
        checkBox_4->setGeometry(QRect(10, 50, 51, 23));
        checkBox_2 = new QCheckBox(groupBox);
        checkBox_2->setObjectName(QStringLiteral("checkBox_2"));
        checkBox_2->setGeometry(QRect(10, 30, 81, 23));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(10, 110, 121, 17));
        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QStringLiteral("checkBox"));
        checkBox->setGeometry(QRect(10, 130, 111, 23));
        buttonBox_2 = new QDialogButtonBox(centralWidget);
        buttonBox_2->setObjectName(QStringLiteral("buttonBox_2"));
        buttonBox_2->setGeometry(QRect(460, 410, 166, 25));
        buttonBox_2->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(10, 360, 131, 17));
        lineEdit_5 = new QLineEdit(centralWidget);
        lineEdit_5->setObjectName(QStringLiteral("lineEdit_5"));
        lineEdit_5->setGeometry(QRect(10, 410, 261, 25));
        checkBox_7 = new QCheckBox(centralWidget);
        checkBox_7->setObjectName(QStringLiteral("checkBox_7"));
        checkBox_7->setEnabled(true);
        checkBox_7->setGeometry(QRect(10, 380, 291, 23));
        checkBox_8 = new QCheckBox(centralWidget);
        checkBox_8->setObjectName(QStringLiteral("checkBox_8"));
        checkBox_8->setGeometry(QRect(10, 60, 101, 23));
        checkBox_9 = new QCheckBox(centralWidget);
        checkBox_9->setObjectName(QStringLiteral("checkBox_9"));
        checkBox_9->setGeometry(QRect(80, 60, 92, 23));
        graphicsView = new QGraphicsView(centralWidget);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));
        graphicsView->setGeometry(QRect(340, 170, 220, 220));
        checkBox_10 = new QCheckBox(centralWidget);
        checkBox_10->setObjectName(QStringLiteral("checkBox_10"));
        checkBox_10->setGeometry(QRect(340, 140, 92, 23));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(580, 230, 77, 80));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_6 = new QLabel(verticalLayoutWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setEnabled(true);

        horizontalLayout->addWidget(label_6);

        lineEdit_6 = new QLineEdit(verticalLayoutWidget);
        lineEdit_6->setObjectName(QStringLiteral("lineEdit_6"));

        horizontalLayout->addWidget(lineEdit_6);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_8 = new QLabel(verticalLayoutWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setEnabled(true);

        horizontalLayout_2->addWidget(label_8);

        lineEdit_8 = new QLineEdit(verticalLayoutWidget);
        lineEdit_8->setObjectName(QStringLiteral("lineEdit_8"));

        horizontalLayout_2->addWidget(lineEdit_8);


        verticalLayout->addLayout(horizontalLayout_2);

        comboBox = new QComboBox(centralWidget);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        comboBox->setEnabled(true);
        comboBox->setGeometry(QRect(350, 30, 151, 25));
        browse_2 = new QPushButton(centralWidget);
        browse_2->setObjectName(QStringLiteral("browse_2"));
        browse_2->setEnabled(true);
        browse_2->setGeometry(QRect(520, 90, 89, 25));
        lineEdit_4 = new QLineEdit(centralWidget);
        lineEdit_4->setObjectName(QStringLiteral("lineEdit_4"));
        lineEdit_4->setEnabled(true);
        lineEdit_4->setGeometry(QRect(350, 60, 261, 25));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 660, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        label->setText(QApplication::translate("MainWindow", "1. Choose World:", 0));
        browse->setText(QApplication::translate("MainWindow", "Browse", 0));
        label1_3->setText(QApplication::translate("MainWindow", "4. Create in package:", 0));
        browse1->setText(QApplication::translate("MainWindow", "Browse", 0));
        label_2->setText(QApplication::translate("MainWindow", "3. Name (.launch):", 0));
        label_3->setText(QApplication::translate("MainWindow", "6. Choose Robot:", 0));
        groupBox->setTitle(QString());
        checkBox_6->setText(QApplication::translate("MainWindow", "debug", 0));
        checkBox_3->setText(QApplication::translate("MainWindow", "use_sim_time", 0));
        checkBox_5->setText(QApplication::translate("MainWindow", "respawn", 0));
        checkBox_4->setText(QApplication::translate("MainWindow", "gui", 0));
        checkBox_2->setText(QApplication::translate("MainWindow", "paused", 0));
        label_5->setText(QApplication::translate("MainWindow", "2. World settings:", 0));
        checkBox->setText(QApplication::translate("MainWindow", "Use default", 0));
        label_7->setText(QApplication::translate("MainWindow", "5. Node name:", 0));
        checkBox_7->setText(QApplication::translate("MainWindow", "Automaticly (first node in CMakeList)", 0));
        checkBox_8->setText(QApplication::translate("MainWindow", "Empty", 0));
        checkBox_9->setText(QApplication::translate("MainWindow", "TurtleBot", 0));
        checkBox_10->setText(QApplication::translate("MainWindow", "Use map", 0));
        label_6->setText(QApplication::translate("MainWindow", "x:", 0));
        label_8->setText(QApplication::translate("MainWindow", "y:", 0));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "None", 0)
         << QApplication::translate("MainWindow", "PR2 robot", 0)
         << QApplication::translate("MainWindow", "Baxter", 0)
         << QApplication::translate("MainWindow", "MyRobot", 0)
        );
        browse_2->setText(QApplication::translate("MainWindow", "Browse", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

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
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QSpacerItem *horizontalSpacer_2;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_21;
    QLineEdit *lineEdit_4;
    QSpacerItem *verticalSpacer_2;
    QLabel *label;
    QHBoxLayout *horizontalLayout_6;
    QCheckBox *checkBox_8;
    QCheckBox *checkBox_9;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_7;
    QCheckBox *checkBox_7;
    QLineEdit *lineEdit_5;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_19;
    QLineEdit *lineEdit;
    QPushButton *browse;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *checkBox_10;
    QHBoxLayout *horizontalLayout_17;
    QGraphicsView *graphicsView;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_8;
    QLineEdit *lineEdit_8;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_6;
    QLineEdit *lineEdit_6;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_5;
    QCheckBox *checkBox;
    QHBoxLayout *horizontalLayout_4;
    QCheckBox *checkBox_3;
    QCheckBox *checkBox_5;
    QHBoxLayout *horizontalLayout_5;
    QCheckBox *checkBox_2;
    QCheckBox *checkBox_4;
    QCheckBox *checkBox_6;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QLabel *label1_3;
    QHBoxLayout *horizontalLayout_18;
    QLineEdit *lineEdit_3;
    QPushButton *browse1;
    QHBoxLayout *horizontalLayout_20;
    QComboBox *comboBox;
    QPushButton *browse_2;
    QSpacerItem *horizontalSpacer;
    QDialogButtonBox *buttonBox_2;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(712, 538);
        MainWindow->setMouseTracking(false);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_2, 4, 10, 1, 1);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setEnabled(true);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);
        label_3->setTextFormat(Qt::AutoText);
        label_3->setScaledContents(false);

        gridLayout->addWidget(label_3, 2, 9, 1, 1);

        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setSpacing(6);
        horizontalLayout_21->setObjectName(QStringLiteral("horizontalLayout_21"));
        lineEdit_4 = new QLineEdit(centralWidget);
        lineEdit_4->setObjectName(QStringLiteral("lineEdit_4"));
        lineEdit_4->setEnabled(true);

        horizontalLayout_21->addWidget(lineEdit_4);


        gridLayout->addLayout(horizontalLayout_21, 4, 9, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_2, 26, 1, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy1);
        QFont font;
        font.setStrikeOut(false);
        font.setKerning(true);
        label->setFont(font);
        label->setTextFormat(Qt::AutoText);
        label->setScaledContents(true);

        gridLayout->addWidget(label, 1, 1, 3, 4);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        checkBox_8 = new QCheckBox(centralWidget);
        checkBox_8->setObjectName(QStringLiteral("checkBox_8"));

        horizontalLayout_6->addWidget(checkBox_8);

        checkBox_9 = new QCheckBox(centralWidget);
        checkBox_9->setObjectName(QStringLiteral("checkBox_9"));

        horizontalLayout_6->addWidget(checkBox_9);


        gridLayout->addLayout(horizontalLayout_6, 5, 1, 1, 1);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QStringLiteral("label_7"));

        verticalLayout_5->addWidget(label_7);

        checkBox_7 = new QCheckBox(centralWidget);
        checkBox_7->setObjectName(QStringLiteral("checkBox_7"));
        checkBox_7->setEnabled(true);

        verticalLayout_5->addWidget(checkBox_7);

        lineEdit_5 = new QLineEdit(centralWidget);
        lineEdit_5->setObjectName(QStringLiteral("lineEdit_5"));

        verticalLayout_5->addWidget(lineEdit_5);


        gridLayout->addLayout(verticalLayout_5, 25, 1, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 0, 1, 1, 1);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setObjectName(QStringLiteral("horizontalLayout_19"));
        lineEdit = new QLineEdit(centralWidget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));

        horizontalLayout_19->addWidget(lineEdit);

        browse = new QPushButton(centralWidget);
        browse->setObjectName(QStringLiteral("browse"));

        horizontalLayout_19->addWidget(browse);


        gridLayout->addLayout(horizontalLayout_19, 4, 1, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        checkBox_10 = new QCheckBox(centralWidget);
        checkBox_10->setObjectName(QStringLiteral("checkBox_10"));

        verticalLayout_2->addWidget(checkBox_10);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setSpacing(6);
        horizontalLayout_17->setObjectName(QStringLiteral("horizontalLayout_17"));
        graphicsView = new QGraphicsView(centralWidget);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));

        horizontalLayout_17->addWidget(graphicsView);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setEnabled(true);

        horizontalLayout_10->addWidget(label_8);

        lineEdit_8 = new QLineEdit(centralWidget);
        lineEdit_8->setObjectName(QStringLiteral("lineEdit_8"));

        horizontalLayout_10->addWidget(lineEdit_8);


        verticalLayout->addLayout(horizontalLayout_10);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setEnabled(true);

        horizontalLayout_7->addWidget(label_6);

        lineEdit_6 = new QLineEdit(centralWidget);
        lineEdit_6->setObjectName(QStringLiteral("lineEdit_6"));

        horizontalLayout_7->addWidget(lineEdit_6);


        verticalLayout->addLayout(horizontalLayout_7);


        horizontalLayout_17->addLayout(verticalLayout);


        verticalLayout_2->addLayout(horizontalLayout_17);


        gridLayout->addLayout(verticalLayout_2, 13, 9, 1, 1);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        verticalLayout_4->addWidget(label_5);

        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QStringLiteral("checkBox"));

        verticalLayout_4->addWidget(checkBox);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        checkBox_3 = new QCheckBox(centralWidget);
        checkBox_3->setObjectName(QStringLiteral("checkBox_3"));

        horizontalLayout_4->addWidget(checkBox_3);

        checkBox_5 = new QCheckBox(centralWidget);
        checkBox_5->setObjectName(QStringLiteral("checkBox_5"));

        horizontalLayout_4->addWidget(checkBox_5);


        verticalLayout_4->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        checkBox_2 = new QCheckBox(centralWidget);
        checkBox_2->setObjectName(QStringLiteral("checkBox_2"));

        horizontalLayout_5->addWidget(checkBox_2);

        checkBox_4 = new QCheckBox(centralWidget);
        checkBox_4->setObjectName(QStringLiteral("checkBox_4"));

        horizontalLayout_5->addWidget(checkBox_4);

        checkBox_6 = new QCheckBox(centralWidget);
        checkBox_6->setObjectName(QStringLiteral("checkBox_6"));

        horizontalLayout_5->addWidget(checkBox_6);


        verticalLayout_4->addLayout(horizontalLayout_5);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        verticalLayout_4->addWidget(label_2);

        lineEdit_2 = new QLineEdit(centralWidget);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));

        verticalLayout_4->addWidget(lineEdit_2);

        label1_3 = new QLabel(centralWidget);
        label1_3->setObjectName(QStringLiteral("label1_3"));

        verticalLayout_4->addWidget(label1_3);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setSpacing(6);
        horizontalLayout_18->setObjectName(QStringLiteral("horizontalLayout_18"));
        lineEdit_3 = new QLineEdit(centralWidget);
        lineEdit_3->setObjectName(QStringLiteral("lineEdit_3"));

        horizontalLayout_18->addWidget(lineEdit_3);

        browse1 = new QPushButton(centralWidget);
        browse1->setObjectName(QStringLiteral("browse1"));

        horizontalLayout_18->addWidget(browse1);


        verticalLayout_4->addLayout(horizontalLayout_18);


        gridLayout->addLayout(verticalLayout_4, 13, 1, 1, 1);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setSpacing(6);
        horizontalLayout_20->setObjectName(QStringLiteral("horizontalLayout_20"));
        comboBox = new QComboBox(centralWidget);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        comboBox->setEnabled(true);

        horizontalLayout_20->addWidget(comboBox);

        browse_2 = new QPushButton(centralWidget);
        browse_2->setObjectName(QStringLiteral("browse_2"));
        browse_2->setEnabled(true);

        horizontalLayout_20->addWidget(browse_2);


        gridLayout->addLayout(horizontalLayout_20, 5, 9, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 4, 0, 1, 1);

        buttonBox_2 = new QDialogButtonBox(centralWidget);
        buttonBox_2->setObjectName(QStringLiteral("buttonBox_2"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(buttonBox_2->sizePolicy().hasHeightForWidth());
        buttonBox_2->setSizePolicy(sizePolicy2);
        buttonBox_2->setSizeIncrement(QSize(0, 0));
        buttonBox_2->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout->addWidget(buttonBox_2, 26, 9, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 712, 22));
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
        label_3->setText(QApplication::translate("MainWindow", "6. Choose Robot:", 0));
        label->setText(QApplication::translate("MainWindow", "1. Choose World:", 0));
        checkBox_8->setText(QApplication::translate("MainWindow", "Empty", 0));
        checkBox_9->setText(QApplication::translate("MainWindow", "TurtleBot", 0));
        label_7->setText(QApplication::translate("MainWindow", "5. Node name:", 0));
        checkBox_7->setText(QApplication::translate("MainWindow", "Auto (first node in CMakeList)", 0));
        browse->setText(QApplication::translate("MainWindow", "Browse", 0));
        checkBox_10->setText(QApplication::translate("MainWindow", "Use map", 0));
        label_8->setText(QApplication::translate("MainWindow", "y:", 0));
        label_6->setText(QApplication::translate("MainWindow", "x:", 0));
        label_5->setText(QApplication::translate("MainWindow", "2. World settings:", 0));
        checkBox->setText(QApplication::translate("MainWindow", "Use default", 0));
        checkBox_3->setText(QApplication::translate("MainWindow", "use_sim_time", 0));
        checkBox_5->setText(QApplication::translate("MainWindow", "respawn", 0));
        checkBox_2->setText(QApplication::translate("MainWindow", "paused", 0));
        checkBox_4->setText(QApplication::translate("MainWindow", "gui", 0));
        checkBox_6->setText(QApplication::translate("MainWindow", "debug", 0));
        label_2->setText(QApplication::translate("MainWindow", "3. Name (.launch):", 0));
        label1_3->setText(QApplication::translate("MainWindow", "4. Create in package:", 0));
        browse1->setText(QApplication::translate("MainWindow", "Browse", 0));
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

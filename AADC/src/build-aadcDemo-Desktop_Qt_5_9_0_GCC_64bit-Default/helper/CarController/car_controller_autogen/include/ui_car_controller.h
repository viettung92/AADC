/********************************************************************************
** Form generated from reading UI file 'car_controller.ui'
**
** Created by: Qt User Interface Compiler version 5.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAR_CONTROLLER_H
#define UI_CAR_CONTROLLER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CarControllerUi
{
public:
    QGridLayout *gridLayout;
    QFrame *frame;
    QGridLayout *gridLayout_5;
    QLCDNumber *lcdNumber_steering;
    QLabel *label_throttle;
    QLabel *label_steering;
    QLCDNumber *lcdNumber_throttle;
    QTabWidget *tabWidget;
    QWidget *tab_2;
    QGridLayout *gridLayout_2;
    QPushButton *pushButton_turnright;
    QLabel *label_lights;
    QPushButton *pushButton_head;
    QPushButton *pushButton_brake;
    QPushButton *pushButton_turnleft;
    QPushButton *pushButton_reverse;
    QPushButton *pushButton_hazard;
    QWidget *tab;
    QGridLayout *gridLayout_3;
    QSpacerItem *horizontalSpacer;
    QSlider *verticalSlider_throttle;
    QSpacerItem *horizontalSpacer_2;
    QSlider *horizontalSlider_steering;
    QRadioButton *radioButton_enable_resetting_slider;
    QWidget *tab_3;
    QTabWidget *tabWidget_2;
    QWidget *tab_4;
    QPushButton *pushButton_action1;
    QPushButton *pushButton_action2;
    QPushButton *pushButton_action3;
    QPushButton *pushButton_action4;
    QPushButton *pushButton_action5;
    QPushButton *pushButton_action6;
    QPushButton *pushButton_action7;
    QPushButton *pushButton_action8;
    QPushButton *pushButton_action9;
    QWidget *tab_5;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_fp_action1;
    QPushButton *pushButton_fp_action2;
    QPushButton *pushButton_fp_action3;
    QPushButton *pushButton_fp_action4;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton_fp_action5;
    QPushButton *pushButton_fp_action6;
    QPushButton *pushButton_fp_action7;
    QPushButton *pushButton_fp_action8;
    QWidget *layoutWidget_2;
    QVBoxLayout *verticalLayout_3;
    QPushButton *pushButton_fp_action9;
    QWidget *tab_6;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout_4;
    QPushButton *pushButton_sa_1;
    QPushButton *pushButton_sa_2;
    QWidget *tab_10;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_4;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QWidget *tab_8;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_5;
    QPushButton *pushButton_acc_action1;
    QPushButton *pushButton_acc_action2;
    QPushButton *pushButton_acc_action3;
    QWidget *tab_7;
    QButtonGroup *buttonGroup_as_actions;
    QButtonGroup *buttonGroup_lf_actions;
    QButtonGroup *buttonGroup_mtp_actions;
    QButtonGroup *buttonGroup_ls_actions;
    QButtonGroup *buttonGroup_lights;
    QButtonGroup *buttonGroup;

    void setupUi(QWidget *CarControllerUi)
    {
        if (CarControllerUi->objectName().isEmpty())
            CarControllerUi->setObjectName(QStringLiteral("CarControllerUi"));
        CarControllerUi->resize(557, 427);
        gridLayout = new QGridLayout(CarControllerUi);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        frame = new QFrame(CarControllerUi);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setMinimumSize(QSize(0, 150));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        gridLayout_5 = new QGridLayout(frame);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        lcdNumber_steering = new QLCDNumber(frame);
        lcdNumber_steering->setObjectName(QStringLiteral("lcdNumber_steering"));

        gridLayout_5->addWidget(lcdNumber_steering, 1, 1, 1, 1);

        label_throttle = new QLabel(frame);
        label_throttle->setObjectName(QStringLiteral("label_throttle"));
        QFont font;
        font.setPointSize(16);
        font.setBold(true);
        font.setItalic(true);
        font.setWeight(75);
        label_throttle->setFont(font);
        label_throttle->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_throttle, 0, 0, 1, 1);

        label_steering = new QLabel(frame);
        label_steering->setObjectName(QStringLiteral("label_steering"));
        label_steering->setFont(font);
        label_steering->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_steering, 0, 1, 1, 1);

        lcdNumber_throttle = new QLCDNumber(frame);
        lcdNumber_throttle->setObjectName(QStringLiteral("lcdNumber_throttle"));

        gridLayout_5->addWidget(lcdNumber_throttle, 1, 0, 1, 1);


        gridLayout->addWidget(frame, 2, 0, 1, 1);

        tabWidget = new QTabWidget(CarControllerUi);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        gridLayout_2 = new QGridLayout(tab_2);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        pushButton_turnright = new QPushButton(tab_2);
        buttonGroup_lights = new QButtonGroup(CarControllerUi);
        buttonGroup_lights->setObjectName(QStringLiteral("buttonGroup_lights"));
        buttonGroup_lights->addButton(pushButton_turnright);
        pushButton_turnright->setObjectName(QStringLiteral("pushButton_turnright"));

        gridLayout_2->addWidget(pushButton_turnright, 6, 1, 1, 1);

        label_lights = new QLabel(tab_2);
        label_lights->setObjectName(QStringLiteral("label_lights"));
        QFont font1;
        font1.setPointSize(18);
        font1.setBold(true);
        font1.setItalic(true);
        font1.setWeight(75);
        label_lights->setFont(font1);
        label_lights->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_lights, 0, 0, 1, 2);

        pushButton_head = new QPushButton(tab_2);
        buttonGroup_lights->addButton(pushButton_head);
        pushButton_head->setObjectName(QStringLiteral("pushButton_head"));

        gridLayout_2->addWidget(pushButton_head, 1, 0, 1, 1);

        pushButton_brake = new QPushButton(tab_2);
        buttonGroup_lights->addButton(pushButton_brake);
        pushButton_brake->setObjectName(QStringLiteral("pushButton_brake"));

        gridLayout_2->addWidget(pushButton_brake, 1, 1, 1, 1);

        pushButton_turnleft = new QPushButton(tab_2);
        buttonGroup_lights->addButton(pushButton_turnleft);
        pushButton_turnleft->setObjectName(QStringLiteral("pushButton_turnleft"));

        gridLayout_2->addWidget(pushButton_turnleft, 6, 0, 1, 1);

        pushButton_reverse = new QPushButton(tab_2);
        buttonGroup_lights->addButton(pushButton_reverse);
        pushButton_reverse->setObjectName(QStringLiteral("pushButton_reverse"));

        gridLayout_2->addWidget(pushButton_reverse, 5, 0, 1, 1);

        pushButton_hazard = new QPushButton(tab_2);
        buttonGroup_lights->addButton(pushButton_hazard);
        pushButton_hazard->setObjectName(QStringLiteral("pushButton_hazard"));

        gridLayout_2->addWidget(pushButton_hazard, 5, 1, 1, 1);

        tabWidget->addTab(tab_2, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        gridLayout_3 = new QGridLayout(tab);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_3->addItem(horizontalSpacer, 0, 0, 1, 1);

        verticalSlider_throttle = new QSlider(tab);
        verticalSlider_throttle->setObjectName(QStringLiteral("verticalSlider_throttle"));
        verticalSlider_throttle->setMinimum(-100);
        verticalSlider_throttle->setMaximum(100);
        verticalSlider_throttle->setValue(0);
        verticalSlider_throttle->setOrientation(Qt::Vertical);

        gridLayout_3->addWidget(verticalSlider_throttle, 0, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_3->addItem(horizontalSpacer_2, 0, 2, 1, 1);

        horizontalSlider_steering = new QSlider(tab);
        horizontalSlider_steering->setObjectName(QStringLiteral("horizontalSlider_steering"));
        horizontalSlider_steering->setMinimum(-100);
        horizontalSlider_steering->setMaximum(100);
        horizontalSlider_steering->setValue(0);
        horizontalSlider_steering->setTracking(true);
        horizontalSlider_steering->setOrientation(Qt::Horizontal);
        horizontalSlider_steering->setInvertedAppearance(false);

        gridLayout_3->addWidget(horizontalSlider_steering, 1, 0, 1, 3);

        radioButton_enable_resetting_slider = new QRadioButton(tab);
        radioButton_enable_resetting_slider->setObjectName(QStringLiteral("radioButton_enable_resetting_slider"));

        gridLayout_3->addWidget(radioButton_enable_resetting_slider, 2, 0, 1, 1);

        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        tabWidget_2 = new QTabWidget(tab_3);
        tabWidget_2->setObjectName(QStringLiteral("tabWidget_2"));
        tabWidget_2->setGeometry(QRect(0, 0, 541, 221));
        tab_4 = new QWidget();
        tab_4->setObjectName(QStringLiteral("tab_4"));
        pushButton_action1 = new QPushButton(tab_4);
        buttonGroup_lf_actions = new QButtonGroup(CarControllerUi);
        buttonGroup_lf_actions->setObjectName(QStringLiteral("buttonGroup_lf_actions"));
        buttonGroup_lf_actions->addButton(pushButton_action1);
        pushButton_action1->setObjectName(QStringLiteral("pushButton_action1"));
        pushButton_action1->setGeometry(QRect(-1, 20, 171, 25));
        pushButton_action2 = new QPushButton(tab_4);
        buttonGroup_lf_actions->addButton(pushButton_action2);
        pushButton_action2->setObjectName(QStringLiteral("pushButton_action2"));
        pushButton_action2->setGeometry(QRect(0, 60, 171, 25));
        pushButton_action3 = new QPushButton(tab_4);
        buttonGroup_lf_actions->addButton(pushButton_action3);
        pushButton_action3->setObjectName(QStringLiteral("pushButton_action3"));
        pushButton_action3->setGeometry(QRect(0, 100, 171, 25));
        pushButton_action4 = new QPushButton(tab_4);
        buttonGroup_lf_actions->addButton(pushButton_action4);
        pushButton_action4->setObjectName(QStringLiteral("pushButton_action4"));
        pushButton_action4->setGeometry(QRect(0, 140, 171, 25));
        pushButton_action5 = new QPushButton(tab_4);
        buttonGroup_lf_actions->addButton(pushButton_action5);
        pushButton_action5->setObjectName(QStringLiteral("pushButton_action5"));
        pushButton_action5->setGeometry(QRect(180, 20, 171, 25));
        pushButton_action6 = new QPushButton(tab_4);
        buttonGroup_lf_actions->addButton(pushButton_action6);
        pushButton_action6->setObjectName(QStringLiteral("pushButton_action6"));
        pushButton_action6->setGeometry(QRect(180, 60, 171, 25));
        pushButton_action7 = new QPushButton(tab_4);
        buttonGroup_lf_actions->addButton(pushButton_action7);
        pushButton_action7->setObjectName(QStringLiteral("pushButton_action7"));
        pushButton_action7->setGeometry(QRect(180, 100, 171, 25));
        pushButton_action8 = new QPushButton(tab_4);
        buttonGroup_lf_actions->addButton(pushButton_action8);
        pushButton_action8->setObjectName(QStringLiteral("pushButton_action8"));
        pushButton_action8->setGeometry(QRect(180, 140, 171, 25));
        pushButton_action9 = new QPushButton(tab_4);
        buttonGroup_lf_actions->addButton(pushButton_action9);
        pushButton_action9->setObjectName(QStringLiteral("pushButton_action9"));
        pushButton_action9->setGeometry(QRect(360, 20, 171, 25));
        tabWidget_2->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QStringLiteral("tab_5"));
        layoutWidget = new QWidget(tab_5);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 10, 195, 171));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_fp_action1 = new QPushButton(layoutWidget);
        buttonGroup_mtp_actions = new QButtonGroup(CarControllerUi);
        buttonGroup_mtp_actions->setObjectName(QStringLiteral("buttonGroup_mtp_actions"));
        buttonGroup_mtp_actions->addButton(pushButton_fp_action1);
        pushButton_fp_action1->setObjectName(QStringLiteral("pushButton_fp_action1"));

        verticalLayout->addWidget(pushButton_fp_action1);

        pushButton_fp_action2 = new QPushButton(layoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action2);
        pushButton_fp_action2->setObjectName(QStringLiteral("pushButton_fp_action2"));

        verticalLayout->addWidget(pushButton_fp_action2);

        pushButton_fp_action3 = new QPushButton(layoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action3);
        pushButton_fp_action3->setObjectName(QStringLiteral("pushButton_fp_action3"));

        verticalLayout->addWidget(pushButton_fp_action3);

        pushButton_fp_action4 = new QPushButton(layoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action4);
        pushButton_fp_action4->setObjectName(QStringLiteral("pushButton_fp_action4"));

        verticalLayout->addWidget(pushButton_fp_action4);

        layoutWidget1 = new QWidget(tab_5);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(190, 10, 150, 171));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        pushButton_fp_action5 = new QPushButton(layoutWidget1);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action5);
        pushButton_fp_action5->setObjectName(QStringLiteral("pushButton_fp_action5"));

        verticalLayout_2->addWidget(pushButton_fp_action5);

        pushButton_fp_action6 = new QPushButton(layoutWidget1);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action6);
        pushButton_fp_action6->setObjectName(QStringLiteral("pushButton_fp_action6"));

        verticalLayout_2->addWidget(pushButton_fp_action6);

        pushButton_fp_action7 = new QPushButton(layoutWidget1);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action7);
        pushButton_fp_action7->setObjectName(QStringLiteral("pushButton_fp_action7"));

        verticalLayout_2->addWidget(pushButton_fp_action7);

        pushButton_fp_action8 = new QPushButton(layoutWidget1);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action8);
        pushButton_fp_action8->setObjectName(QStringLiteral("pushButton_fp_action8"));

        verticalLayout_2->addWidget(pushButton_fp_action8);

        layoutWidget_2 = new QWidget(tab_5);
        layoutWidget_2->setObjectName(QStringLiteral("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(350, 10, 123, 171));
        verticalLayout_3 = new QVBoxLayout(layoutWidget_2);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        pushButton_fp_action9 = new QPushButton(layoutWidget_2);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action9);
        pushButton_fp_action9->setObjectName(QStringLiteral("pushButton_fp_action9"));

        verticalLayout_3->addWidget(pushButton_fp_action9);

        tabWidget_2->addTab(tab_5, QString());
        tab_6 = new QWidget();
        tab_6->setObjectName(QStringLiteral("tab_6"));
        verticalLayoutWidget = new QWidget(tab_6);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 0, 160, 151));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        pushButton_sa_1 = new QPushButton(verticalLayoutWidget);
        buttonGroup_as_actions = new QButtonGroup(CarControllerUi);
        buttonGroup_as_actions->setObjectName(QStringLiteral("buttonGroup_as_actions"));
        buttonGroup_as_actions->addButton(pushButton_sa_1);
        pushButton_sa_1->setObjectName(QStringLiteral("pushButton_sa_1"));

        verticalLayout_4->addWidget(pushButton_sa_1);

        pushButton_sa_2 = new QPushButton(verticalLayoutWidget);
        buttonGroup_as_actions->addButton(pushButton_sa_2);
        pushButton_sa_2->setObjectName(QStringLiteral("pushButton_sa_2"));

        verticalLayout_4->addWidget(pushButton_sa_2);

        tabWidget_2->addTab(tab_6, QString());
        tab_10 = new QWidget();
        tab_10->setObjectName(QStringLiteral("tab_10"));
        gridLayoutWidget = new QWidget(tab_10);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(0, 0, 531, 191));
        gridLayout_4 = new QGridLayout(gridLayoutWidget);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        pushButton = new QPushButton(gridLayoutWidget);
        buttonGroup_ls_actions = new QButtonGroup(CarControllerUi);
        buttonGroup_ls_actions->setObjectName(QStringLiteral("buttonGroup_ls_actions"));
        buttonGroup_ls_actions->addButton(pushButton);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        gridLayout_4->addWidget(pushButton, 0, 0, 1, 1);

        pushButton_2 = new QPushButton(gridLayoutWidget);
        buttonGroup_ls_actions->addButton(pushButton_2);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        gridLayout_4->addWidget(pushButton_2, 0, 1, 1, 1);

        tabWidget_2->addTab(tab_10, QString());
        tab_8 = new QWidget();
        tab_8->setObjectName(QStringLiteral("tab_8"));
        verticalLayoutWidget_2 = new QWidget(tab_8);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(100, 30, 281, 161));
        verticalLayout_5 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        pushButton_acc_action1 = new QPushButton(verticalLayoutWidget_2);
        buttonGroup = new QButtonGroup(CarControllerUi);
        buttonGroup->setObjectName(QStringLiteral("buttonGroup"));
        buttonGroup->addButton(pushButton_acc_action1);
        pushButton_acc_action1->setObjectName(QStringLiteral("pushButton_acc_action1"));

        verticalLayout_5->addWidget(pushButton_acc_action1);

        pushButton_acc_action2 = new QPushButton(verticalLayoutWidget_2);
        buttonGroup->addButton(pushButton_acc_action2);
        pushButton_acc_action2->setObjectName(QStringLiteral("pushButton_acc_action2"));

        verticalLayout_5->addWidget(pushButton_acc_action2);

        pushButton_acc_action3 = new QPushButton(verticalLayoutWidget_2);
        buttonGroup->addButton(pushButton_acc_action3);
        pushButton_acc_action3->setObjectName(QStringLiteral("pushButton_acc_action3"));

        verticalLayout_5->addWidget(pushButton_acc_action3);

        tabWidget_2->addTab(tab_8, QString());
        tabWidget->addTab(tab_3, QString());
        tab_7 = new QWidget();
        tab_7->setObjectName(QStringLiteral("tab_7"));
        tabWidget->addTab(tab_7, QString());

        gridLayout->addWidget(tabWidget, 1, 0, 1, 1);


        retranslateUi(CarControllerUi);

        tabWidget->setCurrentIndex(2);
        tabWidget_2->setCurrentIndex(4);


        QMetaObject::connectSlotsByName(CarControllerUi);
    } // setupUi

    void retranslateUi(QWidget *CarControllerUi)
    {
        CarControllerUi->setWindowTitle(QApplication::translate("CarControllerUi", "Widget", Q_NULLPTR));
        label_throttle->setText(QApplication::translate("CarControllerUi", "Throttle", Q_NULLPTR));
        label_steering->setText(QApplication::translate("CarControllerUi", "Steering", Q_NULLPTR));
        pushButton_turnright->setText(QApplication::translate("CarControllerUi", "Toggle TurnRight", Q_NULLPTR));
        label_lights->setText(QApplication::translate("CarControllerUi", "Lights", Q_NULLPTR));
        pushButton_head->setText(QApplication::translate("CarControllerUi", "Toggle Head", Q_NULLPTR));
        pushButton_brake->setText(QApplication::translate("CarControllerUi", "Toggle Brake", Q_NULLPTR));
        pushButton_turnleft->setText(QApplication::translate("CarControllerUi", "Toggle TurnLeft", Q_NULLPTR));
        pushButton_reverse->setText(QApplication::translate("CarControllerUi", "Toggle Reverse", Q_NULLPTR));
        pushButton_hazard->setText(QApplication::translate("CarControllerUi", "Toggle Hazard", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("CarControllerUi", "Lights", Q_NULLPTR));
        radioButton_enable_resetting_slider->setText(QApplication::translate("CarControllerUi", "enable resetting slider", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("CarControllerUi", "Slider", Q_NULLPTR));
        pushButton_action1->setText(QApplication::translate("CarControllerUi", "AC_LC_HEAD_LIGHT_ON", Q_NULLPTR));
        pushButton_action2->setText(QApplication::translate("CarControllerUi", "AC_LC_HEAD_LIGHT_OFF", Q_NULLPTR));
        pushButton_action3->setText(QApplication::translate("CarControllerUi", "AC_LC_RERVERSE_LIGHT_ON", Q_NULLPTR));
        pushButton_action4->setText(QApplication::translate("CarControllerUi", "AC_LC_RERVERSE_LIGHT_OFF", Q_NULLPTR));
        pushButton_action5->setText(QApplication::translate("CarControllerUi", "AC_LC_TURN_LEFT", Q_NULLPTR));
        pushButton_action6->setText(QApplication::translate("CarControllerUi", "AC_LC_TURN_RIGHT", Q_NULLPTR));
        pushButton_action7->setText(QApplication::translate("CarControllerUi", "AC_LC_TURN_DISABLE", Q_NULLPTR));
        pushButton_action8->setText(QApplication::translate("CarControllerUi", "AC_LC_HAZARD_LIGHT_ON", Q_NULLPTR));
        pushButton_action9->setText(QApplication::translate("CarControllerUi", "AC_LC_HAZARD_LIGHT_OFF", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_4), QApplication::translate("CarControllerUi", "Lights", Q_NULLPTR));
        pushButton_fp_action1->setText(QApplication::translate("CarControllerUi", "AC_FP_GOTO_XY", Q_NULLPTR));
        pushButton_fp_action2->setText(QApplication::translate("CarControllerUi", "AC_FP_PARKING_TRANS", Q_NULLPTR));
        pushButton_fp_action3->setText(QApplication::translate("CarControllerUi", "AC_FP_GO_STRAIGHT", Q_NULLPTR));
        pushButton_fp_action4->setText(QApplication::translate("CarControllerUi", "AC_FP_GOTO_XY_NOSTOP", Q_NULLPTR));
        pushButton_fp_action5->setText(QApplication::translate("CarControllerUi", "AC_FP_LEFT_TURN", Q_NULLPTR));
        pushButton_fp_action6->setText(QApplication::translate("CarControllerUi", "AC_FP_STOP", Q_NULLPTR));
        pushButton_fp_action7->setText(QApplication::translate("CarControllerUi", "AC_FP_CONTINUE", Q_NULLPTR));
        pushButton_fp_action8->setText(QApplication::translate("CarControllerUi", "AC_FP_RIGHT_TURN", Q_NULLPTR));
        pushButton_fp_action9->setText(QApplication::translate("CarControllerUi", "AC_FP_START", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_5), QApplication::translate("CarControllerUi", "FollowPath", Q_NULLPTR));
        pushButton_sa_1->setText(QApplication::translate("CarControllerUi", "AC_SA_STOP_CAR", Q_NULLPTR));
        pushButton_sa_2->setText(QApplication::translate("CarControllerUi", "AC_SA_TEST", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_6), QApplication::translate("CarControllerUi", "ActionStop", Q_NULLPTR));
        pushButton->setText(QApplication::translate("CarControllerUi", "AC_LS_SLOW_RIGHTLANE", Q_NULLPTR));
        pushButton_2->setText(QApplication::translate("CarControllerUi", "AC_LS_NORMAL_RIGHTLANE", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_10), QApplication::translate("CarControllerUi", "LineSpecifier", Q_NULLPTR));
        pushButton_acc_action1->setText(QApplication::translate("CarControllerUi", "PushButton", Q_NULLPTR));
        pushButton_acc_action2->setText(QApplication::translate("CarControllerUi", "PushButton", Q_NULLPTR));
        pushButton_acc_action3->setText(QApplication::translate("CarControllerUi", "PushButton", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_8), QApplication::translate("CarControllerUi", "Seite", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("CarControllerUi", "Actions", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_7), QApplication::translate("CarControllerUi", "Seite", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class CarControllerUi: public Ui_CarControllerUi {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAR_CONTROLLER_H

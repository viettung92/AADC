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
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
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
    QFrame *frame;
    QGridLayout *gridLayout_5;
    QLCDNumber *lcdNumber_steering;
    QLabel *label_throttle;
    QLCDNumber *lcdNumber_throttle;
    QLabel *label_steering;
    QWidget *tab_3;
    QTabWidget *tabWidget_2;
    QWidget *tab_4;
    QWidget *horizontalLayoutWidget_3;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout_7;
    QPushButton *pushButton_lc_action9;
    QPushButton *pushButton_lc_action8;
    QPushButton *pushButton_lc_action5;
    QVBoxLayout *verticalLayout_11;
    QPushButton *pushButton_lc_action4;
    QPushButton *pushButton_lc_action3;
    QPushButton *pushButton_lc_action7;
    QVBoxLayout *verticalLayout_6;
    QPushButton *pushButton_lc_action2;
    QPushButton *pushButton_lc_action1;
    QPushButton *pushButton_lc_action6;
    QWidget *tab_5;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_9;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pushButton_arb_action1;
    QHBoxLayout *horizontalLayout_8;
    QComboBox *comboBox_gotoCom;
    QVBoxLayout *verticalLayout_10;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_11;
    QDoubleSpinBox *doubleSpinBox_posX;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_12;
    QDoubleSpinBox *doubleSpinBox_posY;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_14;
    QDoubleSpinBox *doubleSpinBox_cutOff;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_6;
    QDoubleSpinBox *doubleSpinBox_yaw;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_8;
    QDoubleSpinBox *doubleSpinBox_speed;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_fp_action1;
    QPushButton *pushButton_fp_action9;
    QPushButton *pushButton_fp_action4;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton_fp_action5;
    QPushButton *pushButton_fp_action6;
    QPushButton *pushButton_fp_action8;
    QVBoxLayout *verticalLayout_3;
    QPushButton *pushButton_fp_action3;
    QPushButton *pushButton_fp_action7;
    QPushButton *pushButton_fp_action2;
    QWidget *tab_6;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout_4;
    QPushButton *pushButton_sa_1;
    QPushButton *pushButton_sa_2;
    QWidget *tab_10;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_5;
    QPushButton *pushButton_14;
    QPushButton *pushButton_15;
    QPushButton *pushButton_16;
    QWidget *tab_8;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QWidget *verticalLayoutWidget_6;
    QVBoxLayout *verticalLayout_12;
    QPushButton *buttonGroup_usacc_action1;
    QPushButton *buttonGroup_usacc_action2;
    QPushButton *buttonGroup_usacc_action3;
    QPushButton *buttonGroup_usacc_action4;
    QPushButton *buttonGroup_usacc_action5;
    QPushButton *buttonGroup_usacc_action6;
    QPushButton *buttonGroup_usacc_action7;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_4;
    QPushButton *buttonGroup_usacc_action8;
    QPushButton *buttonGroup_usacc_action10;
    QPushButton *buttonGroup_usacc_action11;
    QPushButton *buttonGroup_usacc_action9;
    QPushButton *buttonGroup_usacc_action12;
    QPushButton *buttonGroup_usacc_action13;
    QFrame *frame_2;
    QGridLayout *gridLayout_6;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *pushButton_arb_action2;
    QLabel *label;
    QLineEdit *lineEdit_arbCom;
    QButtonGroup *buttonGroup_lights;
    QButtonGroup *buttonGroup_ld_actions;
    QButtonGroup *buttonGroup_lf_actions;
    QButtonGroup *buttonGroup_usacc_actions;
    QButtonGroup *buttonGroup_mtp_actions;
    QButtonGroup *buttonGroup_as_actions;
    QButtonGroup *buttonGroup_arb_actions;

    void setupUi(QWidget *CarControllerUi)
    {
        if (CarControllerUi->objectName().isEmpty())
            CarControllerUi->setObjectName(QStringLiteral("CarControllerUi"));
        CarControllerUi->resize(571, 568);
        gridLayout = new QGridLayout(CarControllerUi);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
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
        QFont font;
        font.setPointSize(18);
        font.setBold(true);
        font.setItalic(true);
        font.setWeight(75);
        label_lights->setFont(font);
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

        frame = new QFrame(tab);
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
        QFont font1;
        font1.setPointSize(16);
        font1.setBold(true);
        font1.setItalic(true);
        font1.setWeight(75);
        label_throttle->setFont(font1);
        label_throttle->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_throttle, 0, 0, 1, 1);

        lcdNumber_throttle = new QLCDNumber(frame);
        lcdNumber_throttle->setObjectName(QStringLiteral("lcdNumber_throttle"));

        gridLayout_5->addWidget(lcdNumber_throttle, 1, 0, 1, 1);

        label_steering = new QLabel(frame);
        label_steering->setObjectName(QStringLiteral("label_steering"));
        label_steering->setFont(font1);
        label_steering->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_steering, 0, 1, 1, 1);


        gridLayout_3->addWidget(frame, 3, 0, 1, 3);

        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        tabWidget_2 = new QTabWidget(tab_3);
        tabWidget_2->setObjectName(QStringLiteral("tabWidget_2"));
        tabWidget_2->setGeometry(QRect(0, 80, 540, 441));
        tabWidget_2->setMaximumSize(QSize(540, 500));
        tab_4 = new QWidget();
        tab_4->setObjectName(QStringLiteral("tab_4"));
        horizontalLayoutWidget_3 = new QWidget(tab_4);
        horizontalLayoutWidget_3->setObjectName(QStringLiteral("horizontalLayoutWidget_3"));
        horizontalLayoutWidget_3->setGeometry(QRect(30, 30, 498, 151));
        horizontalLayout_5 = new QHBoxLayout(horizontalLayoutWidget_3);
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        pushButton_lc_action9 = new QPushButton(horizontalLayoutWidget_3);
        buttonGroup_lf_actions = new QButtonGroup(CarControllerUi);
        buttonGroup_lf_actions->setObjectName(QStringLiteral("buttonGroup_lf_actions"));
        buttonGroup_lf_actions->addButton(pushButton_lc_action9);
        pushButton_lc_action9->setObjectName(QStringLiteral("pushButton_lc_action9"));

        verticalLayout_7->addWidget(pushButton_lc_action9);

        pushButton_lc_action8 = new QPushButton(horizontalLayoutWidget_3);
        buttonGroup_lf_actions->addButton(pushButton_lc_action8);
        pushButton_lc_action8->setObjectName(QStringLiteral("pushButton_lc_action8"));

        verticalLayout_7->addWidget(pushButton_lc_action8);

        pushButton_lc_action5 = new QPushButton(horizontalLayoutWidget_3);
        buttonGroup_lf_actions->addButton(pushButton_lc_action5);
        pushButton_lc_action5->setObjectName(QStringLiteral("pushButton_lc_action5"));

        verticalLayout_7->addWidget(pushButton_lc_action5);


        horizontalLayout_5->addLayout(verticalLayout_7);

        verticalLayout_11 = new QVBoxLayout();
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setObjectName(QStringLiteral("verticalLayout_11"));
        pushButton_lc_action4 = new QPushButton(horizontalLayoutWidget_3);
        buttonGroup_lf_actions->addButton(pushButton_lc_action4);
        pushButton_lc_action4->setObjectName(QStringLiteral("pushButton_lc_action4"));

        verticalLayout_11->addWidget(pushButton_lc_action4);

        pushButton_lc_action3 = new QPushButton(horizontalLayoutWidget_3);
        buttonGroup_lf_actions->addButton(pushButton_lc_action3);
        pushButton_lc_action3->setObjectName(QStringLiteral("pushButton_lc_action3"));

        verticalLayout_11->addWidget(pushButton_lc_action3);

        pushButton_lc_action7 = new QPushButton(horizontalLayoutWidget_3);
        buttonGroup_lf_actions->addButton(pushButton_lc_action7);
        pushButton_lc_action7->setObjectName(QStringLiteral("pushButton_lc_action7"));

        verticalLayout_11->addWidget(pushButton_lc_action7);


        horizontalLayout_5->addLayout(verticalLayout_11);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        pushButton_lc_action2 = new QPushButton(horizontalLayoutWidget_3);
        buttonGroup_lf_actions->addButton(pushButton_lc_action2);
        pushButton_lc_action2->setObjectName(QStringLiteral("pushButton_lc_action2"));

        verticalLayout_6->addWidget(pushButton_lc_action2);

        pushButton_lc_action1 = new QPushButton(horizontalLayoutWidget_3);
        buttonGroup_lf_actions->addButton(pushButton_lc_action1);
        pushButton_lc_action1->setObjectName(QStringLiteral("pushButton_lc_action1"));

        verticalLayout_6->addWidget(pushButton_lc_action1);

        pushButton_lc_action6 = new QPushButton(horizontalLayoutWidget_3);
        buttonGroup_lf_actions->addButton(pushButton_lc_action6);
        pushButton_lc_action6->setObjectName(QStringLiteral("pushButton_lc_action6"));

        verticalLayout_6->addWidget(pushButton_lc_action6);


        horizontalLayout_5->addLayout(verticalLayout_6);

        tabWidget_2->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QStringLiteral("tab_5"));
        horizontalLayoutWidget_2 = new QWidget(tab_5);
        horizontalLayoutWidget_2->setObjectName(QStringLiteral("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(0, 210, 533, 111));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_3->setSpacing(25);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        pushButton_arb_action1 = new QPushButton(horizontalLayoutWidget_2);
        buttonGroup_arb_actions = new QButtonGroup(CarControllerUi);
        buttonGroup_arb_actions->setObjectName(QStringLiteral("buttonGroup_arb_actions"));
        buttonGroup_arb_actions->addButton(pushButton_arb_action1);
        pushButton_arb_action1->setObjectName(QStringLiteral("pushButton_arb_action1"));

        horizontalLayout_7->addWidget(pushButton_arb_action1);


        verticalLayout_9->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        comboBox_gotoCom = new QComboBox(horizontalLayoutWidget_2);
        comboBox_gotoCom->setObjectName(QStringLiteral("comboBox_gotoCom"));

        horizontalLayout_8->addWidget(comboBox_gotoCom);


        verticalLayout_9->addLayout(horizontalLayout_8);


        horizontalLayout_3->addLayout(verticalLayout_9);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        label_11 = new QLabel(horizontalLayoutWidget_2);
        label_11->setObjectName(QStringLiteral("label_11"));

        horizontalLayout_9->addWidget(label_11);

        doubleSpinBox_posX = new QDoubleSpinBox(horizontalLayoutWidget_2);
        doubleSpinBox_posX->setObjectName(QStringLiteral("doubleSpinBox_posX"));

        horizontalLayout_9->addWidget(doubleSpinBox_posX);


        verticalLayout_10->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        label_12 = new QLabel(horizontalLayoutWidget_2);
        label_12->setObjectName(QStringLiteral("label_12"));

        horizontalLayout_10->addWidget(label_12);

        doubleSpinBox_posY = new QDoubleSpinBox(horizontalLayoutWidget_2);
        doubleSpinBox_posY->setObjectName(QStringLiteral("doubleSpinBox_posY"));

        horizontalLayout_10->addWidget(doubleSpinBox_posY);


        verticalLayout_10->addLayout(horizontalLayout_10);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        label_14 = new QLabel(horizontalLayoutWidget_2);
        label_14->setObjectName(QStringLiteral("label_14"));

        horizontalLayout_12->addWidget(label_14);

        doubleSpinBox_cutOff = new QDoubleSpinBox(horizontalLayoutWidget_2);
        doubleSpinBox_cutOff->setObjectName(QStringLiteral("doubleSpinBox_cutOff"));

        horizontalLayout_12->addWidget(doubleSpinBox_cutOff);


        verticalLayout_10->addLayout(horizontalLayout_12);


        horizontalLayout_3->addLayout(verticalLayout_10);

        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        label_6 = new QLabel(horizontalLayoutWidget_2);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_4->addWidget(label_6);

        doubleSpinBox_yaw = new QDoubleSpinBox(horizontalLayoutWidget_2);
        doubleSpinBox_yaw->setObjectName(QStringLiteral("doubleSpinBox_yaw"));

        horizontalLayout_4->addWidget(doubleSpinBox_yaw);


        verticalLayout_8->addLayout(horizontalLayout_4);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        label_8 = new QLabel(horizontalLayoutWidget_2);
        label_8->setObjectName(QStringLiteral("label_8"));

        horizontalLayout_6->addWidget(label_8);

        doubleSpinBox_speed = new QDoubleSpinBox(horizontalLayoutWidget_2);
        doubleSpinBox_speed->setObjectName(QStringLiteral("doubleSpinBox_speed"));

        horizontalLayout_6->addWidget(doubleSpinBox_speed);


        verticalLayout_8->addLayout(horizontalLayout_6);


        horizontalLayout_3->addLayout(verticalLayout_8);

        horizontalLayoutWidget = new QWidget(tab_5);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(0, 0, 531, 198));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        pushButton_fp_action1 = new QPushButton(horizontalLayoutWidget);
        buttonGroup_mtp_actions = new QButtonGroup(CarControllerUi);
        buttonGroup_mtp_actions->setObjectName(QStringLiteral("buttonGroup_mtp_actions"));
        buttonGroup_mtp_actions->addButton(pushButton_fp_action1);
        pushButton_fp_action1->setObjectName(QStringLiteral("pushButton_fp_action1"));

        verticalLayout->addWidget(pushButton_fp_action1);

        pushButton_fp_action9 = new QPushButton(horizontalLayoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action9);
        pushButton_fp_action9->setObjectName(QStringLiteral("pushButton_fp_action9"));

        verticalLayout->addWidget(pushButton_fp_action9);

        pushButton_fp_action4 = new QPushButton(horizontalLayoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action4);
        pushButton_fp_action4->setObjectName(QStringLiteral("pushButton_fp_action4"));

        verticalLayout->addWidget(pushButton_fp_action4);


        horizontalLayout->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        pushButton_fp_action5 = new QPushButton(horizontalLayoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action5);
        pushButton_fp_action5->setObjectName(QStringLiteral("pushButton_fp_action5"));

        verticalLayout_2->addWidget(pushButton_fp_action5);

        pushButton_fp_action6 = new QPushButton(horizontalLayoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action6);
        pushButton_fp_action6->setObjectName(QStringLiteral("pushButton_fp_action6"));

        verticalLayout_2->addWidget(pushButton_fp_action6);

        pushButton_fp_action8 = new QPushButton(horizontalLayoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action8);
        pushButton_fp_action8->setObjectName(QStringLiteral("pushButton_fp_action8"));

        verticalLayout_2->addWidget(pushButton_fp_action8);


        horizontalLayout->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        pushButton_fp_action3 = new QPushButton(horizontalLayoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action3);
        pushButton_fp_action3->setObjectName(QStringLiteral("pushButton_fp_action3"));

        verticalLayout_3->addWidget(pushButton_fp_action3);

        pushButton_fp_action7 = new QPushButton(horizontalLayoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action7);
        pushButton_fp_action7->setObjectName(QStringLiteral("pushButton_fp_action7"));

        verticalLayout_3->addWidget(pushButton_fp_action7);

        pushButton_fp_action2 = new QPushButton(horizontalLayoutWidget);
        buttonGroup_mtp_actions->addButton(pushButton_fp_action2);
        pushButton_fp_action2->setObjectName(QStringLiteral("pushButton_fp_action2"));

        verticalLayout_3->addWidget(pushButton_fp_action2);


        horizontalLayout->addLayout(verticalLayout_3);

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
        verticalLayoutWidget_2 = new QWidget(tab_10);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(30, 10, 191, 371));
        verticalLayout_5 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        pushButton_14 = new QPushButton(verticalLayoutWidget_2);
        buttonGroup_ld_actions = new QButtonGroup(CarControllerUi);
        buttonGroup_ld_actions->setObjectName(QStringLiteral("buttonGroup_ld_actions"));
        buttonGroup_ld_actions->addButton(pushButton_14);
        pushButton_14->setObjectName(QStringLiteral("pushButton_14"));

        verticalLayout_5->addWidget(pushButton_14);

        pushButton_15 = new QPushButton(verticalLayoutWidget_2);
        buttonGroup_ld_actions->addButton(pushButton_15);
        pushButton_15->setObjectName(QStringLiteral("pushButton_15"));

        verticalLayout_5->addWidget(pushButton_15);

        pushButton_16 = new QPushButton(verticalLayoutWidget_2);
        buttonGroup_ld_actions->addButton(pushButton_16);
        pushButton_16->setObjectName(QStringLiteral("pushButton_16"));

        verticalLayout_5->addWidget(pushButton_16);

        tabWidget_2->addTab(tab_10, QString());
        tab_8 = new QWidget();
        tab_8->setObjectName(QStringLiteral("tab_8"));
        scrollArea = new QScrollArea(tab_8);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setGeometry(QRect(0, 0, 553, 401));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 551, 399));
        verticalLayoutWidget_6 = new QWidget(scrollAreaWidgetContents);
        verticalLayoutWidget_6->setObjectName(QStringLiteral("verticalLayoutWidget_6"));
        verticalLayoutWidget_6->setGeometry(QRect(0, 6, 282, 391));
        verticalLayout_12 = new QVBoxLayout(verticalLayoutWidget_6);
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setContentsMargins(11, 11, 11, 11);
        verticalLayout_12->setObjectName(QStringLiteral("verticalLayout_12"));
        verticalLayout_12->setContentsMargins(0, 0, 0, 0);
        buttonGroup_usacc_action1 = new QPushButton(verticalLayoutWidget_6);
        buttonGroup_usacc_actions = new QButtonGroup(CarControllerUi);
        buttonGroup_usacc_actions->setObjectName(QStringLiteral("buttonGroup_usacc_actions"));
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action1);
        buttonGroup_usacc_action1->setObjectName(QStringLiteral("buttonGroup_usacc_action1"));

        verticalLayout_12->addWidget(buttonGroup_usacc_action1);

        buttonGroup_usacc_action2 = new QPushButton(verticalLayoutWidget_6);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action2);
        buttonGroup_usacc_action2->setObjectName(QStringLiteral("buttonGroup_usacc_action2"));

        verticalLayout_12->addWidget(buttonGroup_usacc_action2);

        buttonGroup_usacc_action3 = new QPushButton(verticalLayoutWidget_6);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action3);
        buttonGroup_usacc_action3->setObjectName(QStringLiteral("buttonGroup_usacc_action3"));

        verticalLayout_12->addWidget(buttonGroup_usacc_action3);

        buttonGroup_usacc_action4 = new QPushButton(verticalLayoutWidget_6);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action4);
        buttonGroup_usacc_action4->setObjectName(QStringLiteral("buttonGroup_usacc_action4"));

        verticalLayout_12->addWidget(buttonGroup_usacc_action4);

        buttonGroup_usacc_action5 = new QPushButton(verticalLayoutWidget_6);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action5);
        buttonGroup_usacc_action5->setObjectName(QStringLiteral("buttonGroup_usacc_action5"));

        verticalLayout_12->addWidget(buttonGroup_usacc_action5);

        buttonGroup_usacc_action6 = new QPushButton(verticalLayoutWidget_6);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action6);
        buttonGroup_usacc_action6->setObjectName(QStringLiteral("buttonGroup_usacc_action6"));

        verticalLayout_12->addWidget(buttonGroup_usacc_action6);

        buttonGroup_usacc_action7 = new QPushButton(verticalLayoutWidget_6);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action7);
        buttonGroup_usacc_action7->setObjectName(QStringLiteral("buttonGroup_usacc_action7"));

        verticalLayout_12->addWidget(buttonGroup_usacc_action7);

        gridLayoutWidget = new QWidget(scrollAreaWidgetContents);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(280, 10, 251, 391));
        gridLayout_4 = new QGridLayout(gridLayoutWidget);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        buttonGroup_usacc_action8 = new QPushButton(gridLayoutWidget);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action8);
        buttonGroup_usacc_action8->setObjectName(QStringLiteral("buttonGroup_usacc_action8"));

        gridLayout_4->addWidget(buttonGroup_usacc_action8, 9, 0, 1, 1);

        buttonGroup_usacc_action10 = new QPushButton(gridLayoutWidget);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action10);
        buttonGroup_usacc_action10->setObjectName(QStringLiteral("buttonGroup_usacc_action10"));

        gridLayout_4->addWidget(buttonGroup_usacc_action10, 11, 0, 1, 1);

        buttonGroup_usacc_action11 = new QPushButton(gridLayoutWidget);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action11);
        buttonGroup_usacc_action11->setObjectName(QStringLiteral("buttonGroup_usacc_action11"));

        gridLayout_4->addWidget(buttonGroup_usacc_action11, 12, 0, 1, 1);

        buttonGroup_usacc_action9 = new QPushButton(gridLayoutWidget);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action9);
        buttonGroup_usacc_action9->setObjectName(QStringLiteral("buttonGroup_usacc_action9"));

        gridLayout_4->addWidget(buttonGroup_usacc_action9, 10, 0, 1, 1);

        buttonGroup_usacc_action12 = new QPushButton(gridLayoutWidget);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action12);
        buttonGroup_usacc_action12->setObjectName(QStringLiteral("buttonGroup_usacc_action12"));

        gridLayout_4->addWidget(buttonGroup_usacc_action12, 13, 0, 1, 1);

        buttonGroup_usacc_action13 = new QPushButton(gridLayoutWidget);
        buttonGroup_usacc_actions->addButton(buttonGroup_usacc_action13);
        buttonGroup_usacc_action13->setObjectName(QStringLiteral("buttonGroup_usacc_action13"));

        gridLayout_4->addWidget(buttonGroup_usacc_action13, 14, 0, 1, 1);

        scrollArea->setWidget(scrollAreaWidgetContents);
        tabWidget_2->addTab(tab_8, QString());
        frame_2 = new QFrame(tab_3);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setGeometry(QRect(-2, 0, 541, 80));
        frame_2->setMinimumSize(QSize(540, 80));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        gridLayout_6 = new QGridLayout(frame_2);
        gridLayout_6->setSpacing(6);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(10);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        pushButton_arb_action2 = new QPushButton(frame_2);
        buttonGroup_arb_actions->addButton(pushButton_arb_action2);
        pushButton_arb_action2->setObjectName(QStringLiteral("pushButton_arb_action2"));

        horizontalLayout_2->addWidget(pushButton_arb_action2);

        label = new QLabel(frame_2);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_2->addWidget(label);

        lineEdit_arbCom = new QLineEdit(frame_2);
        lineEdit_arbCom->setObjectName(QStringLiteral("lineEdit_arbCom"));

        horizontalLayout_2->addWidget(lineEdit_arbCom);


        gridLayout_6->addLayout(horizontalLayout_2, 0, 0, 1, 1);

        tabWidget->addTab(tab_3, QString());

        gridLayout->addWidget(tabWidget, 2, 0, 1, 1);


        retranslateUi(CarControllerUi);

        tabWidget->setCurrentIndex(2);
        tabWidget_2->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(CarControllerUi);
    } // setupUi

    void retranslateUi(QWidget *CarControllerUi)
    {
        CarControllerUi->setWindowTitle(QApplication::translate("CarControllerUi", "Widget", Q_NULLPTR));
        pushButton_turnright->setText(QApplication::translate("CarControllerUi", "Toggle TurnRight", Q_NULLPTR));
        label_lights->setText(QApplication::translate("CarControllerUi", "Lights", Q_NULLPTR));
        pushButton_head->setText(QApplication::translate("CarControllerUi", "Toggle Head", Q_NULLPTR));
        pushButton_brake->setText(QApplication::translate("CarControllerUi", "Toggle Brake", Q_NULLPTR));
        pushButton_turnleft->setText(QApplication::translate("CarControllerUi", "Toggle TurnLeft", Q_NULLPTR));
        pushButton_reverse->setText(QApplication::translate("CarControllerUi", "Toggle Reverse", Q_NULLPTR));
        pushButton_hazard->setText(QApplication::translate("CarControllerUi", "Toggle Hazard", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("CarControllerUi", "Lights", Q_NULLPTR));
        radioButton_enable_resetting_slider->setText(QApplication::translate("CarControllerUi", "enable resetting slider", Q_NULLPTR));
        label_throttle->setText(QApplication::translate("CarControllerUi", "Throttle", Q_NULLPTR));
        label_steering->setText(QApplication::translate("CarControllerUi", "Steering", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("CarControllerUi", "Slider", Q_NULLPTR));
        pushButton_lc_action9->setText(QApplication::translate("CarControllerUi", "HAZARD_LIGHT_OFF", Q_NULLPTR));
        pushButton_lc_action8->setText(QApplication::translate("CarControllerUi", "HAZARD_LIGHT_ON", Q_NULLPTR));
        pushButton_lc_action5->setText(QApplication::translate("CarControllerUi", "TURN_LEFT", Q_NULLPTR));
        pushButton_lc_action4->setText(QApplication::translate("CarControllerUi", "RERVERSE_LIGHT_OFF", Q_NULLPTR));
        pushButton_lc_action3->setText(QApplication::translate("CarControllerUi", "RERVERSE_LIGHT_ON", Q_NULLPTR));
        pushButton_lc_action7->setText(QApplication::translate("CarControllerUi", "TURN_DISABLE", Q_NULLPTR));
        pushButton_lc_action2->setText(QApplication::translate("CarControllerUi", "HEAD_LIGHT_OFF", Q_NULLPTR));
        pushButton_lc_action1->setText(QApplication::translate("CarControllerUi", "HEAD_LIGHT_ON", Q_NULLPTR));
        pushButton_lc_action6->setText(QApplication::translate("CarControllerUi", "TURN_RIGHT", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_4), QApplication::translate("CarControllerUi", "Lights", Q_NULLPTR));
        pushButton_arb_action1->setText(QApplication::translate("CarControllerUi", "submit", Q_NULLPTR));
        comboBox_gotoCom->clear();
        comboBox_gotoCom->insertItems(0, QStringList()
         << QApplication::translate("CarControllerUi", "AC_FP_GOTO_XY", Q_NULLPTR)
         << QApplication::translate("CarControllerUi", "AC_FP_GOTO_XY_NOSTOP", Q_NULLPTR)
        );
        label_11->setText(QApplication::translate("CarControllerUi", "PosX:  ", Q_NULLPTR));
        label_12->setText(QApplication::translate("CarControllerUi", "PosY:  ", Q_NULLPTR));
        label_14->setText(QApplication::translate("CarControllerUi", "Cut", Q_NULLPTR));
        label_6->setText(QApplication::translate("CarControllerUi", "Yaw:  ", Q_NULLPTR));
        label_8->setText(QApplication::translate("CarControllerUi", "Speed:  ", Q_NULLPTR));
        pushButton_fp_action1->setText(QApplication::translate("CarControllerUi", "SET_FOLLOW_FILE", Q_NULLPTR));
        pushButton_fp_action9->setText(QApplication::translate("CarControllerUi", "RELOAD_XML_FILES", Q_NULLPTR));
        pushButton_fp_action4->setText(QApplication::translate("CarControllerUi", "CHANGE_TO_ORIG_LANE", Q_NULLPTR));
        pushButton_fp_action5->setText(QApplication::translate("CarControllerUi", "LEFT_TURN", Q_NULLPTR));
        pushButton_fp_action6->setText(QApplication::translate("CarControllerUi", "STOP", Q_NULLPTR));
        pushButton_fp_action8->setText(QApplication::translate("CarControllerUi", "RIGHT_TURN", Q_NULLPTR));
        pushButton_fp_action3->setText(QApplication::translate("CarControllerUi", "GO_STRAIGHT", Q_NULLPTR));
        pushButton_fp_action7->setText(QApplication::translate("CarControllerUi", "GO_BACK", Q_NULLPTR));
        pushButton_fp_action2->setText(QApplication::translate("CarControllerUi", "PARKING_TRANS", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_5), QApplication::translate("CarControllerUi", "FollowPath", Q_NULLPTR));
        pushButton_sa_1->setText(QApplication::translate("CarControllerUi", "AC_SA_STOP_CAR", Q_NULLPTR));
        pushButton_sa_2->setText(QApplication::translate("CarControllerUi", "AC_SA_TEST", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_6), QApplication::translate("CarControllerUi", "ActionStop", Q_NULLPTR));
        pushButton_14->setText(QApplication::translate("CarControllerUi", "DRIVE", Q_NULLPTR));
        pushButton_15->setText(QApplication::translate("CarControllerUi", "DRIVE_SLOW", Q_NULLPTR));
        pushButton_16->setText(QApplication::translate("CarControllerUi", "STOP", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_10), QApplication::translate("CarControllerUi", "LaneDetection", Q_NULLPTR));
        buttonGroup_usacc_action1->setText(QApplication::translate("CarControllerUi", "DRIVING_MODE", Q_NULLPTR));
        buttonGroup_usacc_action2->setText(QApplication::translate("CarControllerUi", "INTERSECTION_CHECK_ALL", Q_NULLPTR));
        buttonGroup_usacc_action3->setText(QApplication::translate("CarControllerUi", "INTERS_CHCK_CRSS_TRAF_R", Q_NULLPTR));
        buttonGroup_usacc_action4->setText(QApplication::translate("CarControllerUi", "PARKING_MODE", Q_NULLPTR));
        buttonGroup_usacc_action5->setText(QApplication::translate("CarControllerUi", "PRKNG_CRSS_PLLT_CHCK_CRSS_TRAF", Q_NULLPTR));
        buttonGroup_usacc_action6->setText(QApplication::translate("CarControllerUi", "INTERS_CHCK_ONC_TRAFFIC", Q_NULLPTR));
        buttonGroup_usacc_action7->setText(QApplication::translate("CarControllerUi", "PRKNG_CRSS_CHCK_ONC_TRAFFIC", Q_NULLPTR));
        buttonGroup_usacc_action8->setText(QApplication::translate("CarControllerUi", "OVERT_CHCK_OVERT_ORIG_L", Q_NULLPTR));
        buttonGroup_usacc_action10->setText(QApplication::translate("CarControllerUi", "OVERT_CHECK_OWN_L_STR_LH", Q_NULLPTR));
        buttonGroup_usacc_action11->setText(QApplication::translate("CarControllerUi", "CHCK_NO_MOVEMENT", Q_NULLPTR));
        buttonGroup_usacc_action9->setText(QApplication::translate("CarControllerUi", "OVERT_CHCK_ONC_TRAF", Q_NULLPTR));
        buttonGroup_usacc_action12->setText(QApplication::translate("CarControllerUi", "AC_UA_CHECK_MOVING_AGAIN", Q_NULLPTR));
        buttonGroup_usacc_action13->setText(QApplication::translate("CarControllerUi", "OVERT_CHCK_OWN_LANE_STR", Q_NULLPTR));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_8), QApplication::translate("CarControllerUi", "US ACC", Q_NULLPTR));
        pushButton_arb_action2->setText(QApplication::translate("CarControllerUi", "submit", Q_NULLPTR));
        label->setText(QApplication::translate("CarControllerUi", "Type command:", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("CarControllerUi", "Actions", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class CarControllerUi: public Ui_CarControllerUi {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAR_CONTROLLER_H

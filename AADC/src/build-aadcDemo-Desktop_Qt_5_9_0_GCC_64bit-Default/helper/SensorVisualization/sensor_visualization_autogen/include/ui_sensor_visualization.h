/********************************************************************************
** Form generated from reading UI file 'sensor_visualization.ui'
**
** Created by: Qt User Interface Compiler version 5.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SENSOR_VISUALIZATION_H
#define UI_SENSOR_VISUALIZATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "uswidget.h"

QT_BEGIN_NAMESPACE

class Ui_SensorVisualizationUi
{
public:
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *tab;
    QGridLayout *gridLayout;
    QLabel *label_us_sr;
    QLabel *label_us_r_distance;
    QLabel *label_us_sl;
    QLCDNumber *lcdNumber_us_rcr;
    QLabel *label_us_rcl;
    QLCDNumber *lcdNumber_us_update_sl;
    QLCDNumber *lcdNumber_us_update_sr;
    QLCDNumber *lcdNumber_us_sr;
    QLCDNumber *lcdNumber_us_update_rcr;
    QLabel *label_us_rc;
    QLabel *label_us_rcr;
    QLabel *label_us_r_updaterate;
    QLCDNumber *lcdNumber_us_rc;
    QLCDNumber *lcdNumber_us_update_rc;
    QLCDNumber *lcdNumber_us_update_rcl;
    QLCDNumber *lcdNumber_us_rcl;
    QLCDNumber *lcdNumber_us_sl;
    QWidget *tab_2;
    QGridLayout *gridLayout_2;
    QLCDNumber *lcdNumber_wheel_right_count;
    QLCDNumber *lcdNumber_wheel_left_direction;
    QLCDNumber *lcdNumber_wheel_left_count;
    QLabel *label_wheel_left;
    QLabel *label_wheel_direction;
    QLCDNumber *lcdNumber_wheel_right_direction;
    QLabel *label_wheel_right;
    QLabel *label_wheel_count;
    QLabel *label_wheel_update;
    QLCDNumber *lcdNumber_wheel_update_left;
    QLCDNumber *lcdNumber_wheel_update_right;
    QWidget *tab_3;
    QGridLayout *gridLayout_3;
    QLCDNumber *lcdNumber_imu_angular_x;
    QLabel *label_imu_x;
    QLCDNumber *lcdNumber_imu_acc_x;
    QLCDNumber *lcdNumber_imu_acc_y;
    QLCDNumber *lcdNumber_imu_acc_z;
    QLCDNumber *lcdNumber_imu_angular_y;
    QLCDNumber *lcdNumber_imu_mag_x;
    QLabel *label_imu_mag;
    QLabel *label_imu_y;
    QLabel *label_imu_z;
    QLabel *label_imu_angular;
    QLCDNumber *lcdNumber_imu_mag_y;
    QLCDNumber *lcdNumber_imu_angular_z;
    QLCDNumber *lcdNumber_imu_mag_z;
    QLabel *label_imu_acc;
    QLCDNumber *lcdNumber_imu_update;
    QLabel *label_imu_update;
    QWidget *tab_5;
    QGridLayout *gridLayout_5;
    QLabel *label_sensors_cell4;
    QLabel *label_volt_actuator_overall;
    QLabel *label_sensors_cell2;
    QLabel *label_sensors_cell1;
    QLCDNumber *lcdNumber_sensors_cell1;
    QLabel *label_sensors_cell3;
    QLCDNumber *lcdNumber_sensors_cell2;
    QLabel *label_sensors_cell6;
    QLCDNumber *lcdNumber_volt_actuator_overall;
    QLabel *label_sensors_cell5;
    QLCDNumber *lcdNumber_volt_sensors_overall;
    QLCDNumber *lcdNumber_volt_update;
    QLabel *label_actuator_cell1;
    QLabel *label_volt_actuator;
    QLabel *label_volt_sensors;
    QLabel *label_actuator_cell2;
    QLCDNumber *lcdNumber_actuator_cell1;
    QLCDNumber *lcdNumber_actuator_cell2;
    QLabel *label_volt_update;
    QLabel *label_volt_sensors_overall;
    QLCDNumber *lcdNumber_sensors_cell6;
    QLCDNumber *lcdNumber_sensors_cell3;
    QLCDNumber *lcdNumber_sensors_cell4;
    QLCDNumber *lcdNumber_sensors_cell5;
    QFrame *line;
    QWidget *tab_6;
    QHBoxLayout *horizontalLayout;
    uswidget *us_widget;
    QGridLayout *gridLayout_6;
    QFrame *line_2;
    QLabel *label_ls_samples;
    QLCDNumber *lcdNumber_ls_samples;
    QDoubleSpinBox *doubleSpinBox_ls_max_dist;
    QLabel *label_ls_max_dist;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *SensorVisualizationUi)
    {
        if (SensorVisualizationUi->objectName().isEmpty())
            SensorVisualizationUi->setObjectName(QStringLiteral("SensorVisualizationUi"));
        SensorVisualizationUi->resize(768, 327);
        verticalLayout = new QVBoxLayout(SensorVisualizationUi);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        tabWidget = new QTabWidget(SensorVisualizationUi);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        gridLayout = new QGridLayout(tab);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label_us_sr = new QLabel(tab);
        label_us_sr->setObjectName(QStringLiteral("label_us_sr"));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setItalic(true);
        font.setWeight(75);
        label_us_sr->setFont(font);
        label_us_sr->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_us_sr, 0, 6, 1, 1);

        label_us_r_distance = new QLabel(tab);
        label_us_r_distance->setObjectName(QStringLiteral("label_us_r_distance"));

        gridLayout->addWidget(label_us_r_distance, 1, 0, 1, 1);

        label_us_sl = new QLabel(tab);
        label_us_sl->setObjectName(QStringLiteral("label_us_sl"));
        label_us_sl->setFont(font);
        label_us_sl->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_us_sl, 0, 1, 1, 1);

        lcdNumber_us_rcr = new QLCDNumber(tab);
        lcdNumber_us_rcr->setObjectName(QStringLiteral("lcdNumber_us_rcr"));

        gridLayout->addWidget(lcdNumber_us_rcr, 1, 5, 1, 1);

        label_us_rcl = new QLabel(tab);
        label_us_rcl->setObjectName(QStringLiteral("label_us_rcl"));
        label_us_rcl->setFont(font);
        label_us_rcl->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_us_rcl, 0, 3, 1, 1);

        lcdNumber_us_update_sl = new QLCDNumber(tab);
        lcdNumber_us_update_sl->setObjectName(QStringLiteral("lcdNumber_us_update_sl"));

        gridLayout->addWidget(lcdNumber_us_update_sl, 2, 1, 1, 1);

        lcdNumber_us_update_sr = new QLCDNumber(tab);
        lcdNumber_us_update_sr->setObjectName(QStringLiteral("lcdNumber_us_update_sr"));

        gridLayout->addWidget(lcdNumber_us_update_sr, 2, 6, 1, 1);

        lcdNumber_us_sr = new QLCDNumber(tab);
        lcdNumber_us_sr->setObjectName(QStringLiteral("lcdNumber_us_sr"));

        gridLayout->addWidget(lcdNumber_us_sr, 1, 6, 1, 1);

        lcdNumber_us_update_rcr = new QLCDNumber(tab);
        lcdNumber_us_update_rcr->setObjectName(QStringLiteral("lcdNumber_us_update_rcr"));

        gridLayout->addWidget(lcdNumber_us_update_rcr, 2, 5, 1, 1);

        label_us_rc = new QLabel(tab);
        label_us_rc->setObjectName(QStringLiteral("label_us_rc"));
        label_us_rc->setFont(font);
        label_us_rc->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_us_rc, 0, 4, 1, 1);

        label_us_rcr = new QLabel(tab);
        label_us_rcr->setObjectName(QStringLiteral("label_us_rcr"));
        label_us_rcr->setFont(font);
        label_us_rcr->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_us_rcr, 0, 5, 1, 1);

        label_us_r_updaterate = new QLabel(tab);
        label_us_r_updaterate->setObjectName(QStringLiteral("label_us_r_updaterate"));

        gridLayout->addWidget(label_us_r_updaterate, 2, 0, 1, 1);

        lcdNumber_us_rc = new QLCDNumber(tab);
        lcdNumber_us_rc->setObjectName(QStringLiteral("lcdNumber_us_rc"));

        gridLayout->addWidget(lcdNumber_us_rc, 1, 4, 1, 1);

        lcdNumber_us_update_rc = new QLCDNumber(tab);
        lcdNumber_us_update_rc->setObjectName(QStringLiteral("lcdNumber_us_update_rc"));

        gridLayout->addWidget(lcdNumber_us_update_rc, 2, 4, 1, 1);

        lcdNumber_us_update_rcl = new QLCDNumber(tab);
        lcdNumber_us_update_rcl->setObjectName(QStringLiteral("lcdNumber_us_update_rcl"));

        gridLayout->addWidget(lcdNumber_us_update_rcl, 2, 3, 1, 1);

        lcdNumber_us_rcl = new QLCDNumber(tab);
        lcdNumber_us_rcl->setObjectName(QStringLiteral("lcdNumber_us_rcl"));

        gridLayout->addWidget(lcdNumber_us_rcl, 1, 3, 1, 1);

        lcdNumber_us_sl = new QLCDNumber(tab);
        lcdNumber_us_sl->setObjectName(QStringLiteral("lcdNumber_us_sl"));

        gridLayout->addWidget(lcdNumber_us_sl, 1, 1, 1, 1);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        gridLayout_2 = new QGridLayout(tab_2);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        lcdNumber_wheel_right_count = new QLCDNumber(tab_2);
        lcdNumber_wheel_right_count->setObjectName(QStringLiteral("lcdNumber_wheel_right_count"));

        gridLayout_2->addWidget(lcdNumber_wheel_right_count, 1, 2, 1, 1);

        lcdNumber_wheel_left_direction = new QLCDNumber(tab_2);
        lcdNumber_wheel_left_direction->setObjectName(QStringLiteral("lcdNumber_wheel_left_direction"));

        gridLayout_2->addWidget(lcdNumber_wheel_left_direction, 2, 1, 1, 1);

        lcdNumber_wheel_left_count = new QLCDNumber(tab_2);
        lcdNumber_wheel_left_count->setObjectName(QStringLiteral("lcdNumber_wheel_left_count"));

        gridLayout_2->addWidget(lcdNumber_wheel_left_count, 1, 1, 1, 1);

        label_wheel_left = new QLabel(tab_2);
        label_wheel_left->setObjectName(QStringLiteral("label_wheel_left"));
        QFont font1;
        font1.setPointSize(14);
        font1.setBold(true);
        font1.setItalic(true);
        font1.setWeight(75);
        label_wheel_left->setFont(font1);
        label_wheel_left->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_wheel_left, 0, 1, 1, 1);

        label_wheel_direction = new QLabel(tab_2);
        label_wheel_direction->setObjectName(QStringLiteral("label_wheel_direction"));
        QFont font2;
        font2.setPointSize(16);
        font2.setBold(true);
        font2.setItalic(true);
        font2.setWeight(75);
        label_wheel_direction->setFont(font2);

        gridLayout_2->addWidget(label_wheel_direction, 2, 0, 1, 1);

        lcdNumber_wheel_right_direction = new QLCDNumber(tab_2);
        lcdNumber_wheel_right_direction->setObjectName(QStringLiteral("lcdNumber_wheel_right_direction"));

        gridLayout_2->addWidget(lcdNumber_wheel_right_direction, 2, 2, 1, 1);

        label_wheel_right = new QLabel(tab_2);
        label_wheel_right->setObjectName(QStringLiteral("label_wheel_right"));
        label_wheel_right->setFont(font1);
        label_wheel_right->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_wheel_right, 0, 2, 1, 1);

        label_wheel_count = new QLabel(tab_2);
        label_wheel_count->setObjectName(QStringLiteral("label_wheel_count"));
        label_wheel_count->setFont(font2);

        gridLayout_2->addWidget(label_wheel_count, 1, 0, 1, 1);

        label_wheel_update = new QLabel(tab_2);
        label_wheel_update->setObjectName(QStringLiteral("label_wheel_update"));
        label_wheel_update->setFont(font2);

        gridLayout_2->addWidget(label_wheel_update, 3, 0, 1, 1);

        lcdNumber_wheel_update_left = new QLCDNumber(tab_2);
        lcdNumber_wheel_update_left->setObjectName(QStringLiteral("lcdNumber_wheel_update_left"));

        gridLayout_2->addWidget(lcdNumber_wheel_update_left, 3, 1, 1, 1);

        lcdNumber_wheel_update_right = new QLCDNumber(tab_2);
        lcdNumber_wheel_update_right->setObjectName(QStringLiteral("lcdNumber_wheel_update_right"));

        gridLayout_2->addWidget(lcdNumber_wheel_update_right, 3, 2, 1, 1);

        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QStringLiteral("tab_3"));
        gridLayout_3 = new QGridLayout(tab_3);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        lcdNumber_imu_angular_x = new QLCDNumber(tab_3);
        lcdNumber_imu_angular_x->setObjectName(QStringLiteral("lcdNumber_imu_angular_x"));

        gridLayout_3->addWidget(lcdNumber_imu_angular_x, 1, 2, 1, 1);

        label_imu_x = new QLabel(tab_3);
        label_imu_x->setObjectName(QStringLiteral("label_imu_x"));
        QFont font3;
        font3.setPointSize(20);
        font3.setBold(true);
        font3.setItalic(true);
        font3.setWeight(75);
        label_imu_x->setFont(font3);
        label_imu_x->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_imu_x, 1, 0, 1, 1);

        lcdNumber_imu_acc_x = new QLCDNumber(tab_3);
        lcdNumber_imu_acc_x->setObjectName(QStringLiteral("lcdNumber_imu_acc_x"));

        gridLayout_3->addWidget(lcdNumber_imu_acc_x, 1, 1, 1, 1);

        lcdNumber_imu_acc_y = new QLCDNumber(tab_3);
        lcdNumber_imu_acc_y->setObjectName(QStringLiteral("lcdNumber_imu_acc_y"));

        gridLayout_3->addWidget(lcdNumber_imu_acc_y, 2, 1, 1, 1);

        lcdNumber_imu_acc_z = new QLCDNumber(tab_3);
        lcdNumber_imu_acc_z->setObjectName(QStringLiteral("lcdNumber_imu_acc_z"));

        gridLayout_3->addWidget(lcdNumber_imu_acc_z, 3, 1, 1, 1);

        lcdNumber_imu_angular_y = new QLCDNumber(tab_3);
        lcdNumber_imu_angular_y->setObjectName(QStringLiteral("lcdNumber_imu_angular_y"));

        gridLayout_3->addWidget(lcdNumber_imu_angular_y, 2, 2, 1, 1);

        lcdNumber_imu_mag_x = new QLCDNumber(tab_3);
        lcdNumber_imu_mag_x->setObjectName(QStringLiteral("lcdNumber_imu_mag_x"));

        gridLayout_3->addWidget(lcdNumber_imu_mag_x, 1, 3, 1, 1);

        label_imu_mag = new QLabel(tab_3);
        label_imu_mag->setObjectName(QStringLiteral("label_imu_mag"));
        label_imu_mag->setFont(font1);

        gridLayout_3->addWidget(label_imu_mag, 0, 3, 1, 1);

        label_imu_y = new QLabel(tab_3);
        label_imu_y->setObjectName(QStringLiteral("label_imu_y"));
        label_imu_y->setFont(font3);
        label_imu_y->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_imu_y, 2, 0, 1, 1);

        label_imu_z = new QLabel(tab_3);
        label_imu_z->setObjectName(QStringLiteral("label_imu_z"));
        label_imu_z->setFont(font3);
        label_imu_z->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_imu_z, 3, 0, 1, 1);

        label_imu_angular = new QLabel(tab_3);
        label_imu_angular->setObjectName(QStringLiteral("label_imu_angular"));
        label_imu_angular->setFont(font1);

        gridLayout_3->addWidget(label_imu_angular, 0, 2, 1, 1);

        lcdNumber_imu_mag_y = new QLCDNumber(tab_3);
        lcdNumber_imu_mag_y->setObjectName(QStringLiteral("lcdNumber_imu_mag_y"));

        gridLayout_3->addWidget(lcdNumber_imu_mag_y, 2, 3, 1, 1);

        lcdNumber_imu_angular_z = new QLCDNumber(tab_3);
        lcdNumber_imu_angular_z->setObjectName(QStringLiteral("lcdNumber_imu_angular_z"));

        gridLayout_3->addWidget(lcdNumber_imu_angular_z, 3, 2, 1, 1);

        lcdNumber_imu_mag_z = new QLCDNumber(tab_3);
        lcdNumber_imu_mag_z->setObjectName(QStringLiteral("lcdNumber_imu_mag_z"));

        gridLayout_3->addWidget(lcdNumber_imu_mag_z, 3, 3, 1, 1);

        label_imu_acc = new QLabel(tab_3);
        label_imu_acc->setObjectName(QStringLiteral("label_imu_acc"));
        label_imu_acc->setFont(font1);

        gridLayout_3->addWidget(label_imu_acc, 0, 1, 1, 1);

        lcdNumber_imu_update = new QLCDNumber(tab_3);
        lcdNumber_imu_update->setObjectName(QStringLiteral("lcdNumber_imu_update"));

        gridLayout_3->addWidget(lcdNumber_imu_update, 4, 1, 1, 3);

        label_imu_update = new QLabel(tab_3);
        label_imu_update->setObjectName(QStringLiteral("label_imu_update"));
        label_imu_update->setFont(font);
        label_imu_update->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_imu_update, 4, 0, 1, 1);

        tabWidget->addTab(tab_3, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QStringLiteral("tab_5"));
        gridLayout_5 = new QGridLayout(tab_5);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        label_sensors_cell4 = new QLabel(tab_5);
        label_sensors_cell4->setObjectName(QStringLiteral("label_sensors_cell4"));

        gridLayout_5->addWidget(label_sensors_cell4, 5, 2, 1, 1);

        label_volt_actuator_overall = new QLabel(tab_5);
        label_volt_actuator_overall->setObjectName(QStringLiteral("label_volt_actuator_overall"));

        gridLayout_5->addWidget(label_volt_actuator_overall, 1, 0, 1, 1);

        label_sensors_cell2 = new QLabel(tab_5);
        label_sensors_cell2->setObjectName(QStringLiteral("label_sensors_cell2"));

        gridLayout_5->addWidget(label_sensors_cell2, 3, 2, 1, 1);

        label_sensors_cell1 = new QLabel(tab_5);
        label_sensors_cell1->setObjectName(QStringLiteral("label_sensors_cell1"));

        gridLayout_5->addWidget(label_sensors_cell1, 2, 2, 1, 1);

        lcdNumber_sensors_cell1 = new QLCDNumber(tab_5);
        lcdNumber_sensors_cell1->setObjectName(QStringLiteral("lcdNumber_sensors_cell1"));

        gridLayout_5->addWidget(lcdNumber_sensors_cell1, 2, 3, 1, 1);

        label_sensors_cell3 = new QLabel(tab_5);
        label_sensors_cell3->setObjectName(QStringLiteral("label_sensors_cell3"));

        gridLayout_5->addWidget(label_sensors_cell3, 4, 2, 1, 1);

        lcdNumber_sensors_cell2 = new QLCDNumber(tab_5);
        lcdNumber_sensors_cell2->setObjectName(QStringLiteral("lcdNumber_sensors_cell2"));

        gridLayout_5->addWidget(lcdNumber_sensors_cell2, 3, 3, 1, 1);

        label_sensors_cell6 = new QLabel(tab_5);
        label_sensors_cell6->setObjectName(QStringLiteral("label_sensors_cell6"));

        gridLayout_5->addWidget(label_sensors_cell6, 7, 2, 1, 1);

        lcdNumber_volt_actuator_overall = new QLCDNumber(tab_5);
        lcdNumber_volt_actuator_overall->setObjectName(QStringLiteral("lcdNumber_volt_actuator_overall"));

        gridLayout_5->addWidget(lcdNumber_volt_actuator_overall, 1, 1, 1, 1);

        label_sensors_cell5 = new QLabel(tab_5);
        label_sensors_cell5->setObjectName(QStringLiteral("label_sensors_cell5"));

        gridLayout_5->addWidget(label_sensors_cell5, 6, 2, 1, 1);

        lcdNumber_volt_sensors_overall = new QLCDNumber(tab_5);
        lcdNumber_volt_sensors_overall->setObjectName(QStringLiteral("lcdNumber_volt_sensors_overall"));

        gridLayout_5->addWidget(lcdNumber_volt_sensors_overall, 1, 3, 1, 1);

        lcdNumber_volt_update = new QLCDNumber(tab_5);
        lcdNumber_volt_update->setObjectName(QStringLiteral("lcdNumber_volt_update"));

        gridLayout_5->addWidget(lcdNumber_volt_update, 9, 2, 1, 2);

        label_actuator_cell1 = new QLabel(tab_5);
        label_actuator_cell1->setObjectName(QStringLiteral("label_actuator_cell1"));

        gridLayout_5->addWidget(label_actuator_cell1, 2, 0, 1, 1);

        label_volt_actuator = new QLabel(tab_5);
        label_volt_actuator->setObjectName(QStringLiteral("label_volt_actuator"));
        label_volt_actuator->setFont(font1);

        gridLayout_5->addWidget(label_volt_actuator, 0, 0, 1, 1);

        label_volt_sensors = new QLabel(tab_5);
        label_volt_sensors->setObjectName(QStringLiteral("label_volt_sensors"));
        label_volt_sensors->setFont(font1);

        gridLayout_5->addWidget(label_volt_sensors, 0, 2, 1, 2);

        label_actuator_cell2 = new QLabel(tab_5);
        label_actuator_cell2->setObjectName(QStringLiteral("label_actuator_cell2"));

        gridLayout_5->addWidget(label_actuator_cell2, 3, 0, 1, 1);

        lcdNumber_actuator_cell1 = new QLCDNumber(tab_5);
        lcdNumber_actuator_cell1->setObjectName(QStringLiteral("lcdNumber_actuator_cell1"));

        gridLayout_5->addWidget(lcdNumber_actuator_cell1, 2, 1, 1, 1);

        lcdNumber_actuator_cell2 = new QLCDNumber(tab_5);
        lcdNumber_actuator_cell2->setObjectName(QStringLiteral("lcdNumber_actuator_cell2"));

        gridLayout_5->addWidget(lcdNumber_actuator_cell2, 3, 1, 1, 1);

        label_volt_update = new QLabel(tab_5);
        label_volt_update->setObjectName(QStringLiteral("label_volt_update"));
        label_volt_update->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_volt_update, 9, 0, 1, 2);

        label_volt_sensors_overall = new QLabel(tab_5);
        label_volt_sensors_overall->setObjectName(QStringLiteral("label_volt_sensors_overall"));

        gridLayout_5->addWidget(label_volt_sensors_overall, 1, 2, 1, 1);

        lcdNumber_sensors_cell6 = new QLCDNumber(tab_5);
        lcdNumber_sensors_cell6->setObjectName(QStringLiteral("lcdNumber_sensors_cell6"));

        gridLayout_5->addWidget(lcdNumber_sensors_cell6, 7, 3, 1, 1);

        lcdNumber_sensors_cell3 = new QLCDNumber(tab_5);
        lcdNumber_sensors_cell3->setObjectName(QStringLiteral("lcdNumber_sensors_cell3"));

        gridLayout_5->addWidget(lcdNumber_sensors_cell3, 4, 3, 1, 1);

        lcdNumber_sensors_cell4 = new QLCDNumber(tab_5);
        lcdNumber_sensors_cell4->setObjectName(QStringLiteral("lcdNumber_sensors_cell4"));

        gridLayout_5->addWidget(lcdNumber_sensors_cell4, 5, 3, 1, 1);

        lcdNumber_sensors_cell5 = new QLCDNumber(tab_5);
        lcdNumber_sensors_cell5->setObjectName(QStringLiteral("lcdNumber_sensors_cell5"));

        gridLayout_5->addWidget(lcdNumber_sensors_cell5, 6, 3, 1, 1);

        line = new QFrame(tab_5);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout_5->addWidget(line, 8, 0, 1, 4);

        tabWidget->addTab(tab_5, QString());
        tab_6 = new QWidget();
        tab_6->setObjectName(QStringLiteral("tab_6"));
        horizontalLayout = new QHBoxLayout(tab_6);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        us_widget = new uswidget(tab_6);
        us_widget->setObjectName(QStringLiteral("us_widget"));
        gridLayout_6 = new QGridLayout(us_widget);
        gridLayout_6->setSpacing(6);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        line_2 = new QFrame(us_widget);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        gridLayout_6->addWidget(line_2, 1, 0, 1, 4);

        label_ls_samples = new QLabel(us_widget);
        label_ls_samples->setObjectName(QStringLiteral("label_ls_samples"));
        label_ls_samples->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_ls_samples, 0, 0, 1, 1);

        lcdNumber_ls_samples = new QLCDNumber(us_widget);
        lcdNumber_ls_samples->setObjectName(QStringLiteral("lcdNumber_ls_samples"));

        gridLayout_6->addWidget(lcdNumber_ls_samples, 0, 1, 1, 1);

        doubleSpinBox_ls_max_dist = new QDoubleSpinBox(us_widget);
        doubleSpinBox_ls_max_dist->setObjectName(QStringLiteral("doubleSpinBox_ls_max_dist"));

        gridLayout_6->addWidget(doubleSpinBox_ls_max_dist, 0, 3, 1, 1);

        label_ls_max_dist = new QLabel(us_widget);
        label_ls_max_dist->setObjectName(QStringLiteral("label_ls_max_dist"));
        label_ls_max_dist->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_ls_max_dist, 0, 2, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_6->addItem(verticalSpacer, 2, 0, 1, 1);


        horizontalLayout->addWidget(us_widget);

        tabWidget->addTab(tab_6, QString());

        verticalLayout->addWidget(tabWidget);


        retranslateUi(SensorVisualizationUi);

        tabWidget->setCurrentIndex(4);


        QMetaObject::connectSlotsByName(SensorVisualizationUi);
    } // setupUi

    void retranslateUi(QWidget *SensorVisualizationUi)
    {
        SensorVisualizationUi->setWindowTitle(QApplication::translate("SensorVisualizationUi", "Widget", Q_NULLPTR));
        label_us_sr->setText(QApplication::translate("SensorVisualizationUi", "Side Right", Q_NULLPTR));
        label_us_r_distance->setText(QApplication::translate("SensorVisualizationUi", "Distance [cm]", Q_NULLPTR));
        label_us_sl->setText(QApplication::translate("SensorVisualizationUi", "Side Left", Q_NULLPTR));
        label_us_rcl->setText(QApplication::translate("SensorVisualizationUi", "Rear Left", Q_NULLPTR));
        label_us_rc->setText(QApplication::translate("SensorVisualizationUi", "Rear Center", Q_NULLPTR));
        label_us_rcr->setText(QApplication::translate("SensorVisualizationUi", "Rear Right", Q_NULLPTR));
        label_us_r_updaterate->setText(QApplication::translate("SensorVisualizationUi", "Update Rate [Hz]", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("SensorVisualizationUi", "Ultrasonic Sensors", Q_NULLPTR));
        label_wheel_left->setText(QApplication::translate("SensorVisualizationUi", "Left", Q_NULLPTR));
        label_wheel_direction->setText(QApplication::translate("SensorVisualizationUi", "Direction", Q_NULLPTR));
        label_wheel_right->setText(QApplication::translate("SensorVisualizationUi", "Right", Q_NULLPTR));
        label_wheel_count->setText(QApplication::translate("SensorVisualizationUi", "Count", Q_NULLPTR));
        label_wheel_update->setText(QApplication::translate("SensorVisualizationUi", "Update Rate [Hz]", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("SensorVisualizationUi", "Wheel Encoder", Q_NULLPTR));
        label_imu_x->setText(QApplication::translate("SensorVisualizationUi", "X", Q_NULLPTR));
        label_imu_mag->setText(QApplication::translate("SensorVisualizationUi", "Magnetometer [mG]", Q_NULLPTR));
        label_imu_y->setText(QApplication::translate("SensorVisualizationUi", "Y", Q_NULLPTR));
        label_imu_z->setText(QApplication::translate("SensorVisualizationUi", "Z", Q_NULLPTR));
        label_imu_angular->setText(QApplication::translate("SensorVisualizationUi", "Angular Rate [deg/s]", Q_NULLPTR));
        label_imu_acc->setText(QApplication::translate("SensorVisualizationUi", "Acceleration [g]", Q_NULLPTR));
        label_imu_update->setText(QApplication::translate("SensorVisualizationUi", "Update Rate [Hz]", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("SensorVisualizationUi", "Inertial Measurement Unit", Q_NULLPTR));
        label_sensors_cell4->setText(QApplication::translate("SensorVisualizationUi", "Cell 4 [mV]", Q_NULLPTR));
        label_volt_actuator_overall->setText(QApplication::translate("SensorVisualizationUi", "Overall [mV]", Q_NULLPTR));
        label_sensors_cell2->setText(QApplication::translate("SensorVisualizationUi", "Cell 2 [mV]", Q_NULLPTR));
        label_sensors_cell1->setText(QApplication::translate("SensorVisualizationUi", "Cell 1 [mV]", Q_NULLPTR));
        label_sensors_cell3->setText(QApplication::translate("SensorVisualizationUi", "Cell 3 [mV]", Q_NULLPTR));
        label_sensors_cell6->setText(QApplication::translate("SensorVisualizationUi", "Cell 6 [mV]", Q_NULLPTR));
        label_sensors_cell5->setText(QApplication::translate("SensorVisualizationUi", "Cell 5 [mV]", Q_NULLPTR));
        label_actuator_cell1->setText(QApplication::translate("SensorVisualizationUi", "Cell 1 [mV]", Q_NULLPTR));
        label_volt_actuator->setText(QApplication::translate("SensorVisualizationUi", "Actuator", Q_NULLPTR));
        label_volt_sensors->setText(QApplication::translate("SensorVisualizationUi", "Sensors/Pc", Q_NULLPTR));
        label_actuator_cell2->setText(QApplication::translate("SensorVisualizationUi", "Cell 2 [mV]", Q_NULLPTR));
        label_volt_update->setText(QApplication::translate("SensorVisualizationUi", "Update Rate [Hz]", Q_NULLPTR));
        label_volt_sensors_overall->setText(QApplication::translate("SensorVisualizationUi", "Overall [mV]", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QApplication::translate("SensorVisualizationUi", "Volt Measurement", Q_NULLPTR));
        label_ls_samples->setText(QApplication::translate("SensorVisualizationUi", "samples per scan", Q_NULLPTR));
        label_ls_max_dist->setText(QApplication::translate("SensorVisualizationUi", "laser scanner scaling factor", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab_6), QApplication::translate("SensorVisualizationUi", "US/LS Visual", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SensorVisualizationUi: public Ui_SensorVisualizationUi {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SENSOR_VISUALIZATION_H

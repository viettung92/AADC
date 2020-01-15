/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionExit;
    QAction *actionClear_all;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QGroupBox *groupBox_3;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_2;
    QLabel *label_4;
    QLabel *lblCarState;
    QGraphicsView *viewStateCar;
    QGroupBox *groupBox_2;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *btGetReady;
    QPushButton *btStart;
    QPushButton *btStop;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_3;
    QLabel *label_6;
    QComboBox *cmBoxSections;
    QComboBox *cmBoxManeuvers;
    QLabel *label_7;
    QLabel *label;
    QGroupBox *groupBox;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QFormLayout *formLayout;
    QLabel *label_2;
    QLineEdit *lEdIpAddress;
    QLabel *label_3;
    QLineEdit *lEdPort;
    QPushButton *btConnect;
    QGraphicsView *viewStateConnection;
    QGroupBox *groupBox_4;
    QPushButton *btSendManeuver;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QLineEdit *lEdManeuverFile;
    QPushButton *btSetManeuverFile;
    QPushButton *btLoadManeuver;
    QGraphicsView *viewStateFile;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(448, 666);
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionClear_all = new QAction(MainWindow);
        actionClear_all->setObjectName(QStringLiteral("actionClear_all"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_3->sizePolicy().hasHeightForWidth());
        groupBox_3->setSizePolicy(sizePolicy);
        groupBox_3->setMinimumSize(QSize(0, 100));
        gridLayoutWidget = new QWidget(groupBox_3);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(100, 10, 191, 80));
        gridLayout_2 = new QGridLayout(gridLayoutWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(gridLayoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        QFont font;
        font.setPointSize(15);
        label_4->setFont(font);

        gridLayout_2->addWidget(label_4, 0, 0, 1, 1);

        lblCarState = new QLabel(gridLayoutWidget);
        lblCarState->setObjectName(QStringLiteral("lblCarState"));
        lblCarState->setFont(font);

        gridLayout_2->addWidget(lblCarState, 0, 1, 1, 1);

        viewStateCar = new QGraphicsView(groupBox_3);
        viewStateCar->setObjectName(QStringLiteral("viewStateCar"));
        viewStateCar->setGeometry(QRect(350, 30, 50, 50));

        gridLayout->addWidget(groupBox_3, 5, 0, 1, 1);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        sizePolicy.setHeightForWidth(groupBox_2->sizePolicy().hasHeightForWidth());
        groupBox_2->setSizePolicy(sizePolicy);
        groupBox_2->setMinimumSize(QSize(0, 180));
        verticalLayoutWidget = new QWidget(groupBox_2);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(230, 20, 161, 151));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        btGetReady = new QPushButton(verticalLayoutWidget);
        btGetReady->setObjectName(QStringLiteral("btGetReady"));

        verticalLayout->addWidget(btGetReady);

        btStart = new QPushButton(verticalLayoutWidget);
        btStart->setObjectName(QStringLiteral("btStart"));

        verticalLayout->addWidget(btStart);

        btStop = new QPushButton(verticalLayoutWidget);
        btStop->setObjectName(QStringLiteral("btStop"));

        verticalLayout->addWidget(btStop);

        gridLayoutWidget_2 = new QWidget(groupBox_2);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(40, 50, 160, 80));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        label_6 = new QLabel(gridLayoutWidget_2);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout_3->addWidget(label_6, 0, 0, 1, 1);

        cmBoxSections = new QComboBox(gridLayoutWidget_2);
        cmBoxSections->setObjectName(QStringLiteral("cmBoxSections"));

        gridLayout_3->addWidget(cmBoxSections, 0, 1, 1, 1);

        cmBoxManeuvers = new QComboBox(gridLayoutWidget_2);
        cmBoxManeuvers->setObjectName(QStringLiteral("cmBoxManeuvers"));

        gridLayout_3->addWidget(cmBoxManeuvers, 1, 1, 1, 1);

        label_7 = new QLabel(gridLayoutWidget_2);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_3->addWidget(label_7, 1, 0, 1, 1);


        gridLayout->addWidget(groupBox_2, 4, 0, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setPixmap(QPixmap(QString::fromUtf8(":/resources/helloworld-transparent.png")));
        label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label, 2, 2, 1, 1);

        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy1.setHorizontalStretch(100);
        sizePolicy1.setVerticalStretch(100);
        sizePolicy1.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy1);
        groupBox->setMinimumSize(QSize(250, 150));
        verticalLayoutWidget_2 = new QWidget(groupBox);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(110, 20, 211, 91));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        formLayout = new QFormLayout();
        formLayout->setSpacing(6);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        label_2 = new QLabel(verticalLayoutWidget_2);
        label_2->setObjectName(QStringLiteral("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        lEdIpAddress = new QLineEdit(verticalLayoutWidget_2);
        lEdIpAddress->setObjectName(QStringLiteral("lEdIpAddress"));

        formLayout->setWidget(1, QFormLayout::FieldRole, lEdIpAddress);

        label_3 = new QLabel(verticalLayoutWidget_2);
        label_3->setObjectName(QStringLiteral("label_3"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_3);

        lEdPort = new QLineEdit(verticalLayoutWidget_2);
        lEdPort->setObjectName(QStringLiteral("lEdPort"));

        formLayout->setWidget(2, QFormLayout::FieldRole, lEdPort);


        verticalLayout_2->addLayout(formLayout);

        btConnect = new QPushButton(verticalLayoutWidget_2);
        btConnect->setObjectName(QStringLiteral("btConnect"));

        verticalLayout_2->addWidget(btConnect);

        viewStateConnection = new QGraphicsView(groupBox);
        viewStateConnection->setObjectName(QStringLiteral("viewStateConnection"));
        viewStateConnection->setGeometry(QRect(350, 90, 50, 50));

        gridLayout->addWidget(groupBox, 0, 0, 1, 1);

        groupBox_4 = new QGroupBox(centralWidget);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        sizePolicy1.setHeightForWidth(groupBox_4->sizePolicy().hasHeightForWidth());
        groupBox_4->setSizePolicy(sizePolicy1);
        groupBox_4->setMinimumSize(QSize(100, 120));
        btSendManeuver = new QPushButton(groupBox_4);
        btSendManeuver->setObjectName(QStringLiteral("btSendManeuver"));
        btSendManeuver->setGeometry(QRect(110, 100, 200, 23));
        horizontalLayoutWidget = new QWidget(groupBox_4);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 30, 401, 31));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        lEdManeuverFile = new QLineEdit(horizontalLayoutWidget);
        lEdManeuverFile->setObjectName(QStringLiteral("lEdManeuverFile"));

        horizontalLayout->addWidget(lEdManeuverFile);

        btSetManeuverFile = new QPushButton(horizontalLayoutWidget);
        btSetManeuverFile->setObjectName(QStringLiteral("btSetManeuverFile"));

        horizontalLayout->addWidget(btSetManeuverFile);

        btLoadManeuver = new QPushButton(groupBox_4);
        btLoadManeuver->setObjectName(QStringLiteral("btLoadManeuver"));
        btLoadManeuver->setGeometry(QRect(110, 70, 200, 23));
        viewStateFile = new QGraphicsView(groupBox_4);
        viewStateFile->setObjectName(QStringLiteral("viewStateFile"));
        viewStateFile->setGeometry(QRect(350, 80, 50, 50));

        gridLayout->addWidget(groupBox_4, 3, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 448, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        MainWindow->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuFile->addAction(actionClear_all);
        menuFile->addAction(actionExit);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "AADC2018 Jury Application 1.0", Q_NULLPTR));
        actionExit->setText(QApplication::translate("MainWindow", "E&xit", Q_NULLPTR));
        actionClear_all->setText(QApplication::translate("MainWindow", "Clear all", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "Car", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "State:", Q_NULLPTR));
        lblCarState->setText(QApplication::translate("MainWindow", "TextLabel", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Commands", Q_NULLPTR));
        btGetReady->setText(QApplication::translate("MainWindow", "GetReady", Q_NULLPTR));
        btStart->setText(QApplication::translate("MainWindow", "Start", Q_NULLPTR));
        btStop->setText(QApplication::translate("MainWindow", "Stop", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "Section:", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "Maneuver:", Q_NULLPTR));
        label->setText(QString());
        groupBox->setTitle(QApplication::translate("MainWindow", "Connection", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "IP address", Q_NULLPTR));
        lEdIpAddress->setText(QApplication::translate("MainWindow", "127.0.0.1", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "Port", Q_NULLPTR));
        lEdPort->setText(QApplication::translate("MainWindow", "1234", Q_NULLPTR));
        btConnect->setText(QApplication::translate("MainWindow", "Connect", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "Maneuver file", Q_NULLPTR));
        btSendManeuver->setText(QApplication::translate("MainWindow", "Send maneuver file", Q_NULLPTR));
        btSetManeuverFile->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        btLoadManeuver->setText(QApplication::translate("MainWindow", "Load maneuver file", Q_NULLPTR));
        menuFile->setTitle(QApplication::translate("MainWindow", "&File", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

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
#include <QtWidgets/QTextEdit>
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
    QGroupBox *groupBox_4;
    QWidget *horizontalLayoutWidget;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout;
    QLabel *label_9;
    QLineEdit *lEdManeuverFile;
    QPushButton *btSetManeuverFile;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *btLoadManeuver;
    QPushButton *btSendManeuver;
    QGraphicsView *viewStateFile;
    QWidget *horizontalLayoutWidget_4;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_8;
    QLineEdit *lEdRoadSignMap;
    QPushButton *btSetRoadSignMap;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *btSendRoadSignMap;
    QWidget *horizontalLayoutWidget_5;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_5;
    QLineEdit *lEdOpenDriveMap;
    QPushButton *btSetOpenDriveMap;
    QHBoxLayout *horizontalLayout_8;
    QPushButton *btSendOpenDriveMap;
    QGraphicsView *viewStateManeuverFileSent;
    QGraphicsView *viewStateRoadSignMapSent;
    QGraphicsView *viewStateOpenDriveMapSent;
    QGroupBox *groupBox;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QFormLayout *formLayout;
    QLabel *label_2;
    QLineEdit *lEdIpAddress;
    QLabel *label_3;
    QLineEdit *lEdPort;
    QPushButton *btConnect;
    QPushButton *btDisconnect;
    QGraphicsView *viewStateConnection;
    QLabel *label;
    QGroupBox *groupBox_3;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_2;
    QLabel *label_4;
    QLabel *lblCarState;
    QGraphicsView *viewStateCar;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_4;
    QTextEdit *logField;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_2;
    QGridLayout *gridLayout_3;
    QLabel *label_6;
    QComboBox *cmBoxSections;
    QComboBox *cmBoxManeuvers;
    QLabel *label_7;
    QVBoxLayout *verticalLayout;
    QPushButton *btGetReady;
    QPushButton *btStart;
    QPushButton *btStop;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(545, 907);
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
        groupBox_4 = new QGroupBox(centralWidget);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(100);
        sizePolicy.setVerticalStretch(100);
        sizePolicy.setHeightForWidth(groupBox_4->sizePolicy().hasHeightForWidth());
        groupBox_4->setSizePolicy(sizePolicy);
        groupBox_4->setMinimumSize(QSize(100, 260));
        horizontalLayoutWidget = new QWidget(groupBox_4);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 30, 431, 58));
        verticalLayout_4 = new QVBoxLayout(horizontalLayoutWidget);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_9 = new QLabel(horizontalLayoutWidget);
        label_9->setObjectName(QStringLiteral("label_9"));

        horizontalLayout->addWidget(label_9);

        lEdManeuverFile = new QLineEdit(horizontalLayoutWidget);
        lEdManeuverFile->setObjectName(QStringLiteral("lEdManeuverFile"));

        horizontalLayout->addWidget(lEdManeuverFile);

        btSetManeuverFile = new QPushButton(horizontalLayoutWidget);
        btSetManeuverFile->setObjectName(QStringLiteral("btSetManeuverFile"));

        horizontalLayout->addWidget(btSetManeuverFile);


        verticalLayout_4->addLayout(horizontalLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        btLoadManeuver = new QPushButton(horizontalLayoutWidget);
        btLoadManeuver->setObjectName(QStringLiteral("btLoadManeuver"));

        horizontalLayout_3->addWidget(btLoadManeuver);

        btSendManeuver = new QPushButton(horizontalLayoutWidget);
        btSendManeuver->setObjectName(QStringLiteral("btSendManeuver"));

        horizontalLayout_3->addWidget(btSendManeuver);


        verticalLayout_4->addLayout(horizontalLayout_3);

        viewStateFile = new QGraphicsView(groupBox_4);
        viewStateFile->setObjectName(QStringLiteral("viewStateFile"));
        viewStateFile->setGeometry(QRect(450, 30, 50, 21));
        viewStateFile->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        viewStateFile->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        horizontalLayoutWidget_4 = new QWidget(groupBox_4);
        horizontalLayoutWidget_4->setObjectName(QStringLiteral("horizontalLayoutWidget_4"));
        horizontalLayoutWidget_4->setGeometry(QRect(10, 90, 431, 93));
        verticalLayout_5 = new QVBoxLayout(horizontalLayoutWidget_4);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_8 = new QLabel(horizontalLayoutWidget_4);
        label_8->setObjectName(QStringLiteral("label_8"));

        horizontalLayout_5->addWidget(label_8);

        lEdRoadSignMap = new QLineEdit(horizontalLayoutWidget_4);
        lEdRoadSignMap->setObjectName(QStringLiteral("lEdRoadSignMap"));

        horizontalLayout_5->addWidget(lEdRoadSignMap);

        btSetRoadSignMap = new QPushButton(horizontalLayoutWidget_4);
        btSetRoadSignMap->setObjectName(QStringLiteral("btSetRoadSignMap"));

        horizontalLayout_5->addWidget(btSetRoadSignMap);


        verticalLayout_5->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        btSendRoadSignMap = new QPushButton(horizontalLayoutWidget_4);
        btSendRoadSignMap->setObjectName(QStringLiteral("btSendRoadSignMap"));

        horizontalLayout_6->addWidget(btSendRoadSignMap);


        verticalLayout_5->addLayout(horizontalLayout_6);

        horizontalLayoutWidget_5 = new QWidget(groupBox_4);
        horizontalLayoutWidget_5->setObjectName(QStringLiteral("horizontalLayoutWidget_5"));
        horizontalLayoutWidget_5->setGeometry(QRect(10, 190, 431, 58));
        verticalLayout_6 = new QVBoxLayout(horizontalLayoutWidget_5);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        label_5 = new QLabel(horizontalLayoutWidget_5);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout_7->addWidget(label_5);

        lEdOpenDriveMap = new QLineEdit(horizontalLayoutWidget_5);
        lEdOpenDriveMap->setObjectName(QStringLiteral("lEdOpenDriveMap"));

        horizontalLayout_7->addWidget(lEdOpenDriveMap);

        btSetOpenDriveMap = new QPushButton(horizontalLayoutWidget_5);
        btSetOpenDriveMap->setObjectName(QStringLiteral("btSetOpenDriveMap"));

        horizontalLayout_7->addWidget(btSetOpenDriveMap);


        verticalLayout_6->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        btSendOpenDriveMap = new QPushButton(horizontalLayoutWidget_5);
        btSendOpenDriveMap->setObjectName(QStringLiteral("btSendOpenDriveMap"));

        horizontalLayout_8->addWidget(btSendOpenDriveMap);


        verticalLayout_6->addLayout(horizontalLayout_8);

        viewStateManeuverFileSent = new QGraphicsView(groupBox_4);
        viewStateManeuverFileSent->setObjectName(QStringLiteral("viewStateManeuverFileSent"));
        viewStateManeuverFileSent->setGeometry(QRect(450, 60, 50, 21));
        viewStateManeuverFileSent->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        viewStateManeuverFileSent->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        viewStateRoadSignMapSent = new QGraphicsView(groupBox_4);
        viewStateRoadSignMapSent->setObjectName(QStringLiteral("viewStateRoadSignMapSent"));
        viewStateRoadSignMapSent->setGeometry(QRect(450, 120, 50, 50));
        viewStateOpenDriveMapSent = new QGraphicsView(groupBox_4);
        viewStateOpenDriveMapSent->setObjectName(QStringLiteral("viewStateOpenDriveMapSent"));
        viewStateOpenDriveMapSent->setGeometry(QRect(450, 190, 50, 50));

        gridLayout->addWidget(groupBox_4, 1, 0, 1, 1);

        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        sizePolicy.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy);
        groupBox->setMinimumSize(QSize(250, 150));
        verticalLayoutWidget_2 = new QWidget(groupBox);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(110, 20, 211, 114));
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

        btDisconnect = new QPushButton(verticalLayoutWidget_2);
        btDisconnect->setObjectName(QStringLiteral("btDisconnect"));

        verticalLayout_2->addWidget(btDisconnect);

        viewStateConnection = new QGraphicsView(groupBox);
        viewStateConnection->setObjectName(QStringLiteral("viewStateConnection"));
        viewStateConnection->setGeometry(QRect(350, 30, 50, 50));

        gridLayout->addWidget(groupBox, 0, 0, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setPixmap(QPixmap(QString::fromUtf8(":/resources/helloworld-transparent.png")));
        label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label, 2, 2, 1, 1);

        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        sizePolicy.setHeightForWidth(groupBox_3->sizePolicy().hasHeightForWidth());
        groupBox_3->setSizePolicy(sizePolicy);
        groupBox_3->setMinimumSize(QSize(0, 300));
        gridLayoutWidget = new QWidget(groupBox_3);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(100, 10, 227, 73));
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

        viewStateCar = new QGraphicsView(gridLayoutWidget);
        viewStateCar->setObjectName(QStringLiteral("viewStateCar"));

        gridLayout_2->addWidget(viewStateCar, 0, 2, 1, 1);

        horizontalLayoutWidget_2 = new QWidget(groupBox_3);
        horizontalLayoutWidget_2->setObjectName(QStringLiteral("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(0, 100, 511, 191));
        horizontalLayout_4 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        logField = new QTextEdit(horizontalLayoutWidget_2);
        logField->setObjectName(QStringLiteral("logField"));
        logField->setEnabled(true);
        logField->setFocusPolicy(Qt::ClickFocus);
        logField->setTextInteractionFlags(Qt::TextSelectableByKeyboard|Qt::TextSelectableByMouse);

        horizontalLayout_4->addWidget(logField);


        gridLayout->addWidget(groupBox_3, 5, 0, 1, 1);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        sizePolicy.setHeightForWidth(groupBox_2->sizePolicy().hasHeightForWidth());
        groupBox_2->setSizePolicy(sizePolicy);
        groupBox_2->setMinimumSize(QSize(0, 120));
        horizontalLayout_2 = new QHBoxLayout(groupBox_2);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setSpacing(6);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout_3->addWidget(label_6, 0, 0, 1, 1);

        cmBoxSections = new QComboBox(groupBox_2);
        cmBoxSections->setObjectName(QStringLiteral("cmBoxSections"));

        gridLayout_3->addWidget(cmBoxSections, 0, 1, 1, 1);

        cmBoxManeuvers = new QComboBox(groupBox_2);
        cmBoxManeuvers->setObjectName(QStringLiteral("cmBoxManeuvers"));

        gridLayout_3->addWidget(cmBoxManeuvers, 1, 1, 1, 1);

        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_3->addWidget(label_7, 1, 0, 1, 1);


        horizontalLayout_2->addLayout(gridLayout_3);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        btGetReady = new QPushButton(groupBox_2);
        btGetReady->setObjectName(QStringLiteral("btGetReady"));

        verticalLayout->addWidget(btGetReady);

        btStart = new QPushButton(groupBox_2);
        btStart->setObjectName(QStringLiteral("btStart"));

        verticalLayout->addWidget(btStart);

        btStop = new QPushButton(groupBox_2);
        btStop->setObjectName(QStringLiteral("btStop"));

        verticalLayout->addWidget(btStop);


        horizontalLayout_2->addLayout(verticalLayout);


        gridLayout->addWidget(groupBox_2, 2, 0, 3, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 545, 21));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "AADC2018 Jury Application 1.1", Q_NULLPTR));
        actionExit->setText(QApplication::translate("MainWindow", "E&xit", Q_NULLPTR));
        actionClear_all->setText(QApplication::translate("MainWindow", "Clear all", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "Files", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindow", "Maneuver file", Q_NULLPTR));
        btSetManeuverFile->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        btLoadManeuver->setText(QApplication::translate("MainWindow", "Load maneuver file", Q_NULLPTR));
        btSendManeuver->setText(QApplication::translate("MainWindow", "Send maneuver file", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindow", "Road sign map", Q_NULLPTR));
        btSetRoadSignMap->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        btSendRoadSignMap->setText(QApplication::translate("MainWindow", "Send road sign map", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "OpenDrive map", Q_NULLPTR));
        btSetOpenDriveMap->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        btSendOpenDriveMap->setText(QApplication::translate("MainWindow", "Send OpenDrive map", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("MainWindow", "Connection", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "IP address", Q_NULLPTR));
        lEdIpAddress->setText(QApplication::translate("MainWindow", "127.0.0.1", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "Port", Q_NULLPTR));
        lEdPort->setText(QApplication::translate("MainWindow", "1234", Q_NULLPTR));
        btConnect->setText(QApplication::translate("MainWindow", "Connect", Q_NULLPTR));
        btDisconnect->setText(QApplication::translate("MainWindow", "Disconnect", Q_NULLPTR));
        label->setText(QString());
        groupBox_3->setTitle(QApplication::translate("MainWindow", "Car", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "State:", Q_NULLPTR));
        lblCarState->setText(QApplication::translate("MainWindow", "no state", Q_NULLPTR));
        logField->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Log messages:</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Commands", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "Section:", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "Maneuver:", Q_NULLPTR));
        btGetReady->setText(QApplication::translate("MainWindow", "GetReady", Q_NULLPTR));
        btStart->setText(QApplication::translate("MainWindow", "Start", Q_NULLPTR));
        btStop->setText(QApplication::translate("MainWindow", "Stop", Q_NULLPTR));
        menuFile->setTitle(QApplication::translate("MainWindow", "&File", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

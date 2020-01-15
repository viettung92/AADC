/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#include "CarControllerWidget.h"

cCarControllerWidget::cCarControllerWidget(QWidget *parent) : QWidget(parent), m_ui(new Ui_CarControllerUi)
{
    m_ui->setupUi(this);

    // Rearrange buttons id 0...5 for lights
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(1), 0); // Head
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(2), 1); // Brake
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(4), 2); // Reverse
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(5), 3); // Hazard
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(3), 4); // TurnLeft
    m_ui->buttonGroup_lights->setId(m_ui->buttonGroup_lights->buttons().at(0), 5); // TurnRight

    {
        const QRegExp re("pushButton_lc_action(\\d+)");
        foreach(QPushButton *button, findChildren<QPushButton*>(re)) {
            (void) re.indexIn(button->objectName());  // we know it matches, but we need the captured text
            const int id = re.cap(1).toInt();
            m_ui->buttonGroup_lf_actions->setId(button, id); // assuming the button is already in the button group
        }
    }


    {
        const QRegExp re("pushButton_fp_action(\\d+)");
        foreach(QPushButton *button, findChildren<QPushButton*>(re)) {
            (void) re.indexIn(button->objectName());  // we know it matches, but we need the captured text
            const int id = re.cap(1).toInt();
            m_ui->buttonGroup_mtp_actions->setId(button, id); // assuming the button is already in the button group
        }
    }

    m_ui->buttonGroup_as_actions->setId(m_ui->buttonGroup_as_actions->buttons().at(0), 1);
    m_ui->buttonGroup_as_actions->setId(m_ui->buttonGroup_as_actions->buttons().at(1), 2);

    {
        const QRegExp re("buttonGroup_usacc_action(\\d+)");
        foreach(QPushButton *button, findChildren<QPushButton*>(re)) {
            (void) re.indexIn(button->objectName());  // we know it matches, but we need the captured text
            const int id = re.cap(1).toInt();
            m_ui->buttonGroup_usacc_actions->setId(button, id); // assuming the button is already in the button group
        }
    }


    // Rearrange buttons id 39... 41 for LaneDetection filter actions
    m_ui->buttonGroup_ld_actions->setId(m_ui->buttonGroup_ld_actions->buttons().at(0), 1);
    m_ui->buttonGroup_ld_actions->setId(m_ui->buttonGroup_ld_actions->buttons().at(1), 2);
    m_ui->buttonGroup_ld_actions->setId(m_ui->buttonGroup_ld_actions->buttons().at(2), 3);


    {
        const QRegExp re("pushButton_arb_action(\\d+)");
        foreach(QPushButton *button, findChildren<QPushButton*>(re)) {
            (void) re.indexIn(button->objectName());  // we know it matches, but we need the captured text
            const int id = re.cap(1).toInt();
            m_ui->buttonGroup_arb_actions->setId(button, id); // assuming the button is already in the button group
        }
    }

    /*Arbitrary Command*/

    m_ui->lineEdit_arbCom->setText("42");
    m_ui->doubleSpinBox_posX->setValue(1.0);
    m_ui->doubleSpinBox_speed->setValue(0.6);

    m_ui->doubleSpinBox_posX->setMaximum(5.0);
    m_ui->doubleSpinBox_posX->setMinimum(-5.0);
    m_ui->doubleSpinBox_posX->setSingleStep(0.1);
    m_ui->doubleSpinBox_posY->setMaximum(5.0);
    m_ui->doubleSpinBox_posY->setMinimum(-5.0);
    m_ui->doubleSpinBox_posY->setSingleStep(0.1);
    m_ui->doubleSpinBox_yaw->setMaximum(6.28);
    m_ui->doubleSpinBox_yaw->setMinimum(-6.28);
    m_ui->doubleSpinBox_yaw->setSingleStep(0.1);
    m_ui->doubleSpinBox_speed->setMaximum(1.0);
    m_ui->doubleSpinBox_speed->setMinimum(0.0);
    m_ui->doubleSpinBox_speed->setSingleStep(0.1);


    m_timer.setInterval(50);
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
    m_timer.start();
}


cCarControllerWidget::~cCarControllerWidget()
{
    delete m_ui;
}

void cCarControllerWidget::setSpeed(int value)
{
    m_ui->lcdNumber_throttle->display(value);
    emit throttleReceived((float) value);
}

void cCarControllerWidget::setSteering(int value)
{
    m_ui->lcdNumber_steering->display(value);
    emit steeringReceived(value);
}

QButtonGroup* cCarControllerWidget::getLightButtonGroup()
{
    return m_ui->buttonGroup_lights;
}

QButtonGroup* cCarControllerWidget::getArbitraryButtonGroup()
{
    return m_ui->buttonGroup_arb_actions;
}
QButtonGroup* cCarControllerWidget::getLightActionButtonGroup()
{
    return m_ui->buttonGroup_lf_actions;
}
QButtonGroup* cCarControllerWidget::getMTPActionButtonGroup()
{
    return m_ui->buttonGroup_mtp_actions;
}

QButtonGroup* cCarControllerWidget::getASActionButtonGroup()
{
    return m_ui->buttonGroup_as_actions;
}

QButtonGroup* cCarControllerWidget::getUSACCActionButtonGroup()
{
    return m_ui->buttonGroup_usacc_actions;
}

QButtonGroup* cCarControllerWidget::getLDActionButtonGroup()
{
    return m_ui->buttonGroup_ld_actions;
}

void cCarControllerWidget::update()
{
    if (m_ui->radioButton_enable_resetting_slider->isChecked())
    {
        int steering = m_ui->horizontalSlider_steering->value();
        int speed = m_ui->verticalSlider_throttle->value();

        if (steering > 0)
            steering--;
        if (steering < 0)
            steering++;

        if (speed > 0)
            speed--;
        if (speed < 0)
            speed++;

        m_ui->horizontalSlider_steering->setValue(steering);
        m_ui->verticalSlider_throttle->setValue(speed);
    }

    setSteering(m_ui->horizontalSlider_steering->value());
    setSpeed(m_ui->verticalSlider_throttle->value());
}


int cCarControllerWidget::getCarPose(TPoseStruct::Data &ps){
    ps.f32PosX = m_ui->doubleSpinBox_posX->value();
    ps.f32PosY = m_ui->doubleSpinBox_posY->value();
    ps.f32Yaw = m_ui->doubleSpinBox_yaw->value();
    ps.f32CarSpeed = m_ui->doubleSpinBox_speed->value();
    ps.f32Radius = m_ui->doubleSpinBox_cutOff->value(); //wird missbruacht als cutoff container
    return m_ui->comboBox_gotoCom->currentIndex();
}

int cCarControllerWidget::getArbitraryCommand(){
    return m_ui->lineEdit_arbCom->text().toInt();
}

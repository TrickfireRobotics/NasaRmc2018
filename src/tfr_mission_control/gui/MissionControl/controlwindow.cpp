#include "controlwindow.h"
#include "ui_controlwindow.h"

ControlWindow::ControlWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControlWindow)
{
    ui->setupUi(this);
    connect(ui->fwButton, SIGNAL(clicked()),this, SLOT(testClick()));
}

void ControlWindow::testClick()
{
    ui->fwButton->setText("clicked");
}

ControlWindow::~ControlWindow()
{
    delete ui;
}

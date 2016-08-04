#include "qtfile.h"


using namespace std;

MainWindow::MainWindow(){
    slider_vl = new QSlider(Qt::Horizontal); 
    slider_vl->setRange(0, 100);
    lineEdit_vl = new QLineEdit();
    QObject::connect(slider_vl, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged_vl(int)));

    slider_vh = new QSlider(Qt::Horizontal); 
    slider_vh->setRange(0, 100);
    lineEdit_vh = new QLineEdit();
    QObject::connect(slider_vh, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged_vh(int)));

    slider_sl = new QSlider(Qt::Horizontal); 
    slider_sl->setRange(0, 100);
    lineEdit_sl = new QLineEdit();
    QObject::connect(slider_sl, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged_sl(int)));

    slider_sh = new QSlider(Qt::Horizontal); 
    slider_sh->setRange(0, 100);
    lineEdit_sh = new QLineEdit();
    QObject::connect(slider_sh, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged_sh(int)));

    slider_hl = new QSlider(Qt::Horizontal); 
    slider_hl->setRange(0, 500);
    lineEdit_hl = new QLineEdit();
    QObject::connect(slider_hl, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged_hl(int)));

    slider_hh = new QSlider(Qt::Horizontal); 
    slider_hh->setRange(0, 500);
    lineEdit_hh = new QLineEdit();
    QObject::connect(slider_hh, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged_hh(int)));
 
    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(slider_vl);
    layout->addWidget(lineEdit_vl);
        layout->addWidget(slider_vh);
    layout->addWidget(lineEdit_vh);
        layout->addWidget(slider_sl);
    layout->addWidget(lineEdit_sl);
        layout->addWidget(slider_sh);
    layout->addWidget(lineEdit_sh);
        layout->addWidget(slider_hl);
    layout->addWidget(lineEdit_hl);
        layout->addWidget(slider_hh);
    layout->addWidget(lineEdit_hh);
 
    QWidget *wrapper = new QWidget();
    wrapper->setLayout(layout);
    setCentralWidget(wrapper);
    //Set initial values
    slider_sl->setValue(80);
    slider_sh->setValue(110);
    slider_vl->setValue(50);
    slider_vh->setValue(110);
    slider_hl->setValue(180);
    slider_hh->setValue(250);

}
 
MainWindow::~MainWindow(){}


void MainWindow::onValueChanged_vl(int value){
    int pos = slider_vl->sliderPosition();

    myfile.open ("pipe.txt");
    myfile << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "\n";
    myfile.close();

    QString text = "Value Low " + QString::number(pos);
    lineEdit_vl->setText(text);
}
void MainWindow::onValueChanged_vh(int value){
    int pos = slider_vh->sliderPosition();

    myfile.open ("pipe.txt");
    myfile << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "\n";
    myfile.close();

    QString text = "Value High " + QString::number(pos);
    lineEdit_vh->setText(text);
}
void MainWindow::onValueChanged_sl(int value){
    int pos = slider_sl->sliderPosition();

    myfile.open ("pipe.txt");
    myfile << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "\n";
    myfile.close();

    QString text = "Saturation Low " + QString::number(pos);
    lineEdit_sl->setText(text);
}
void MainWindow::onValueChanged_sh(int value){
    int pos = slider_sh->sliderPosition();

    myfile.open ("pipe.txt");
    myfile << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "\n";
    myfile.close();

    QString text = "Saturation High " + QString::number(pos);
    lineEdit_sh->setText(text);
}
void MainWindow::onValueChanged_hl(int value){
    int pos = slider_hl->sliderPosition();

    myfile.open ("pipe.txt");
    myfile << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "\n";
    myfile.close();

    QString text = "Hue Low " + QString::number(pos);
    lineEdit_hl->setText(text);
}
void MainWindow::onValueChanged_hh(int value){
    int pos = slider_hh->sliderPosition();

    myfile.open ("pipe.txt");
    myfile << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "\n";
    myfile.close();
    
    QString text = "Hue High " + QString::number(pos);
    lineEdit_hh->setText(text);
}
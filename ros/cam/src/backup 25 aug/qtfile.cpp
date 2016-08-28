#include "qtfile.h"


using namespace std;
static void HSVToRGB(float h, float s, float v, float *r, float *g, float *b)
{
    if (h < 0) h = 0;
    if (h > 359) h = 359;
    if (s < 0) s = 0;
    if (s > 1) s = 100;
    if (v < 0) v = 0;
    if (v > 1) v = 100;

    float tmp = h/60.0;
    int hi = floor(tmp);
    float f = tmp - hi;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);
    
    switch (hi) {
        case 0:
            *r = v;
            *g = t;
            *b = p;
            break;
        case 1:
            *r = q;
            *g = v;
            *b = p;
            break;
        case 2:
            *r = p;
            *g = v;
            *b = t;
            break;
        case 3:
            *r = p;
            *g = q;
            *b = v;
            break;
        case 4:
            *r = t;
            *g = p;
            *b = v;
            break;
        case 5:
            *r = v;
            *g = p;
            *b = q;
            break;
    }
}
MainWindow::MainWindow(){
    printf("Started constructor\n");
    toggle = 0;

    resize(QDesktopWidget().availableGeometry(this).size() * 0.2);

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
    
    but = new QPushButton("set", this);
    but->setGeometry(QRect(QPoint(90, 250),
    QSize(30, 30)));
    QObject::connect(but, SIGNAL (released()), this, SLOT (handleButton()));
    layout->addWidget(but);

    bar = new QProgressBar();
    bar->setStyleSheet("::chunk {"
                    "background-color: "
                    "qlineargradient(x0: 0, x2: 1, "
                    "stop: 0 green, stop: 0.6 green, "
                    "stop: 0.8 orange, "
                    "stop: 1 red"
                    ")}");
    int val = bar->value();
    int pos = QStyle::sliderPositionFromValue(bar->minimum(), bar->maximum(), val, bar->width());

    QPainter p(bar);

    QLinearGradient linearGrad(bar->rect().topLeft(), bar->rect().bottomRight());
    linearGrad.setColorAt(0, Qt::red);
    linearGrad.setColorAt(0.2, QColor(255, 165, 0));
    linearGrad.setColorAt(1, Qt::green);
    QRect rect_linear(bar->rect().topLeft(), bar->rect().bottomRight());
    p.fillRect(rect_linear, linearGrad);

    p.setPen(Qt::lightGray);
    p.setBrush(QBrush(Qt::lightGray));
    p.drawRect(pos, 0, width(), height());
    bar->setValue(100);
    bar->setTextVisible(false);
    
    layout->addWidget(bar);
    printf("ended cons\n");
}
 
MainWindow::~MainWindow(){}

void MainWindow::handleButton(){
    toggle = 1;

    myfile.open ("pipe.txt");
    myfile  << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "|"
            << toggle << "\n";
    myfile.close();

    //Close window and put toggle to invalid number just before closing
    toggle = 2;
    close();

}

void MainWindow::onValueChanged_vl(int value){
    int pos = slider_vl->sliderPosition();



    float r;
    float g;
    float b;
    float h = 0;
    float s = 0;
    float v = pos/100.0;
    HSVToRGB(h,s,v,&r,&g,&b);
    int red = (int)255*r;
    int green = (int)255*g;
    int blue = (int)255*b;
    std::cout << red << " " << green << " " << blue << endl;
    QString ss = "::sub-page:horizontal {"
                    "background-color: "
                    "qlineargradient(x0: 0, x2: 1, "
                    "stop: 0 rgba(%1, %2, %3, 255)"
                    ")}";
    slider_vl->setStyleSheet(ss.arg(red).arg(green).arg(blue));




    myfile.open ("pipe.txt");
    myfile  << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "|"
            << toggle << "\n";
    myfile.close();

    QString text = "Value Low " + QString::number(pos);
    lineEdit_vl->setText(text);
}
void MainWindow::onValueChanged_vh(int value){
    int pos = slider_vh->sliderPosition();


    float r;
    float g;
    float b;
    float h = 0;
    float s = 0;
    float v = pos/100.0;
    HSVToRGB(h,s,v,&r,&g,&b);
    int red = (int)255*r;
    int green = (int)255*g;
    int blue = (int)255*b;
    std::cout << red << " " << green << " " << blue << endl;
    QString ss = "::sub-page:horizontal {"
                    "background-color: "
                    "qlineargradient(x0: 0, x2: 1, "
                    "stop: 0 rgba(%1, %2, %3, 255)"
                    ")}";
    slider_vh->setStyleSheet(ss.arg(red).arg(green).arg(blue));



    myfile.open ("pipe.txt");
    myfile  << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "|"
            << toggle << "\n";
    myfile.close();

    QString text = "Value High " + QString::number(pos);
    lineEdit_vh->setText(text);
}
void MainWindow::onValueChanged_sl(int value){
    int pos = slider_sl->sliderPosition();

    float r;
    float g;
    float b;
    float h = 0;
    float s = pos/100.0;
    float v = 1.0;
    HSVToRGB(h,s,v,&r,&g,&b);
    int red = (int)255*r;
    int green = (int)255*g;
    int blue = (int)255*b;
    std::cout << red << " " << green << " " << blue << endl;
    QString ss = "::sub-page:horizontal {"
                    "background-color: "
                    "qlineargradient(x0: 0, x2: 1, "
                    "stop: 0 rgba(%1, %2, %3, 255)"
                    ")}";
    slider_sl->setStyleSheet(ss.arg(red).arg(green).arg(blue));


    myfile.open ("pipe.txt");
    myfile  << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "|"
            << toggle << "\n";
    myfile.close();

    QString text = "Saturation Low " + QString::number(pos);
    lineEdit_sl->setText(text);
}
void MainWindow::onValueChanged_sh(int value){
    int pos = slider_sh->sliderPosition();

    float r;
    float g;
    float b;
    float h = 0;
    float s = pos/100.0;
    float v = 1.0;
    HSVToRGB(h,s,v,&r,&g,&b);
    int red = (int)255*r;
    int green = (int)255*g;
    int blue = (int)255*b;
    std::cout << red << " " << green << " " << blue << endl;
    QString ss = "::sub-page:horizontal {"
                    "background-color: "
                    "qlineargradient(x0: 0, x2: 1, "
                    "stop: 0 rgba(%1, %2, %3, 255)"
                    ")}";
    slider_sh->setStyleSheet(ss.arg(red).arg(green).arg(blue));



    myfile.open ("pipe.txt");
    myfile  << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "|"
            << toggle << "\n";
    myfile.close();

    QString text = "Saturation High " + QString::number(pos);
    lineEdit_sh->setText(text);
}
void MainWindow::onValueChanged_hl(int value){
    int pos = slider_hl->sliderPosition();

    float r;
    float g;
    float b;
    float h = (float)pos;
    float s = 1.0;
    float v = 1.0;
    HSVToRGB(h,s,v,&r,&g,&b);
    int red = (int)255*r;
    int green = (int)255*g;
    int blue = (int)255*b;
    std::cout << red << " " << green << " " << blue << endl;
    QString ss = "::sub-page:horizontal {"
                    "background-color: "
                    "qlineargradient(x0: 0, x2: 1, "
                    "stop: 0 rgba(%1, %2, %3, 255)"
                    ")}";
    slider_hl->setStyleSheet(ss.arg(red).arg(green).arg(blue));

    myfile.open ("pipe.txt");
    myfile  << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "|"
            << toggle << "\n";
    myfile.close();

    QString text = "Hue Low " + QString::number(pos);
    lineEdit_hl->setText(text);
}
void MainWindow::onValueChanged_hh(int value){
    int pos = slider_hh->sliderPosition();



    float r;
    float g;
    float b;
    float h = (float)pos;
    float s = 1.0;
    float v = 1.0;
    HSVToRGB(h,s,v,&r,&g,&b);
    int red = (int)255*r;
    int green = (int)255*g;
    int blue = (int)255*b;
    std::cout << red << " " << green << " " << blue << endl;
    QString ss = "::sub-page:horizontal {"
                    "background-color: "
                    "qlineargradient(x0: 0, x2: 1, "
                    "stop: 0 rgba(%1, %2, %3, 255)"
                    ")}";
    slider_hh->setStyleSheet(ss.arg(red).arg(green).arg(blue));

    myfile.open ("pipe.txt");
    myfile  << slider_sl->sliderPosition() << "|" 
            << slider_sh->sliderPosition() << "|"
            << slider_vl->sliderPosition() << "|"
            << slider_vh->sliderPosition() << "|"
            << slider_hl->sliderPosition() << "|"
            << slider_hh->sliderPosition() << "|"
            << toggle << "\n";
    myfile.close();
    
    QString text = "Hue High " + QString::number(pos);
    lineEdit_hh->setText(text);
}
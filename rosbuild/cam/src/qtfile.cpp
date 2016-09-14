#include "qtfile.h"

using namespace std;

int _vl = 49;
int _vh = 100;

int _hl = 192;
int _hh = 238;

int _sl = 50;
int _sh = 100;

static void HSVToRGB(float h, float s, float v, float *r, float *g, float *b){
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
    slider_hl->setRange(0, 450);
    lineEdit_hl = new QLineEdit();
    QObject::connect(slider_hl, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged_hl(int)));

    slider_hh = new QSlider(Qt::Horizontal); 
    slider_hh->setRange(0, 450);
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
    
    slider_sl->setValue(_sl);
    slider_sh->setValue(_sh);
    slider_vl->setValue(_vl);
    slider_vh->setValue(_vh);
    slider_hl->setValue(_hl);
    slider_hh->setValue(_hh);

    float r_l;
    float g_l;
    float b_l;
    HSVToRGB((float)_hl, (float)_sl/100.0, (float)_vl/100.0, &r_l, &g_l, &b_l);
    float r_h;
    float g_h;
    float b_h;
    HSVToRGB((float)_hh, (float)_sh/100.0, (float)_vh/100.0, &r_h, &g_h, &b_h);
    but = new QPushButton("set", this);
    but->setGeometry(QRect(QPoint(90, 250),
    QSize(30, 30)));
    QObject::connect(but, SIGNAL (released()), this, SLOT (handleButton()));
    layout->addWidget(but);

    bar_low = new QProgressBar();
    QString sl = "::chunk {"
                    "background-color: "
                    "qlineargradient(x0: 0, x2: 1, "
                    "stop: 0 rgba(%1, %2, %3, 255)"
                    ")}";
    bar_low->setStyleSheet(sl.arg((int)255*r_l).arg((int)255*g_l).arg((int)255*b_l));
    bar_low->setValue(100);
    bar_low->setTextVisible(false);
    
    layout->addWidget(bar_low);
    bar_high = new QProgressBar();
    QString sh = "::chunk {"
                    "background-color: "
                    "qlineargradient(x0: 0, x2: 1, "
                    "stop: 0 rgba(%1, %2, %3, 255)"
                    ")}";
    bar_high->setStyleSheet(sh.arg((int)255*r_h).arg((int)255*g_h).arg((int)255*b_h));
    bar_high->setValue(100);
    bar_high->setTextVisible(false);
    
    layout->addWidget(bar_high);
    printf("ended cons\n");
}
 
MainWindow::~MainWindow(){}

void MainWindow::handleButton(){
    toggle = 1;

    myfile.open ("pipe.txt");
    myfile  << slider_sl->sliderPosition() << "\n" 
            << slider_sh->sliderPosition() << "\n"
            << slider_vl->sliderPosition() << "\n"
            << slider_vh->sliderPosition() << "\n"
            << slider_hl->sliderPosition() << "\n"
            << slider_hh->sliderPosition() << "\n"
            << toggle;
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
    myfile  << slider_sl->sliderPosition() << "\n" 
            << slider_sh->sliderPosition() << "\n"
            << slider_vl->sliderPosition() << "\n"
            << slider_vh->sliderPosition() << "\n"
            << slider_hl->sliderPosition() << "\n"
            << slider_hh->sliderPosition() << "\n"
            << toggle;
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
    myfile  << slider_sl->sliderPosition() << "\n" 
            << slider_sh->sliderPosition() << "\n"
            << slider_vl->sliderPosition() << "\n"
            << slider_vh->sliderPosition() << "\n"
            << slider_hl->sliderPosition() << "\n"
            << slider_hh->sliderPosition() << "\n"
            << toggle;
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
    myfile  << slider_sl->sliderPosition() << "\n" 
            << slider_sh->sliderPosition() << "\n"
            << slider_vl->sliderPosition() << "\n"
            << slider_vh->sliderPosition() << "\n"
            << slider_hl->sliderPosition() << "\n"
            << slider_hh->sliderPosition() << "\n"
            << toggle;
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
    myfile  << slider_sl->sliderPosition() << "\n" 
            << slider_sh->sliderPosition() << "\n"
            << slider_vl->sliderPosition() << "\n"
            << slider_vh->sliderPosition() << "\n"
            << slider_hl->sliderPosition() << "\n"
            << slider_hh->sliderPosition() << "\n"
            << toggle;
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
    myfile  << slider_sl->sliderPosition() << "\n" 
            << slider_sh->sliderPosition() << "\n"
            << slider_vl->sliderPosition() << "\n"
            << slider_vh->sliderPosition() << "\n"
            << slider_hl->sliderPosition() << "\n"
            << slider_hh->sliderPosition() << "\n"
            << toggle;
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
    myfile << slider_sl->sliderPosition() << "\n" 
            << slider_sh->sliderPosition() << "\n"
            << slider_vl->sliderPosition() << "\n"
            << slider_vh->sliderPosition() << "\n"
            << slider_hl->sliderPosition() << "\n"
            << slider_hh->sliderPosition() << "\n"
            << toggle;
    myfile.close();
    
    QString text = "Hue High " + QString::number(pos);
    lineEdit_hh->setText(text);
}
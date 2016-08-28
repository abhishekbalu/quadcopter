#ifndef _QTFILE_H
#define _QTFILE_H
 
#include <QMainWindow>
#include <QApplication>
#include <QVBoxLayout>
#include <QSlider>
#include <QLineEdit>
#include <QMessageBox>
#include <QDebug>
#include <iostream>
#include <fstream>
using namespace std;
class MainWindow: public QMainWindow{
    Q_OBJECT
    QSlider *slider_vl;
    QLineEdit *lineEdit_vl;
    QSlider *slider_vh;
    QLineEdit *lineEdit_vh;
    QSlider *slider_sl;
    QLineEdit *lineEdit_sl;
    QSlider *slider_sh;
    QLineEdit *lineEdit_sh;
    QSlider *slider_hl;
    QLineEdit *lineEdit_hl;
    QSlider *slider_hh;
    QLineEdit *lineEdit_hh;
 
public:
    ofstream myfile;
    MainWindow();
    ~MainWindow();
 
private slots:
    void onValueChanged_vl(int value);
    void onValueChanged_vh(int value);
    void onValueChanged_sl(int value);
    void onValueChanged_sh(int value);
    void onValueChanged_hl(int value);
    void onValueChanged_hh(int value);
};
 
#endif
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <thread>
//#include <sstream>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


//const auto serialPortInfos = QSerialPortInfo::availablePorts();

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QList<QSerialPortInfo> serialPortInfos = QSerialPortInfo::availablePorts();

    unsigned char recAddress;
    int recSpeed;
    //unsigned char contactAddress;
    unsigned char message [8] = {01, 06, 00, 0x14, 00, 00, 00, 00};
    unsigned char answer [8] = {01, 03, 00, 0x14, 00, 02, 00, 00};
    unsigned char message2 [8] = {01, 06, 00, 0x15, 00, 00, 00, 00};
    //unsigned char bufbuf[10];


private slots:
    uint16_t MODBUS_CRC16_v3(const unsigned char* buf, unsigned int len);

    void logs(unsigned char send[8], int n);

    void logs2(unsigned char send[8], int n);

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_comboBox_speed_activated(int index);

    void on_comboBox_address_activated(int index);

    void on_pushButton_3_clicked();

    void on_cBox_COM_activated(int index);

    void on_pushButton_open_clicked();

    void on_spinBox_address_valueChanged(int arg1);

    void on_pushButton_4_clicked();

    void on_comboBox_setSpeed_activated(int index);

    void on_pushButton_5_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort *m_serial;

};
#endif // MAINWINDOW_H

#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_serial(new QSerialPort)
{
    ui->setupUi(this);

    // Settings
    //m_serial->setPortName("COM3");
    //m_serial->setBaudRate(115200); // меняется с кнопки

    m_serial->setDataBits(QSerialPort::DataBits::Data8);
    m_serial->setParity(QSerialPort::Parity::NoParity);
    m_serial->setStopBits(QSerialPort::StopBits::OneStop);
    m_serial->setFlowControl(QSerialPort::FlowControl::NoFlowControl);


    //ui->cBox_COM->addItem(portInfo.portName());//добавление в комбо бокс
}

uint16_t MainWindow::MODBUS_CRC16_v3(const unsigned char* buf, unsigned int len)
{
    static const uint16_t table[256] = {
                                        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
                                        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
                                        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
                                        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
                                        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
                                        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
                                        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
                                        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
                                        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
                                        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
                                        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
                                        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
                                        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
                                        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
                                        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
                                        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
                                        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
                                        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
                                        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
                                        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
                                        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
                                        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
                                        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
                                        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
                                        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
                                        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
                                        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
                                        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
                                        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
                                        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
                                        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
                                        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

    uint8_t XOR = 0;
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        XOR = (*buf++) ^ crc;
        crc >>= 8;
        crc ^= table[XOR];
    }

    return crc;
}

void MainWindow::logs(unsigned char send[8], int n)
{
    QString str = QString("записано ");
    for (int i = 0; i < n; i++)//
    {
        str += QString::number(send[i], 16);
        str += " ";
    }
    ui->listWidget->addItem(str);
}

void MainWindow::logs2(unsigned char send[8], int n)
{
    QString str = QString("отправлено ");
    for (int i = 0; i < n; i++)//
    {
        str += QString::number(send[i], 16);
        str += " ";
    }
    ui->listWidget->addItem(str);
}

MainWindow::~MainWindow()
{
    if(m_serial->isOpen())
        m_serial->close();

    delete m_serial;
    delete ui;
}

void MainWindow::on_pushButton_clicked()//вывод ответа
{
    char buf[9];//------------------------------------------------------------------------------------
    int n = m_serial->read(buf, 9);//чтение из м-сериал в буффер

    qDebug() << n;

    QDebug deb = qDebug();
    for(int i=0;i<9;i++)
        deb << QString::number(static_cast<uint8_t>(buf[i]), 16) << " ";

    QString add = QString();
    add += std::to_string(buf[4]);
    //qDebug() << add;
    ui->lineEditAnswer->setText(add);

    QString str = QString("получено ");
    for (int i = 0; i < 9; i++)
    {
        str += QString::number(static_cast<uint8_t>(buf[i]), 16);
        str += " ";
    }
    ui->listWidget->addItem(str);



}


void MainWindow::on_pushButton_2_clicked()//отправить сообщение
{

    unsigned short crc = MODBUS_CRC16_v3(answer, 6);

    answer[6] = (crc & 0xFF);
    answer[7] = (crc >> 8);

    //QDebug deb = qDebug();
    //for(int i=0;i<8;i++)
        //deb << "Send: " << QString::number(answer[i], 16) << " ";

    m_serial->flush();
    m_serial->write((char*)(answer), 8);//отправить


    logs2(answer, 8);
}



void MainWindow::on_comboBox_speed_activated(int index)
{
    int speed[] {115200, 9600};//т.к. не вышло вытащить текст из индекса комбобокса, делаем массив в котором числа по индексам совпадают с боксом
    m_serial->setBaudRate(speed[index]);//устанавливаем ту скорость которая выбрана
}


void MainWindow::on_comboBox_address_activated(int index)
{
    unsigned char contactAddress[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    //unsigned char buf[12] = {address[index], 0x03, 0x00, 0x14, 0x00, 0x01, 0xC4, 0x0E};
    message[0] = contactAddress[index];
    answer[0] = contactAddress[index];
    message2[0] = contactAddress[index];
}




void MainWindow::on_pushButton_3_clicked()
{

    ui->cBox_COM->clear();
    serialPortInfos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &portInfo : serialPortInfos)
    {
        //qDebug() << "\n"
                 //<< "Port:" << portInfo.portName() << "\n";

        ui->cBox_COM->addItem(portInfo.portName());//добавление в комбо бокс

    }


}


void MainWindow::on_cBox_COM_activated(int index)
{
    std::string ports[10];

    if(m_serial->isOpen())
    {
        m_serial->close();
    }

    m_serial->setPortName(serialPortInfos[index].portName());//подключает порт
    qDebug() << serialPortInfos[index].portName();

    ui->listWidget->addItem(serialPortInfos[index].portName() + " выбран");

    m_serial->setBaudRate(115200); // по умолчанию будет стоять эта скорость , с кнопки она изменится
    //получается мы при выборе порта и подключаем его и скорость задаёт
}





void MainWindow::on_pushButton_open_clicked()
{
    //qDebug() << ui->pushButton_3->text(); //получаем то что написано на кнопке
    //ui->pushButton_3->setText("вкл");//устанавливаем текст

    if(ui->pushButton_open->text() == "Подключить")
    {
        if (m_serial->open(QIODevice::ReadWrite))
        {
            ui->listWidget->addItem("порт подключен");
        } else
        {
            qDebug() << m_serial->errorString();
            ui->listWidget->addItem("ОШИБКА: порт не открыт");
        }

        //qDebug() << "меняем на вкл и отключаем порт";
        ui->pushButton_open->setText("Отключить");
    }
    else if(ui->pushButton_open->text() == "Отключить")
    {
        if(m_serial->isOpen())
        {
            m_serial->close();
            ui->listWidget->addItem("порт отключен");
        }

        //qDebug() << "меняем на выкл и подключаем порт";
        ui->pushButton_open->setText("Подключить");
    }



}


void MainWindow::on_spinBox_address_valueChanged(int arg1)
{
    ui->spinBox_address->setRange(1, 10);
    //int a;
    recAddress = ui->spinBox_address->value();


    //qDebug() << std::hex << a;

    /*                          //для работы этого кусочка нужен sstream
    std::ostringstream ss;
    ss << "0x" << std::hex << a;
    std::string result = ss.str();

    std::string addr = ss.str();
    qDebug() << addr << '\n';
    */


}

void MainWindow::on_pushButton_4_clicked()
{
    message[5] = recAddress;


    //qDebug() << address;
    /*for (int i; i < 8; i++)
    {
        qDebug() << (unsigned)message[i];
    }
    qDebug() << '\n';*/

    unsigned short crc = MODBUS_CRC16_v3(message, 6);
    //int a = (crc >> 8);
    //int b = (crc & 0xFF);
    //qDebug() << a << " " << b;

    message[6] = (crc & 0xFF); //b
    message[7] = (crc >> 8); //a

    /*for (int i; i < 8; i++)
    {
        qDebug() << message[i];
    }
    qDebug() << '\n';*/

    m_serial->flush();
    m_serial->write((char*)(message), 8);//отправка команды

    /*
    QString str = QString("записано ");
    //unsigned char message16 [8];
    for (auto i: message)
    {
        str += QString::number(message[i], 16);
        str += " ";
    }
    ui->listWidget->addItem(str);*/

    logs(message, 8);
}


void MainWindow::on_comboBox_setSpeed_activated(int index)
{
    //int speed[] {115200, 9600};//т.к. не вышло вытащить текст из индекса комбобокса, делаем массив в котором числа по индексам совпадают с боксом
    unsigned char speedCode[] {7, 3};
    recSpeed = speedCode[index];





}


void MainWindow::on_pushButton_5_clicked()
{
    message2[5] = recSpeed;

    //for (int i; i < 8; i++)
    //{
    //    qDebug() << (unsigned)message2[i];
    //}
    //qDebug() << '\n';

    unsigned short crc2 = MODBUS_CRC16_v3(message2, 6);


    message2[6] = (crc2 & 0xFF); //b
    message2[7] = (crc2 >> 8); //a


    QDebug deb = qDebug();
    for(int i=0;i<8;i++)
        deb << "Send: " << QString::number(message2[i], 16) << " ";

    m_serial->flush();
    m_serial->write((char*)(message2), 8);//отправка команды

    logs(message2, 8);
}


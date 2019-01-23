#include <iostream>
#include "feedbackupdater.h"

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

FeedbackUpdater::FeedbackUpdater(QObject *parent)
    : QObject(parent), feedbackUpdateInterval(200)
{
    //std::cerr << tc::brwn << Q_FUNC_INFO << ": If you use this constructor remember to call Init(*engine) after"
    //          << tc::none << "\n";
}

FeedbackUpdater::FeedbackUpdater(QQmlApplicationEngine *engine, QObject *parent)
    : QObject(parent), feedbackUpdateInterval(200)
{
    Init(engine);
}

FeedbackUpdater::~FeedbackUpdater(){
    //xcom_->Release();
    delete myTimer_;
}

void FeedbackUpdater::Init(QQmlApplicationEngine *engine){

    appEngine_ = engine;
    //xcom_ = ortos::xcom::XCOMInterface::GetInstance();

    myTimer_ = new QTimer(this);
    myTimer_->start(feedbackUpdateInterval);
    QObject::connect(myTimer_, SIGNAL (timeout()), this, SLOT (l_wTt_Slot()));
    QObject::connect(myTimer_, SIGNAL (timeout()), this, SLOT (r_wTt_Slot()));
    QObject::connect(myTimer_, SIGNAL (timeout()), this, SLOT (l_Q_Slot()));
    QObject::connect(myTimer_, SIGNAL (timeout()), this, SLOT (r_Q_Slot()));
}

void FeedbackUpdater::ReadwTt(QString &wTtString, const std::string topicGroup)
{
    /*xcom_->Synchronize();

    int ret = odh_.ReadArm_wTt(v6Container_, topicGroup);
    if(ret == ORTOS_RV_OK){
        wTtString = QString::fromStdString(FUTILS::ArrayToString(v6Container_.d.data,6,' '));
    }else{
        wTtString = "*No incoming data*";
    }*/
}

void FeedbackUpdater::ReadArmQ(QString &armQ, const std::string topicGroup)
{
    /*xcom_->Synchronize();

    int ret = odh_.ReadArmFeedback(armFbkContainer_, topicGroup);
    if(ret == ORTOS_RV_OK){
        armQ = QString::fromStdString(FUTILS::ArrayToString(armFbkContainer_.d.q,ortosdata::numJoints,' '));
    }else{
        armQ = "*No incoming data*";
    }*/
}

void FeedbackUpdater::copyToClipboard(QString newText)
{
    QClipboard *clipboard = QGuiApplication::clipboard();
    clipboard->setText(newText);
}

QString FeedbackUpdater::get_l_wTt()
{
    return l_wTt_qs;
}

QString FeedbackUpdater::get_r_wTt()
{
    return r_wTt_qs;
}

QString FeedbackUpdater::get_l_Q()
{
    return l_Q_qs;
}

QString FeedbackUpdater::get_r_Q()
{
    return r_Q_qs;
}


void FeedbackUpdater::l_wTt_Slot()
{
    //ReadwTt(l_wTt_qs, ortosdata::topicnames::left);

    //qDebug() << l_wTt_qs;
    /*std::cout << tc::grnL << "l_wTt: " << tc::none;
    FUTILS::PrintArray(v6Container_.d.data, 6, ' ');
    std::cout << std::endl;*/
    emit l_wTt_FeedbackUpdate();
}

void FeedbackUpdater::r_wTt_Slot()
{
    //ReadwTt(r_wTt_qs, ortosdata::topicnames::right);
    emit r_wTt_FeedbackUpdate();
}

void FeedbackUpdater::l_Q_Slot()
{
    //ReadArmQ(l_Q_qs, ortosdata::topicnames::left);
    emit l_Q_FeedbackUpdate();
}

void FeedbackUpdater::r_Q_Slot()
{
    //ReadArmQ(r_Q_qs, ortosdata::topicnames::right);
    emit r_Q_FeedbackUpdate();
}

QVector<double> FeedbackUpdater::generateRandFloatVector(int size)
{
    QVector<double> randVect(size);

    for (int i=0; i<randVect.size();i++) {
        randVect[i] = fRand(0.0,1.0);
    }
    return randVect;
}


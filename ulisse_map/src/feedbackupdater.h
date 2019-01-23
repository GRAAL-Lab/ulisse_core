#ifndef FEEDBACKUPDATER_H
#define FEEDBACKUPDATER_H

#include <QtGui>
#include <QQmlApplicationEngine>
#include <QObject>
#include <QVector>
//#include <control_baxter/roc.h>

class FeedbackUpdater : public QObject
{
    Q_OBJECT
    QQmlApplicationEngine *appEngine_;
    QTimer *myTimer_;
    Q_PROPERTY( QString l_wTt READ get_l_wTt NOTIFY l_wTt_FeedbackUpdate )
    Q_PROPERTY( QString r_wTt READ get_r_wTt NOTIFY r_wTt_FeedbackUpdate )
    Q_PROPERTY( QString l_Q READ get_l_Q NOTIFY l_Q_FeedbackUpdate )
    Q_PROPERTY( QString r_Q READ get_r_Q NOTIFY r_Q_FeedbackUpdate )
    QString l_wTt_qs, r_wTt_qs, l_Q_qs, r_Q_qs;
    int feedbackUpdateInterval;

    //ortos::xcom::XCOMInterface* xcom_;
    //ortosdata::DataHelper odh_;
    //ortosdata::Vector6Container v6Container_;
    //ortosdata::ArmFeedbackContainer armFbkContainer_;

    QVector<double> generateRandFloatVector(int size);

public:
    explicit FeedbackUpdater(QObject *parent = 0);
    explicit FeedbackUpdater(QQmlApplicationEngine *engine, QObject *parent = 0);
    virtual ~FeedbackUpdater();
    void Init(QQmlApplicationEngine *engine);
    void ReadwTt(QString &wTtString, const std::string topicGroup);
    void ReadArmQ(QString &armString, const std::string topicGroup);

    Q_INVOKABLE void copyToClipboard(QString value);

    QString get_l_wTt();
    QString get_r_wTt();
    QString get_l_Q();
    QString get_r_Q();

    //Q_INVOKABLE void someFunction(int i);

signals:
    void l_wTt_FeedbackUpdate();
    void r_wTt_FeedbackUpdate();
    void l_Q_FeedbackUpdate();
    void r_Q_FeedbackUpdate();

public slots:
    void l_wTt_Slot();
    void r_wTt_Slot();
    void l_Q_Slot();
    void r_Q_Slot();
};

#endif // FEEDBACKUPDATER_H


#include <ros/ros.h>
#include "coordinator.h"
#include <navigationISL/neighborInfo.h>
#include <navigationISL/robotInfo.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>


//#define numOfRobots 5


class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();


    Coordinator coordinator;

private:
     bool shutdown;

  //   navigationISL::robotInfo currentStatus;

     ros::NodeHandle n;

     ros::Subscriber communicationManagerSubscriber;

     ros::Publisher networkinfoPublisher;

     void handleCoordinatorUpdate(navigationISL::neighborInfo info);

     bool readInitialPoses(QString filepath);

     double bin[numOfRobots+1][4];// positions including itself

     int adjM[numOfRobots+1][numOfRobots+1];

     bool dataReceived[numOfRobots+1];

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void  rosStarted();
   void  rosStartFailed();

};

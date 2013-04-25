#include "coordinator.h"

#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>


Coordinator::Coordinator()
{
    QString path = QDir::homePath();
     path.append("/fuerte_workspace/sandbox/configISL.json");


     if(!readConfigFile(path)){

         qDebug()<< "Read Config File Failed!!!";

      }



}


bool Coordinator::readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {

        this->ro = result["ro"].toDouble();
        qDebug()<<result["ro"].toString();

        this->cf = result["linkCost"].toDouble();
        qDebug()<<result["linkCost"].toString();

        this->maxGameMove = result["maxGameMove"].toDouble();
        qDebug()<<result["maxGameMove"].toString();

        this->checkPSPeriod = result["checkPSPeriod"].toDouble();
        qDebug()<<result["checkPSPeriod"].toString();


    }
    file.close();
    return true;

}



int Coordinator::checkPairwiseStability(int adjM[][numOfRobots+1], double bin[][4])
{
    int stable;

    for (int i=1; i<=numOfRobots-1; i++)
    {
        for(int j=i+1; j<=numOfRobots; j++)
        {

            if ( (adjM[j][i]==1) && (adjM[i][j]==1) && (i!=j) ) // test on severing a link
            {

                double payoff1 = calcPayoff( adjM, bin, i);
                double payoff2 = calcPayoff( adjM, bin, j);

                adjM[i][j] = 0;
                adjM[j][i] = 0;
                double payoff1N = calcPayoff( adjM, bin, i);
                double payoff2N = calcPayoff( adjM, bin, j);

                adjM[i][j] = 1;
                adjM[j][i] = 1;

                if ( (payoff1 < payoff1N) || (payoff2 < payoff2N) || (payoff1<0 || payoff2<0) )
                {
                    adjM[i][j] = 0;
                    adjM[j][i] = 0;

                    stable = 0;
                    return stable;
                }
            }
            else if ( (adjM[j][i]==0) && (adjM[i][j]==0) && (i!=j) ) // test on establishing a link
            {
                double payoffG1 = calcPayoff( adjM, bin, i);
                double payoffG2 = calcPayoff( adjM, bin, j);

                adjM[i][j] = 1;
                adjM[j][i] = 1;
                double payoffG1N = calcPayoff( adjM, bin, i);
                double payoffG2N = calcPayoff( adjM, bin, j);

                adjM[i][j] = 0;
                adjM[j][i] = 0;

                if ( (payoffG1 < payoffG1N) && (payoffG2 < payoffG2N) && (payoffG1N>0) && (payoffG2N>0) )
                {
                    adjM[i][j] = 1;
                    adjM[j][i] = 1;

                    stable = 0;
                    return stable;
                }

            }
        }
    }

    stable = 1;
    return stable;
}


double Coordinator::calcPayoff(int adjM[][numOfRobots+1], double bin[][4], int id)
{
    //const int SPILLOVER_PAYOFF = 12;

    double  cf_ = this->cf;//8;//params[2]; // cost parameter

    double ro = this->ro;

    //int payoffID = params[9];

    double payoff;

    //if (payoffID==SPILLOVER_PAYOFF)
    //{

        int ni = 0; // number of robot-id's neighbor

        double distTot = 0;
        for(int i=1; i<=numOfRobots; i++)
        {
            if ( (adjM[i][id]==1) && (adjM[id][i]==1) && (id!=i) )
            {
                    distTot = distTot + sqrt((bin[id][1]-bin[i][1])*(bin[id][1]-bin[i][1]) + (bin[id][2]-bin[i][2])*(bin[id][2]-bin[i][2]));

                    ni = ni + 1;
            }

        }

        double B=1;
        if (distTot>0)
        {
            B = ro/(distTot/ni);
        }

        double vplus = ni;//*B;

        double vminus = 0;

        for(int j=1;j<=numOfRobots; j++)
        {
            if ( (adjM[j][id]==1) && (adjM[id][j]==1) && (id!=j) )
            {
                int nj = 0;
                for(int i=1; i<=numOfRobots; i++)
                {
                    if ( (adjM[i][j]==1) && (adjM[j][i]==1) && (j!=i) )
                    {
                        nj = nj + 1;
                    }
                }

                double dij = sqrt((bin[id][1]-bin[j][1])*(bin[id][1]-bin[j][1]) + (bin[id][2]-bin[j][2])*(bin[id][2]-bin[j][2]));

                vminus = vminus + (ro/(dij))/nj;
            }
        }

        payoff = vplus + vminus - cf_*ni;

    //}

    return payoff;
}


void Coordinator::playGame(int linksR[][numOfRobots+1], int links[][numOfRobots+1], double bin[][4])
{

    int MAX_GAME_MOVE = this->maxGameMove;
    int periodCPS = this->checkPSPeriod; //checking period for pairwise stability

    int **linkListNoPlayed;
    linkListNoPlayed = matrixInt(1,numOfRobots*(numOfRobots-1)/2,1,2);

    int cnt = 1;
    for (int i=1; i<=numOfRobots-1; i++)
    {
        for(int j=i+1; j<=numOfRobots; j++)
        {
            linkListNoPlayed[cnt][1] = i;
            linkListNoPlayed[cnt][2] = j;
            cnt = cnt + 1;
        }
    }
    int linkListSize = cnt - 1;
    int iter;
    for(int ii=1; ii<=MAX_GAME_MOVE; ii++)
    {


        if ((ii%periodCPS)==0)
        {

            int stable = checkPairwiseStability(links, bin);


            if (stable==1)
            {
                //iterPG = ii;
                break;
            }

        }

       // double temp1 = (double)(rand()% 100 + 1)/(RAND_MAX + 1);
       // int linkID = (int)((((long int)rand()/(double)(RAND_MAX + 1))*linkListSize)) + 1;
        int linkID = 0;
        if (linkListSize==1){
            linkID = 1;
        }
        else
        {
            linkID = round((rand() % (linkListSize)) + 1);
        }
        qDebug()<<linkID;
        qDebug()<<"linkList "<<linkListSize;
        int iR = linkListNoPlayed[linkID][1];
        int idR = linkListNoPlayed[linkID][2];

        if (linkListSize>1)
        {
            //delete this link from linkListNoPlayed
            int cnt = 1;
            int cntN = 1;
            for (int i=1; i<=numOfRobots-1; i++)
            {
                for(int j=i+1; j<=numOfRobots; j++)
                {
                    if ( (cnt!=linkID) && (cnt<=linkListSize) )
                    {
                        linkListNoPlayed[cntN][1] = linkListNoPlayed[cnt][1];
                        linkListNoPlayed[cntN][2] = linkListNoPlayed[cnt][2];
                        cntN = cntN + 1;
                    }
                    cnt = cnt + 1;
                }
            }
            linkListSize = cntN - 1;
        }
        else
        {
            int cnt = 1;
            for (int i=1; i<=numOfRobots-1; i++)
            {
                for(int j=i+1; j<=numOfRobots; j++)
                {
                    linkListNoPlayed[cnt][1] = i;
                    linkListNoPlayed[cnt][2] = j;
                    cnt = cnt + 1;
                }
            }
            linkListSize = cnt - 1;
        }

        int i = iR;
        int id = idR;

        if ( (links[i][id]==1) && (links[id][i]==1) )
        {
            double payoffi = calcPayoff( links, bin, i) ;
            double payoffid = calcPayoff( links, bin, id);

            links[i][id] = 0;
            links[id][i] = 0;

            double payoffNexti = calcPayoff(links, bin, i);
            double payoffNextid = calcPayoff(links, bin, id);

            links[i][id] = 1;
            links[id][i] = 1;

            if ( (payoffNexti > payoffi) || (payoffNextid > payoffid) )
            {
                links[i][id] = 0;
                links[id][i] = 0;
            }
            else if ( ( payoffi<0) || (payoffid<0) )
            {
                links[i][id] = 0;
                links[id][i] = 0;
            }

        }
        else if ( (links[i][id]==0) || (links[id][i]==0) )
        {
            double payoffi = calcPayoff(links, bin, i);
            double payoffid = calcPayoff(links, bin, id);

            links[i][id] = 1;
            links[id][i] = 1;

            double payoffNexti = calcPayoff(links, bin, i);
            double payoffNextid = calcPayoff(links, bin, id);

            links[i][id] = 0;
            links[id][i] = 0;

            if ( (payoffNexti > payoffi) && (payoffNextid > payoffid) && (payoffNexti>0) && (payoffNextid>0) )
            {
                links[i][id] = 1;
                links[id][i] = 1;
            }
        }

    }

    for (int i=1; i<=numOfRobots-1; i++)
    {
        linksR[i][i] = 0;
        for(int j=i+1; j<=numOfRobots; j++)
        {
            linksR[i][j] = links[i][j];
            linksR[j][i] = links[j][i];
        }
    }


    free_matrixInt(linkListNoPlayed,1,numOfRobots*(numOfRobots-1)/2,1,2);

}


int ** Coordinator::matrixInt(long nrl, long nrh, long ncl, long nch)
/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
    long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
    int **m;

    /* allocate pointers to rows */
    m=(int **) malloc((size_t)((nrow+NR_END)*sizeof(int*)));
    if (!m) nrerror("allocation failure 1 in matrix()");
    m += NR_END;
    m -= nrl;

    /* allocate rows and set pointers to them */
    m[nrl]=(int *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(int)));
    if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
    m[nrl] += NR_END;
    m[nrl] -= ncl;

    for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

    /* return pointer to array of pointers to rows */
    return m;
}


void Coordinator::free_matrixInt(int **m, long nrl, long nrh, long ncl, long nch)
/* free a double matrix allocated by matrix() */
{
    free((FREE_ARG) (m[nrl]+ncl-NR_END));
    free((FREE_ARG) (m+nrl-NR_END));
}

void Coordinator::nrerror(char error_text[])
/* Numerical Recipes standard error handler */
{
    fprintf(stderr,"Numerical Recipes run-time error...\n");
    fprintf(stderr,"%s\n",error_text);
    fprintf(stderr,"...now exiting to system...\n");
    exit(1);
}



#ifndef COORDINATOR_H
#define COORDINATOR_H

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <QString>

#define numOfRobots 3

#define NR_END 1
#define FREE_ARG char*

class Coordinator
{
public:
    Coordinator();

    double ro;
    double cf;

    int maxGameMove;
    int checkPSPeriod;

    bool readConfigFile(QString);

    void playGame(int [][numOfRobots+1], int [][numOfRobots+1], double [][4]);
    int checkPairwiseStability(int [][numOfRobots+1], double [][4]);
    double calcPayoff(int [][numOfRobots+1], double [][4], int );

    int **matrixInt(long nrl, long nrh, long ncl, long nch);
    void free_matrixInt(int **m, long nrl, long nrh, long ncl, long nch);
    void nrerror(char error_text[]);

};

#endif // COORDINATOR_H

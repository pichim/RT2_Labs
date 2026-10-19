#pragma once

#include "mbed.h"

class GPA
{
public:
    typedef struct GPADATA {
        float fexc;
        float absGyu;
        float angGyu;
        float absGyr;
        float angGyr;
        float Umag;
        float Ymag;
        float Rmag;
        bool MeasPointFinished;
        bool MeasFinished;
        int ind;
    } gpadata_t;

    GPA() {};
    GPA(float fMin, float fMax, int NfexcDes, float Aexc0, float Aexc1, float Ts);
    GPA(float fMin,
        float fMax,
        int NfexcDes,
        int NperMin,
        int NmeasMin,
        float Ts,
        float Aexc0,
        float Aexc1,
        int Nstart,
        int Nsweep);
    GPA(float f0,
        float f1,
        float *fexcDes,
        int NfexcDes,
        int NperMin,
        int NmeasMin,
        float Ts,
        float Aexc0,
        float Aexc1,
        int Nstart,
        int Nsweep);
    GPA(float *fexcDes,
        int NfexcDes,
        int NperMin,
        int NmeasMin,
        float Ts,
        float Aexc0,
        float Aexc1,
        int Nstart,
        int Nsweep);
    GPA(float fMin,
        float fMax,
        int NfexcDes,
        int NperMin,
        int NmeasMin,
        float Ts,
        float Aexc0,
        float Aexc1,
        int Nstart,
        int Nsweep,
        bool doPrint);
    void setup(float, float, int, float, float, float);
    float operator()(float inp, float out) { return update(inp, out); }

    virtual ~GPA();

    void setParameters(float fMin,
                       float fMax,
                       int NfexcDes,
                       int NperMin,
                       int NmeasMin,
                       float Ts,
                       float Aexc0,
                       float Aexc1,
                       int Nstart,
                       int Nsweep,
                       bool doPrint);
    void reset();
    float update(float inp, float out);

    void printGPAfexcDes();
    void printGPAmeasPara();
    void printGPAmeasTime();
    void printNfexcDes();
    void printLine();
    void printLongLine();
    float get_Ts();
    uint8_t get_i();
    void getGPAdata(float *);
    bool new_data_available;
    bool meas_is_finished;
    bool start_now;
    int status;

private:
    int NfexcDes;
    int NperMin;
    int NmeasMin;
    float Ts;
    float *fexcDes;
    float aAexcDes;
    float bAexcDes;

    float fnyq;
    double pi2;
    float pi4;
    double pi2Ts;
    float piDiv2;
    float rad2deg;
    float div12pi;
    float div812pi;

    int Nmeas;
    int Nper;
    double dfexc;
    double fexc;
    float fexcPast;
    float dfexcj;
    int f_iter;
    int j_iter;
    double scaleG;
    double cr;
    double ci;
    double *sU;
    double *sY;
    double sinarg;
    float sinargR;
    int NmeasTotal;
    float Aexc;
    float AexcPast;
    double pi2Tsfexc;
    int Nstart;
    int Nsweep;
    int Nsweep_i;
    float AexcOut;
    uint8_t testpoint;

    gpadata_t gpaData;
    bool doPrint;

    void assignParameters(int NfexcDes, int NperMin, int NmeasMin, float Ts, int Nstart, int Nsweep);
    void calculateDecreasingAmplitudeCoefficients(float Aexc0, float Aexc1);
    void initializeConstants(float Ts);
    void assignFilterStorage();
    void fexcDesLogspace(float fMin, float fMax, int NfexcDes);
    void calcGPAmeasPara(float fexcDes_i);
    float wrapAngle(float angle);
};

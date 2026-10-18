/*
    GPA: Frequency point wise gain and phase analyser to measure a frequency respone function (FRF) of a dynamical
system

         Hint:        If the plant has a pole at zero, is unstable or weakly damped the measurement has to be perfomed
                      in closed loop (this is NOT tfestimate, the algorithm is based on the one point DFT).
         Assumption:  The system is and remains at the desired steady state when the measurment starts

         Note:        The amplitude drops with 1/fexc, if you're using Axc1 = Aexc0/fMax then d/dt exc = const.,
                      this is recommended if your controller does not have a rolloff. If a desired frequency point
                      is not measured (could not be reached) try to increase Nmeas.


    Instantiate option 0: ("Not a Jedi yet" users, for logarithmic equaly spaced frequency points)

        GPA(float fMin, float fMax, int NfexcDes, float Aexc0, float Aexc1, float Ts)

            fMin:       Minimal desired frequency that should be measured in Hz
            fMax:       Maximal desired frequency that should be measured in Hz
            NfexcDes:   Number of logarithmic equaly spaced frequency points between fMin and fMax
            Aexc0:      Excitation amplitude at fMin
            Aexc1:      Excitation amplitude at fMax
            Ts:         Sampling time in sec

            Default values are as follows:
            int NperMin  = 3;
            int NmeasMin = (int)ceil(1.0f/Ts);
            int Nstart   = (int)ceil(3.0f/Ts);
            int Nsweep   = (int)ceil(0.0f/Ts);

    Instantiate option 1: ("Jedi or Sith Lord", for logarithmic equaly spaced frequency points)

        GPA(float fMin, float fMax, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int
Nstart, int Nsweep)

            fMin:       Minimal desired frequency that should be measured in Hz
            fMax:       Maximal desired frequency that should be measured in Hz
            NfexcDes:   Number of logarithmic equaly spaced frequency points
            NperMin:    Minimal number of periods that are used for each frequency point
            NmeasMin:   Minimal number of samples that are used for each frequency point
            Ts:         Sampling time in sec
            Aexc0:      Excitation amplitude at fMin
            Aexc1:      Excitation amplitude at fMax
            Nstart:     Minimal number of samples to sweep to the first frequency point (can be equal 0)
            Nsweep:     Minimal number of samples to sweep from freq. point to freq. point (can be equal 0)


    Instantiate option 2: (for a second, refined frequency grid measurement)

        GPA(float f0, float f1, float *fexcDes, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float
Aexc1, int Nstart, int Nsweep)

            f0:         Frequency point for the calculation of Aexc0 in Hz (should be chosen like in the first
measurement) f1:         Frequency point for the calculation of Aexc1 in Hz (should be chosen like in the first
measurement) *fexcDes:   Sorted frequency point array in Hz NfexcDes:   Length of fexcDes

            For the other parameters see above.

    Instantiate option 3: (for an arbitary but sorted frequency grid measurement)

        GPA(float *fexcDes, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int
Nsweep)

            *fexcDes:   Sorted frequency point array in Hz
            Aexc0:      Excitation amplitude at fexcDes[0]
            Aexc1:      Excitation amplitude at fexcDes[NfexcDes-1]
            NfexcDes:   Length of fexcDes

            For the other parameters see above.


    Block diagram:

                w (const.)    exc(2)                C: controller
                |              |                    P: plant
                v   e          v
     exc(1) --> o   ->| C |--->o------->| P |----------> out (y)
                ^ -                |             |
                |                   --> inp (u)  |  exc (R): excitation signal
                |                                |  inp (U): input plant
                 --------------------------------   out (Y): output plant


    Pseudo code for an open loop measurement:

        - Measuring the plant P = Gyu = Gyr:

            u = w + exc;
            ... write output u here! it follows exc(k+1) ...
            exc = Wobble(exc, y);

            Closed loop FRF calculus with a stabilizing controller C:
                S  = 1/(1 + C*P);  % ( exc1 -> e ,   1/(1 + C*P) ) sensitivity, contr. error rejection, robustness
(1/max|S|) T  = 1 - S;        % ( w -> y    , C*P/(1 + C*P) ) compl. sensitivity, tracking performance CS = C*S; % (
exc1 -> u ,   C/(1 + C*P) ) control effort, disturbance plant output on plant input PS = P*S;          % ( exc2 -> y ,
P/(1 + C*P) ) compliance, disturbance plant input on plant output


    Pseudo code for a closed loop measurement with stabilizing controller C:

        Excitation at excitation input (1):

        - Measuring the plant P = Gyu and the closed loop tf T = PC/(1 + PC) = Gyr:

            u = C(w - y + exc);
            ... write output u here! it follows exc(k+1) ...
            exc = Wobble(u, y);

            Closed loop FRF calculus:
                S  = 1 - T;
                PS = P*S;
                CS = T/P;
                C  = C/S;

        Excitation at excitation input (2):

        - Measuring the plant P = Gyu and the closed loop tf PS = P/(1 + PC) = Gyr:

            u = C(w - y) + exc;
            ... write output u here! it follows exc(k+1) ...
            exc = Wobble(u, y);

            Closed loop FRF calculus:
                S  = PS/P;
                T  = 1 - S;
                CS = T/P;
                C  = C/S;


    Usage:
        exc(k+1) = myGPA(inp(k), out(k)) does update the internal states of the
        gpa at the timestep k and returns the excitation signal for the timestep
        k+1. The FRF data are plotted to a terminal (Putty) over a serial
        connection and look as follows:

--------------------------------------------------------------------------------
  fexc[Hz]    |Gyu|    deg(Gyu)  |Gyr|    deg(Gyr)   |U|       |Y|       |R|
--------------------------------------------------------------------------------
 5.0000e-02 1.001e+00   -0.309 1.001e+00   -0.309 4.000e-01 4.000e-01 4.005e-01
    .           .         .        .         .        .         .         .
    .           .         .        .         .        .         .         .
    .           .         .        .         .        .         .         .

    In Matlab you can use the editor as follows:
        data = [... insert measurement data here ...];
        gyu = frd(data(:,2).*exp(1i*data(:,3)*pi/180), data(:,1), Ts, 'Units', 'Hz');
        gyr = frd(data(:,4).*exp(1i*data(:,5)*pi/180), data(:,1), Ts, 'Units', 'Hz');

    If you're evaluating more than one measurement which contain equal frequency points use:
        data = [data1; data2];
        [~, ind] = unique(data(:,1), 'stable');
        data = data(ind,:);


    Autor and Copyrigth: 2018 / 2019 / 2020 / M.E. Peter

*/

#include "GPA.h"

#include "math.h"
#include "mbed.h"
#define pi 3.14159265358979323846 // M_PI

using namespace std;

// -----------------------------------------------------------------------------
//      instantiate
// -----------------------------------------------------------------------------

GPA::GPA(float fMin, float fMax, int NfexcDes, float Aexc0, float Aexc1, float Ts)
{
    setup(fMin, fMax, NfexcDes, Aexc0, Aexc1, Ts);
}

GPA::GPA(float fMin,
         float fMax,
         int NfexcDes,
         int NperMin,
         int NmeasMin,
         float Ts,
         float Aexc0,
         float Aexc1,
         int Nstart,
         int Nsweep)
{
    doPrint = true;
    new_data_available = false;
    meas_is_finished = false;
    start_now = false;
    setParameters(fMin, fMax, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep, doPrint);
}

GPA::GPA(float f0,
         float f1,
         float *fexcDes,
         int NfexcDes,
         int NperMin,
         int NmeasMin,
         float Ts,
         float Aexc0,
         float Aexc1,
         int Nstart,
         int Nsweep)
{
    doPrint = true;
    new_data_available = false;
    meas_is_finished = false;

    assignParameters(NfexcDes, NperMin, NmeasMin, Ts, Nstart, Nsweep);

    // convert fexcDes from float to float, it is assumed that it is sorted
    this->fexcDes = (float *)malloc(NfexcDes * sizeof(float));
    for (int i = 0; i < NfexcDes; i++) {
        this->fexcDes[i] = (float)fexcDes[i];
    }

    calculateDecreasingAmplitudeCoefficients(Aexc0, Aexc1);
    initializeConstants(Ts);
    assignFilterStorage();
    reset();
}

GPA::GPA(
    float *fexcDes, int NfexcDes, int NperMin, int NmeasMin, float Ts, float Aexc0, float Aexc1, int Nstart, int Nsweep)
{
    doPrint = true;
    assignParameters(NfexcDes, NperMin, NmeasMin, Ts, Nstart, Nsweep);

    // convert fexcDes from float to float, it is assumed that it is sorted
    this->fexcDes = (float *)malloc(NfexcDes * sizeof(float));
    for (int i = 0; i < NfexcDes; i++) {
        this->fexcDes[i] = fexcDes[i];
    }

    calculateDecreasingAmplitudeCoefficients(Aexc0, Aexc1);
    initializeConstants(Ts);
    assignFilterStorage();
    reset();
}

GPA::GPA(float fMin,
         float fMax,
         int NfexcDes,
         int NperMin,
         int NmeasMin,
         float Ts,
         float Aexc0,
         float Aexc1,
         int Nstart,
         int Nsweep,
         bool doPrint)
{
    setParameters(fMin, fMax, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep, doPrint);
}

// -----------------------------------------------------------------------------
void GPA::setup(float fMin, float fMax, int NfexcDes, float Aexc0, float Aexc1, float Ts)
{
    doPrint = false;
    this->Ts = Ts;
    int NperMin = 3;
    int NmeasMin = (int)ceil(1.0f / Ts);
    int Nstart = (int)ceil(3.0f / Ts);
    int Nsweep = (int)ceil(0.0f / Ts);
    new_data_available = false;
    meas_is_finished = false;
    start_now = false;
    status = 0;

    setParameters(fMin, fMax, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep, doPrint);
}

// ------------------- deconstructor ---------------------
GPA::~GPA() {}

float GPA::get_Ts() { return this->Ts; }
uint8_t GPA::get_i() { return (uint8_t)testpoint; }
void GPA::setParameters(float fMin,
                        float fMax,
                        int NfexcDes,
                        int NperMin,
                        int NmeasMin,
                        float Ts,
                        float Aexc0,
                        float Aexc1,
                        int Nstart,
                        int Nsweep,
                        bool doPrint)
{
    this->doPrint = doPrint;
    assignParameters(NfexcDes, NperMin, NmeasMin, Ts, Nstart, Nsweep);

    // calculate logarithmic spaced frequency points
    fexcDes = (float *)malloc(NfexcDes * sizeof(float));
    fexcDesLogspace(fMin, fMax, NfexcDes);

    calculateDecreasingAmplitudeCoefficients(Aexc0, Aexc1);
    initializeConstants(Ts);
    assignFilterStorage();
    reset();
}

void GPA::reset()
{
    Nmeas = 0;
    Nper = 0;
    dfexc = 0.0;
    fexc = 0.0;
    fexcPast = 0.0f;
    dfexcj = 0.0f;
    f_iter = 1; // iterating through desired frequency points
    j_iter = 1; // iterating through measurement points w.r.t. reachable frequency
    scaleG = 0.0;
    cr = 0.0;
    ci = 0.0;
    for (int i = 0; i < 3; i++) {
        sU[i] = 0.0;
        sY[i] = 0.0;
    }
    sinarg = 0.0;
    sinargR = 0.0f;
    NmeasTotal = 0;
    Aexc = 0.0f;
    pi2Tsfexc = 0.0;
    Nsweep_i = Nstart;
    AexcOut = 0.0f;
    gpaData.fexc = 0.0f;
    gpaData.absGyu = 0.0f;
    gpaData.angGyu = 0.0f;
    gpaData.absGyr = 0.0f;
    gpaData.angGyr = 0.0f;
    gpaData.Umag = 0.0f;
    gpaData.Ymag = 0.0f;
    gpaData.Rmag = 0.0f;
    gpaData.MeasPointFinished = false;
    gpaData.MeasFinished = false;
    gpaData.ind = -1;
    status = 0;
}

// -----------------------------------------------------------------------------
//      update (operator)
// -----------------------------------------------------------------------------

float GPA::update(float inp, float out)
{
    // a new frequency point has been reached
    if (status != 2)
        return 0;
    if (j_iter == 1) {
        // user info
        if (f_iter == 1) {
            start_now = true;
        }

        // get a new unique frequency point
        while (fexc == fexcPast) {
            // measurement finished
            if (f_iter > NfexcDes) {
                gpaData.MeasPointFinished = false;
                gpaData.MeasFinished = true;
                meas_is_finished = true;
                status = 0;
                return 0.0f;
            }
            calcGPAmeasPara(fexcDes[f_iter - 1]);
            // secure fexc is not higher or equal to nyquist frequency
            if (fexc > fnyq) {
                fexc = fexcPast;
            }
            // no frequency found
            if (fexc == fexcPast) {
                f_iter += 1;
            } else {
                Aexc = aAexcDes / fexc + bAexcDes;
                pi2Tsfexc = pi2Ts * fexc;
            }
        }
        // filter scaling
        scaleG = 1.0 / sqrt((double)Nmeas);
        // filter coefficients
        cr = cos(pi2Tsfexc);
        ci = sin(pi2Tsfexc);
        // set filter storage zero
        for (int i = 0; i < 3; i++) {
            sU[i] = 0.0;
            sY[i] = 0.0;
        }
        gpaData.MeasPointFinished = false;
    }
    // perfomre the sweep or measure
    if (j_iter <= Nsweep_i) {
        dfexcj = ((float)j_iter - 1.0f) / ((float)Nsweep_i - 1.0f);
        dfexcj = div12pi * sinf(pi4 * dfexcj) - div812pi * sinf((float)pi2 * dfexcj) + dfexcj;
        dfexc = fexcPast + (fexc - fexcPast) * dfexcj;
        AexcOut = AexcPast + (Aexc - AexcPast) * dfexcj;
    } else {
        dfexc = fexc;
        AexcOut = Aexc;
        // one point DFT filter step for signal su
        sU[0] = scaleG * inp + 2.0 * cr * sU[1] - sU[2];
        sU[2] = sU[1];
        sU[1] = sU[0];
        // one point DFT filter step for signal sy
        sY[0] = scaleG * out + 2.0 * cr * sY[1] - sY[2];
        sY[2] = sY[1];
        sY[1] = sY[0];
    }
    // copy starting value for ang(R)
    if (j_iter == 1 || j_iter == Nsweep_i + 1)
        sinargR = sinarg;
    // measurement of frequencypoint is finished
    if (j_iter == Nmeas + Nsweep_i) {
        fexcPast = fexc;
        AexcPast = Aexc;
        Nsweep_i = Nsweep;
        // calculate the one point dft
        float Ureal = (float)(2.0 * scaleG * (cr * sU[1] - sU[2]));
        float Uimag = (float)(2.0 * scaleG * ci * sU[1]);
        float Yreal = (float)(2.0 * scaleG * (cr * sY[1] - sY[2]));
        float Yimag = (float)(2.0 * scaleG * ci * sY[1]);
        // calculate magnitude and angle
        float Umag = sqrtf(Ureal * Ureal + Uimag * Uimag);
        float Ymag = sqrtf(Yreal * Yreal + Yimag * Yimag);
        gpaData.fexc = (float)fexc;
        gpaData.absGyu = Ymag / Umag;
        gpaData.angGyu = wrapAngle(atan2f(Yimag, Yreal) - atan2f(Uimag, Ureal)) * rad2deg;
        gpaData.absGyr = Ymag / Aexc;
        gpaData.angGyr = wrapAngle(atan2f(Yimag, Yreal) + fmod(piDiv2 - sinargR, (float)pi2)) * rad2deg;
        gpaData.Umag = Umag;
        gpaData.Ymag = Ymag;
        gpaData.Rmag = Aexc;
        gpaData.MeasPointFinished = true;
        gpaData.ind++;
        // user info
        testpoint = (uint8_t)f_iter + 100;
        new_data_available = true;
        // printf("new_data_available = true;\r\n");
        if (doPrint) {
            // printf("%11.4e %9.3e %8.3f %9.3e %8.3f %9.3e %9.3e %9.3e\r\n", gpaData.fexc, gpaData.absGyu,
            // gpaData.angGyu, gpaData.absGyr, gpaData.angGyr, gpaData.Umag, gpaData.Ymag, gpaData.Rmag);
        }
        f_iter += 1;
        j_iter = 1;
    } else {
        j_iter += 1;
    }
    // calculate the excitation
    sinarg = fmod(sinarg + pi2Ts * dfexc, pi2);
    NmeasTotal += 1;
    return AexcOut * sinf(sinarg);
}

// -----------------------------------------------------------------------------
//      private functions
// -----------------------------------------------------------------------------

void GPA::assignParameters(int NfexcDes, int NperMin, int NmeasMin, float Ts, int Nstart, int Nsweep)
{
    this->NfexcDes = NfexcDes;
    this->NperMin = NperMin;
    this->NmeasMin = NmeasMin;
    this->Ts = Ts;
    this->Nstart = Nstart;
    this->Nsweep = Nsweep;
}

void GPA::calculateDecreasingAmplitudeCoefficients(float Aexc0, float Aexc1)
{
    // calculate coefficients for decreasing amplitude (1/fexc)
    this->aAexcDes = (Aexc1 - Aexc0) / (1.0f / fexcDes[NfexcDes - 1] - 1.0f / fexcDes[0]);
    this->bAexcDes = Aexc0 - aAexcDes / fexcDes[0];
}

void GPA::initializeConstants(float Ts)
{
    fnyq = 1.0f / 2.0f / Ts;
    pi2 = 2.0 * pi;
    pi4 = 4.0f * (float)pi;
    pi2Ts = 2.0 * pi * (double)Ts; // 2.0*pi*Ts;
    piDiv2 = (float)pi / 2.0f;
    rad2deg = 180.0f / (float)pi;
    div12pi = 1.0f / (12.0f * (float)pi);
    div812pi = 8.0f / (12.0f * (float)pi);
}

void GPA::assignFilterStorage()
{
    sU = (double *)malloc(3 * sizeof(double));
    sY = (double *)malloc(3 * sizeof(double));
}

void GPA::fexcDesLogspace(float fMin, float fMax, int NfexcDes)
{
    // calculate logarithmic spaced frequency points
    float Gain = log10f(fMax / fMin) / ((float)NfexcDes - 1.0f);
    float expon = 0.0f;
    for (int i = 0; i < NfexcDes; i++) {
        fexcDes[i] = fMin * powf(10.0f, expon);
        expon += Gain;
    }
}

void GPA::calcGPAmeasPara(float fexcDes_i)
{
    // Nmeas has to be an integer
    Nper = NperMin;
    Nmeas = (int)floor((float)Nper / fexcDes_i / Ts + 0.5f);
    //  secure that the minimal number of measurements is fullfilled
    int Ndelta = NmeasMin - Nmeas;
    if (Ndelta > 0) {
        Nper = (int)ceil((float)NmeasMin * fexcDes_i * Ts);
        Nmeas = (int)floor((float)Nper / fexcDes_i / Ts + 0.5f);
    }
    // evaluating reachable frequency
    fexc = (float)((double)Nper / (double)Nmeas / (double)Ts);
}

float GPA::wrapAngle(float angle)
{
    // wrap angle from (-2pi,2pi) into (-pi,pi)
    if (abs(angle) > (float)pi)
        angle -= copysignf(-(float)pi2, angle); // -1*sign(angle)*2*pi + angle;
    return angle;
}

void GPA::printLongLine()
{
    // printf("-------------------------------------------------------------------------------------------------------------------------------\r\n");
}

// -----------------------------------------------------------------------------
//      public functions (mainly for debugging)
// -----------------------------------------------------------------------------

void GPA::printGPAfexcDes()
{
    printLine();
    for (int i = 0; i < NfexcDes; i++) {
        printf("%9.4f\r\n", fexcDes[i]);
    }
}

void GPA::printGPAmeasPara()
{
    printLine();
    printf(" fexcDes[Hz]   fexc[Hz]     Aexc      Nmeas   Nper  Nsweep_i\r\n");
    printLine();
    for (int i = 0; i < NfexcDes; i++) {
        calcGPAmeasPara(fexcDes[i]);
        if (fexc == fexcPast || fexc >= fnyq) {
            fexc = 0.0;
            Aexc = 0.0f;
            Nmeas = 0;
            Nper = 0;
            Nsweep_i = 0;
        } else {
            Aexc = aAexcDes / fexc + bAexcDes;
            fexcPast = fexc;
            AexcPast = Aexc;
        }
        NmeasTotal += Nmeas;
        NmeasTotal += Nsweep_i;
        printf("%11.4e %12.4e %10.3e %7i %6i %7i\r\n", fexcDes[i], (float)fexc, Aexc, Nmeas, Nper, Nsweep_i);
        //        wait(0.01);
        Nsweep_i = Nsweep;
    }
    printGPAmeasTime();
    reset();
}

void GPA::printGPAmeasTime()
{
    printLine();
    printf(" Number of data points :  %11i\r\n", NmeasTotal);
    printf(" Measurment time in sec: %12.2f\r\n", (float)NmeasTotal * Ts);
}

void GPA::printNfexcDes()
{
    printLine();
    printf("  Number of frequancy points:  %3i\r\n", NfexcDes);
}

void GPA::printLine()
{
    printf("--------------------------------------------------------------------------------\r\n");
}

void GPA::getGPAdata(float *val)
{
    val[0] = gpaData.fexc;
    val[1] = gpaData.absGyu;
    val[2] = gpaData.angGyu;
    val[3] = gpaData.absGyr;
    val[4] = gpaData.angGyr;
    val[5] = gpaData.Umag;
    val[6] = gpaData.Ymag;
    val[7] = gpaData.Rmag;
    new_data_available = false;
}

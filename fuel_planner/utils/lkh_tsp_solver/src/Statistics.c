#include "LKH.h"

static int TrialsMin, TrialsMax, TrialSum, Successes;
static GainType CostMin, CostMax, CostSum;
static double TimeMin, TimeMax, TimeSum;

void InitializeStatistics()
{
    TrialSum = Successes = 0;
    CostSum = 0;
    TimeSum = 0.0;
    TrialsMin = INT_MAX;
    TrialsMax = 0;
    TimeMin = DBL_MAX;
    TimeMax = 0;
    CostMin = PLUS_INFINITY;
    CostMax = MINUS_INFINITY;
}

void UpdateStatistics(GainType Cost, double Time)
{
    if (Trial < TrialsMin)
        TrialsMin = Trial;
    if (Trial > TrialsMax)
        TrialsMax = Trial;
    TrialSum += Trial;
    if (Cost <= Optimum)
        Successes++;
    if (Cost < CostMin)
        CostMin = Cost;
    if (Cost > CostMax)
        CostMax = Cost;
    CostSum += Cost;
    if (Time < TimeMin)
        TimeMin = Time;
    if (Time > TimeMax)
        TimeMax = Time;
    TimeSum += Time;
}

void PrintStatistics()
{
    int _Runs = Runs, _TrialsMin = TrialsMin;
    double _TimeMin = TimeMin;
    GainType _Optimum = Optimum;

    // printff("Successes/Runs = %d/%d\n", Successes, Runs);
    if (_Runs == 0)
        _Runs = 1;
    if (_TrialsMin > TrialsMax)
        _TrialsMin = 0;
    if (_TimeMin > TimeMax)
        _TimeMin = 0;
    if (CostMin <= CostMax && CostMin != PLUS_INFINITY) {
        // printff
        //     ("Cost.min = " GainFormat ", Cost.avg = %0.2f, Cost.max = "
        //      GainFormat "\n", CostMin, (double) CostSum / _Runs, CostMax);
        if (_Optimum == MINUS_INFINITY)
            _Optimum = BestCost;
        if (_Optimum != 0);
            // printff
            //     ("Gap.min = %0.4f%%, Gap.avg = %0.4f%%, Gap.max = %0.4f%%\n",
            //      100.0 * (CostMin - _Optimum) / _Optimum,
            //      100.0 * ((double) CostSum / _Runs - _Optimum) / _Optimum,
            //      100.0 * (CostMax - _Optimum) / _Optimum);

    }
    // printff("Trials.min = %d, Trials.avg = %0.1f, Trials.max = %d\n",
    //         _TrialsMin, 1.0 * TrialSum / _Runs, TrialsMax);
    // printff
    //     ("Time.min = %0.2f sec., Time.avg = %0.2f sec., Time.max = %0.2f sec.\n",
    //      fabs(_TimeMin), fabs(TimeSum) / _Runs, fabs(TimeMax));
}

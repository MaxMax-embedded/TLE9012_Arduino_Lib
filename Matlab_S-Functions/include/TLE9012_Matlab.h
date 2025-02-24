#ifndef _TLE9012_MATLAB_H_
#define _TLE9012_MATLAB_H_

#ifdef __cplusplus
extern "C" {
#endif

    void init_tle9012(void);
    void get_cell_voltages(float*);
    void get_cell_voltages_with_current(float*, float*);
    void tle9012_Terminate(void);


#ifdef __cplusplus
}
#endif

#endif
//
// Created by 15873 on 2025-03-17.
//

#ifndef DYNAMIC_H
#define DYNAMIC_H
#include "stm32f405xx.h"
#include "arm_math.h"
#include "memory.h"


class DOF6Dynamic {
public:
    explicit DOF6Dynamic( float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT,float g);
    ~DOF6Dynamic();

    void Yr_clc(const float* q,const float* dq,const float* ddq, float* Yr);
private:
    const float RAD_TO_DEG = 57.29577951308232f;
    struct DyArmConfig_t
    {
        float L_BASE;
        float D_BASE;
        float L_ARM;
        float L_FOREARM;
        float D_ELBOW;
        float L_WRIST;
        float gravity;
    };
    DyArmConfig_t dyarmConfig;
    // const float min_parameters[36];
    void scalarMultiply(const float* vec, float scalar, float* result);
    void crossProduct(const float* A, const float* B, float* C);
    void crossProduct3(const float* A, const float* B, const float* C,  float* output);
    void MatrixAddition2(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        const int _m, const int _n);
    void MatrixAddition3(const float* _matrix1, const float* _matrix2,const float* _matrix3, float* _matrixOut,
                        const int _m, const int _n);
    void MatrixMultiplication(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        const int _m, const int _l, const int _n);
    void MDHTrans(float alpha,float a,float d,float theta,float* R, float* R_T, float* P );
    void motion_para_clc(const float *R_inv,const float dq,const float ddq,const float* w_pre,const float* dw_pre,
        const float* dv_pre,const float* Po, const float* axis, float* w,float* dw,float* dv);
    void getK(const float* vec, float* K);
    void getS(const float* vec, float* S);
    void getHi(const float* wi,const float* dwi,const float* dvi,float* Hi);
    void getAi(const float* wi,const float* dwi,const float* dvi,float* Ai);
    void get_Yf_Yn(int id,const float* R,const float* H,const float* A,const float* Yf_next,
        const float* Yn_next,const float* Po,float* Yf,float* Yn);
    void Ymatrix_clc(const float* q,const float* dq,const float* ddq, float* Y);

};



#endif //DYNAMIC_H

//
// Created by 15873 on 2025-03-17.
//

#include "dynamic.h"

#include <cstring>

inline float cosf_(float x)
{
    return arm_cos_f32(x);
}

inline float sinf_(float x)
{
    return arm_sin_f32(x);
}
DOF6Dynamic::DOF6Dynamic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT, float g)
: dyarmConfig(DyArmConfig_t{L_BS, D_BS, L_AM, L_FA, D_EW, L_WT, g})
{
}


void DOF6Dynamic::scalarMultiply(const float* vec, float scalar, float* result) {
    for (int i = 0; i < 3; i++) {
        result[i] = vec[i] * scalar;
    }
}
void DOF6Dynamic::crossProduct(const float* A, const float* B, float* C) {
    C[0] = A[1] * B[2] - A[2] * B[1];
    C[1] = A[2] * B[0] - A[0] * B[2];
    C[2] = A[0] * B[1] - A[1] * B[0];
}
void DOF6Dynamic::crossProduct3(const float* A, const float* B, const float* C,  float* output)
{
    float temp1[3];
    crossProduct(B, C, temp1);
    crossProduct(A, temp1, output);
}
void DOF6Dynamic::MatrixAddition2(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        const int _m, const int _n)
{
    int i, j;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            _matrixOut[i * _n + j] = _matrix1[i * _n + j] + _matrix2[i * _n + j];
        }
    }
}
void DOF6Dynamic::MatrixAddition3(const float* _matrix1, const float* _matrix2,const float* _matrix3, float* _matrixOut,
                        const int _m, const int _n)
{
    int i, j;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            _matrixOut[i * _n + j] = _matrix1[i * _n + j] + _matrix2[i * _n + j]+_matrix3[i * _n + j];
        }
    }
}
void DOF6Dynamic::MatrixMultiplication(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        const int _m, const int _l, const int _n)
{
    float tmp;
    int i, j, k;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            tmp = 0.0f;
            for (k = 0; k < _l; k++)
            {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }
    }
}
void DOF6Dynamic::MDHTrans(float alpha,float a,float d,float theta,float* R, float* R_T,float* P )
{
    float sa,ca,st,ct;
    sa = sinf_(alpha);
    ca = cosf_(alpha);
    st = sinf_(theta);
    ct = cosf_(theta);
    R[0] = ct;
    R[1] = -st;
    R[2] = 0;
    R[3] = st*ca;
    R[4] = ct*ca;
    R[5] = -sa;
    R[6] = st*sa;
    R[7] = ct*sa;
    R[8] = ca;

    R_T[0] = R[0];
    R_T[1] = R[3];
    R_T[2] = R[6];
    R_T[3] = R[1];
    R_T[4] = R[4];
    R_T[5] = R[7];
    R_T[6] = R[2];
    R_T[7] = R[5];
    R_T[8] = R[8];

    P[0] = a;
    P[1] = -d*sa;
    P[2] = d*ca;
}
void DOF6Dynamic::motion_para_clc(const float *R_inv,const float dq,const float ddq,const float* w_pre,const float* dw_pre,
        const float* dv_pre,const float* Po, const float* axis, float* w,float* dw,float* dv)
{
    float temp1[3];
    float temp2[3];
    float temp3[3];
    float temp4[3];
    float temp5[3];

    MatrixMultiplication(R_inv, w_pre, temp1, 3, 3, 1);
    scalarMultiply(axis, dq, temp2);
    MatrixMultiplication(R_inv, dw_pre, temp3, 3, 3, 1);
    crossProduct(temp1, temp2, temp4);
    scalarMultiply(axis, ddq, temp5);

    MatrixAddition2(temp1, temp2, w, 3, 1);
    MatrixAddition3(temp3, temp4,temp5, dw, 3, 1);

    crossProduct(dw_pre, Po, temp1);
    crossProduct3(w_pre, w_pre, Po, temp2);
    MatrixAddition3(temp1, temp2, dv_pre,temp3, 3, 1);
    MatrixMultiplication(R_inv, temp3, dv, 3, 3, 1);
}
void DOF6Dynamic::getK(const float* vec, float* K)
{
    for (int i = 0; i < 18; i++) {
        K[i] = 0.0f;
    }
    K[0] = vec[0];
    K[1] = vec[1];
    K[2] = vec[2];

    K[7] = vec[0];
    K[9] = vec[1];
    K[10] = vec[2];

    K[14] = vec[0];
    K[16] = vec[1];
    K[17] = vec[2];

}
void DOF6Dynamic::getS(const float* vec, float* S)
{
    S[0] =0.0f;
    S[1] = -vec[2];
    S[2] = vec[1];
    S[3] = vec[2];
    S[4] = 0.0f;
    S[5] = -vec[0];
    S[6] = -vec[1];
    S[7] = vec[0];
    S[8] = 0.0f;
}
void DOF6Dynamic::getHi(const float* wi,const float* dwi,const float* dvi,float* Hi)
{
    float temp1[9];
    float temp2[9];
    float temp3[9];
    float temp4[9];
    getS(dwi, temp1);
    getS(wi,temp2);
    MatrixMultiplication(temp2, temp2, temp3, 3, 3, 3);
    MatrixAddition2(temp1, temp3, temp4, 3, 3);

    float H[30];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            H[i * 10 + j] = 0.0f;
        }
        for (int j = 0; j < 3; j++)
        {
            H[i * 10 + j + 6] = temp4[3 * i + j];
        }
        for (int j = 0; j < 1; j++)
        {
            H[i * 10 + j + 9] = dvi[1*i+j];
        }
    }
    memcpy(Hi, H, sizeof(float)*30);
}
void DOF6Dynamic::getAi(const float* wi,const float* dwi,const float* dvi,float* Ai)
{
    float temp1[18];
    float temp2[9];
    float temp3[18];
    float temp4[18];
    getK(dwi, temp1);
    getS(wi,temp2);
    MatrixMultiplication(temp2, temp1, temp3, 3, 3, 6);
    MatrixAddition2(temp1, temp3, temp4, 3, 6);

    getS(dvi,temp2);

    float A[30];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            A[i * 10 + j] = temp4[6*i+j];
        }
        for (int j = 0; j < 3; j++)
        {
            A[i * 10 + j + 6] = -temp2[3 * i + j];
        }
        for (int j = 0; j < 1; j++)
        {
            A[i * 10 + j + 9] = 0.0f;
        }
    }
    memcpy(Ai, A, sizeof(float)*30);
}
void DOF6Dynamic::get_Yf_Yn(int id,const float* R,const float* H,const float* A,const float* Yf_next,const float* Yn_next,const float* Po,float* Yf,float* Yn)
{
    int c1 = (id-1)*10+1-1;
    int c2 = (id-1)*10+10-1;
    float Yf_temp[180];
    float Yn_temp[180];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < c1; j++)
        {
            Yf_temp[i*60+j] = 0.0f;
            Yn_temp[i*60+j] = 0.0f;
        }
        for (int j = 0; j < c2-c1+1; j++)
        {
            Yf_temp[i*60+j+c1] = H[10*i+j];
            Yn_temp[i*60+j+c1] = A[10*i+j];
        }
        for (int j = 0; j < 60-c2-1; j++)
        {
            Yf_temp[i*60+c2+1+j] = 0.0f;
            Yn_temp[i*60+c2+1+j] = 0.0f;
        }
    }
    float temp1[180];
    MatrixMultiplication(R, Yf_next, temp1, 3, 3, 60);
    MatrixAddition2(Yf_temp, temp1, Yf, 3, 60);
    float temp2[180];
    MatrixMultiplication(R, Yn_next, temp2, 3, 3, 60);
    float temp3[9];
    getS(Po, temp3);
    float temp4[180];
    MatrixMultiplication(temp3, temp1, temp4, 3,3, 60);
    MatrixAddition3(Yn_temp,temp2, temp4, Yn, 3, 60);

}
void DOF6Dynamic::Ymatrix_clc(const float* q,const float* dq,const float* ddq, float* Y)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3], q5 = q[4], q6 = q[5];
    float dq1 = dq[0], dq2 = dq[1], dq3 = dq[2], dq4 = dq[3], dq5 = dq[4], dq6 = dq[5];
    float ddq1 = ddq[0], ddq2 = ddq[1], ddq3 = ddq[2], ddq4 = ddq[3], ddq5 = ddq[4], ddq6 = ddq[5];

    float axis1[3] = {0.0f, 0.0f, 1.0f};
    float axis2[3] = {0.0f, 0.0f, 1.0f};
    float axis3[3] = {0.0f, 0.0f, 1.0f};
    float axis4[3] = {0.0f, 0.0f, 1.0f};
    float axis5[3] = {0.0f, 0.0f, 1.0f};
    float axis6[3] = {0.0f, 0.0f, 1.0f};

    float R01[9], R12[9], R23[9], R34[9], R45[9], R56[9];
    float R01T[9], R12T[9], R23T[9], R34T[9], R45T[9], R56T[9];
    float P01[3], P12[3], P23[3], P34[3], P45[3], P56[3];

    MDHTrans(0.0f             , 0.0f                  , dyarmConfig.L_BASE   , q1, R01, R01T, P01);
    MDHTrans(-90.0f/RAD_TO_DEG, dyarmConfig.D_BASE    , 0.0f                 , q2, R12, R12T, P12);
    MDHTrans(0.0f             , dyarmConfig.L_ARM     , 0.0f                 , q3, R23, R23T, P23);
    MDHTrans(-90.0f/RAD_TO_DEG, dyarmConfig.D_ELBOW   , dyarmConfig.L_FOREARM, q4, R34, R34T, P34);
    MDHTrans(90.0f/RAD_TO_DEG , 0.0f                  , 0.0f                 , q5, R45, R45T, P45);
    MDHTrans(-90.0f/RAD_TO_DEG, 0.0f                  , dyarmConfig.L_WRIST  , q6, R56, R56T, P56);

    float w00[3] = {0.0f, 0.0f, 0.0f};
    float dw00[3] = {0.0f, 0.0f, 0.0f};
    float dv00[3] = {0.0f, 0.0f, dyarmConfig.gravity};

    float w11[3], w22[3], w33[3], w44[3], w55[3], w66[3];
    float dw11[3], dw22[3], dw33[3], dw44[3], dw55[3], dw66[3];
    float dv11[3], dv22[3], dv33[3], dv44[3], dv55[3], dv66[3];
    motion_para_clc(R01T,dq1,ddq1,w00,dw00,dv00,P01,axis1,w11,dw11,dv11);
    motion_para_clc(R12T,dq2,ddq2,w11,dw11,dv11,P12,axis2,w22,dw22,dv22);
    motion_para_clc(R23T,dq3,ddq3,w22,dw22,dv22,P23,axis3,w33,dw33,dv33);
    motion_para_clc(R34T,dq4,ddq4,w33,dw33,dv33,P34,axis4,w44,dw44,dv44);
    motion_para_clc(R45T,dq5,ddq5,w44,dw44,dv44,P45,axis5,w55,dw55,dv55);
    motion_para_clc(R56T,dq6,ddq6,w55,dw55,dv55,P56,axis6,w66,dw66,dv66);

    float H1[30], H2[30], H3[30], H4[30], H5[30], H6[30];
    float A1[30], A2[30], A3[30], A4[30], A5[30], A6[30];
    getHi(w11,dw11,dv11,H1);
    getHi(w22,dw22,dv22,H2);
    getHi(w33,dw33,dv33,H3);
    getHi(w44,dw44,dv44,H4);
    getHi(w55,dw55,dv55,H5);
    getHi(w66,dw66,dv66,H6);

    getAi(w11,dw11,dv11,A1);
    getAi(w22,dw22,dv22,A2);
    getAi(w33,dw33,dv33,A3);
    getAi(w44,dw44,dv44,A4);
    getAi(w55,dw55,dv55,A5);
    getAi(w66,dw66,dv66,A6);

    float Yf6[180], Yf5[180], Yf4[180], Yf3[180], Yf2[180], Yf1[180];
    float Yn6[180], Yn5[180], Yn4[180], Yn3[180], Yn2[180], Yn1[180];

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 50; j++)
        {
            Yf6[i*60+j] = 0.0f;
            Yn6[i*60+j] = 0.0f;
        }
        for (int j = 0; j < 10; j++)
        {
            Yf6[i*60+j+50] = H6[10*i+j];
            Yn6[i*60+j+50] = A6[10*i+j];
        }
    }

    get_Yf_Yn(5,R56,H5,A5,Yf6,Yn6,P56,Yf5,Yn5);
    get_Yf_Yn(4,R45,H4,A4,Yf5,Yn5,P45,Yf4,Yn4);
    get_Yf_Yn(3,R34,H3,A3,Yf4,Yn4,P34,Yf3,Yn3);
    get_Yf_Yn(2,R23,H2,A2,Yf3,Yn3,P23,Yf2,Yn2);
    get_Yf_Yn(1,R12,H1,A1,Yf2,Yn2,P12,Yf1,Yn1);

    float Y1[60], Y2[60], Y3[60], Y4[60], Y5[60], Y6[60];
    MatrixMultiplication(axis1, Yn1, Y1, 1, 3, 60);
    MatrixMultiplication(axis2, Yn2, Y2, 1, 3, 60);
    MatrixMultiplication(axis3, Yn3, Y3, 1, 3, 60);
    MatrixMultiplication(axis4, Yn4, Y4, 1, 3, 60);
    MatrixMultiplication(axis5, Yn5, Y5, 1, 3, 60);
    MatrixMultiplication(axis6, Yn6, Y6, 1, 3, 60);
    float Y_temp[360];

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 60; j++) {
            if(i==0) Y_temp[i * 60 + j] = Y1[j];
            else if(i==1) Y_temp[i * 60 + j] = Y2[j];
            else if(i==2) Y_temp[i * 60 + j] = Y3[j];
            else if(i==3) Y_temp[i * 60 + j] = Y4[j];
            else if(i==4) Y_temp[i * 60 + j] = Y5[j];
            else if(i==5) Y_temp[i * 60 + j] = Y6[j];
        }
    }
    memcpy(Y, Y_temp, sizeof(float)*360);
}
void DOF6Dynamic::Yr_clc(const float* q,const float* dq,const float* ddq, float* Yr)
{
    float Y_temp[360];
    float Yr_temp[216];
    const int SELECTED_COLS[36] = {5,
                                  10,11,12,14,15,16,17,
                                  20,21,22,24,25,26,27,
                                  30,31,32,34,35,36,37,
                                  40,41,42,44,45,46,47,
                                  50,51,52,54,55,56,57};
    Ymatrix_clc(q,dq,ddq,Y_temp);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 36; j++) {
            Yr_temp[i * 36 + j] = Y_temp[i * 60 + SELECTED_COLS[j]];  // 线性索引取值
        }
    }
    memcpy(Yr, Yr_temp, sizeof(float)*6*36);
}


/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

__kernel void matrix_mult(
    const int Ndim,
    const int Mdim,
    const int Pdim,
    __global const float* A,
    __global const float* B,
    __global float* C)
{
    int i = get_global_id(0);
    int j = get_global_id(1);

    int k;
    float tmp;

    if ((i < Ndim) && (j < Mdim)) {
        tmp = 0.0;
        for (k = 0; k < Pdim; k++)
            tmp += A[i*Pdim + k] * B[k*Mdim + j];
        C[i*Mdim + j] = tmp;
    }
}

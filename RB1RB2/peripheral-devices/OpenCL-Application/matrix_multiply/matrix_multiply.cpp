/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <CL/cl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <fstream>

#pragma comment(lib, "OpenCL.lib")

cl_uint status;
cl_platform_id platform;
cl_device_id device;
cl_context context;
cl_command_queue commandQueue;
cl_kernel kernel;
cl_program program;
cl_event prof_event;
cl_mem memObjects[3] = { 0, 0, 0 };

const int Ndim = 20;
const int Mdim = 20;
const int Pdim = 20;

//  read file to string
int convertToString(const char *filename, std::string& str) {
    using std::fstream;
    char* filestr;
    fstream f(filename, (fstream::in | fstream::binary));
    if (f.is_open()) {
        size_t len;
        f.seekg(0, fstream::end);
        len = (size_t)f.tellg();
        f.seekg(0, fstream::beg);
        filestr = (char *) malloc(len + 1);
        f.read(filestr, len);
        filestr[len] = '\0';
        str = filestr;
        free(filestr);
        f.close();
        return 0;
    }
    std::cout << "open file filed" << std::endl;
    return -1;
}

void cl_init() {
    status = clGetPlatformIDs(1, &platform, NULL);
    if (status < 0) {
      perror("Couldn't identify a platform");
      exit(1);
    }
    clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);
    if (status < 0) {
      perror("Couldn't access any devices");
      exit(1);
    }
    context = clCreateContext(NULL, 1, &device, NULL, NULL, NULL);
    commandQueue = clCreateCommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE, NULL);

    if (commandQueue == NULL) {
        perror("Failed to create commandQueue for device 0.");
    }
}

void cl_create_kernel() {
    using std::cout;
    using std::endl;
    using std::string;
    const char * filename = "Matrix multiply.cl";
    string sourceStr;
    status = convertToString(filename, sourceStr);
    const char * source = sourceStr.c_str();
    size_t sourceSize[] = { strlen(source) };
    program = clCreateProgramWithSource(context, 1, &source, sourceSize, NULL);
    status = clBuildProgram(program, 1, &device, NULL, NULL, NULL);
    if (status != 0) {
        printf("clBuildProgram failed : %d\n", status);
        char tbuf[0x10000];
        clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0x10000, tbuf, NULL);
        printf("\n%s\n", tbuf);
    }
    kernel = clCreateKernel(program, "matrix_mult", NULL);
}

void cl_set_kernel_arg() {
    using std::cout;
    using std::endl;
    status = clSetKernelArg(kernel, 0, sizeof(int), &Ndim);
    if (status) {
        cout << "arguments set error" << endl;
    }

    status = clSetKernelArg(kernel, 1, sizeof(int), &Mdim);
    if (status) {
        cout << "arguments set error" << endl;
    }

    status = clSetKernelArg(kernel, 2, sizeof(int), &Pdim);
    if (status) {
        cout << "arguments set error" << endl;
    }

    status = clSetKernelArg(kernel, 3, sizeof(cl_mem), &memObjects[0]);
    if (status) {
        cout << "arguments set error" << endl;
    }

    status = clSetKernelArg(kernel, 4, sizeof(cl_mem), &memObjects[1]);
    if (status) {
        cout << "arguments set error" << endl;
    }

    status = clSetKernelArg(kernel, 5, sizeof(cl_mem), &memObjects[2]);
    if (status) {
        cout << "arguments set error" << endl;
    }
}

void cont_runtime() {
    using std::cout;
    using std::endl;
    cl_ulong ev_start_time = (cl_ulong)0;
    cl_ulong ev_end_time = (cl_ulong)0;
    uint64_t rum_time;

    status = clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_QUEUED, sizeof(cl_ulong), &ev_start_time, NULL);

    status = clGetEventProfilingInfo(prof_event, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &ev_end_time, NULL);

    rum_time = ev_end_time - ev_start_time;
    cout << "run time :" << rum_time << endl;
}

void free_mem(float *mem) {
    if (mem)
        free(mem);
}

void cl_release() {
    clReleaseMemObject(memObjects[2]);
    clReleaseMemObject(memObjects[1]);
    clReleaseMemObject(memObjects[0]);
    clReleaseProgram(program);
    clReleaseCommandQueue(commandQueue);
    clReleaseContext(context);
}

int main() {
    using std::cout;
    using std::endl;
    int i, j;
    int size_A = Ndim * Pdim;
    int size_B = Pdim * Mdim;
    int size_C = Ndim * Mdim;
    float *MEM_A = new float[size_A * sizeof(float)];
    float *MEM_B = new float[size_B * sizeof(float)];
    float *MEM_C = new float[size_C * sizeof(float)];

    for (i = 0; i < size_A; i++)
        MEM_A[i] = 1.0f * (1.0f * i + 1.0);
    for (i = 0; i < size_B; i++)
        MEM_B[i] = 1.0f * (1.0f * i + 1.0);

    cl_init();

    memObjects[0] = clCreateBuffer(context, CL_MEM_READ_ONLY |  CL_MEM_COPY_HOST_PTR,
                                   sizeof(float)* size_A, MEM_A, NULL);
    memObjects[1] = clCreateBuffer(context, CL_MEM_READ_ONLY |  CL_MEM_COPY_HOST_PTR,
                                   sizeof(float)* size_B, MEM_B, NULL);
    memObjects[2] = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                   sizeof(float)* size_C, MEM_C, NULL);
    if (memObjects[0] == NULL || memObjects[1] == NULL ||memObjects[2] == NULL)
        perror("Error in clCreateBuffer.\n");

    cl_create_kernel();
    cl_set_kernel_arg();

    size_t global[2];
    global[0] = (size_t)Ndim;
    global[1] = (size_t)Mdim;

    status = clEnqueueNDRangeKernel(commandQueue, kernel, 2, NULL, global, NULL, 0, NULL, &prof_event);
    clFinish(commandQueue);

    cont_runtime();

    status = clEnqueueReadBuffer(commandQueue, memObjects[2], CL_TRUE, 0, sizeof(float)* size_C,
                                 MEM_C, 0, NULL, NULL);

    printf("\nArray A:\n");
    for (i = 0; i < Ndim; i++) {
        for (j = 0; j < Pdim; j++)
             printf("%.3f\t", MEM_A[i*Pdim + j]);
        printf("\n");
    }
    printf("\nArray B:\n");
    for (i = 0; i < Pdim; i++) {
        for (j = 0; j < Mdim; j++)
             printf("%.3f\t", MEM_B[i*Mdim + j]);
        printf("\n");
    }
    printf("\nArray C:\n");
    for (i = 0; i < Ndim; i++) {
        for (j = 0; j < Mdim; j++)
             printf("%.3f\t", MEM_C[i*Mdim + j]);
        printf("\n");
    }

    cout << endl;

    free_mem(MEM_A);
    free_mem(MEM_B);
    free_mem(MEM_C);

    cl_release();

    return 0;
}


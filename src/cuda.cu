#include <stdio.h>
#include <iostream>
using namespace std;

#include <cuda.h>
#include <cuda_runtime.h>

#include <cublas_v2.h>

#include <Eigen/Dense>
#include <Eigen/Core>

//const int N = 131072; //2^17
const int N = 7;



__global__
void hello(float * b )
{
    int index = blockIdx.x*blockDim.x + threadIdx.x;
    b[index] *= 2;
}


void displayDeviceInfo()
{
    int devCount;
    cudaGetDeviceCount(&devCount);
    printf( "[# of cuda devices] : %d \n", devCount );
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0 );
    printf( "[Device Name] : %s\n", prop.name );
    printf( "[maxThreadsPerMultiProcessor] : %d\n", prop.maxThreadsPerMultiProcessor );
    printf( "[maxThreadsPerBlock] : %d\n", prop.maxThreadsPerBlock );
}

void printArray( int * ary, int nCount )
{
    printf( "[ ");

    if( nCount < 15 )
    {
    for( int i=0 ; i<nCount ; i++ )
        printf( "%3d, ", ary[i]);
    }
    else
    {
        for( int i=0 ; i<5 ; i++ )
            printf( "%3d, ", ary[i]);

        printf( "., ., ., ");

        for( int i=nCount-6 ; i<nCount ; i++ )
            printf( "%3d, ", ary[i]);

    }

    printf( "]\n");
}


void printArrayFloat( float * ary, int nCount )
{
    printf( "[ ");

    if( nCount < 15 )
    {
    for( int i=0 ; i<nCount ; i++ )
        printf( "%3.3f, ", ary[i]);
    }
    else
    {
        for( int i=0 ; i<5 ; i++ )
            printf( "%3.3f, ", ary[i]);

        printf( "., ., ., ");

        for( int i=nCount-6 ; i<nCount ; i++ )
            printf( "%3.3f, ", ary[i]);

    }

    printf( "]\n");
}

void printMatrixColMajor( float *mat, int rows, int cols )
{
     printf( "[ ");
    //note: matrix is in col-major format
    for( int i=0 ; i<rows ; i++ )
    {
        for( int j=0 ; j<cols ; j++ )
        {
            int index = i + j*cols;
            printf( "%5.3f, ", mat[index]);
        }
        printf( "\n");
    }
    printf( "]\n");
}


int verifyVector( int len, float * cpu, float * gpu )
{
    //verify
    int notMatchCount = 0 ;
    for( int i=0 ; i<N*N ; i++ )
    {
        if( cpu[i] != gpu[i] )
        {
            //cout << "Does not match at : "<< i << endl;
            notMatchCount++;
        }
    }
    cout << "Does not match at : `"<< notMatchCount << "` locations"<< endl;
    return notMatchCount;

}


int testmain()
{


    displayDeviceInfo();


    //int b[N] = {15, 10, 6, 0, -11, 1, 0};
    float *b = new float[N];
    for( int i=0 ; i<N ; i++ )
        b[i] = (float) ( rand()%100 ) - 50.0;
    printArrayFloat(b,N);

    Eigen::VectorXf eig_b(N);
    for( int i=0 ; i<N ; i++ )
        eig_b(i) = b[i];

    std::cout << "B (eigen): " << eig_b.transpose() << std::endl;
    Eigen::MatrixXf eig_out = eig_b * eig_b.transpose();
    std::cout << "WIth Eigen B*B': "<< eig_out << std::endl;




    float *b_dev, *by_dev;
    const int isize = N*sizeof(float);
    cudaMalloc((void **)&b_dev, isize );
    cudaMalloc((void **)&by_dev, isize );




//    float outb[N];

//    cudaMalloc( (void**)&b_dev, isize );
//    cudaMemcpy( b_dev, b, isize, cudaMemcpyHostToDevice );

//    dim3 dimBlock( 1024, 1 );
//    dim3 dimGrid( N/1024, 1 );
//    hello<<<dimGrid, dimBlock>>>(b_dev);
//    cudaMemcpy( outb, b_dev, isize, cudaMemcpyDeviceToHost );

//    printArray( b, N );
//    printArray( outb, N );



    cublasHandle_t handle;
    cublasCreate(&handle);


//    cublasSetVector(N, sizeof(float), (void*)b, 1, b_dev, 1);
//    int resultIndx;
//    //max
//    cublasIsamax(handle, N, b_dev, 1, &resultIndx);
//    printf( "Max [%d]: %f\n", resultIndx, b[resultIndx-1] );


//    //min
//    cublasIsamin(handle, N, b_dev, 1, &resultIndx);
//    printf( "Min [%d]: %f\n", resultIndx, b[resultIndx-1] );



    // cublas rank-1 update

    //create and populate matrix A
    float *A_dev;
    cudaMalloc((void**)&A_dev, N*N*sizeof(float));
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(eig_b.rows(), eig_b.rows() );
    std::cout << "Matrix A : "<< A << std::endl;


    cublasSetVector(N, sizeof(float), (void*)b, 1, b_dev, 1);
    cublasSetVector(N, sizeof(float), (void*)b, 1, by_dev, 1);
    cublasSetMatrix(A.rows(), A.cols(), sizeof(float), (void*)A.data(), A.rows(), (void*)A_dev, A.rows() );


    float alpha = 1.0f;
    cublasSger( handle, A.rows(), A.cols(), &alpha, b_dev, 1, by_dev, 1, A_dev, A.rows() );


    float * resA = new float[N*N];
    float * resVec = new float[N];
    cublasGetMatrix(A.rows(), A.cols(), sizeof(float), A_dev, A.rows(), resA, A.rows() );
    cublasGetVector(N, sizeof(float), by_dev, 1, resVec, 1 );

    //std::cout << "Resulting A: "<< A  << std::endl;
    cout << "resulting matrix : "; printMatrixColMajor( resA, N, N );

    //verify
    float *cpu = eig_out.data();
    verifyVector( N*N, cpu, resA );



    cudaFree((void*)b_dev);
    cublasDestroy(handle);
    delete [] b;


    return EXIT_SUCCESS;
}

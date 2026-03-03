eigen_add_benchmark(bench_blocking_sizes bench_blocking_sizes.cpp)
eigen_add_benchmark(bench_aocl bench_aocl.cpp)

if(BLAS_FOUND)
  eigen_add_benchmark(bench_blas_gemm bench_blas_gemm.cpp
    LIBRARIES ${BLAS_LIBRARIES}
    DEFINITIONS HAVE_BLAS)
endif()

add_executable(print_blocking print_blocking.cpp)
target_include_directories(print_blocking PRIVATE ${EIGEN_SOURCE_DIR})
target_compile_options(print_blocking PRIVATE -O3 -DNDEBUG)

# Eigen 3.4 (header-only, MPL2). An installed copy is preferred; otherwise the release
# archive is fetched once into the build directory and checked against its SHA-256.
#
# Offline: pass -DINS10_EIGEN_URL=<path to eigen-3.4.0.tar.gz> or
# -DFETCHCONTENT_SOURCE_DIR_EIGEN=<extracted eigen-3.4.0 folder>.
set(INS10_EIGEN_URL "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"
    CACHE STRING "Location of the Eigen 3.4.0 source archive")
set(INS10_EIGEN_SHA256 "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72")

find_package(Eigen3 3.4 QUIET NO_MODULE)
if(TARGET Eigen3::Eigen)
    message(STATUS "Eigen ${Eigen3_VERSION} found in ${Eigen3_DIR}")
else()
    include(FetchContent)
    FetchContent_Declare(eigen
        URL "${INS10_EIGEN_URL}"
        URL_HASH SHA256=${INS10_EIGEN_SHA256}
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        # Only the headers are wanted: a subfolder without CMakeLists.txt keeps
        # FetchContent from configuring Eigen's own tests and install rules.
        SOURCE_SUBDIR ins10-headers-only)
    FetchContent_MakeAvailable(eigen)
    add_library(ins10_eigen INTERFACE)
    target_include_directories(ins10_eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")
    target_compile_definitions(ins10_eigen INTERFACE EIGEN_MPL2_ONLY)
    add_library(Eigen3::Eigen ALIAS ins10_eigen)
    message(STATUS "Eigen 3.4.0 fetched into ${eigen_SOURCE_DIR}")
endif()

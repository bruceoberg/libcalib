project(libcalib)
set(CMAKE_STATIC_LIBRARY_PREFIX "")	# don't prefix library name with "lib"

add_library(libcalib STATIC)

target_sources(libcalib
    PRIVATE
		include/libcalib.h
		include/libcalib_common.h
		include/libcalib_fusion.h
		include/libcalib_magcal.h
		include/libcalib_mahony.h
		include/libcalib_nxp.h
		include/libcalib_quality.h
		include/libcalib_sampler.h
		src/calibrator.cpp
		src/fusion.cpp
		src/magcal.cpp
		src/mahony.cpp
		src/matrix.cpp
		src/matrix.h
        src/nxp.cpp
		src/quality.cpp
		src/sampler.cpp
)

target_include_directories(libcalib PUBLIC ${PROJECT_SOURCE_DIR}/include)

if(TARGET Fusion)
    target_link_libraries(libcalib PUBLIC Fusion)
endif()
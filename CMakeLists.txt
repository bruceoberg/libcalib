project(libcalib)
set(CMAKE_STATIC_LIBRARY_PREFIX "")	# don't prefix library name with "lib"

add_library(libcalib STATIC)

target_sources(libcalib
    PRIVATE
		include/libcalib/ahrs.h
		include/libcalib/calibrator.h
		include/libcalib/common.h
		include/libcalib/fusion.h
		include/libcalib/magcal.h
		include/libcalib/mahony.h
		include/libcalib/nxp.h
		include/libcalib/quality.h
		include/libcalib/sampler.h
		src/calibrator.cpp
		src/fusion.cpp
		src/magcal.cpp
		src/mahony.cpp
		src/matrix.cpp
		src/matrix.h
		src/nxp.cpp
		src/quality.cpp
		src/sampler.cpp
		src/sphere.cpp
		src/sphere.h
)

target_include_directories(libcalib PUBLIC ${PROJECT_SOURCE_DIR}/include)

if(TARGET Fusion)
    target_link_libraries(libcalib PUBLIC Fusion)
endif()
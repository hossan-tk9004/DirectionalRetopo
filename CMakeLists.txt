cmake_minimum_required(VERSION 3.21)

project(DirectionalRetopo VERSION 0.1.0 LANGUAGES CXX)

if(NOT WIN32)
    message(FATAL_ERROR "DirectionalRetopo targets Maya 2024.2 on Windows x64 only.")
endif()

if(NOT CMAKE_GENERATOR STREQUAL "Visual Studio 17 2022")
    message(FATAL_ERROR
        "Use the Visual Studio 17 2022 generator. Run build_maya2024.bat. "
        "Current generator: ${CMAKE_GENERATOR}")
endif()

if(NOT MSVC)
    message(FATAL_ERROR "DirectionalRetopo requires the Visual Studio 2022 MSVC compiler.")
endif()

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "DirectionalRetopo requires an x64 build. Configure with -A x64.")
endif()

if(CMAKE_VS_PLATFORM_TOOLSET AND NOT CMAKE_VS_PLATFORM_TOOLSET STREQUAL "v143")
    message(FATAL_ERROR
        "DirectionalRetopo requires the MSVC v143 platform toolset. "
        "Current toolset: ${CMAKE_VS_PLATFORM_TOOLSET}")
endif()

set(MAYA_LOCATION
    "C:/Program Files/Autodesk/Maya2024"
    CACHE PATH
    "Autodesk Maya 2024.2 installation root")

set(MAYA_DEVKIT_ROOT
    "D:/RD/AITools/MayaPlungins/devkit/maya2024.2/devkitBase"
    CACHE PATH
    "Extracted Autodesk Maya 2024 Update 2 DevKit devkitBase root")

set(MAYA_INCLUDE_DIR "${MAYA_DEVKIT_ROOT}/include")
set(MAYA_LIBRARY_DIR "${MAYA_DEVKIT_ROOT}/lib")
set(MAYA_API_HEADER "${MAYA_INCLUDE_DIR}/maya/MTypes.h")

foreach(required_path IN ITEMS
        "${MAYA_LOCATION}/bin/maya.exe"
        "${MAYA_INCLUDE_DIR}/maya/MFnPlugin.h"
        "${MAYA_API_HEADER}"
        "${MAYA_LIBRARY_DIR}/Foundation.lib"
        "${MAYA_LIBRARY_DIR}/OpenMaya.lib"
        "${MAYA_LIBRARY_DIR}/OpenMayaRender.lib"
        "${MAYA_LIBRARY_DIR}/OpenMayaUI.lib"
        "${MAYA_LIBRARY_DIR}/tbb.lib")
    if(NOT EXISTS "${required_path}")
        message(FATAL_ERROR "Required Maya 2024.2 file was not found: ${required_path}")
    endif()
endforeach()

file(STRINGS "${MAYA_API_HEADER}" MAYA_API_VERSION_LINE
    REGEX "^#define[ \t]+MAYA_API_VERSION[ \t]+[0-9]+")

if(NOT MAYA_API_VERSION_LINE MATCHES "MAYA_API_VERSION[ \t]+20240200")
    message(FATAL_ERROR
        "The configured DevKit is not Maya 2024 Update 2. "
        "Expected MAYA_API_VERSION 20240200, found: ${MAYA_API_VERSION_LINE}")
endif()

function(add_maya_imported_library target_name library_name)
    set(library_path "${MAYA_LIBRARY_DIR}/${library_name}.lib")
    if(NOT EXISTS "${library_path}")
        message(FATAL_ERROR "Required Maya library was not found: ${library_path}")
    endif()

    add_library(${target_name} UNKNOWN IMPORTED)
    set_target_properties(${target_name} PROPERTIES
        IMPORTED_LOCATION "${library_path}")
endfunction()

add_maya_imported_library(Maya::Foundation Foundation)
add_maya_imported_library(Maya::OpenMaya OpenMaya)

add_library(DirectionalRetopoRemeshContract STATIC
    src/Solver/RemeshContract.cpp)
target_include_directories(DirectionalRetopoRemeshContract PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_features(DirectionalRetopoRemeshContract PUBLIC cxx_std_17)
target_compile_definitions(DirectionalRetopoRemeshContract PRIVATE NOMINMAX)
target_compile_options(DirectionalRetopoRemeshContract PRIVATE
    /EHsc
    /W4
    /permissive-
    /Zc:__cplusplus)
set_target_properties(DirectionalRetopoRemeshContract PROPERTIES
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
add_maya_imported_library(Maya::OpenMayaRender OpenMayaRender)
add_maya_imported_library(Maya::OpenMayaUI OpenMayaUI)
add_maya_imported_library(Maya::TBB tbb)

set(AUTOREMESHER_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/autoremesher")

add_library(DirectionalRetopoAutoRemesherCore STATIC
    "${AUTOREMESHER_ROOT}/src/AutoRemesher/constrainedleastsquares.cpp"
    "${AUTOREMESHER_ROOT}/src/AutoRemesher/meshseparator.cpp"
    "${AUTOREMESHER_ROOT}/src/AutoRemesher/mixedintegerleastsquares.cpp"
    "${AUTOREMESHER_ROOT}/src/AutoRemesher/positionkey.cpp"
    "${AUTOREMESHER_ROOT}/src/AutoRemesher/quadextractor.cpp"
    "${AUTOREMESHER_ROOT}/src/AutoRemesher/quadparameterizer.cpp"
    "${AUTOREMESHER_ROOT}/src/AutoRemesher/surfacemesh.cpp"
    "${AUTOREMESHER_ROOT}/third_party/isotropicremesher/axisalignedboundingboxtree.cpp")

target_include_directories(DirectionalRetopoAutoRemesherCore
    PUBLIC
        "${AUTOREMESHER_ROOT}/include"
    PRIVATE
        "${AUTOREMESHER_ROOT}/third_party/isotropicremesher")

target_include_directories(DirectionalRetopoAutoRemesherCore SYSTEM
    PUBLIC
        "${AUTOREMESHER_ROOT}/third_party/eigen"
    PRIVATE
        "${MAYA_INCLUDE_DIR}")

target_link_libraries(DirectionalRetopoAutoRemesherCore PUBLIC Maya::TBB)
target_compile_features(DirectionalRetopoAutoRemesherCore PUBLIC cxx_std_17)
target_compile_definitions(DirectionalRetopoAutoRemesherCore PRIVATE
    AUTO_REMESHER_DEV=0
    NOMINMAX
    _USE_MATH_DEFINES
    _CRT_SECURE_NO_WARNINGS)
target_compile_options(DirectionalRetopoAutoRemesherCore PRIVATE
    /EHsc
    /W3
    /permissive-
    /Zc:__cplusplus)
set_target_properties(DirectionalRetopoAutoRemesherCore PROPERTIES
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

option(DIRECTIONAL_RETOPO_BUILD_TESTS
    "Build AutoRemesher adapter and SurfaceConformer smoke tests" ON)
if(DIRECTIONAL_RETOPO_BUILD_TESTS)
    enable_testing()
    add_executable(AutoRemesherCoreSmokeTest
        tests/AutoRemesherCoreSmokeTest.cpp)
    target_link_libraries(AutoRemesherCoreSmokeTest PRIVATE
        DirectionalRetopoAutoRemesherCore)
    target_compile_features(AutoRemesherCoreSmokeTest PRIVATE cxx_std_17)
    target_compile_definitions(AutoRemesherCoreSmokeTest PRIVATE
        NOMINMAX
        _USE_MATH_DEFINES)
    target_compile_options(AutoRemesherCoreSmokeTest PRIVATE
        /EHsc
        /W4
        /permissive-
        /Zc:__cplusplus)
    set_target_properties(AutoRemesherCoreSmokeTest PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_test(NAME AutoRemesherCoreSmokeTest
        COMMAND AutoRemesherCoreSmokeTest)
    set_tests_properties(AutoRemesherCoreSmokeTest PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${MAYA_LOCATION}/bin")

    add_executable(AutoRemesherAdapterSmokeTest
        tests/AutoRemesherAdapterSmokeTest.cpp
        src/Remesh/AutoRemesherAdapter.cpp
        src/Remesh/BoundaryConformer.cpp
        src/Remesh/OrderedBoundaryCorrespondence.cpp
        src/Remesh/QuadResultValidator.cpp
        src/Remesh/SurfaceConformer.cpp)
    target_include_directories(AutoRemesherAdapterSmokeTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_include_directories(AutoRemesherAdapterSmokeTest SYSTEM PRIVATE
        "${MAYA_INCLUDE_DIR}")
    target_link_libraries(AutoRemesherAdapterSmokeTest PRIVATE
        DirectionalRetopoAutoRemesherCore
        Maya::OpenMaya
        Maya::Foundation)
    target_compile_features(AutoRemesherAdapterSmokeTest PRIVATE cxx_std_17)
    target_compile_definitions(AutoRemesherAdapterSmokeTest PRIVATE
        NOMINMAX
        _USE_MATH_DEFINES)
    target_compile_options(AutoRemesherAdapterSmokeTest PRIVATE
        /EHsc
        /W4
        /permissive-
        /Zc:__cplusplus)
    set_target_properties(AutoRemesherAdapterSmokeTest PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_test(NAME AutoRemesherAdapterSmokeTest
        COMMAND AutoRemesherAdapterSmokeTest)
    set_tests_properties(AutoRemesherAdapterSmokeTest PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${MAYA_LOCATION}/bin")

    add_executable(SurfaceConformerSmokeTest
        tests/SurfaceConformerSmokeTest.cpp
        src/Remesh/BoundaryConformer.cpp
        src/Remesh/OrderedBoundaryCorrespondence.cpp
        src/Remesh/SurfaceConformer.cpp)
    target_include_directories(SurfaceConformerSmokeTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_include_directories(SurfaceConformerSmokeTest SYSTEM PRIVATE
        "${MAYA_INCLUDE_DIR}")
    target_link_libraries(SurfaceConformerSmokeTest PRIVATE
        Maya::OpenMaya
        Maya::Foundation)
    target_compile_features(SurfaceConformerSmokeTest PRIVATE cxx_std_17)
    target_compile_definitions(SurfaceConformerSmokeTest PRIVATE NOMINMAX)
    target_compile_options(SurfaceConformerSmokeTest PRIVATE
        /EHsc
        /W4
        /permissive-
        /Zc:__cplusplus)
    set_target_properties(SurfaceConformerSmokeTest PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_test(NAME SurfaceConformerSmokeTest
        COMMAND SurfaceConformerSmokeTest)
    set_tests_properties(SurfaceConformerSmokeTest PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${MAYA_LOCATION}/bin")

    add_executable(BoundaryConformerSmokeTest
        tests/BoundaryConformerSmokeTest.cpp
        src/Remesh/BoundaryConformer.cpp
        src/Remesh/OrderedBoundaryCorrespondence.cpp
        src/Remesh/QuadResultValidator.cpp)
    target_include_directories(BoundaryConformerSmokeTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_include_directories(BoundaryConformerSmokeTest SYSTEM PRIVATE
        "${MAYA_INCLUDE_DIR}")
    target_link_libraries(BoundaryConformerSmokeTest PRIVATE
        Maya::OpenMaya
        Maya::Foundation)
    target_compile_features(BoundaryConformerSmokeTest PRIVATE cxx_std_17)
    target_compile_definitions(BoundaryConformerSmokeTest PRIVATE NOMINMAX)
    target_compile_options(BoundaryConformerSmokeTest PRIVATE
        /EHsc
        /W4
        /permissive-
        /Zc:__cplusplus)
    set_target_properties(BoundaryConformerSmokeTest PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_test(NAME BoundaryConformerSmokeTest
        COMMAND BoundaryConformerSmokeTest)
    set_tests_properties(BoundaryConformerSmokeTest PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${MAYA_LOCATION}/bin")

    add_executable(ScaleFidelitySmokeTest
        tests/ScaleFidelitySmokeTest.cpp
        src/Remesh/AutoRemesherAdapter.cpp
        src/Remesh/BoundaryConformer.cpp
        src/Remesh/OrderedBoundaryCorrespondence.cpp
        src/Remesh/QuadResultValidator.cpp
        src/Remesh/SurfaceConformer.cpp)
    target_include_directories(ScaleFidelitySmokeTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_include_directories(ScaleFidelitySmokeTest SYSTEM PRIVATE
        "${MAYA_INCLUDE_DIR}")
    target_link_libraries(ScaleFidelitySmokeTest PRIVATE
        DirectionalRetopoAutoRemesherCore
        Maya::OpenMaya
        Maya::Foundation)
    target_compile_features(ScaleFidelitySmokeTest PRIVATE cxx_std_17)
    target_compile_definitions(ScaleFidelitySmokeTest PRIVATE
        NOMINMAX
        _USE_MATH_DEFINES)
    target_compile_options(ScaleFidelitySmokeTest PRIVATE
        /EHsc
        /W4
        /permissive-
        /Zc:__cplusplus)
    set_target_properties(ScaleFidelitySmokeTest PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_test(NAME ScaleFidelitySmokeTest COMMAND ScaleFidelitySmokeTest)
    set_tests_properties(ScaleFidelitySmokeTest PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${MAYA_LOCATION}/bin")

    add_executable(CurvatureEstimatorSmokeTest
        tests/CurvatureEstimatorSmokeTest.cpp
        src/Field/CurvatureEstimator.cpp
        src/Field/DensityFieldBuilder.cpp
        src/Mesh/MeshTopologyCache.cpp)
    target_include_directories(CurvatureEstimatorSmokeTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_include_directories(CurvatureEstimatorSmokeTest SYSTEM PRIVATE
        "${MAYA_INCLUDE_DIR}")
    target_link_libraries(CurvatureEstimatorSmokeTest PRIVATE
        Maya::OpenMaya
        Maya::Foundation)
    target_compile_features(CurvatureEstimatorSmokeTest PRIVATE cxx_std_17)
    target_compile_definitions(CurvatureEstimatorSmokeTest PRIVATE NOMINMAX)
    target_compile_options(CurvatureEstimatorSmokeTest PRIVATE
        /EHsc
        /W4
        /permissive-
        /Zc:__cplusplus)
    set_target_properties(CurvatureEstimatorSmokeTest PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_test(NAME CurvatureEstimatorSmokeTest COMMAND CurvatureEstimatorSmokeTest)
    set_tests_properties(CurvatureEstimatorSmokeTest PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${MAYA_LOCATION}/bin")
    add_executable(BoundaryLockedTransitionSmokeTest
        tests/BoundaryLockedTransitionSmokeTest.cpp
        src/Remesh/BoundaryGeometryValidator.cpp
        src/Remesh/TransitionCollarBuilder.cpp
        src/Remesh/QuadResultValidator.cpp
        src/Remesh/BoundaryLockedPatchBuilder.cpp)
    target_include_directories(BoundaryLockedTransitionSmokeTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_include_directories(BoundaryLockedTransitionSmokeTest SYSTEM PRIVATE
        "${MAYA_INCLUDE_DIR}")
    target_link_libraries(BoundaryLockedTransitionSmokeTest PRIVATE
        Maya::OpenMaya
        Maya::Foundation)
    target_compile_features(BoundaryLockedTransitionSmokeTest PRIVATE cxx_std_17)
    target_compile_definitions(BoundaryLockedTransitionSmokeTest PRIVATE NOMINMAX)
    target_compile_options(BoundaryLockedTransitionSmokeTest PRIVATE
        /EHsc
        /W4
        /permissive-
        /Zc:__cplusplus)
    set_target_properties(BoundaryLockedTransitionSmokeTest PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_test(NAME BoundaryLockedTransitionSmokeTest
        COMMAND BoundaryLockedTransitionSmokeTest)
    set_tests_properties(BoundaryLockedTransitionSmokeTest PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${MAYA_LOCATION}/bin")

    add_executable(BoundaryCompatibilityDensitySmokeTest
        tests/BoundaryCompatibilityDensitySmokeTest.cpp
        src/Remesh/BoundaryCompatibilityDensity.cpp
        src/Mesh/MeshTopologyCache.cpp)
    target_include_directories(BoundaryCompatibilityDensitySmokeTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_include_directories(BoundaryCompatibilityDensitySmokeTest SYSTEM PRIVATE
        "${MAYA_INCLUDE_DIR}")
    target_link_libraries(BoundaryCompatibilityDensitySmokeTest PRIVATE
        Maya::OpenMaya
        Maya::Foundation)
    target_compile_features(BoundaryCompatibilityDensitySmokeTest PRIVATE cxx_std_17)
    target_compile_definitions(BoundaryCompatibilityDensitySmokeTest PRIVATE NOMINMAX)
    target_compile_options(BoundaryCompatibilityDensitySmokeTest PRIVATE
        /EHsc
        /W4
        /permissive-
        /Zc:__cplusplus)
    set_target_properties(BoundaryCompatibilityDensitySmokeTest PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    add_test(NAME BoundaryCompatibilityDensitySmokeTest
        COMMAND BoundaryCompatibilityDensitySmokeTest)
    set_tests_properties(BoundaryCompatibilityDensitySmokeTest PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${MAYA_LOCATION}/bin")

endif()

add_library(DirectionalRetopo SHARED
    src/pluginMain.cpp
    src/Brush/RayCaster.cpp
    src/Field/CrossFieldMath.cpp
    src/Field/CurvatureEstimator.cpp
    src/Field/DensityFieldBuilder.cpp
    src/Field/DirectionFieldBuilder.cpp
    src/Integration/MayaRemeshInputAdapter.cpp
    src/Mesh/BoundaryExtractor.cpp
    src/Mesh/MeshTopologyCache.cpp
    src/Paint/PaintRegionSolver.cpp
    src/Paint/RegionPreviewCalculator.cpp
    src/Remesh/BoundaryCompatibilityDensity.cpp
    src/Paint/StrokeData.cpp
    src/Paint/StrokeProcessor.cpp
    src/Remesh/AutoRemesherAdapter.cpp
    src/Remesh/BoundaryGeometryValidator.cpp
    src/Remesh/BoundaryLockedPatchBuilder.cpp
    src/Remesh/TransitionCollarBuilder.cpp
    src/Remesh/BoundaryConformer.cpp
    src/Remesh/OrderedBoundaryCorrespondence.cpp
    src/Remesh/PatchTriangulator.cpp
    src/Remesh/QuadResultValidator.cpp
    src/Remesh/SurfaceConformer.cpp
    src/Tool/DirectionalRetopoContext.cpp
    src/Tool/DirectionalRetopoContextCommand.cpp
    src/Tool/PythonRuntimeBridge.cpp
    src/Tool/DirectionalRetopoToolCommand.cpp
    src/Viewport/DirectionalRetopoBrushCursor.cpp
    src/Viewport/DirectionalRetopoQuadPreview.cpp
    src/Viewport/DirectionalRetopoTargetDisplay.cpp
    src/Viewport/FieldVisualizer.cpp
    src/Viewport/RegionVisualizer.cpp
    src/Viewport/QuadPreviewModel.cpp
    src/Viewport/TargetDisplayModel.cpp)

target_include_directories(DirectionalRetopo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src")

target_include_directories(DirectionalRetopo SYSTEM PRIVATE
    "${MAYA_INCLUDE_DIR}")

target_link_libraries(DirectionalRetopo PRIVATE
    DirectionalRetopoRemeshContract
    DirectionalRetopoAutoRemesherCore
    Maya::OpenMaya
    Maya::OpenMayaRender
    Maya::OpenMayaUI
    Maya::Foundation)

target_compile_features(DirectionalRetopo PRIVATE cxx_std_17)

target_compile_definitions(DirectionalRetopo PRIVATE
    NT_PLUGIN
    REQUIRE_IOSTREAM
    _BOOL
    _CRT_SECURE_NO_WARNINGS)

target_compile_options(DirectionalRetopo PRIVATE
    /EHsc
    /W4
    /permissive-
    /Zc:__cplusplus)

set_target_properties(DirectionalRetopo PROPERTIES
    OUTPUT_NAME "DirectionalRetopo"
    PREFIX ""
    SUFFIX ".mll"
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
    PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>"
    COMPILE_PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")

message(STATUS "Maya installation: ${MAYA_LOCATION}")
message(STATUS "Maya 2024.2 DevKit: ${MAYA_DEVKIT_ROOT}")
message(STATUS "Maya API version: 20240200")
message(STATUS "CMake generator: ${CMAKE_GENERATOR}")
message(STATUS "CMake platform: ${CMAKE_VS_PLATFORM_NAME}")
message(STATUS "MSVC platform toolset: ${CMAKE_VS_PLATFORM_TOOLSET}")
message(STATUS "MSVC toolset version: ${CMAKE_VS_PLATFORM_TOOLSET_VERSION}")

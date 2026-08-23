#pragma once

#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Remesh/LocalPatch.h"
#include "Remesh/QuadPatchResult.h"
#include "Remesh/QuadResultValidator.h"
#include "Remesh/SurfaceConformer.h"

#include <AutoRemesher/QuadParameterizer>
#include <AutoRemesher/Vector2>
#include <AutoRemesher/Vector3>

#include <cstddef>
#include <string>
#include <vector>

namespace directional_retopo {

struct AutoRemesherInput final
{
    std::size_t componentId = 0;
    std::vector<AutoRemesher::Vector3> vertices;
    std::vector<std::vector<std::size_t>> triangles;
    std::vector<int> sourceFaceIds;
    std::vector<AutoRemesher::Vector3> guidance;
    std::vector<double> targetEdgeLengths;
    std::vector<unsigned char> curvatureLimitedTriangles;
    std::vector<double> faceScaling;
    std::vector<double> faceScalingU;
    std::vector<double> faceScalingV;
    double patchAverageEdgeLength = 0.0;
    double baseTargetEdgeLength = 0.0;
    double globalScaling = 0.0;
    double minimumTargetEdgeLength = 0.0;
    double meanTargetEdgeLength = 0.0;
    double maximumTargetEdgeLength = 0.0;
    std::size_t curvatureLimitedTriangleCount = 0U;
};

struct ParameterizationResult final
{
    bool success = false;
    std::vector<std::vector<AutoRemesher::Vector2>> triangleUvs;
    std::vector<AutoRemesher::Vector3> solverGuidance;
    std::vector<int> cornerRotations;
    std::vector<std::size_t> singularVertices;
    double meanGuidanceDeviationDegrees = 0.0;
    double maximumGuidanceDeviationDegrees = 0.0;
};

struct AutoRemesherAdapterSettings final
{
    double hardEdgeDegrees = 90.0;
    double geometryEpsilon = 1.0e-12;
};

class AutoRemesherAdapter final
{
public:
    [[nodiscard]] const AutoRemesherAdapterSettings& settings() const noexcept;
    void setSettings(const AutoRemesherAdapterSettings& settings) noexcept;

    bool buildInput(
        const TriangulatedPatch& patch,
        const DirectionFieldData& directionField,
        const DensityFieldData& densityField,
        AutoRemesherInput& input,
        std::string& diagnostic) const;

    bool parameterize(
        const AutoRemesherInput& input,
        ParameterizationResult& result,
        std::string& diagnostic) const;

    bool extractQuads(
        const AutoRemesherInput& input,
        const ParameterizationResult& parameterization,
        QuadPatchResult& result,
        std::string& diagnostic) const;

    bool conformToSurface(
        const TriangulatedPatch& patch,
        QuadPatchResult& result,
        std::string& diagnostic) const;

    bool validateResult(
        const TriangulatedPatch& patch,
        QuadPatchResult& result,
        std::string& diagnostic) const;
    bool finalizeBoundaryLocked(
        const TriangulatedPatch& completeSourcePatch,
        QuadPatchResult& result,
        std::string& diagnostic) const;


    [[nodiscard]] QuadComponentSolveReport solve(
        const TriangulatedPatch& patch,
        const DirectionFieldData& directionField,
        const DensityFieldData& densityField) const noexcept;

private:
    AutoRemesherAdapterSettings settings_;
    SurfaceConformer surfaceConformer_;
    QuadResultValidator validator_;
};

}  // namespace directional_retopo

#include "Remesh/AutoRemesherAdapter.h"

#include <AutoRemesher/QuadExtractor>

#include <maya/MVector.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <utility>

namespace directional_retopo {
namespace {

using Clock = std::chrono::steady_clock;
using Edge = std::pair<std::size_t, std::size_t>;

double elapsedMilliseconds(const Clock::time_point& begin, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

double median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    return values.size() % 2U == 0U
        ? (values[middle - 1U] + values[middle]) * 0.5
        : values[middle];
}

bool finiteVector(const AutoRemesher::Vector3& value)
{
    return std::isfinite(value.x()) && std::isfinite(value.y()) &&
        std::isfinite(value.z());
}

AutoRemesher::Vector3 toAutoRemesher(const MPoint& point)
{
    return AutoRemesher::Vector3(point.x, point.y, point.z);
}

AutoRemesher::Vector3 toAutoRemesher(const MVector& vector)
{
    return AutoRemesher::Vector3(vector.x, vector.y, vector.z);
}

MVector toMaya(const AutoRemesher::Vector3& vector)
{
    return MVector(vector.x(), vector.y(), vector.z());
}

double crossDeviationDegrees(
    const AutoRemesher::Vector3& first,
    const AutoRemesher::Vector3& second,
    double epsilon)
{
    MVector a = toMaya(first);
    MVector b = toMaya(second);
    if (a.length() <= epsilon || b.length() <= epsilon) {
        return 90.0;
    }
    a.normalize();
    b.normalize();
    const double angle = std::acos(std::clamp(a * b, -1.0, 1.0));
    constexpr double halfPi = 1.57079632679489661923;
    double reduced = std::fmod(angle, halfPi);
    reduced = std::min(reduced, halfPi - reduced);
    return reduced * 180.0 / 3.14159265358979323846;
}

}  // namespace

const AutoRemesherAdapterSettings& AutoRemesherAdapter::settings() const noexcept
{
    return settings_;
}

void AutoRemesherAdapter::setSettings(
    const AutoRemesherAdapterSettings& settings) noexcept
{
    settings_ = settings;
    settings_.hardEdgeDegrees = std::clamp(settings_.hardEdgeDegrees, 0.0, 180.0);
    settings_.geometryEpsilon = std::max(settings_.geometryEpsilon, 0.0);
}

bool AutoRemesherAdapter::buildInput(
    const TriangulatedPatch& patch,
    const DirectionFieldData& directionField,
    const DensityFieldData& densityField,
    AutoRemesherInput& input,
    std::string& diagnostic) const
{
    input = AutoRemesherInput();
    input.componentId = patch.componentId;
    if (patch.empty()) {
        diagnostic = "Triangulated patch is empty.";
        return false;
    }
    input.vertices.reserve(patch.vertices.size());
    for (const PatchVertex& vertex : patch.vertices) {
        const AutoRemesher::Vector3 converted = toAutoRemesher(vertex.position);
        if (!finiteVector(converted)) {
            diagnostic = "Patch contains a non-finite source vertex.";
            return false;
        }
        input.vertices.push_back(converted);
    }

    std::set<Edge> uniqueEdges;
    std::vector<double> targetEdgeLengths;
    targetEdgeLengths.reserve(patch.triangles.size());
    input.triangles.reserve(patch.triangles.size());
    input.sourceFaceIds.reserve(patch.triangles.size());
    input.guidance.reserve(patch.triangles.size());
    input.targetEdgeLengths.reserve(patch.triangles.size());
    input.curvatureLimitedTriangles.reserve(patch.triangles.size());
    for (const PatchTriangle& triangle : patch.triangles) {
        if (triangle.vertexIndices[0] >= patch.vertices.size() ||
            triangle.vertexIndices[1] >= patch.vertices.size() ||
            triangle.vertexIndices[2] >= patch.vertices.size() ||
            triangle.vertexIndices[0] == triangle.vertexIndices[1] ||
            triangle.vertexIndices[1] == triangle.vertexIndices[2] ||
            triangle.vertexIndices[2] == triangle.vertexIndices[0]) {
            diagnostic = "Patch contains an invalid triangle vertex index.";
            return false;
        }
        const int sourceFaceId = triangle.sourceFaceId;
        const FaceDirectionField* faceDirection = directionField.face(sourceFaceId);
        const FaceDensity* faceDensity = densityField.face(sourceFaceId);
        if (faceDirection == nullptr || !faceDirection->valid) {
            std::ostringstream message;
            message << "Direction Field is invalid for source face " << sourceFaceId << '.';
            diagnostic = message.str();
            return false;
        }
        if (faceDensity == nullptr || !faceDensity->valid ||
            !std::isfinite(faceDensity->targetEdgeLength) ||
            faceDensity->targetEdgeLength <= settings_.geometryEpsilon) {
            std::ostringstream message;
            message << "Density Field is invalid for source face " << sourceFaceId << '.';
            diagnostic = message.str();
            return false;
        }

        const MPoint& a = patch.vertices[triangle.vertexIndices[0]].position;
        const MPoint& b = patch.vertices[triangle.vertexIndices[1]].position;
        const MPoint& c = patch.vertices[triangle.vertexIndices[2]].position;
        MVector triangleNormal = (b - a) ^ (c - a);
        if (triangleNormal.length() <= settings_.geometryEpsilon) {
            diagnostic = "Patch contains a zero-area triangle.";
            return false;
        }
        triangleNormal.normalize();
        MVector guidance = faceDirection->uDirection -
            triangleNormal * (faceDirection->uDirection * triangleNormal);
        if (guidance.length() <= settings_.geometryEpsilon) {
            guidance = faceDirection->vDirection -
                triangleNormal * (faceDirection->vDirection * triangleNormal);
        }
        if (guidance.length() <= settings_.geometryEpsilon) {
            diagnostic = "Face guidance cannot be projected to a patch triangle.";
            return false;
        }
        guidance.normalize();

        input.triangles.push_back({
            triangle.vertexIndices[0],
            triangle.vertexIndices[1],
            triangle.vertexIndices[2]});
        input.sourceFaceIds.push_back(sourceFaceId);
        input.guidance.push_back(toAutoRemesher(guidance));
        input.targetEdgeLengths.push_back(faceDensity->targetEdgeLength);
        input.curvatureLimitedTriangles.push_back(
            faceDensity->curvatureLimited ? 1U : 0U);
        targetEdgeLengths.push_back(faceDensity->targetEdgeLength);
        for (std::size_t corner = 0; corner < 3U; ++corner) {
            const std::size_t first = triangle.vertexIndices[corner];
            const std::size_t second = triangle.vertexIndices[(corner + 1U) % 3U];
            uniqueEdges.insert(first < second ? Edge(first, second) : Edge(second, first));
        }
    }

    double totalEdgeLength = 0.0;
    for (const Edge& edge : uniqueEdges) {
        totalEdgeLength += (patch.vertices[edge.second].position -
            patch.vertices[edge.first].position).length();
    }
    input.patchAverageEdgeLength = uniqueEdges.empty()
        ? 0.0
        : totalEdgeLength / static_cast<double>(uniqueEdges.size());
    input.baseTargetEdgeLength = median(targetEdgeLengths);
    if (!std::isfinite(input.patchAverageEdgeLength) ||
        input.patchAverageEdgeLength <= settings_.geometryEpsilon ||
        !std::isfinite(input.baseTargetEdgeLength) ||
        input.baseTargetEdgeLength <= settings_.geometryEpsilon) {
        diagnostic = "Patch average edge length or target edge length is invalid.";
        return false;
    }

    input.globalScaling = input.baseTargetEdgeLength / input.patchAverageEdgeLength;
    input.minimumTargetEdgeLength = *std::min_element(
        input.targetEdgeLengths.begin(),
        input.targetEdgeLengths.end());
    input.maximumTargetEdgeLength = *std::max_element(
        input.targetEdgeLengths.begin(),
        input.targetEdgeLengths.end());
    double targetSum = 0.0;
    for (const double target : input.targetEdgeLengths) {
        targetSum += target;
    }
    input.meanTargetEdgeLength = targetSum /
        static_cast<double>(input.targetEdgeLengths.size());
    input.curvatureLimitedTriangleCount = static_cast<std::size_t>(
        std::count(
            input.curvatureLimitedTriangles.begin(),
            input.curvatureLimitedTriangles.end(),
            static_cast<unsigned char>(1U)));
    input.faceScaling.reserve(input.targetEdgeLengths.size());
    input.faceScalingU.assign(input.targetEdgeLengths.size(), 1.0);
    input.faceScalingV.assign(input.targetEdgeLengths.size(), 1.0);
    for (const double targetEdgeLength : input.targetEdgeLengths) {
        input.faceScaling.push_back(targetEdgeLength / input.baseTargetEdgeLength);
    }
    diagnostic = "External triangle guidance and per-triangle target lengths prepared.";
    return true;
}

bool AutoRemesherAdapter::parameterize(
    const AutoRemesherInput& input,
    ParameterizationResult& result,
    std::string& diagnostic) const
{
    result = ParameterizationResult();
    AutoRemesher::QuadParameterizer::Result upstreamResult;
    const bool succeeded = AutoRemesher::QuadParameterizer::parameterize(
        input.vertices,
        input.triangles,
        &input.guidance,
        input.globalScaling,
        settings_.hardEdgeDegrees,
        &upstreamResult,
        &input.faceScaling,
        &input.faceScalingU,
        &input.faceScalingV);
    if (!succeeded || upstreamResult.triangleUvs.size() != input.triangles.size() ||
        upstreamResult.field.size() != input.guidance.size()) {
        diagnostic = "AutoRemesher QuadParameterizer returned failure or incomplete output.";
        return false;
    }
    for (const auto& triangleUvs : upstreamResult.triangleUvs) {
        if (triangleUvs.size() != 3U) {
            diagnostic = "AutoRemesher returned an invalid triangle UV count.";
            return false;
        }
        for (const AutoRemesher::Vector2& uv : triangleUvs) {
            if (!std::isfinite(uv.x()) || !std::isfinite(uv.y())) {
                diagnostic = "AutoRemesher returned non-finite triangle UVs.";
                return false;
            }
        }
    }

    double deviationSum = 0.0;
    for (std::size_t triangleIndex = 0;
         triangleIndex < input.guidance.size();
         ++triangleIndex) {
        const double deviation = crossDeviationDegrees(
            input.guidance[triangleIndex],
            upstreamResult.field[triangleIndex],
            settings_.geometryEpsilon);
        deviationSum += deviation;
        result.maximumGuidanceDeviationDegrees =
            std::max(result.maximumGuidanceDeviationDegrees, deviation);
    }
    result.meanGuidanceDeviationDegrees = input.guidance.empty()
        ? 0.0
        : deviationSum / static_cast<double>(input.guidance.size());
    result.triangleUvs = std::move(upstreamResult.triangleUvs);
    result.solverGuidance = std::move(upstreamResult.field);
    result.cornerRotations = std::move(upstreamResult.cornerRotations);
    result.singularVertices = std::move(upstreamResult.singularVertices);
    result.success = true;
    std::ostringstream message;
    message << "Parameterization succeeded; maximum 4-RoSy guidance deviation "
            << result.maximumGuidanceDeviationDegrees << " degrees.";
    diagnostic = message.str();
    return true;
}

bool AutoRemesherAdapter::extractQuads(
    const AutoRemesherInput& input,
    const ParameterizationResult& parameterization,
    QuadPatchResult& result,
    std::string& diagnostic) const
{
    result.clear();
    result.componentId = input.componentId;
    result.targetEdgeLength = input.baseTargetEdgeLength;
    if (!parameterization.success) {
        diagnostic = "Quad extraction received no valid parameterization.";
        return false;
    }
    AutoRemesher::QuadExtractor extractor(
        &input.vertices,
        &input.triangles,
        &parameterization.triangleUvs);
    extractor.setSingularVertices(&parameterization.singularVertices);
    if (!extractor.extract()) {
        diagnostic = "AutoRemesher QuadExtractor returned failure.";
        return false;
    }
    result.rawVertices.reserve(extractor.remeshedVertices().size());
    for (const AutoRemesher::Vector3& vertex : extractor.remeshedVertices()) {
        if (!finiteVector(vertex)) {
            diagnostic = "AutoRemesher QuadExtractor returned a non-finite vertex.";
            result.clear();
            result.componentId = input.componentId;
            return false;
        }
        result.rawVertices.emplace_back(vertex.x(), vertex.y(), vertex.z());
    }
    result.polygons.reserve(extractor.remeshedQuads().size());
    for (const std::vector<std::size_t>& polygon : extractor.remeshedQuads()) {
        result.polygons.push_back(polygon);
    }
    if (result.rawVertices.empty() || result.polygons.empty()) {
        diagnostic = "AutoRemesher QuadExtractor returned an empty result.";
        return false;
    }
    diagnostic = "Quad extraction succeeded.";
    return true;
}

bool AutoRemesherAdapter::conformToSurface(
    const TriangulatedPatch& patch,
    QuadPatchResult& result,
    std::string& diagnostic) const
{
    return surfaceConformer_.conform(patch, result, diagnostic);
}

bool AutoRemesherAdapter::validateResult(
    const TriangulatedPatch& patch,
    QuadPatchResult& result,
    std::string& diagnostic) const
{
    const bool valid = validator_.validate(patch, result);
    diagnostic = result.diagnosticMessage;
    return valid;
}
bool AutoRemesherAdapter::finalizeBoundaryLocked(
    const TriangulatedPatch& completeSourcePatch,
    QuadPatchResult& result,
    std::string& diagnostic) const
{
    if (!result.boundaryLocked) {
        diagnostic = "Finalization requires a Boundary-Locked Patch result.";
        return false;
    }
    if (!conformToSurface(completeSourcePatch, result, diagnostic)) {
        return false;
    }
    return validateResult(completeSourcePatch, result, diagnostic);
}


QuadComponentSolveReport AutoRemesherAdapter::solve(
    const TriangulatedPatch& patch,
    const DirectionFieldData& directionField,
    const DensityFieldData& densityField) const noexcept
{
    QuadComponentSolveReport report;
    report.componentId = patch.componentId;
    report.patchVertexCount = patch.vertices.size();
    report.patchTriangleCount = patch.triangles.size();
    const Clock::time_point totalStart = Clock::now();
    try {
        AutoRemesherInput input;
        std::string diagnostic;
        if (!buildInput(patch, directionField, densityField, input, diagnostic)) {
            report.diagnosticMessage = diagnostic;
            report.timings.totalMilliseconds =
                elapsedMilliseconds(totalStart, Clock::now());
            return report;
        }
        report.minimumEffectiveTargetEdgeLength =
            input.minimumTargetEdgeLength;
        report.meanEffectiveTargetEdgeLength = input.meanTargetEdgeLength;
        report.maximumEffectiveTargetEdgeLength =
            input.maximumTargetEdgeLength;
        report.curvatureLimitedTriangleCount =
            input.curvatureLimitedTriangleCount;

        const Clock::time_point parameterizationStart = Clock::now();
        ParameterizationResult parameterization;
        report.parameterizationSuccess = parameterize(
            input,
            parameterization,
            diagnostic);
        report.timings.parameterizationMilliseconds = elapsedMilliseconds(
            parameterizationStart,
            Clock::now());
        report.maximumGuidanceDeviationDegrees =
            parameterization.maximumGuidanceDeviationDegrees;
        if (!report.parameterizationSuccess) {
            report.diagnosticMessage = diagnostic;
            report.timings.totalMilliseconds =
                elapsedMilliseconds(totalStart, Clock::now());
            return report;
        }

        const Clock::time_point extractionStart = Clock::now();
        report.extractionSuccess = extractQuads(
            input,
            parameterization,
            report.result,
            diagnostic);
        report.timings.extractionMilliseconds = elapsedMilliseconds(
            extractionStart,
            Clock::now());
        if (!report.extractionSuccess) {
            report.diagnosticMessage = diagnostic;
            report.result.clear();
            report.result.componentId = patch.componentId;
            report.timings.totalMilliseconds =
                elapsedMilliseconds(totalStart, Clock::now());
            return report;
        }

        const Clock::time_point conformationStart = Clock::now();
        report.conformationSuccess = conformToSurface(
            patch,
            report.result,
            diagnostic);
        report.timings.conformationMilliseconds = elapsedMilliseconds(
            conformationStart,
            Clock::now());
        if (!report.conformationSuccess) {
            report.diagnosticMessage = diagnostic;
            report.result.clear();
            report.result.componentId = patch.componentId;
            report.timings.totalMilliseconds =
                elapsedMilliseconds(totalStart, Clock::now());
            return report;
        }

        const Clock::time_point validationStart = Clock::now();
        if (!validateResult(patch, report.result, diagnostic)) {
            report.diagnosticMessage = diagnostic;
            report.result.clear();
            report.result.componentId = patch.componentId;
        } else {
            report.diagnosticMessage = diagnostic;
        }
        report.timings.validationMilliseconds = elapsedMilliseconds(
            validationStart,
            Clock::now());
    } catch (const std::exception& exception) {
        report.result.clear();
        report.result.componentId = patch.componentId;
        report.diagnosticMessage = std::string("AutoRemesher exception: ") + exception.what();
    } catch (...) {
        report.result.clear();
        report.result.componentId = patch.componentId;
        report.diagnosticMessage = "AutoRemesher raised an unknown exception.";
    }
    report.timings.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
    return report;
}

}  // namespace directional_retopo

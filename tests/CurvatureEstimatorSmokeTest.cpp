#include "Field/CurvatureEstimator.h"
#include "Field/DensityFieldBuilder.h"

#include <maya/MVector.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

MVector tiltedNormal(double degrees)
{
    const double angle = degrees * kPi / 180.0;
    return MVector(std::sin(angle), 0.0, std::cos(angle));
}

void addPair(
    std::vector<directional_retopo::MeshFaceTopology>& faces,
    std::vector<directional_retopo::MeshEdgeTopology>& edges,
    double angleDegrees)
{
    using namespace directional_retopo;
    const int firstFace = static_cast<int>(faces.size());
    const int secondFace = firstFace + 1;
    const int edgeId = static_cast<int>(edges.size());

    MeshFaceTopology first;
    first.edgeIds.push_back(edgeId);
    first.adjacentFaceIds.push_back(secondFace);
    first.worldNormal = MVector(0.0, 0.0, 1.0);
    first.worldGeometryValid = true;
    faces.push_back(first);

    MeshFaceTopology second;
    second.edgeIds.push_back(edgeId);
    second.adjacentFaceIds.push_back(firstFace);
    second.worldNormal = tiltedNormal(angleDegrees);
    second.worldGeometryValid = true;
    faces.push_back(second);

    MeshEdgeTopology edge;
    edge.faceIds = {firstFace, secondFace};
    edge.worldLength = 1.0;
    edges.push_back(edge);
}

double curvatureTarget(
    double baseTarget,
    double curvature,
    const directional_retopo::CurvatureEstimatorSettings& settings)
{
    if (curvature <= settings.geometryEpsilon) {
        return baseTarget;
    }
    const double desiredVariation =
        settings.desiredNormalVariationDegrees * kPi / 180.0;
    return std::min(
        baseTarget,
        std::max(desiredVariation / curvature, baseTarget / 5.0));
}

}  // namespace

int main()
{
    using namespace directional_retopo;
    std::vector<MeshFaceTopology> faces;
    std::vector<MeshEdgeTopology> edges;
    addPair(faces, edges, 0.0);
    addPair(faces, edges, 10.0);
    addPair(faces, edges, 30.0);

    CurvatureEstimator estimator;
    CurvatureEstimatorSettings settings = estimator.settings();
    settings.neighborSpreadIterations = 0;
    estimator.setSettings(settings);

    std::vector<FaceCurvature> output;
    CurvatureEstimateMetrics metrics;
    if (!estimator.estimate(faces, edges, output, &metrics) ||
        output.size() != faces.size() || metrics.validFaceCount != faces.size()) {
        std::cerr << "Curvature estimation failed\n";
        return EXIT_FAILURE;
    }
    const double flat = output[0].indicator;
    const double medium = output[2].indicator;
    const double high = output[4].indicator;
    const double flatTarget = curvatureTarget(1.0, flat, settings);
    const double mediumTarget = curvatureTarget(1.0, medium, settings);
    const double highTarget = curvatureTarget(1.0, high, settings);
    if (flat > 1.0e-12 || !(medium > flat) || !(high > medium) ||
        std::abs(flatTarget - 1.0) > 1.0e-12 ||
        !(mediumTarget < flatTarget) || !(highTarget < mediumTarget) ||
        highTarget < 0.2 - 1.0e-12) {
        std::cerr << "Curvature classes or refinement clamp are incorrect\n";
        return EXIT_FAILURE;
    }

    PaintRegionData region;
    PaintRegionComponent component;
    for (std::size_t faceIndex = 0U; faceIndex < faces.size(); ++faceIndex) {
        component.coreFaceIds.push_back(static_cast<int>(faceIndex));
        component.allFaceIds.push_back(static_cast<int>(faceIndex));
    }
    for (std::size_t edgeIndex = 0U; edgeIndex < edges.size(); ++edgeIndex) {
        BoundaryEdge boundary;
        boundary.edgeId = static_cast<int>(edgeIndex);
        boundary.insideFaceId = edges[edgeIndex].faceIds.front();
        component.boundaryEdges.push_back(boundary);
    }
    region.components.push_back(component);

    DensityFieldBuilder densityBuilder;
    DensityFieldBuilderSettings densitySettings = densityBuilder.settings();
    densitySettings.mode = DensityMode::Auto;
    densitySettings.edgeLengthScale = 1.0;
    densitySettings.curvature.neighborSpreadIterations = 0;
    densityBuilder.setSettings(densitySettings);
    DensityFieldData density;
    DensityFieldBuildMetrics densityMetrics;
    if (!densityBuilder.build(region, faces, edges, density, &densityMetrics) ||
        density.perFace.size() != faces.size() ||
        !density.perFace[0].valid || !density.perFace[2].valid ||
        !density.perFace[4].valid ||
        std::abs(density.perFace[0].targetEdgeLength - 1.0) > 1.0e-10 ||
        !(density.perFace[2].targetEdgeLength <
          density.perFace[0].targetEdgeLength) ||
        !(density.perFace[4].targetEdgeLength <
          density.perFace[2].targetEdgeLength) ||
        density.perFace[4].targetEdgeLength < 0.2 - 1.0e-10 ||
        densityMetrics.curvatureConstrainedFaceCount != 4U ||
        densityMetrics.maximumAppliedCurvatureRefinementFactor >
            densitySettings.maximumCurvatureRefinementFactor + 1.0e-10) {
        std::cerr << "Curvature-aware Auto Density or clamp is incorrect\n";
        return EXIT_FAILURE;
    }

    std::cout << "Curvature estimator smoke test: success\n"
              << "flat indicator/target: " << flat << '/' << flatTarget << '\n'
              << "medium indicator/target: " << medium << '/'
              << mediumTarget << '\n'
              << "high indicator/target: " << high << '/' << highTarget << '\n';
    std::cout << "Auto Density flat/medium/high target: "
              << density.perFace[0].targetEdgeLength << '/'
              << density.perFace[2].targetEdgeLength << '/'
              << density.perFace[4].targetEdgeLength << '\n'
              << "Curvature constrained faces: "
              << densityMetrics.curvatureConstrainedFaceCount << '\n';

    densitySettings.mode = DensityMode::Manual;
    densitySettings.manualTargetEdgeLength = 0.75;
    densitySettings.edgeLengthScale = 2.0;
    densityBuilder.setSettings(densitySettings);
    if (!densityBuilder.build(region, faces, edges, density, &densityMetrics) ||
        density.mode != DensityMode::Manual ||
        densityMetrics.mode != DensityMode::Manual) {
        std::cerr << "Manual Density build failed\n";
        return EXIT_FAILURE;
    }
    for (const FaceDensity& faceDensity : density.perFace) {
        if (!faceDensity.valid) {
            continue;
        }
        if (faceDensity.autoDerived || faceDensity.curvatureLimited ||
            std::abs(faceDensity.baseTargetEdgeLength - 1.5) > 1.0e-10) {
            std::cerr << "Manual Density requested target semantics are incorrect\n";
            return EXIT_FAILURE;
        }
    }
    std::cout << "Manual Density requested target: "
              << density.perFace[0].baseTargetEdgeLength << '\n';
    return EXIT_SUCCESS;
}

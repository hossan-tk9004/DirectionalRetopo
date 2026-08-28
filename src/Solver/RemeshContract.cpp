#include "Solver/RemeshContract.h"

#include <algorithm>

namespace directional_retopo::solver {
namespace {

bool fail(std::string* reason, const char* message) noexcept
{
    if (reason != nullptr) {
        *reason = message;
    }
    return false;
}

}  // namespace

bool SourceMeshSnapshot::valid(std::string* reason) const noexcept
{
    if (vertices.empty() || faces.empty() || triangles.empty()) {
        return fail(reason, "Source mesh snapshot is empty.");
    }
    for (const SourceVertex& vertex : vertices) {
        if (!vertex.position.finite() || !vertex.normal.finite()) {
            return fail(reason, "Source mesh snapshot contains a non-finite vertex.");
        }
    }
    for (const SourceEdge& edge : edges) {
        if (edge.vertexIndices[0] >= vertices.size() ||
            edge.vertexIndices[1] >= vertices.size() ||
            edge.vertexIndices[0] == edge.vertexIndices[1]) {
            return fail(reason, "Source mesh snapshot contains an invalid edge.");
        }
    }
    for (const SourceTriangle& triangle : triangles) {
        if (triangle.faceIndex >= faces.size()) {
            return fail(reason, "Source mesh triangle has an invalid face index.");
        }
        for (const std::size_t index : triangle.vertexIndices) {
            if (index >= vertices.size()) {
                return fail(reason, "Source mesh triangle has an invalid vertex index.");
            }
        }
        const Vec3 normal =
            (vertices[triangle.vertexIndices[1]].position -
             vertices[triangle.vertexIndices[0]].position)
                .cross(vertices[triangle.vertexIndices[2]].position -
                       vertices[triangle.vertexIndices[0]].position);
        if (!normal.finite() || normal.squaredLength() <= 1.0e-24) {
            return fail(reason, "Source mesh snapshot contains a zero-area triangle.");
        }
    }
    return true;
}

bool RemeshInput::valid(std::string* reason) const noexcept
{
    if (!sourceMesh.valid(reason)) {
        return false;
    }
    if (components.empty()) {
        return fail(reason, "Remesh input contains no Region components.");
    }
    if (directionField.size() != sourceMesh.faces.size() ||
        densityField.size() != sourceMesh.faces.size()) {
        return fail(reason, "Remesh field arrays do not match the source face count.");
    }
    if (settings.topologyBlendWidth < 1U || settings.maximumRetryAttempts < 1U ||
        !(settings.geometryEpsilon > 0.0) || !(settings.areaEpsilon > 0.0)) {
        return fail(reason, "Remesh settings contain an invalid limit or tolerance.");
    }
    for (const RegionComponent& component : components) {
        if (component.coreFaceIndices.empty() || component.allFaceIndices.empty() ||
            component.fixedBoundaryLoops.empty() ||
            component.transitionRingDepthByFace.size() != sourceMesh.faces.size()) {
            return fail(reason, "Remesh Region component is incomplete.");
        }
        for (const OrderedBoundaryLoop& loop : component.fixedBoundaryLoops) {
            if (!loop.closed || loop.vertexIndices.size() < 3U) {
                return fail(reason, "Only closed ordered source boundaries are supported.");
            }
            if (std::any_of(
                    loop.vertexIndices.begin(),
                    loop.vertexIndices.end(),
                    [this](std::size_t index) {
                        return index >= sourceMesh.vertices.size();
                    })) {
                return fail(reason, "Region boundary contains an invalid vertex index.");
            }
        }
    }
    return true;
}

const char* failureCodeName(FailureCode code) noexcept
{
    switch (code) {
    case FailureCode::Success: return "Success";
    case FailureCode::InvalidInput: return "InvalidInput";
    case FailureCode::RegionTooSmall: return "RegionTooSmall";
    case FailureCode::PatchBuildFailed: return "PatchBuildFailed";
    case FailureCode::ParameterizationFailed: return "ParameterizationFailed";
    case FailureCode::QuadExtractionFailed: return "QuadExtractionFailed";
    case FailureCode::SurfaceConformationFailed: return "SurfaceConformationFailed";
    case FailureCode::InnerBoundaryInvalid: return "InnerBoundaryInvalid";
    case FailureCode::BoundaryLoopMismatch: return "BoundaryLoopMismatch";
    case FailureCode::TransitionBuildFailed: return "TransitionBuildFailed";
    case FailureCode::FinalValidationFailed: return "FinalValidationFailed";
    case FailureCode::ZeroAreaPolygon: return "ZeroAreaPolygon";
    case FailureCode::BoundaryCrossing: return "BoundaryCrossing";
    case FailureCode::NonManifoldResult: return "NonManifoldResult";
    case FailureCode::UnknownFailure: return "UnknownFailure";
    }
    return "UnknownFailure";
}

}  // namespace directional_retopo::solver

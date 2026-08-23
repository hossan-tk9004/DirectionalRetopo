#include <AutoRemesher/QuadExtractor>
#include <AutoRemesher/QuadParameterizer>
#include <AutoRemesher/Vector3>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

namespace {

bool finiteVector(const AutoRemesher::Vector3& value)
{
    return std::isfinite(value.x()) && std::isfinite(value.y()) &&
        std::isfinite(value.z());
}

}  // namespace

int main()
{
    constexpr std::size_t columns = 5;
    constexpr std::size_t rows = 4;
    constexpr double targetEdgeLength = 1.0;

    std::vector<AutoRemesher::Vector3> vertices;
    vertices.reserve(columns * rows);
    for (std::size_t row = 0; row < rows; ++row) {
        for (std::size_t column = 0; column < columns; ++column) {
            vertices.emplace_back(
                static_cast<double>(column),
                static_cast<double>(row),
                0.0);
        }
    }

    std::vector<std::vector<std::size_t>> triangles;
    triangles.reserve((columns - 1) * (rows - 1) * 2);
    for (std::size_t row = 0; row + 1 < rows; ++row) {
        for (std::size_t column = 0; column + 1 < columns; ++column) {
            const std::size_t lowerLeft = row * columns + column;
            const std::size_t lowerRight = lowerLeft + 1;
            const std::size_t upperLeft = lowerLeft + columns;
            const std::size_t upperRight = upperLeft + 1;
            triangles.push_back({lowerLeft, lowerRight, upperRight});
            triangles.push_back({lowerLeft, upperRight, upperLeft});
        }
    }

    std::vector<AutoRemesher::Vector3> guidance(
        triangles.size(),
        AutoRemesher::Vector3(1.0, 0.0, 0.0));
    std::vector<double> faceScaling(triangles.size(), 1.0);
    std::vector<double> directionalScaling(triangles.size(), 1.0);

    double totalEdgeLength = 0.0;
    std::size_t edgeCount = 0;
    for (const auto& triangle : triangles) {
        for (std::size_t corner = 0; corner < 3; ++corner) {
            const AutoRemesher::Vector3 edge =
                vertices[triangle[(corner + 1) % 3]] - vertices[triangle[corner]];
            totalEdgeLength += edge.length();
            ++edgeCount;
        }
    }
    const double approximateAverageEdgeLength =
        totalEdgeLength / static_cast<double>(edgeCount);
    const double globalScaling = targetEdgeLength / approximateAverageEdgeLength;

    AutoRemesher::QuadParameterizer::Result parameterization;
    const bool parameterized = AutoRemesher::QuadParameterizer::parameterize(
        vertices,
        triangles,
        &guidance,
        globalScaling,
        90.0,
        &parameterization,
        &faceScaling,
        &directionalScaling,
        &directionalScaling);
    if (!parameterized || parameterization.triangleUvs.size() != triangles.size()) {
        std::cerr << "parameterization failed" << std::endl;
        return 1;
    }
    for (const auto& triangleUvs : parameterization.triangleUvs) {
        if (triangleUvs.size() != 3) {
            std::cerr << "invalid triangle UV count" << std::endl;
            return 2;
        }
        for (const auto& uv : triangleUvs) {
            if (!std::isfinite(uv.x()) || !std::isfinite(uv.y())) {
                std::cerr << "non-finite parameterization" << std::endl;
                return 3;
            }
        }
    }

    AutoRemesher::QuadExtractor extractor(
        &vertices,
        &triangles,
        &parameterization.triangleUvs);
    extractor.setSingularVertices(&parameterization.singularVertices);
    const bool extracted = extractor.extract();
    if (!extracted || extractor.remeshedVertices().empty() ||
        extractor.remeshedQuads().empty()) {
        std::cerr << "quad extraction failed" << std::endl;
        return 4;
    }
    for (const AutoRemesher::Vector3& vertex : extractor.remeshedVertices()) {
        if (!finiteVector(vertex)) {
            std::cerr << "non-finite result vertex" << std::endl;
            return 5;
        }
    }

    std::size_t quadCount = 0;
    std::size_t nonQuadCount = 0;
    for (const auto& polygon : extractor.remeshedQuads()) {
        if (polygon.size() == 4) {
            ++quadCount;
        } else {
            ++nonQuadCount;
        }
    }

    std::cout << "parameterization success\n"
              << "quad extraction success\n"
              << "input vertices: " << vertices.size() << '\n'
              << "input triangles: " << triangles.size() << '\n'
              << "result vertices: " << extractor.remeshedVertices().size() << '\n'
              << "result polygons: " << extractor.remeshedQuads().size() << '\n'
              << "quads: " << quadCount << '\n'
              << "non-quads: " << nonQuadCount << std::endl;
    return 0;
}

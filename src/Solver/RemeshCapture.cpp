#include "Solver/RemeshCapture.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <limits>
#include <sstream>
#include <type_traits>
#include <utility>

namespace directional_retopo::solver {
namespace {

constexpr std::array<char, 32> kMagic = {
    'D','i','r','e','c','t','i','o','n','a','l','R','e','t','o','p','o',
    'R','e','m','e','s','h','I','n','p','u','t','\0','\0','\0','\0'};
constexpr std::uint32_t kEndianMarker = 0x01020304U;
constexpr std::uint64_t kMaximumContainerElements = 100000000ULL;
constexpr std::uint64_t kMaximumStringBytes = 64ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

class Writer final
{
public:
    explicit Writer(std::ostream& stream) : stream_(stream) {}

    template<typename T>
    void pod(const T& value)
    {
        static_assert(std::is_arithmetic_v<T> || std::is_enum_v<T>);
        stream_.write(reinterpret_cast<const char*>(&value), sizeof(T));
    }

    void boolean(bool value)
    {
        pod(static_cast<std::uint8_t>(value ? 1U : 0U));
    }

    void size(std::size_t value)
    {
        pod(static_cast<std::uint64_t>(value));
    }

    void string(const std::string& value)
    {
        size(value.size());
        stream_.write(value.data(), static_cast<std::streamsize>(value.size()));
    }

    template<typename T, typename Function>
    void vector(const std::vector<T>& values, Function writeValue)
    {
        size(values.size());
        for (const T& value : values) {
            writeValue(value);
        }
    }

    [[nodiscard]] bool good() const noexcept { return stream_.good(); }

private:
    std::ostream& stream_;
};

class Reader final
{
public:
    explicit Reader(std::istream& stream) : stream_(stream) {}

    template<typename T>
    bool pod(T& value)
    {
        static_assert(std::is_arithmetic_v<T> || std::is_enum_v<T>);
        stream_.read(reinterpret_cast<char*>(&value), sizeof(T));
        return stream_.good();
    }

    bool boolean(bool& value)
    {
        std::uint8_t stored = 0U;
        if (!pod(stored) || stored > 1U) {
            return false;
        }
        value = stored != 0U;
        return true;
    }

    bool size(std::size_t& value, std::uint64_t maximum = kMaximumContainerElements)
    {
        std::uint64_t stored = 0U;
        if (!pod(stored) || stored > maximum ||
            stored > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
            return false;
        }
        value = static_cast<std::size_t>(stored);
        return true;
    }

    bool string(std::string& value)
    {
        std::size_t count = 0U;
        if (!size(count, kMaximumStringBytes)) {
            return false;
        }
        value.resize(count);
        stream_.read(value.data(), static_cast<std::streamsize>(count));
        return stream_.good();
    }

    template<typename T, typename Function>
    bool vector(std::vector<T>& values, Function readValue)
    {
        std::size_t count = 0U;
        if (!size(count)) {
            return false;
        }
        values.resize(count);
        for (T& value : values) {
            if (!readValue(value)) {
                return false;
            }
        }
        return true;
    }

private:
    std::istream& stream_;
};

void writeVec3(Writer& writer, const Vec3& value)
{
    writer.pod(value.x);
    writer.pod(value.y);
    writer.pod(value.z);
}

bool readVec3(Reader& reader, Vec3& value)
{
    return reader.pod(value.x) && reader.pod(value.y) && reader.pod(value.z);
}

void writeIndices(Writer& writer, const std::vector<std::size_t>& values)
{
    writer.vector(values, [&writer](std::size_t value) { writer.size(value); });
}

bool readIndices(Reader& reader, std::vector<std::size_t>& values)
{
    return reader.vector(values, [&reader](std::size_t& value) {
        return reader.size(value);
    });
}

void writeSourceIds(Writer& writer, const std::vector<SourceId>& values)
{
    writer.vector(values, [&writer](SourceId value) { writer.pod(value); });
}

bool readSourceIds(Reader& reader, std::vector<SourceId>& values)
{
    return reader.vector(values, [&reader](SourceId& value) {
        return reader.pod(value);
    });
}

void writeInput(Writer& writer, const RemeshInput& input)
{
    writer.vector(input.sourceMesh.vertices, [&writer](const SourceVertex& value) {
        writeVec3(writer, value.position);
        writeVec3(writer, value.normal);
        writer.pod(value.sourceVertexId);
        writeIndices(writer, value.adjacentVertexIndices);
        writeIndices(writer, value.edgeIndices);
        writeIndices(writer, value.faceIndices);
    });
    writer.vector(input.sourceMesh.edges, [&writer](const SourceEdge& value) {
        writer.size(value.vertexIndices[0]);
        writer.size(value.vertexIndices[1]);
        writeIndices(writer, value.faceIndices);
        writer.pod(value.sourceEdgeId);
        writer.pod(value.length);
        writer.boolean(value.originalMeshBoundary);
    });
    writer.vector(input.sourceMesh.faces, [&writer](const SourceFace& value) {
        writeIndices(writer, value.vertexIndices);
        writeIndices(writer, value.edgeIndices);
        writeIndices(writer, value.adjacentFaceIndices);
        writeIndices(writer, value.triangleIndices);
        writeVec3(writer, value.center);
        writeVec3(writer, value.normal);
        writer.pod(value.sourceFaceId);
        writer.boolean(value.geometryValid);
    });
    writer.vector(input.sourceMesh.triangles, [&writer](const SourceTriangle& value) {
        writer.size(value.vertexIndices[0]);
        writer.size(value.vertexIndices[1]);
        writer.size(value.vertexIndices[2]);
        writer.size(value.faceIndex);
    });
    writer.vector(input.components, [&writer](const RegionComponent& value) {
        writer.size(value.componentId);
        writeIndices(writer, value.coreFaceIndices);
        writeIndices(writer, value.transitionFaceIndices);
        writeIndices(writer, value.allFaceIndices);
        writer.vector(value.transitionRingDepthByFace, [&writer](int depth) {
            writer.pod(static_cast<std::int32_t>(depth));
        });
        writer.vector(value.fixedBoundaryLoops, [&writer](const OrderedBoundaryLoop& loop) {
            writeIndices(writer, loop.vertexIndices);
            writeIndices(writer, loop.edgeIndices);
            writeSourceIds(writer, loop.sourceVertexIds);
            writeSourceIds(writer, loop.sourceEdgeIds);
            writer.boolean(loop.closed);
            writer.boolean(loop.touchesOriginalMeshBoundary);
        });
    });
    writer.vector(input.directionField, [&writer](const FaceDirection& value) {
        writeVec3(writer, value.normal);
        writeVec3(writer, value.uDirection);
        writeVec3(writer, value.vDirection);
        writer.pod(value.paintConstraintWeight);
        writer.pod(value.topologyGuidanceWeight);
        writer.boolean(value.valid);
    });
    writer.vector(input.densityField, [&writer](const FaceDensity& value) {
        writer.pod(value.requestedTargetEdgeLength);
        writer.pod(value.effectiveTargetEdgeLength);
        writer.pod(value.scaleU);
        writer.pod(value.scaleV);
        writer.boolean(value.curvatureConstrained);
        writer.boolean(value.valid);
    });
    writer.pod(input.settings.topologyBlendWidth);
    writer.pod(input.settings.topologyPolicy);
    writer.pod(input.settings.trianglePolicy);
    writer.pod(input.settings.maximumRetryAttempts);
    writer.pod(input.settings.geometryEpsilon);
    writer.pod(input.settings.areaEpsilon);
    writer.boolean(input.settings.retainDebugResults);
}

bool readInput(Reader& reader, RemeshInput& input)
{
    input = RemeshInput();
    if (!reader.vector(input.sourceMesh.vertices, [&reader](SourceVertex& value) {
            return readVec3(reader, value.position) && readVec3(reader, value.normal) &&
                reader.pod(value.sourceVertexId) &&
                readIndices(reader, value.adjacentVertexIndices) &&
                readIndices(reader, value.edgeIndices) && readIndices(reader, value.faceIndices);
        }) ||
        !reader.vector(input.sourceMesh.edges, [&reader](SourceEdge& value) {
            return reader.size(value.vertexIndices[0]) && reader.size(value.vertexIndices[1]) &&
                readIndices(reader, value.faceIndices) && reader.pod(value.sourceEdgeId) &&
                reader.pod(value.length) && reader.boolean(value.originalMeshBoundary);
        }) ||
        !reader.vector(input.sourceMesh.faces, [&reader](SourceFace& value) {
            return readIndices(reader, value.vertexIndices) && readIndices(reader, value.edgeIndices) &&
                readIndices(reader, value.adjacentFaceIndices) &&
                readIndices(reader, value.triangleIndices) && readVec3(reader, value.center) &&
                readVec3(reader, value.normal) && reader.pod(value.sourceFaceId) &&
                reader.boolean(value.geometryValid);
        }) ||
        !reader.vector(input.sourceMesh.triangles, [&reader](SourceTriangle& value) {
            return reader.size(value.vertexIndices[0]) && reader.size(value.vertexIndices[1]) &&
                reader.size(value.vertexIndices[2]) && reader.size(value.faceIndex);
        }) ||
        !reader.vector(input.components, [&reader](RegionComponent& value) {
            if (!reader.size(value.componentId) || !readIndices(reader, value.coreFaceIndices) ||
                !readIndices(reader, value.transitionFaceIndices) ||
                !readIndices(reader, value.allFaceIndices) ||
                !reader.vector(value.transitionRingDepthByFace, [&reader](int& depth) {
                    std::int32_t stored = 0;
                    if (!reader.pod(stored)) return false;
                    depth = static_cast<int>(stored);
                    return true;
                })) {
                return false;
            }
            return reader.vector(value.fixedBoundaryLoops, [&reader](OrderedBoundaryLoop& loop) {
                return readIndices(reader, loop.vertexIndices) && readIndices(reader, loop.edgeIndices) &&
                    readSourceIds(reader, loop.sourceVertexIds) &&
                    readSourceIds(reader, loop.sourceEdgeIds) && reader.boolean(loop.closed) &&
                    reader.boolean(loop.touchesOriginalMeshBoundary);
            });
        }) ||
        !reader.vector(input.directionField, [&reader](FaceDirection& value) {
            return readVec3(reader, value.normal) && readVec3(reader, value.uDirection) &&
                readVec3(reader, value.vDirection) && reader.pod(value.paintConstraintWeight) &&
                reader.pod(value.topologyGuidanceWeight) && reader.boolean(value.valid);
        }) ||
        !reader.vector(input.densityField, [&reader](FaceDensity& value) {
            return reader.pod(value.requestedTargetEdgeLength) &&
                reader.pod(value.effectiveTargetEdgeLength) && reader.pod(value.scaleU) &&
                reader.pod(value.scaleV) && reader.boolean(value.curvatureConstrained) &&
                reader.boolean(value.valid);
        }) ||
        !reader.pod(input.settings.topologyBlendWidth) ||
        !reader.pod(input.settings.topologyPolicy) ||
        !reader.pod(input.settings.trianglePolicy) ||
        !reader.pod(input.settings.maximumRetryAttempts) ||
        !reader.pod(input.settings.geometryEpsilon) ||
        !reader.pod(input.settings.areaEpsilon) ||
        !reader.boolean(input.settings.retainDebugResults)) {
        return false;
    }
    return true;
}

void writeQuality(Writer& writer, const QualityMetrics& value)
{
    writer.size(value.quadCount);
    writer.size(value.triangleCount);
    writer.size(value.nGonCount);
    writer.size(value.boundaryCrossingCount);
    writer.size(value.nonManifoldEdgeCount);
    writer.size(value.zeroAreaPolygonCount);
    writer.pod(value.maximumBoundaryDisplacement);
    writer.pod(value.meanSurfaceDistance);
    writer.pod(value.p95SurfaceDistance);
    writer.pod(value.maximumSurfaceDistance);
    writer.pod(value.meanCoreDirectionDeviationDegrees);
    writer.pod(value.maximumCoreDirectionDeviationDegrees);
    writer.pod(value.requestedCoreEdgeLength);
    writer.pod(value.actualCoreEdgeLength);
}

bool readQuality(Reader& reader, QualityMetrics& value)
{
    return reader.size(value.quadCount) && reader.size(value.triangleCount) &&
        reader.size(value.nGonCount) && reader.size(value.boundaryCrossingCount) &&
        reader.size(value.nonManifoldEdgeCount) && reader.size(value.zeroAreaPolygonCount) &&
        reader.pod(value.maximumBoundaryDisplacement) && reader.pod(value.meanSurfaceDistance) &&
        reader.pod(value.p95SurfaceDistance) && reader.pod(value.maximumSurfaceDistance) &&
        reader.pod(value.meanCoreDirectionDeviationDegrees) &&
        reader.pod(value.maximumCoreDirectionDeviationDegrees) &&
        reader.pod(value.requestedCoreEdgeLength) && reader.pod(value.actualCoreEdgeLength);
}

void writeSummary(Writer& writer, const CapturedSolveSummary& summary)
{
    writer.pod(summary.status);
    writer.pod(summary.failureCode);
    writer.pod(summary.topologySignature);
    writer.vector(summary.components, [&writer](const CapturedComponentSummary& component) {
        writer.size(component.componentId);
        writer.pod(component.status);
        writer.pod(component.failureCode);
        writer.size(component.vertexCount);
        writer.size(component.polygonCount);
        writeQuality(writer, component.quality);
        writer.pod(component.topologySignature);
        writer.string(component.diagnosticMessage);
    });
}

bool readSummary(Reader& reader, CapturedSolveSummary& summary)
{
    return reader.pod(summary.status) && reader.pod(summary.failureCode) &&
        reader.pod(summary.topologySignature) &&
        reader.vector(summary.components, [&reader](CapturedComponentSummary& component) {
            return reader.size(component.componentId) && reader.pod(component.status) &&
                reader.pod(component.failureCode) && reader.size(component.vertexCount) &&
                reader.size(component.polygonCount) && readQuality(reader, component.quality) &&
                reader.pod(component.topologySignature) && reader.string(component.diagnosticMessage);
        });
}

std::uint64_t fnv(std::uint64_t hash, std::uint64_t value) noexcept
{
    for (unsigned int byte = 0U; byte < 8U; ++byte) {
        hash ^= (value >> (byte * 8U)) & 0xffU;
        hash *= kFnvPrime;
    }
    return hash;
}

std::vector<std::size_t> canonicalPolygon(const std::vector<std::size_t>& polygon)
{
    std::vector<std::size_t> best;
    for (const bool reverse : {false, true}) {
        for (std::size_t offset = 0U; offset < polygon.size(); ++offset) {
            std::vector<std::size_t> candidate;
            candidate.reserve(polygon.size());
            for (std::size_t index = 0U; index < polygon.size(); ++index) {
                const std::size_t position = reverse
                    ? (offset + polygon.size() - index) % polygon.size()
                    : (offset + index) % polygon.size();
                candidate.push_back(polygon[position]);
            }
            if (best.empty() || candidate < best) {
                best = std::move(candidate);
            }
        }
    }
    return best;
}

std::uint64_t componentSignature(const ComponentResult& component)
{
    std::uint64_t hash = kFnvOffset;
    hash = fnv(hash, component.componentId);
    hash = fnv(hash, static_cast<std::uint64_t>(component.status));
    hash = fnv(hash, static_cast<std::uint64_t>(component.failureCode));
    for (const Vec3& vertex : component.vertices) {
        for (const double value : {vertex.x, vertex.y, vertex.z}) {
            hash = fnv(hash, static_cast<std::uint64_t>(
                static_cast<std::int64_t>(std::llround(value * 1.0e6))));
        }
    }
    std::vector<std::vector<std::size_t>> polygons;
    polygons.reserve(component.polygons.size());
    for (const ResultPolygon& polygon : component.polygons) {
        polygons.push_back(canonicalPolygon(polygon.vertexIndices));
    }
    std::sort(polygons.begin(), polygons.end());
    for (const std::vector<std::size_t>& polygon : polygons) {
        hash = fnv(hash, polygon.size());
        for (const std::size_t value : polygon) {
            hash = fnv(hash, value);
        }
    }
    return hash;
}

bool sameComponent(
    const CapturedComponentSummary& expected,
    const CapturedComponentSummary& actual,
    std::string& diagnostic)
{
    if (expected.componentId != actual.componentId || expected.status != actual.status ||
        expected.failureCode != actual.failureCode || expected.vertexCount != actual.vertexCount ||
        expected.polygonCount != actual.polygonCount ||
        expected.quality.quadCount != actual.quality.quadCount ||
        expected.quality.triangleCount != actual.quality.triangleCount ||
        expected.quality.nGonCount != actual.quality.nGonCount ||
        expected.topologySignature != actual.topologySignature) {
        std::ostringstream message;
        message << "Component " << expected.componentId
                << " differs (status/code/vertices/polygons/Q/T/N/hash).";
        diagnostic = message.str();
        return false;
    }
    return true;
}

}  // namespace

CapturedSolveSummary summarizeResult(const RemeshResult& result) noexcept
{
    CapturedSolveSummary summary;
    summary.status = result.status;
    summary.failureCode = result.failureCode;
    summary.topologySignature = kFnvOffset;
    try {
        summary.components.reserve(result.components.size());
        for (const ComponentResult& source : result.components) {
            CapturedComponentSummary component;
            component.componentId = source.componentId;
            component.status = source.status;
            component.failureCode = source.failureCode;
            component.vertexCount = source.vertices.size();
            component.polygonCount = source.polygons.size();
            component.quality = source.quality;
            component.topologySignature = componentSignature(source);
            component.diagnosticMessage = source.diagnosticMessage;
            summary.topologySignature = fnv(summary.topologySignature, component.topologySignature);
            summary.components.push_back(std::move(component));
        }
    } catch (...) {
        summary.components.clear();
        summary.topologySignature = 0U;
    }
    return summary;
}

std::uint64_t remeshInputSignature(const RemeshInput& input) noexcept
{
    try {
        std::ostringstream stream(std::ios::binary | std::ios::out);
        Writer writer(stream);
        writeInput(writer, input);
        const std::string bytes = stream.str();
        std::uint64_t hash = kFnvOffset;
        for (const unsigned char value : bytes) {
            hash ^= value;
            hash *= kFnvPrime;
        }
        return hash;
    } catch (...) {
        return 0U;
    }
}

bool saveRemeshCapture(
    const std::string& path,
    const RemeshCaptureRecord& record,
    std::string& diagnostic) noexcept
{
    diagnostic.clear();
    try {
        std::ofstream stream(path, std::ios::binary | std::ios::trunc);
        if (!stream) {
            diagnostic = "Could not open the Remesh capture file for writing: " + path;
            return false;
        }
        Writer writer(stream);
        stream.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));
        writer.pod(kRemeshCaptureFormatVersion);
        writer.pod(kEndianMarker);
        writer.string(record.label);
        writeInput(writer, record.input);
        writer.boolean(record.hasExpectedResult);
        if (record.hasExpectedResult) {
            writeSummary(writer, record.expectedResult);
        }
        if (!writer.good()) {
            diagnostic = "Writing the Remesh capture file failed: " + path;
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        diagnostic = std::string("Remesh capture write exception: ") + exception.what();
    } catch (...) {
        diagnostic = "Unknown Remesh capture write exception.";
    }
    return false;
}

bool loadRemeshCapture(
    const std::string& path,
    RemeshCaptureRecord& record,
    std::string& diagnostic) noexcept
{
    diagnostic.clear();
    record = RemeshCaptureRecord();
    try {
        std::ifstream stream(path, std::ios::binary);
        if (!stream) {
            diagnostic = "Could not open the Remesh capture file for reading: " + path;
            return false;
        }
        std::array<char, kMagic.size()> magic{};
        stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
        Reader reader(stream);
        std::uint32_t version = 0U;
        std::uint32_t endian = 0U;
        if (!stream.good() || magic != kMagic || !reader.pod(version) ||
            !reader.pod(endian)) {
            diagnostic = "The file is not a DirectionalRetopo RemeshInput capture.";
            return false;
        }
        if (version != kRemeshCaptureFormatVersion) {
            diagnostic = "Unsupported RemeshInput capture version: " + std::to_string(version);
            return false;
        }
        if (endian != kEndianMarker) {
            diagnostic = "RemeshInput capture endian marker does not match this build.";
            return false;
        }
        if (!reader.string(record.label) || !readInput(reader, record.input) ||
            !reader.boolean(record.hasExpectedResult) ||
            (record.hasExpectedResult && !readSummary(reader, record.expectedResult))) {
            diagnostic = "The RemeshInput capture is truncated or malformed.";
            return false;
        }
        std::string validation;
        if (!record.input.valid(&validation)) {
            diagnostic = "The deserialized RemeshInput is invalid: " + validation;
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        diagnostic = std::string("Remesh capture read exception: ") + exception.what();
    } catch (...) {
        diagnostic = "Unknown Remesh capture read exception.";
    }
    return false;
}

bool replayMatchesCapture(
    const CapturedSolveSummary& expected,
    const RemeshResult& actual,
    std::string& diagnostic)
{
    diagnostic.clear();
    const CapturedSolveSummary observed = summarizeResult(actual);
    if (expected.status != observed.status || expected.failureCode != observed.failureCode) {
        diagnostic = "SolveStatus or FailureCode differs from the Maya capture.";
        return false;
    }
    if (expected.components.size() != observed.components.size()) {
        diagnostic = "Component count differs from the Maya capture.";
        return false;
    }
    for (std::size_t index = 0U; index < expected.components.size(); ++index) {
        if (!sameComponent(expected.components[index], observed.components[index], diagnostic)) {
            return false;
        }
    }
    if (expected.topologySignature != observed.topologySignature) {
        diagnostic = "Aggregate topology signature differs from the Maya capture.";
        return false;
    }
    return true;
}

}  // namespace directional_retopo::solver

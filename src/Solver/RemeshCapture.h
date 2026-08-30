#pragma once

#include "Solver/RemeshContract.h"

#include <cstdint>
#include <string>
#include <vector>

namespace directional_retopo::solver {

constexpr std::uint32_t kRemeshCaptureFormatVersion = 1U;

struct CapturedComponentSummary final
{
    std::size_t componentId = 0U;
    SolveStatus status = SolveStatus::Failed;
    FailureCode failureCode = FailureCode::UnknownFailure;
    std::size_t vertexCount = 0U;
    std::size_t polygonCount = 0U;
    QualityMetrics quality;
    std::uint64_t topologySignature = 0U;
    std::string diagnosticMessage;
};

struct CapturedSolveSummary final
{
    SolveStatus status = SolveStatus::Failed;
    FailureCode failureCode = FailureCode::UnknownFailure;
    std::uint64_t topologySignature = 0U;
    std::vector<CapturedComponentSummary> components;
};

struct RemeshCaptureRecord final
{
    std::string label;
    RemeshInput input;
    bool hasExpectedResult = false;
    CapturedSolveSummary expectedResult;
};

[[nodiscard]] CapturedSolveSummary summarizeResult(
    const RemeshResult& result) noexcept;

[[nodiscard]] std::uint64_t remeshInputSignature(
    const RemeshInput& input) noexcept;

[[nodiscard]] bool saveRemeshCapture(
    const std::string& path,
    const RemeshCaptureRecord& record,
    std::string& diagnostic) noexcept;

[[nodiscard]] bool loadRemeshCapture(
    const std::string& path,
    RemeshCaptureRecord& record,
    std::string& diagnostic) noexcept;

[[nodiscard]] bool replayMatchesCapture(
    const CapturedSolveSummary& expected,
    const RemeshResult& actual,
    std::string& diagnostic);

}  // namespace directional_retopo::solver

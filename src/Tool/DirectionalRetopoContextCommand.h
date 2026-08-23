#pragma once

#include <maya/MPxContextCommand.h>
#include <maya/MStatus.h>

namespace directional_retopo {

class DirectionalRetopoContext;

class DirectionalRetopoContextCommand final : public MPxContextCommand
{
public:
    DirectionalRetopoContextCommand() = default;
    ~DirectionalRetopoContextCommand() override = default;

    static void* creator();

    MPxContext* makeObj() override;
    MStatus appendSyntax() override;
    MStatus doEditFlags() override;
    MStatus doQueryFlags() override;

private:
    DirectionalRetopoContext* context_ = nullptr;
};

}  // namespace directional_retopo

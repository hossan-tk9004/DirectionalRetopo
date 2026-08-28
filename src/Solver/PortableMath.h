#pragma once

#include <cmath>

namespace directional_retopo::solver {

struct Vec2 final
{
    double x = 0.0;
    double y = 0.0;

    [[nodiscard]] bool finite() const noexcept
    {
        return std::isfinite(x) && std::isfinite(y);
    }
};

struct Vec3 final
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    [[nodiscard]] Vec3 operator+(const Vec3& other) const noexcept
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    [[nodiscard]] Vec3 operator-(const Vec3& other) const noexcept
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    [[nodiscard]] Vec3 operator*(double scalar) const noexcept
    {
        return {x * scalar, y * scalar, z * scalar};
    }

    [[nodiscard]] Vec3 operator/(double scalar) const noexcept
    {
        return {x / scalar, y / scalar, z / scalar};
    }

    Vec3& operator+=(const Vec3& other) noexcept
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    [[nodiscard]] double dot(const Vec3& other) const noexcept
    {
        return x * other.x + y * other.y + z * other.z;
    }

    [[nodiscard]] Vec3 cross(const Vec3& other) const noexcept
    {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x};
    }

    [[nodiscard]] double squaredLength() const noexcept
    {
        return dot(*this);
    }

    [[nodiscard]] double length() const noexcept
    {
        return std::sqrt(squaredLength());
    }

    [[nodiscard]] Vec3 normalized(double epsilon = 1.0e-12) const noexcept
    {
        const double magnitude = length();
        return finite() && std::isfinite(magnitude) && magnitude > epsilon
            ? *this / magnitude
            : Vec3();
    }

    [[nodiscard]] bool finite() const noexcept
    {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

[[nodiscard]] inline Vec3 operator*(double scalar, const Vec3& vector) noexcept
{
    return vector * scalar;
}

}  // namespace directional_retopo::solver

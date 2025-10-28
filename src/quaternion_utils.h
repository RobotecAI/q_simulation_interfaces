/* Copyright 2025, Robotec.ai sp. z o.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

inline tf2::Quaternion AxisAngleToQuaternion(const tf2::Vector3& axis, const double angle)
{
    tf2::Quaternion q;

    // Threshold for considering axis as non-zero
    constexpr double kAxisEpsilon = 1e-6;
    if (axis.length() > kAxisEpsilon)
    {
        q = tf2::Quaternion(axis, angle);
        q.normalize();
    }
    else
    {
        // Default to no rotation if vector is zero
        q.setValue(0.0, 0.0, 0.0, 1.0);
    }

    if (std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w()))
    {
        q.setValue(0.0, 0.0, 0.0, 1.0);
    }

    return q;
}

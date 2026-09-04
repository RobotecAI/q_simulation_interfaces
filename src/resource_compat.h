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

#include <type_traits>

//! Compatibility helpers for the resource fields of simulation_interfaces.
//!
//! simulation_interfaces 2.0 replaced the flat `uri` and `resource_string` fields of
//! SpawnEntity::Request, LoadWorld::Request and Spawnable with a single nested
//! simulation_interfaces/msg/Resource member, named `entity_resource` or `world_resource`
//! depending on the message. The accessors below resolve the field at compile time, so the
//! panel builds against both the 1.x and the 2.x message definitions.

namespace ResourceCompat
{
    template <typename T, typename = void>
    struct HasEntityResource : std::false_type
    {
    };

    template <typename T>
    struct HasEntityResource<T, std::void_t<decltype(std::declval<T&>().entity_resource)>> : std::true_type
    {
    };

    template <typename T, typename = void>
    struct HasWorldResource : std::false_type
    {
    };

    template <typename T>
    struct HasWorldResource<T, std::void_t<decltype(std::declval<T&>().world_resource)>> : std::true_type
    {
    };
} // namespace ResourceCompat

//! Returns the `uri` field of a message, whether it is nested in a Resource member or not.
//! Constness of the message is preserved in the returned reference.
template <typename T>
auto& ResourceUri(T& message)
{
    if constexpr (ResourceCompat::HasEntityResource<T>::value)
    {
        return message.entity_resource.uri;
    }
    else if constexpr (ResourceCompat::HasWorldResource<T>::value)
    {
        return message.world_resource.uri;
    }
    else
    {
        return message.uri;
    }
}

//! Returns the `resource_string` field of a message, whether it is nested in a Resource member or not.
//! Constness of the message is preserved in the returned reference.
template <typename T>
auto& ResourceString(T& message)
{
    if constexpr (ResourceCompat::HasEntityResource<T>::value)
    {
        return message.entity_resource.resource_string;
    }
    else if constexpr (ResourceCompat::HasWorldResource<T>::value)
    {
        return message.world_resource.resource_string;
    }
    else
    {
        return message.resource_string;
    }
}

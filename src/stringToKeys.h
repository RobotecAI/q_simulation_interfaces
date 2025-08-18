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

#include <unordered_map>
#include <string>

const std::map<int, std::string> FeatureToName
        {
                {0,  "SPAWNING"},
                {1,  "DELETING"},
                {2,  "NAMED_POSES"},
                {3,  "POSE_BOUNDS"},
                {4,  "ENTITY_TAGS"},
                {5,  "ENTITY_BOUNDS"},
                {6,  "ENTITY_BOUNDS_BOX"},
                {7,  "ENTITY_BOUNDS_CONVEX"},
                {8,  "ENTITY_CATEGORIES"},
                {9,  "SPAWNING_RESOURCE_STRING"},
                {10, "ENTITY_STATE_GETTING"},
                {11, "ENTITY_STATE_SETTING"},
                {12, "ENTITY_INFO_GETTING"},
                {13, "ENTITY_INFO_SETTING"},
                {14, "SPAWNABLES"},
                {20, "SIMULATION_RESET"},
                {21, "SIMULATION_RESET_TIME"},
                {22, "SIMULATION_RESET_STATE"},
                {23, "SIMULATION_RESET_SPAWNED"},
                {24, "SIMULATION_STATE_GETTING"},
                {25, "SIMULATION_STATE_SETTING"},
                {26, "SIMULATION_STATE_PAUSE"},
                {31, "STEP_SIMULATION_SINGLE"},
                {32, "STEP_SIMULATION_MULTIPLE"},
                {33, "STEP_SIMULATION_ACTION"},
        };
const std::map<int, std::string> FeatureDescription
        {
                {0,  "Supports spawn interface (SpawnEntity)."},
                {1,  "Supports deleting entities (DeleteEntity)."},
                {2,  "Supports predefined named poses (GetNamedPoses)."},
                {3,  "Supports pose bounds (GetNamedPoseBounds)."},
                {4,  "Supports entity tags in interfaces using EntityFilters, such as GetEntities."},
                {5,  "Supports entity bounds (GetEntityBounds)."},
                {6,  "Supports entity filtering with bounds with TYPE_BOX."},
                {7,  "Supports entity filtering with Bounds TYPE_CONVEX_HULL."},
                {8,  "Supports entity categories, such as in use with EntityFilters or SetEntityInfo."},
                {9,  "Supports SpawnEntity resource_string field."},
                {10, "Supports GetEntityState interface."},
                {11, "Supports SetEntityState interface."},
                {12, "Supports GetEntityInfo interface."},
                {13, "Supports SetEntityInfo interface."},
                {14, "Supports GetSpawnables interface."},
                {20, "Supports one or more ways to reset the simulation through ResetSimulation."},
                {21, "Supports SCOPE_TIME flag for resetting"},
                {22, "Supports SCOPE_STATE flag for resetting."},
                {23, "Supports SCOPE_SPAWNED flag for resetting."},
                {24, "Supports GetSimulationState interface."},
                {25, "Supports SetSimulationState interface. Check SIMULATION_STATE_PAUSE feature for setting STATE_PAUSED."},
                {26, "Supports the STATE_PAUSED SimulationState in SetSimulationState interface."},
                {31, "Supports single stepping through simulation with StepSimulation interface."},
                {32, "Supports multi-stepping through simulation, either through StepSimulation service or StepSimulation action."},
                {33, "Supports SimulateSteps action interface."},
        };
const std::map<std::string, int> ScopeNameToId
        {
                {"SCOPE_DEFAULT", 0},
                {"SCOPE_TIME",    1},
                {"SCOPE_STATE",   2},
                {"SCOPE_SPAWNED", 4},
                {"SCOPE_ALL",     255},
        };

const std::map<int, std::string> ErrorIdToName
        {
                {0, "RESULT_FEATURE_UNSUPPORTED"},
                {2, "RESULT_NOT_FOUND"},
                {3, "RESULT_INCORRECT_STATE"},
                {4, "RESULT_OPERATION_FAILED"},
        };
const std::map<int, std::string> SimStateIdToName
        {
                {0, "STATE_STOPPED"},
                {1, "STATE_PLAYING"},
                {2, "STATE_PAUSED"},
                {3, "STATE_QUITTING"},
        };


const std::map<std::string, int> SimStateNameToId
        {
                {"STATE_STOPPED",  0},
                {"STATE_PLAYING",  1},
                {"STATE_PAUSED",   2},
                {"STATE_QUITTING", 3},
        };

#pragma once
#include <unordered_map>
#include <string>

const std::map<int, std::string> FeatureToName
{
        {0, "SPAWNING"},
        {1, "DELETING"},
        {2, "NAMED_POSES"},
        {3, "POSE_BOUNDS"},
        {4, "ENTITY_TAGS"},
        {5, "ENTITY_BOUNDS"},
        {6, "ENTITY_BOUNDS_BOX"},
        {7, "ENTITY_BOUNDS_CONVEX"},
        {8, "ENTITY_CATEGORIES"},
        {9, "SPAWNING_RESOURCE_STRING"},
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

const std::map<std::string, int> ScopeNameToId
{
        {"SCOPE_DEFAULT", 0},
        {"SCOPE_TIME", 1},
        {"SCOPE_STATE", 2},
        {"SCOPE_SPAWNED", 4},
        {"SCOPE_ALL", 255},
};
/**
 * @file yaml_config_parser.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef YAML_CONFIG_PARSER_H
#define YAML_CONFIG_PARSER_H

#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "limxsdk/macros.h"

namespace limxsdk {
namespace ability {

struct LIMX_SDK_API AbilityConfig {
    std::string name;
    std::string type;
    bool autostart;
    YAML::Node config;
};

struct LIMX_SDK_API LibraryConfig {
    std::string library;
    std::vector<AbilityConfig> abilities;
};

struct LIMX_SDK_API SystemConfig {
    std::string robotIp;
    std::string robotType;
    std::vector<LibraryConfig> libraries;
};

class LIMX_SDK_API YamlConfigParser {
public:
    static SystemConfig parse(const std::string& yamlPath) {
        SystemConfig config;
        try {
            YAML::Node yamlConfig = YAML::LoadFile(yamlPath);
            
            // Parse system configuration
            config.robotIp = "127.0.0.1";
            const char* env_robot_ip = std::getenv("ROBOT_IP");
            if (env_robot_ip != nullptr) {
                config.robotIp = std::string(env_robot_ip);
            } else {
                if (yamlConfig["robot_ip"]) {
                    config.robotIp = yamlConfig["robot_ip"].as<std::string>();
                }
            }
            
            if (yamlConfig["robot_type"]) {
                config.robotType = yamlConfig["robot_type"].as<std::string>();
            }
            
            // Parse libraries
            if (yamlConfig["libraries"]) {
                for (const auto& libraryNode : yamlConfig["libraries"]) {
                    LibraryConfig library;
                    
                    if (libraryNode["library"]) {
                        library.library = libraryNode["library"].as<std::string>();
                        // Automatically append .dll or .so suffix
                        if (library.library.find_last_of('.') == std::string::npos) {
                            #ifdef _WIN32
                                library.library += ".dll";
                            #else
                                library.library += ".so";
                            #endif
                        }
                    }
                    
                    // Parse abilities for this library
                    if (libraryNode["abilities"]) {
                        for (const auto& abilityNode : libraryNode["abilities"]) {
                            AbilityConfig ability;
                            if (!abilityNode["name"] || !abilityNode["type"]) {
                                std::cerr << "Error: Ability is missing required 'name' or 'type' field in config" << std::endl;
                                abort();
                            }

                            ability.name = abilityNode["name"].as<std::string>();
                            ability.type = abilityNode["type"].as<std::string>();
                            
                            if (abilityNode["autostart"]) {
                                ability.autostart = abilityNode["autostart"].as<bool>();
                            } else {
                                ability.autostart = false;
                            }
                            
                            if (abilityNode["config"]) {
                                ability.config = abilityNode["config"];
                            }
                            
                            library.abilities.push_back(ability);
                        }
                    }
                    
                    config.libraries.push_back(library);
                }
            }
        } catch (const YAML::Exception& e) {
            std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
        }
        
        return config;
    }
};

} // namespace ability
} // namespace limxsdk

#endif // YAML_CONFIG_PARSER_H

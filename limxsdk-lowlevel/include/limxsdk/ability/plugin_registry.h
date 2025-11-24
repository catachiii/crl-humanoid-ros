/**
 * @file plugin_registry.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef PLUGIN_REGISTRY_H
#define PLUGIN_REGISTRY_H
#include <mutex>
#include <string>
#include <unordered_map>
#include <memory>
#include <functional>
#include <iostream>
#include "base_ability.h"
#include "limxsdk/macros.h"

namespace limxsdk {
namespace ability {
namespace PluginRegistry {

std::map<std::string, std::function<void*()>>& getPluginToFactoryMap();

std::recursive_mutex& getPluginToFactoryMapMutex();

// Register a plugin
template<typename Derived, typename Base>
void registerPlugin(const std::string& derivedClassName, const std::string& baseClassName) {
    std::unique_lock<std::recursive_mutex> lock(getPluginToFactoryMapMutex());
    PluginRegistry::getPluginToFactoryMap()[derivedClassName] = []() { return static_cast<Base*>(new Derived()); };
}

// Create an instance of a plugin
template<typename Base>
std::unique_ptr<Base> create(const std::string& className) {
    std::unique_lock<std::recursive_mutex> lock(getPluginToFactoryMapMutex());
    auto it = PluginRegistry::getPluginToFactoryMap().find(className);
    if (it != PluginRegistry::getPluginToFactoryMap().end()) {
        return std::unique_ptr<Base>(static_cast<Base*>(it->second()));
    }
    std::cerr << "Plugin not found: " << className << std::endl;
    return nullptr;
}
} // namespace PluginRegistry
} // namespace ability
} // namespace limxsdk

// Macro definitions
#define LIMX_REGISTER_PLUGIN_INTERNAL_WITH_MESSAGE(Derived, Base, UniqueID, Message) \
  namespace \
  { \
  struct ProxyExec ## UniqueID \
  { \
    typedef  Derived _derived; \
    typedef  Base _base; \
    ProxyExec ## UniqueID() \
    { \
      if (!std::string(Message).empty()) { \
        std::cout << "[INFO] " << Message << std::endl; } \
      limxsdk::ability::PluginRegistry::registerPlugin<_derived, _base>(#Derived, #Base); \
    } \
  }; \
  static ProxyExec ## UniqueID g_register_plugin_ ## UniqueID; \
  }  // namespace


#define LIMX_REGISTER_PLUGIN_INTERNAL_HOP1_WITH_MESSAGE(Derived, Base, UniqueID, Message) \
  LIMX_REGISTER_PLUGIN_INTERNAL_WITH_MESSAGE(Derived, Base, UniqueID, Message)

// Simplified ABILITY registration macro, automatically using limxsdk::ability::BaseAbility as base class
#define LIMX_REGISTER_ABILITY(Derived) \
  LIMX_REGISTER_PLUGIN_INTERNAL_HOP1_WITH_MESSAGE(Derived, limxsdk::ability::BaseAbility, __COUNTER__, "")

// ABILITY registration macro with message
#define LIMX_REGISTER_ABILITY_WITH_MESSAGE(Derived, Message) \
  LIMX_REGISTER_PLUGIN_INTERNAL_HOP1_WITH_MESSAGE(Derived, limxsdk::ability::BaseAbility, __COUNTER__, Message)

#endif // PLUGIN_REGISTRY_H

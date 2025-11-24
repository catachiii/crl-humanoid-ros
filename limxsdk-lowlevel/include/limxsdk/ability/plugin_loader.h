/**
 * @file plugin_loader.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef PLUGIN_LOADER_H
#define PLUGIN_LOADER_H

#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <mutex>
#include <fstream>
#include "limxsdk/macros.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#include <unistd.h>
#endif

namespace limxsdk {
namespace ability {

class LIMX_SDK_API PluginLoader {
public:
    PluginLoader(const std::string& path) : path_(path), handle_(nullptr) {}
    ~PluginLoader() { unload(); }
    
    bool load() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (handle_) {
            std::cerr << "Plugin already loaded: " << path_ << std::endl;
            return false;
        }
        
#ifdef _WIN32
        // Windows platform: Use LoadLibrary to load DLL
        handle_ = LoadLibraryA(path_.c_str());
        if (!handle_) {
            std::cerr << "Failed to load plugin: " << path_ 
                      << " - Error: " << GetLastError() << std::endl;
            return false;
        }
#else
        // POSIX platform: Use dlopen to load SO
        // Clear previous errors
        dlerror();
        handle_ = dlopen(path_.c_str(), RTLD_LAZY | RTLD_GLOBAL);
        const char* error = dlerror();
        if (error) {
            std::cerr << "Failed to load plugin: " << path_ 
                      << " - " << error << std::endl;
            handle_ = nullptr;
            return false;
        }
#endif
        
        std::cout << "Successfully loaded plugin: " << path_ << std::endl;
        return true;
    }
    
    void unload() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (handle_) {
#ifdef _WIN32
            // Windows platform: Use FreeLibrary to unload DLL
            if (!FreeLibrary(static_cast<HMODULE>(handle_))) {
                std::cerr << "Failed to unload plugin: " << path_ 
                          << " - Error: " << GetLastError() << std::endl;
            }
#else
            // POSIX platform: Use dlclose to unload SO
            if (dlclose(handle_) != 0) {
                std::cerr << "Failed to unload plugin: " << path_ 
                          << " - " << dlerror() << std::endl;
            }
#endif
            handle_ = nullptr;
            std::cout << "Unloaded plugin: " << path_ << std::endl;
        }
    }
    
    bool isLoaded() const {
        return handle_ != nullptr;
    }
    
    std::string path() const {
        return path_;
    }
    
private:
    std::string path_;
    void* handle_;
    mutable std::mutex mutex_;
};

class PluginManager {
public:
    static PluginManager& getInstance() {
        static PluginManager instance;
        return instance;
    }

    bool fileExists(const std::string& filename) {
        std::ifstream infile(filename);
        return infile.good();
    }
    
    bool loadPlugin(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Resolve plugin path
        std::string resolvedPath;
        
        // Check if path is absolute (Unix-style or Windows-style)
        if (path[0] == '/' || (path.length() >= 2 && path[1] == ':')) {
            resolvedPath = path;  // Use absolute path directly
        } else {
            // Get current executable path
            std::string exePath;
            
#ifdef _WIN32
            // Windows: Use GetModuleFileName to get executable path
            char buffer[MAX_PATH];
            GetModuleFileNameA(NULL, buffer, MAX_PATH);
            exePath = buffer;
#else
            // Linux/Unix: Use /proc/self/exe to get executable path
            char buffer[1024];
            ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
            if (len != -1) {
                buffer[len] = '\0';
                exePath = buffer;
            }
#endif
            
            if (exePath.empty()) {
                std::cout << "Failed to retrieve executable path" << std::endl;
                return false;
            }
            
            // Extract directory from executable path
            size_t lastSlash = exePath.find_last_of("/\\");
            std::string exeDir = (lastSlash != std::string::npos) ? 
                                exePath.substr(0, lastSlash) : "";

            // Get parent directory of the executable
            size_t parentSlash = exeDir.find_last_of("/\\");
            std::string parentDir = (parentSlash != std::string::npos) ? 
                                  exeDir.substr(0, parentSlash) : "";
            
            // 1. Try plugin in executable directory
            resolvedPath = exeDir + "/" + path;
            
            // Normalize path separators for Windows
            #ifdef _WIN32
            for (char& c : resolvedPath) if (c == '/') c = '\\';
            #endif
            
            // Check if plugin exists in executable directory
            if (!fileExists(resolvedPath)) {
                // 2. Try plugin in "lib" subdirectory
                resolvedPath = exeDir + "/lib/" + path;
                
                // Normalize path separators for Windows
                #ifdef _WIN32
                for (char& c : resolvedPath) if (c == '/') c = '\\';
                #endif
                
                if (!fileExists(resolvedPath)) {
                    // 3. Try "lib" subdirectory in parent directory
                    resolvedPath = parentDir + "/lib/" + path;

                    // Normalize path separators for Windows
                    #ifdef _WIN32
                    for (char& c : resolvedPath) if (c == '/') c = '\\';
                    #endif

                    if (!fileExists(resolvedPath)) {
                      // 4. Try "lib" limxsdk::ability::path::lib()
                      resolvedPath = limxsdk::ability::path::lib() + "/" + path;

                      // Normalize path separators for Windows
                      #ifdef _WIN32
                      for (char& c : resolvedPath) if (c == '/') c = '\\';
                      #endif

                      if (!fileExists(resolvedPath)) {
                        std::cout << "Plugin not found: " << path << std::endl;
                        return false;
                      }
                    }
                }
            }
        }
        
        // Verify file existence
        if (!fileExists(resolvedPath)) {
            std::cout << "Plugin not found: " << resolvedPath << std::endl;
            return false;
        }
        
        // Check if plugin is already loaded
        for (const auto& loader : loaders_) {
            if (loader->isLoaded() && loader->path() == resolvedPath) {
                return true;
            }
        }
    
        auto loader = std::unique_ptr<PluginLoader>(new PluginLoader(resolvedPath));
        if (loader->load()) {
            loaders_.push_back(std::move(loader));
            return true;
        }
        
        return false;
    }
    
private:
    PluginManager() = default;
    ~PluginManager() = default;
    PluginManager(const PluginManager&) = delete;
    PluginManager& operator=(const PluginManager&) = delete;
    
    std::vector<std::unique_ptr<PluginLoader>> loaders_;
    mutable std::mutex mutex_;
};

} // namespace ability
} // namespace limxsdk

#endif // PLUGIN_LOADER_H

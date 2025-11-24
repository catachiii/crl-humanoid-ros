/**
 * @file ability_manager.h
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef ABILITY_MANAGER_H
#define ABILITY_MANAGER_H

#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <functional>
#include <atomic>
#include <iostream>
#include <sstream>
#include <cstring>
#include <algorithm> 

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #define INVALID_SOCKET -1
    #define SOCKET int
#endif

#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"
#include "limxsdk/ability/robot_data.h"
#include "limxsdk/ability/base_ability.h"
#include "limxsdk/ability/plugin_registry.h"
#include "limxsdk/ability/plugin_loader.h"
#include "limxsdk/ability/yaml_config_parser.h"

namespace limxsdk {
namespace ability {

class AbilityManager;

class LIMX_SDK_API RemoteCliServer {
public:
    using CommandHandler = std::function<std::string(const std::vector<std::string>&)>;
    
    RemoteCliServer(int port, AbilityManager* abilityManager);
    ~RemoteCliServer() {
        stop();
    }
    
    bool start() {
        if (running_) {
            return true;
        }
        
        #ifdef _WIN32
        // Initialize Winsock
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            std::cerr << "Failed to initialize Winsock" << std::endl;
            return false;
        }
        #endif
        
        // Create socket
        serverSocket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket_ < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            #ifdef _WIN32
            WSACleanup();
            #endif
            return false;
        }
        
        // Set socket options
        int opt = 1;
        if (setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, 
                      reinterpret_cast<const char*>(&opt), sizeof(opt)) < 0) {
            std::cerr << "Failed to set socket options" << std::endl;
            closeSocket(serverSocket_);
            #ifdef _WIN32
            WSACleanup();
            #endif
            return false;
        }
        
        // Bind socket
        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons(port_);
        
        if (bind(serverSocket_, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) < 0) {
            std::cerr << "Failed to bind socket" << std::endl;
            closeSocket(serverSocket_);
            #ifdef _WIN32
            WSACleanup();
            #endif
            return false;
        }
        
        // Listen for connections
        if (listen(serverSocket_, 5) < 0) {
            std::cerr << "Failed to listen on socket" << std::endl;
            closeSocket(serverSocket_);
            #ifdef _WIN32
            WSACleanup();
            #endif
            return false;
        }
        
        // Start server thread
        running_ = true;
        serverThread_ = std::thread(&RemoteCliServer::serverThread, this);
        
        std::cout << "Remote CLI server started on port " << port_ << std::endl;
        return true;
    }

    void stop() {
        if (!running_) {
            return;
        }
        
        running_ = false;
        
        // Close all client connections
        {
            std::lock_guard<std::mutex> lock(clientsMutex_);
            for (SOCKET socket : clientSockets_) {
                closeSocket(socket);
            }
            clientSockets_.clear();
        }
        
        // Close server socket
        if (serverSocket_ != INVALID_SOCKET) {
            closeSocket(serverSocket_);
            serverSocket_ = INVALID_SOCKET;
        }
        
        // Wait for server thread to finish
        if (serverThread_.joinable()) {
            serverThread_.join();
        }
        
        #ifdef _WIN32
        WSACleanup();
        #endif
        
        std::cout << "Remote CLI server stopped" << std::endl;
    }
    
    void registerCommand(const std::string& command, CommandHandler handler, const std::string& helpText) {
        commandHandlers_[command] = std::make_pair(handler, helpText);
    }

    std::string getHelpText() const {
        std::stringstream ss;
        ss << "\nAvailable commands:\n";
        
        for (const auto& pair : commandHandlers_) {
            ss << "  " << pair.first << ": " << pair.second.second << "\n";
        }
        
        return ss.str();
    }
    
private:
    void serverThread() {
        while (running_) {
            // Accept client connection
            sockaddr_in clientAddr{};
            #ifdef _WIN32
            int clientAddrLen = sizeof(clientAddr);
            #else
            socklen_t clientAddrLen = sizeof(clientAddr);
            #endif
            SOCKET clientSocket = accept(serverSocket_, reinterpret_cast<sockaddr*>(&clientAddr), &clientAddrLen);
            
            if (clientSocket < 0) {
                if (running_) {
                    std::cerr << "Failed to accept client connection" << std::endl;
                }
                continue;
            }
            
            // Add client socket to list
            {
                std::lock_guard<std::mutex> lock(clientsMutex_);
                clientSockets_.push_back(clientSocket);
            }
            
            // Handle client connection
            #ifdef _WIN32
            char clientIP[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(clientAddr.sin_addr), clientIP, INET_ADDRSTRLEN);
            std::cout << "New client connected: " << clientIP << ":" << ntohs(clientAddr.sin_port) << std::endl;
            #else
            std::cout << "New client connected: " << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port) << std::endl;
            #endif
            
            handleClient(clientSocket);
            
            // Remove client socket from list
            {
                std::lock_guard<std::mutex> lock(clientsMutex_);
                auto it = std::find(clientSockets_.begin(), clientSockets_.end(), clientSocket);
                if (it != clientSockets_.end()) {
                    closeSocket(*it);
                    clientSockets_.erase(it);
                }
            }
        }
    }

    void handleClient(SOCKET clientSocket) {
        // Send welcome message
        std::string welcomeMsg = "LIMX SDK Remote CLI\nType 'help' for available commands.\n";
        send(clientSocket, welcomeMsg.c_str(), welcomeMsg.length(), 0);
        
        char buffer[1024];
        while (running_) {
            // Send prompt
            std::string prompt = "limx> ";
            send(clientSocket, prompt.c_str(), prompt.length(), 0);
            
            // Receive command
            memset(buffer, 0, sizeof(buffer));
            #ifdef _WIN32
            int bytesRead = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
            #else
            ssize_t bytesRead = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
            #endif
            
            if (bytesRead <= 0) {
                // Client disconnected
                break;
            }
            
            // Process command
            std::string commandLine(buffer);
            // Remove newline characters
            commandLine.erase(std::remove(commandLine.begin(), commandLine.end(), '\n'), commandLine.end());
            commandLine.erase(std::remove(commandLine.begin(), commandLine.end(), '\r'), commandLine.end());
            
            if (commandLine.empty()) {
                continue;
            }
            
            // Parse command
            std::vector<std::string> args = parseCommand(commandLine);
            if (args.empty()) {
                continue;
            }
            
            // Find command handler
            std::string response;
            auto it = commandHandlers_.find(args[0]);
            if (it != commandHandlers_.end()) {
                response = it->second.first(args);
            } else {
                response = "Unknown command: " + args[0] + "\n" + getHelpText();
            }
            
            // Send response
            response += "\n";
            send(clientSocket, response.c_str(), response.length(), 0);
            
            // Check if exiting
            if (args[0] == "exit") {
                break;
            }
        }
        
        std::cout << "Client disconnected" << std::endl;
    }

    std::vector<std::string> parseCommand(const std::string& commandLine) {
        std::vector<std::string> args;
        std::string currentArg;
        bool inQuotes = false;

        for (char c : commandLine) {
            if (c == '\"') {
                inQuotes = !inQuotes;
            } else if (c == ' ' && !inQuotes) {
                if (!currentArg.empty()) {
                    args.push_back(currentArg);
                    currentArg.clear();
                }
            } else {
                currentArg += c;
            }
        }
        if (!currentArg.empty()) {
            args.push_back(currentArg);
        }
        return args;
    }
    
    // Cross-platform helper function to close sockets
    void closeSocket(SOCKET sock) {
        #ifdef _WIN32
        closesocket(sock);
        #else
        close(sock);
        #endif
    }
    
    int port_;
    AbilityManager* abilityManager_;
    std::atomic<bool> running_;
    std::thread serverThread_;
    #ifdef _WIN32
    SOCKET serverSocket_;
    std::vector<SOCKET> clientSockets_;
    #else
    int serverSocket_;
    std::vector<int> clientSockets_;
    #endif
    mutable std::mutex clientsMutex_;
    
    std::unordered_map<std::string, std::pair<CommandHandler, std::string>> commandHandlers_;
};

class LIMX_SDK_API AbilityManager {
public:
    AbilityManager(const std::string& configPath) {
        // Initialize remote CLI server
        cliServer_ = std::unique_ptr<RemoteCliServer>(new RemoteCliServer(8888, this));

        SystemConfig config = YamlConfigParser::parse(configPath);

        // Apply system configuration
        std::cout << "Robot IP: " << config.robotIp << std::endl;
        std::cout << "Robot Type: " << config.robotType << std::endl;

        robotData_ = std::unique_ptr<RobotData>(new RobotData(config.robotIp, config.robotType));

        // Load libraries and abilities
        for (const auto& library : config.libraries) {
            // Load abilities from this library
            for (const auto& ability : library.abilities) {
                if (!loadAbility(library.library, ability.name, ability.type, ability.config)) {
                    std::cerr << "Failed to load ability: " << ability.name << " from " << library.library << std::endl;
                }
                
                // Auto-start if configured
                if (ability.autostart) {
                    startAbility(ability.name);
                }
            }
        }
    }

    ~AbilityManager() {
        // Stop all abilities
        for (auto& pair : abilities_) {
            pair.second->stop();
        }
        
        stopRemoteServer();
    }
    
    bool loadAbility(const std::string& soPath, const std::string& abilityName, const std::string& className, const YAML::Node& config = YAML::Node()) {
        // Load plugin library
        if (!PluginManager::getInstance().loadPlugin(soPath)) {
            std::cerr << "Failed to load plugin library: " << soPath << std::endl;
            robotData_->get_robot_instance()->publishDiagnostic("ability/" + abilityName, "load", -1, 2, "Failed to load plugin library: " + soPath);
            return false;
        }
        
        // Create ability instance
        auto ability = PluginRegistry::create<BaseAbility>(className);
        if (!ability) {
            std::cerr << "Failed to create ability instance: " << className << std::endl;
            robotData_->get_robot_instance()->publishDiagnostic("ability/" + abilityName, "load", -1, 2, "Failed to create ability instance: " + className);
            return false;
        }
        
        // Initialize ability
        if (!ability->on_init(config)) {
            std::cerr << "Failed to initialize ability: " << abilityName << std::endl;
            robotData_->get_robot_instance()->publishDiagnostic("ability/" + abilityName, "load", -1, 2, "Failed to initialize ability: " + abilityName);
            return false;
        }
        
        // Store ability
        ability->name_ = abilityName;
        ability->type_ = className;
        ability->robot_ = robotData_.get();

        abilities_[abilityName] = std::move(ability);
        robotData_->get_robot_instance()->publishDiagnostic("ability/" + abilityName, "load", 0, 0, "Successfully loaded ability: " + abilityName + " (" + className + ")");
        std::cout << "Successfully loaded ability: " << abilityName << " (" << className << ")" << std::endl;
        return true;
    }

    bool startAbility(const std::string& abilityName) {
        auto it = abilities_.find(abilityName);
        if (it == abilities_.end()) {
            std::cerr << "Ability not found: " << abilityName << std::endl;
            robotData_->get_robot_instance()->publishDiagnostic("ability/" + abilityName, "start", -1, 2, "Ability not found: " + abilityName);
            return false;
        }
        
        if (it->second->isRunning()) {
            std::cout << "Ability already running: " << abilityName << std::endl;
            return true;
        }
        
        it->second->start();
        return true;
    }

    bool stopAbility(const std::string& abilityName) {
        auto it = abilities_.find(abilityName);
        if (it == abilities_.end()) {
            std::cerr << "Ability not found: " << abilityName << std::endl;
            robotData_->get_robot_instance()->publishDiagnostic("ability/" + abilityName, "stop", -1, 2, "Ability not found: " + abilityName);
            return false;
        }
        
        if (!it->second->isRunning()) {
            std::cout << "Ability not running: " << abilityName << std::endl;
            return true;
        }
        
        it->second->stop();
        return true;
    }

    bool isAbilityRunning(const std::string& abilityName) const {
        auto it = abilities_.find(abilityName);
        if (it == abilities_.end()) {
            return false;
        }
        
        return it->second->isRunning();
    }

    std::string listAbilities() const {
        std::string abilities;
        for (const auto& pair : abilities_) {
            abilities += "\n  * ";
            abilities += pair.first;
            abilities += " [state: ";
            abilities += (pair.second->isRunning() ? "running" : "stopped");
            abilities += ", type: " +  pair.second->getType() + "]";
        }
        return abilities;
    }
    
    // Remote CLI server methods
    bool startRemoteServer() {
        return cliServer_->start();
    }

    void stopRemoteServer() {
        cliServer_->stop();
    }
    
    std::unordered_map<std::string, std::unique_ptr<BaseAbility>> abilities_;
    std::unique_ptr<RemoteCliServer> cliServer_;
    std::unique_ptr<RobotData> robotData_;
};


RemoteCliServer::RemoteCliServer(int port, AbilityManager* abilityManager)
    : port_(port), abilityManager_(abilityManager), running_(false), serverSocket_(INVALID_SOCKET) {
    // Register built-in commands
    registerCommand("help", [this](const std::vector<std::string>& args) {
        return getHelpText();
    }, "Show this help message");
    
    registerCommand("list", [this](const std::vector<std::string>& args) {
        std::stringstream ss;
        ss << "Available abilities:";
        ss << abilityManager_->listAbilities();
        return ss.str();
    }, "List all available abilities");
    
    registerCommand("start", [this](const std::vector<std::string>& args) {
        if (args.size() < 2) {
            return "Usage: start <ability_name>" + getHelpText();
        }
        
        if (abilityManager_->startAbility(args[1])) {
            return "Successfully started ability: " + args[1];
        } else {
            return "Failed to start ability: " + args[1];
        }
    }, "Start an ability");
    
    registerCommand("stop", [this](const std::vector<std::string>& args) {
        if (args.size() < 2) {
            return "Usage: stop <ability_name>" + getHelpText();
        }
        
        if (abilityManager_->stopAbility(args[1])) {
            return "Successfully stopped ability: " + args[1];
        } else {
            return "Failed to stop ability: " + args[1];
        }
    }, "Stop an ability");
    
    registerCommand("switch", [this](const std::vector<std::string>& args) {
        if (args.size() < 2) {
            return "Usage: switch \"<stop ability1> <stop ability2> ...\" \"<start ability3> <start ability4> ...\"\n" + getHelpText();
        }
        
        std::vector<std::string> stopList;
        std::vector<std::string> startList;
        bool inStopSection = false;
        bool inStartSection = false;
        
        for (size_t i = 1; i < args.size(); ++i) {
            // Split by spaces
            std::istringstream iss(args[i]);
            std::string ability;
            while (iss >> ability) {
                if (i == 1) {
                    stopList.push_back(ability);
                } else if (i == 2) {
                    startList.push_back(ability);
                }
            }
        }
        
        std::stringstream result;
        
        // First stop specified abilities
        for (const auto& ability : stopList) {
            if (abilityManager_->stopAbility(ability)) {
                result << "Stopped: " << ability << std::endl;
            } else {
                result << "Failed to stop: " << ability << std::endl;
            }
        }
        
        // Then start specified abilities
        for (const auto& ability : startList) {
            if (abilityManager_->startAbility(ability)) {
                result << "Started: " << ability << std::endl;
            } else {
                result << "Failed to start: " << ability << std::endl;
            }
        }
        
        return result.str();
    }, "Switch between abilities: switch \"<stop abilities>\" \"<start abilities>\"");
    
    registerCommand("exit", [this](const std::vector<std::string>& args) {
        return "Goodbye!";
    }, "Exit the CLI");
}

} // namespace ability
} // namespace limxsdk

#endif // ABILITY_MANAGER_H

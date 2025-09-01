#pragma once

// ROS2 includes
#include "rclcpp/rclcpp.hpp"

// MuJoCo includes
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// Standard libraries
#include <array>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>

#include "crl_humanoid_monitor/MuJoCoMonitorNode.h"

namespace crl::humanoid::monitor {

    template <typename States, typename Machines, typename TransitionsCont, std::size_t N, typename MonitorNodeType>
    class MuJoCoMonitorApp {
    public:
        MuJoCoMonitorApp(const std::string& to_monitor, const TransitionsCont& trans_cont, const std::array<Machines, N>& monitoring)
            : to_monitor_(to_monitor), trans_cont_(trans_cont), monitoring_(monitoring) {

            // Initialize GLFW
            if (!glfwInit()) {
                throw std::runtime_error("Failed to initialize GLFW");
            }

            // Create window
            window_ = glfwCreateWindow(1400, 1000, "MuJoCo Monitor", nullptr, nullptr);
            if (!window_) {
                glfwTerminate();
                throw std::runtime_error("Failed to create GLFW window");
            }

            glfwMakeContextCurrent(window_);
            glfwSwapInterval(1); // Enable vsync to match display refresh rate (like official MuJoCo examples)

            // Set callbacks
            glfwSetWindowUserPointer(window_, this);
            glfwSetKeyCallback(window_, keyCallback);
            glfwSetMouseButtonCallback(window_, mouseButtonCallback);
            glfwSetCursorPosCallback(window_, mouseMoveCallback);
            glfwSetScrollCallback(window_, scrollCallback);

            // Initialize ROS2
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }

            // Create executor
            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

            // Initialize state
            showInfo_ = true;
            processIsRunning_ = true;

            // Initialize FSM overlay
            updateAvailableStates();

            // Debug: Print enum values
            RCLCPP_INFO(rclcpp::get_logger("MuJoCoMonitorApp"), "State enum values - ESTOP: %d, STAND: %d, WALK: %d",
                      static_cast<int>(States::ESTOP), static_cast<int>(States::STAND), static_cast<int>(States::WALK));
            RCLCPP_INFO(rclcpp::get_logger("MuJoCoMonitorApp"), "MuJoCo GUI is ready. After the simulation node prints \"Start robot.\", you may connect to the robot by clicking \"C\"");

            // Don't auto-connect to avoid startup crashes
            // User can press 'C' to connect manually
        }

        ~MuJoCoMonitorApp() {
            // Cleanup GLFW
            if (window_) {
                glfwDestroyWindow(window_);
            }
            glfwTerminate();
        }

        void run() {
            while (!glfwWindowShouldClose(window_)) {
                // Poll events with exception handling
                try {
                    glfwPollEvents();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("monitor_app"), "GLFW event polling error: %s", e.what());
                    // Continue running but log the error
                } catch (...) {
                    RCLCPP_ERROR(rclcpp::get_logger("monitor_app"), "Unknown error in GLFW event polling");
                    // Continue running but log the error
                }

                // Process ROS2 callbacks efficiently (following MuJoCo's non-blocking pattern)
                if (runnerNode_ && processIsRunning_) {
                    try {
                        // Use non-blocking spin_some with zero timeout for better performance
                        executor_->spin_some(std::chrono::milliseconds(0));
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("monitor_app"), "ROS2 callback processing error: %s", e.what());
                        // Continue running but log the error
                    } catch (...) {
                        RCLCPP_ERROR(rclcpp::get_logger("monitor_app"), "Unknown error in ROS2 callback processing");
                        // Continue running but log the error
                    }
                }

                // Update monitor node
                if (runnerNode_) {
                    try {
                        runnerNode_->update();
                        // Update current FSM state
                        updateCurrentState();
                        // Update camera follow if enabled
                        updateCameraFollow();
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("monitor_app"), "Monitor update error: %s", e.what());
                        // Continue running but log the error
                    } catch (...) {
                        RCLCPP_ERROR(rclcpp::get_logger("monitor_app"), "Unknown error in monitor update");
                        // Continue running but log the error
                    }
                }

                // Simple clear screen
                int width, height;
                glfwGetFramebufferSize(window_, &width, &height);
                glViewport(0, 0, width, height);

                // Set background color based on connection status
                if (runnerNode_) {
                    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black background when connected
                } else {
                    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black background when disconnected
                }
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                // Enable MuJoCo rendering now that crashes are fixed
                if (runnerNode_) {
                    try {
                        runnerNode_->render();
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("monitor_app"), "Rendering error: %s", e.what());
                        // If rendering fails, just continue without crashing
                    } catch (...) {
                        RCLCPP_ERROR(rclcpp::get_logger("monitor_app"), "Unknown rendering error");
                        // If rendering fails, just continue without crashing
                    }
                }

                // Render simple status text overlay
                try {
                    renderTextOverlay(width, height);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Text overlay error: %s", e.what());
                    // Continue without text overlay
                } catch (...) {
                    RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Unknown text overlay error");
                    // Continue without text overlay
                }

                // Calculate FPS
                updateFPS();

                // Swap buffers (blocking call due to v-sync, following official MuJoCo pattern)
                glfwSwapBuffers(window_);

                // Remove artificial delay - let VSync control the frame rate naturally
            }
        }

    private:
        static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
            auto* app = static_cast<MuJoCoMonitorApp*>(glfwGetWindowUserPointer(window));
            if (app) {
                app->handleKey(key, scancode, action, mods);
            } else {
                // This should not happen
                printf("ERROR: App pointer is null in keyCallback!\n");
            }
        }

        static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
            auto* app = static_cast<MuJoCoMonitorApp*>(glfwGetWindowUserPointer(window));
            app->handleMouseButton(button, action, mods);
        }

        static void mouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
            auto* app = static_cast<MuJoCoMonitorApp*>(glfwGetWindowUserPointer(window));
            app->handleMouseMove(xpos, ypos);
        }

        static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
            auto* app = static_cast<MuJoCoMonitorApp*>(glfwGetWindowUserPointer(window));
            app->handleScroll(xoffset, yoffset);
        }

        void handleKey(int key, [[maybe_unused]] int scancode, int action, [[maybe_unused]] int mods) {

            if (action == GLFW_PRESS) {
                switch (key) {
                    case GLFW_KEY_C:
                        RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "C key pressed - Connect/Disconnect");
                        // Connect/Disconnect
                        if (!runnerNode_) {
                            connectToRobot();
                        } else {
                            disconnectFromRobot();
                        }
                        break;
                    case GLFW_KEY_SPACE:
                        RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "Space key pressed - Play/Pause");
                        // Play/Pause
                        processIsRunning_ = !processIsRunning_;
                        break;
                    case GLFW_KEY_R:
                        RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "R key pressed - Restart");
                        if (runnerNode_) {
                            runnerNode_->restart();
                        }
                        break;
                    case GLFW_KEY_H:
                        RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "H key pressed - Toggle info");
                        // Toggle info display
                        showInfo_ = !showInfo_;
                        break;
                    case GLFW_KEY_F:
                        RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "F key pressed - Toggle FSM overlay");
                        // Toggle FSM overlay
                        showFsmOverlay_ = !showFsmOverlay_;
                        break;
                    case GLFW_KEY_E:
                        RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "E key pressed - Toggle Elastic Band");
                        // Toggle elastic band support
                        if (runnerNode_) {
                            runnerNode_->toggleElasticBand();
                        }
                        break;
                    case GLFW_KEY_T:
                        RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "T key pressed - Toggle camera follow");
                        // Toggle camera follow mode
                        toggleCameraFollow();
                        break;
                    case GLFW_KEY_UP:
                        // Navigate FSM states up
                        if (showFsmOverlay_ && runnerNode_ && !availableStates_.empty()) {
                            selectedTransition_ = (selectedTransition_ > 0) ? selectedTransition_ - 1 : availableStates_.size() - 1;
                            RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "Selected state: %d (%s)",
                                      static_cast<int>(availableStates_[selectedTransition_]),
                                      stateToString(availableStates_[selectedTransition_]));
                        }
                        break;
                    case GLFW_KEY_DOWN:
                        // Navigate FSM states down
                        if (showFsmOverlay_ && runnerNode_ && !availableStates_.empty()) {
                            selectedTransition_ = (selectedTransition_ + 1) % availableStates_.size();
                            RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "Selected state: %d (%s)",
                                      static_cast<int>(availableStates_[selectedTransition_]),
                                      stateToString(availableStates_[selectedTransition_]));
                        }
                        break;
                    case GLFW_KEY_ENTER:
                        // Execute selected transition
                        if (showFsmOverlay_ && runnerNode_ && !availableStates_.empty()) {
                            executeSelectedTransition();
                        }
                        break;
                    case GLFW_KEY_ESCAPE:
                        RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "ESC key pressed - Exit");
                        glfwSetWindowShouldClose(window_, GLFW_TRUE);
                        break;
                }
            }
        }

        void handleMouseButton(int button, int action, [[maybe_unused]] int mods) {
            // Track mouse button state for camera control
            if (button >= 0 && button < 3) {
                if (action == GLFW_PRESS) {
                    mousePressed_[button] = true;
                    // Store initial mouse position for delta calculations
                    glfwGetCursorPos(window_, &lastMouseX_, &lastMouseY_);
                } else if (action == GLFW_RELEASE) {
                    mousePressed_[button] = false;
                }
            }
        }

        void handleMouseMove(double xpos, double ypos) {
            if (!runnerNode_) return;

            // Calculate mouse movement delta
            double dx = xpos - lastMouseX_;
            double dy = ypos - lastMouseY_;

            // Get window size for relative movement scaling
            int width, height;
            glfwGetFramebufferSize(window_, &width, &height);

            // Scale movement based on window size
            dx /= width;
            dy /= height;

            try {
                // Access camera through the monitor node
                auto& camera = runnerNode_->getCamera();

                // Left mouse button: rotate camera (orbit)
                if (mousePressed_[GLFW_MOUSE_BUTTON_LEFT]) {
                    // Horizontal movement rotates around vertical axis (azimuth)
                    camera.azimuth -= dx * 360.0; // 360 degrees per full window width (inverted for natural feel)

                    // Vertical movement changes elevation
                    camera.elevation -= dy * 180.0; // 180 degrees per full window height

                    // Clamp elevation to prevent flipping
                    if (camera.elevation > 89.0) camera.elevation = 89.0;
                    if (camera.elevation < -89.0) camera.elevation = -89.0;

                    // Normalize azimuth to [0, 360)
                    while (camera.azimuth < 0) camera.azimuth += 360.0;
                    while (camera.azimuth >= 360.0) camera.azimuth -= 360.0;
                }

                // Middle mouse button: pan (translate lookat point)
                else if (mousePressed_[GLFW_MOUSE_BUTTON_MIDDLE]) {
                    // Calculate camera's right and up vectors for panning
                    double azimuthRad = camera.azimuth * M_PI / 180.0;
                    double elevationRad = camera.elevation * M_PI / 180.0;

                    // Camera right vector (for horizontal panning)
                    double rightX = -sin(azimuthRad);
                    double rightY = cos(azimuthRad);
                    double rightZ = 0.0;

                    // Camera up vector (for vertical panning)
                    double upX = -cos(azimuthRad) * sin(elevationRad);
                    double upY = -sin(azimuthRad) * sin(elevationRad);
                    double upZ = cos(elevationRad);

                    // Pan speed proportional to distance from target
                    double panSpeed = camera.distance * 0.5;

                    // Apply panning to lookat point
                    camera.lookat[0] += (rightX * dx + upX * dy) * panSpeed;
                    camera.lookat[1] += (rightY * dx + upY * dy) * panSpeed;
                    camera.lookat[2] += (rightZ * dx + upZ * dy) * panSpeed;
                }

            } catch (const std::exception& e) {
                RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Camera control error: %s", e.what());
            }

            // Update last mouse position
            lastMouseX_ = xpos;
            lastMouseY_ = ypos;
        }

        void handleScroll([[maybe_unused]] double xoffset, double yoffset) {
            if (!runnerNode_) return;

            try {
                // Access camera through the monitor node
                auto& camera = runnerNode_->getCamera();

                // Scroll controls zoom (distance from target)
                double zoomFactor = 1.0 - yoffset * 0.1; // 10% zoom per scroll step
                camera.distance *= zoomFactor;

                // Clamp distance to reasonable bounds
                if (camera.distance < 0.1) camera.distance = 0.1;
                if (camera.distance > 50.0) camera.distance = 50.0;

            } catch (const std::exception& e) {
                RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Camera zoom error: %s", e.what());
            }
        }

        void updateFPS() {
            frameCount_++;
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - fpsUpdateTime_).count();

            // Update FPS every 500ms
            if (elapsed >= 500) {
                currentFps_ = (frameCount_ * 1000.0) / elapsed;
                frameCount_ = 0;
                fpsUpdateTime_ = currentTime;
            }
        }

        void toggleCameraFollow() {
            cameraFollowEnabled_ = !cameraFollowEnabled_;
        }

        void updateCameraFollow() {
            if (!cameraFollowEnabled_ || !runnerNode_) return;

            try {
                // Get current robot position
                auto robotPosition = runnerNode_->getRobotBasePosition();
                auto& camera = runnerNode_->getCamera();

                // Update camera lookat point to follow robot
                // Keep the viewing angles (azimuth, elevation, distance) unchanged
                camera.lookat[0] = robotPosition[0];
                camera.lookat[1] = robotPosition[1];
                camera.lookat[2] = robotPosition[2];

            } catch (const std::exception& e) {
                // Use a static counter for throttling instead of rclcpp clock
                static auto last_warning = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warning).count() > 1000) {
                    RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Failed to update camera follow: %s", e.what());
                    last_warning = now;
                }
            }
        }

        void connectToRobot() {
            if (!runnerNode_) {
                try {
                    runnerNode_ = std::make_shared<MonitorNodeType>(to_monitor_, trans_cont_, monitoring_);
                    if (runnerNode_) {
                        if (runnerNode_->initializeNode()) {
                            // Initialize OpenGL rendering context now that OpenGL context is current
                            try {
                                runnerNode_->initializeRenderingContext();
                                executor_->add_node(runnerNode_);

                                // Set to PLAY mode when connecting/reconnecting
                                processIsRunning_ = true;
                                RCLCPP_INFO(rclcpp::get_logger("MuJoCoMonitorApp"), "Connected to robot: %s", to_monitor_.c_str());
                            } catch (const std::exception& e) {
                                runnerNode_.reset();
                            }
                        } else {
                            runnerNode_.reset();
                        }
                    } else {
                    }
                } catch (const std::exception& e) {
                    runnerNode_.reset();
                }
            }
        }

        void disconnectFromRobot() {
            if (runnerNode_) {
                executor_->remove_node(runnerNode_);
                runnerNode_.reset();
            }
        }

        void renderTextOverlay([[maybe_unused]] int width, int height) {
            // Draw instructions and status overlay using MuJoCo text rendering

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(0, width, height, 0, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            glDisable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            // Draw semi-transparent background for instructions
            glBegin(GL_QUADS);
            glColor4f(0.0f, 0.0f, 0.0f, 0.7f); // Semi-transparent black
            glVertex2f(10, 10);
            glVertex2f(380, 10);
            glVertex2f(380, 400); // Increased height to accommodate elastic band and camera follow info
            glVertex2f(10, 400);
            glEnd();

            // Draw connection status indicator
            glBegin(GL_QUADS);
            if (runnerNode_) {
                glColor4f(0.0f, 1.0f, 0.0f, 0.9f); // Bright green when connected
            } else {
                glColor4f(1.0f, 0.0f, 0.0f, 0.9f); // Bright red when disconnected
            }
            glVertex2f(20, 20);
            glVertex2f(40, 20);
            glVertex2f(40, 40);
            glVertex2f(20, 40);
            glEnd();

            // FSM overlay background if enabled and connected
            if (showFsmOverlay_ && runnerNode_) {
                glBegin(GL_QUADS);
                glColor4f(0.0f, 0.0f, 0.0f, 0.8f); // Semi-transparent black
                glVertex2f(width - 400, 10);
                glVertex2f(width - 10, 10);
                glVertex2f(width - 10, 300);
                glVertex2f(width - 400, 300);
                glEnd();
            }

            // Use MuJoCo text rendering if we have a connected node
            if (runnerNode_) {
                try {
                    auto& context = runnerNode_->getContext();

                    // Status text
                    mjr_text(mjFONT_NORMAL, "CONNECTED", &context,
                             20.0f / width, 1.0f - 35.0f / height,
                             0.0f, 1.0f, 0.0f);

                    // Instructions
                    mjr_text(mjFONT_NORMAL, "CONTROLS:", &context,
                             20.0f / width, 1.0f - 70.0f / height,
                             1.0f, 1.0f, 1.0f);

                    mjr_text(mjFONT_NORMAL, "C - Connect/Disconnect", &context,
                             20.0f / width, 1.0f - 95.0f / height,
                             0.8f, 0.8f, 0.8f);

                    mjr_text(mjFONT_NORMAL, "SPACE - Play/Pause", &context,
                             20.0f / width, 1.0f - 120.0f / height,
                             0.8f, 0.8f, 0.8f);

                    mjr_text(mjFONT_NORMAL, "R - Restart Robot", &context,
                             20.0f / width, 1.0f - 145.0f / height,
                             0.8f, 0.8f, 0.8f);

                    mjr_text(mjFONT_NORMAL, "F - Toggle FSM Overlay", &context,
                             20.0f / width, 1.0f - 170.0f / height,
                             0.8f, 0.8f, 0.8f);

                    // Show elastic band status and control
                    bool elasticBandEnabled = runnerNode_ ? runnerNode_->isElasticBandEnabled() : false;
                    std::string elasticText = "E - Elastic Band: " + std::string(elasticBandEnabled ? "ON" : "OFF");
                    mjr_text(mjFONT_NORMAL, elasticText.c_str(), &context,
                             20.0f / width, 1.0f - 195.0f / height,
                             elasticBandEnabled ? 0.0f : 0.8f, elasticBandEnabled ? 1.0f : 0.8f, 0.8f);

                    // Show camera follow status and control
                    std::string followText = "T - Camera Follow: " + std::string(cameraFollowEnabled_ ? "ON" : "OFF");
                    mjr_text(mjFONT_NORMAL, followText.c_str(), &context,
                             20.0f / width, 1.0f - 220.0f / height,
                             cameraFollowEnabled_ ? 0.0f : 0.8f, cameraFollowEnabled_ ? 1.0f : 0.8f, 0.8f);

                    mjr_text(mjFONT_NORMAL, "ESC - Exit", &context,
                             20.0f / width, 1.0f - 245.0f / height,
                             0.8f, 0.8f, 0.8f);

                    // Mouse controls section
                    mjr_text(mjFONT_NORMAL, "MOUSE CONTROLS:", &context,
                             20.0f / width, 1.0f - 280.0f / height,
                             1.0f, 1.0f, 1.0f);

                    mjr_text(mjFONT_NORMAL, "Left Mouse - Rotate Camera", &context,
                             20.0f / width, 1.0f - 305.0f / height,
                             0.8f, 0.8f, 0.8f);

                    mjr_text(mjFONT_NORMAL, "Middle Mouse - Pan Camera", &context,
                             20.0f / width, 1.0f - 330.0f / height,
                             0.8f, 0.8f, 0.8f);

                    mjr_text(mjFONT_NORMAL, "Scroll Wheel - Zoom", &context,
                             20.0f / width, 1.0f - 355.0f / height,
                             0.8f, 0.8f, 0.8f);

                    // FSM overlay
                    if (showFsmOverlay_ && runnerNode_) {
                        renderFsmOverlay(width, height, context);
                    }

                } catch (const std::exception& e) {
                    // If MuJoCo text rendering fails, fall back to simple indicators
                    RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Text rendering failed: %s", e.what());
                }
            } else {
                // When disconnected, show simple text indication
                // Draw some basic indicator text using OpenGL primitives
                // Status indicator shows red, which indicates disconnected state
            }

            // FPS display at bottom left (always visible when connected)
            if (runnerNode_) {
                try {
                    auto& context = runnerNode_->getContext();
                    std::string fpsText = "FPS: " + std::to_string(static_cast<int>(currentFps_ + 0.5));
                    mjr_text(mjFONT_NORMAL, fpsText.c_str(), &context,
                             20.0f / width, 0.0f / height,
                             1.0f, 1.0f, 0.0f);
                } catch (const std::exception& e) {
                    // Silently ignore FPS display errors
                }
            }

            glDisable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);
        }

        void updateAvailableStates() {
            availableStates_.clear();

            // Add all possible states for navigation - match simulator order
            availableStates_.push_back(States::ESTOP);
            availableStates_.push_back(States::STAND);
            availableStates_.push_back(States::WALK);

            // Reset selection if needed
            if (selectedTransition_ >= availableStates_.size()) {
                selectedTransition_ = 0;
            }
        }        void updateCurrentState() {
            if (runnerNode_) {
                try {
                    std::array<States, N> states;
                    runnerNode_->getCurrentStates(states);
                    if (!states.empty()) {
                        [[maybe_unused]] States oldState = currentState_;
                        currentState_ = states[0];

                        updateAvailableStates(); // Update available transitions
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Failed to get current state (last known: %d (%s)): %s",
                              static_cast<int>(currentState_), stateToString(currentState_), e.what());
                }
            }
        }

        void executeSelectedTransition() {
            if (selectedTransition_ < availableStates_.size()) {
                States targetState = availableStates_[selectedTransition_];

                if (targetState == currentState_) {
                    RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "Already in target state %d (%s)",
                              static_cast<int>(targetState), stateToString(targetState));
                    return;
                }

                // Check if transition is valid
                auto possible = trans_cont_.possible(currentState_, targetState);
                if (possible == crl::fsm::TransitionsPossible::YES) {
                    try {
                        RCLCPP_INFO(rclcpp::get_logger("monitor_app"), "Executing transition from %d (%s) to %d (%s)",
                                  static_cast<int>(currentState_), stateToString(currentState_),
                                  static_cast<int>(targetState), stateToString(targetState));
                        runnerNode_->executeTransition(targetState);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("monitor_app"), "Failed to execute transition: %s", e.what());
                    }
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Invalid transition from %d (%s) to %d (%s)",
                              static_cast<int>(currentState_), stateToString(currentState_),
                              static_cast<int>(targetState), stateToString(targetState));
                }
            }
        }

        const char* stateToString(States state) {
            // Debug: Log the state value
            RCLCPP_DEBUG(rclcpp::get_logger("monitor_app"), "Converting state with value: %d", static_cast<int>(state));

            switch (state) {
                case States::ESTOP: return "ESTOP";
                case States::STAND: return "STAND";
                case States::WALK: return "WALK";
                default:
                    RCLCPP_WARN(rclcpp::get_logger("monitor_app"), "Unknown state value: %d", static_cast<int>(state));
                    return "UNKNOWN";
            }
        }

        void renderFsmOverlay(int width, int height, mjrContext& context) {
            // Fixed pixel spacing from edges - 250 pixels from right edge for text start
            float textStartX = (width - 300.0f) / width;
            float startY = 1.0f - 70.0f / height;

            // Debug logging
            RCLCPP_DEBUG(rclcpp::get_logger("monitor_app"), "Rendering FSM overlay with current state: %d (%s)",
                        static_cast<int>(currentState_), stateToString(currentState_));

            // Title
            mjr_text(mjFONT_NORMAL, "FINITE STATE MACHINE", &context,
                     textStartX, startY,
                     1.0f, 1.0f, 1.0f);
            // Available transitions header
            mjr_text(mjFONT_NORMAL, "Available Transitions:", &context,
                     textStartX, 1.0f - 100.0f / height,
                     1.0f, 1.0f, 1.0f);

            // List all states with transition status - using fixed pixel spacing
            for (std::size_t i = 0; i < availableStates_.size(); ++i) {
                States state = availableStates_[i];
                float yPos = 1.0f - (130.0f + i * 25.0f) / height; // Fixed 25 pixel spacing between items

                // Check if transition is possible
                auto possible = trans_cont_.possible(currentState_, state);
                bool isValid = (possible == crl::fsm::TransitionsPossible::YES);
                bool isCurrent = (state == currentState_);
                bool isSelected = (i == selectedTransition_);

                // Color coding
                float r, g, b;
                if (isCurrent) {
                    r = 0.0f; g = 1.0f; b = 1.0f; // Cyan for current state
                } else if (isValid) {
                    r = 0.0f; g = 1.0f; b = 0.0f; // Green for valid transitions
                } else {
                    r = 1.0f; g = 0.0f; b = 0.0f; // Red for invalid transitions
                }

                // Highlight selection
                std::string prefix = isSelected ? "> " : "  ";
                std::string suffix = isCurrent ? " (CURRENT)" : (isValid ? " (AVAILABLE)" : " (BLOCKED)");

                std::string stateText = prefix + std::string(stateToString(state)) + " (" + std::to_string(static_cast<int>(state)) + ")" + suffix;                mjr_text(mjFONT_NORMAL, stateText.c_str(), &context,
                         textStartX, yPos,
                         r, g, b);
            }

            // Instructions with fixed pixel spacing
            mjr_text(mjFONT_NORMAL, "UP/DOWN: Navigate", &context,
                     textStartX, 1.0f - (130.0f + availableStates_.size() * 25.0f + 30.0f) / height,
                     0.8f, 0.8f, 0.8f);

            mjr_text(mjFONT_NORMAL, "ENTER: Execute", &context,
                     textStartX, 1.0f - (130.0f + availableStates_.size() * 25.0f + 50.0f) / height,
                     0.8f, 0.8f, 0.8f);
        }

    private:
        // Window
        GLFWwindow* window_ = nullptr;
        bool showInfo_ = true;
        bool processIsRunning_ = true;

        // Camera control
        bool cameraFollowEnabled_ = true;

        // ROS2
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
        std::shared_ptr<MonitorNodeType> runnerNode_ = nullptr;

        // Configuration
        const std::string to_monitor_;
        const TransitionsCont trans_cont_;
        const std::array<Machines, N> monitoring_;

        // FSM overlay state
        bool showFsmOverlay_ = true;
        std::size_t selectedTransition_ = 0;
        std::vector<States> availableStates_;
        States currentState_ = States::ESTOP;

        // Mouse control state
        bool mousePressed_[3] = {false, false, false}; // Left, right, middle mouse buttons
        double lastMouseX_ = 0.0;
        double lastMouseY_ = 0.0;

        // FPS tracking
        std::chrono::steady_clock::time_point lastFrameTime_ = std::chrono::steady_clock::now();
        double currentFps_ = 0.0;
        int frameCount_ = 0;
        std::chrono::steady_clock::time_point fpsUpdateTime_ = std::chrono::steady_clock::now();
    };

    /**
     * Helper function to create default MuJoCo monitor app.
     */
    template <typename States, typename Machines, typename TransitionsContGen, std::size_t N>
    auto make_mujoco_monitor_app(const std::string& to_monitor, const TransitionsContGen& trans_cont, const std::array<Machines, N>& monitoring) {
        using TransitionsContType = std::invoke_result_t<TransitionsContGen>;
        using MuJoCoMonitorNodeType = crl::humanoid::monitor::MuJoCoMonitorNode<States, Machines, TransitionsContType, N>;
        return MuJoCoMonitorApp<States, Machines, TransitionsContType, N, MuJoCoMonitorNodeType>(to_monitor, trans_cont(), monitoring);
    }

}  // namespace crl::humanoid::monitor

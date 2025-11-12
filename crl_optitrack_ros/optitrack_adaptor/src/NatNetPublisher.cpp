#include "optitrack_ros2_adaptor/NatNetPublisher.h"

#include <NatNetCAPI.h>

#include <cstdlib>
#include <chrono>

using namespace std::chrono_literals;

NatNetPublisher::NatNetPublisher(const std::string& name)
    : Node(name) {
    // ros2 config params
    declare_parameter<bool>("auto_find", false);
    declare_parameter<int>("connection_type", 0);
    declare_parameter<uint16_t>("server_command_port", 1510);
    declare_parameter<uint16_t>("server_data_port", 1511);
    declare_parameter<std::string>("server_address", "127.0.0.1");
    declare_parameter<std::string>("local_address", "127.0.0.1");
    declare_parameter<std::string>("multicast_address", "239.255.42.99");

    bool auto_find = get_parameter("auto_find").as_bool();
    if (auto_find) {
        sNatNetDiscoveredServer servers;
        int num_obtained = 1;
        NatNet_BroadcastServerDiscovery(&servers, &num_obtained, discover_time_ms);

        if (num_obtained == 0) {
            RCLCPP_ERROR(get_logger(), "No server found when automatically discovering.");
        } else if (num_obtained > 1) {
            RCLCPP_ERROR(get_logger(), "Too many servers found. Please manually specify");
        }

        if (servers.serverDescription.bConnectionInfoValid) {
            // normal server
            std::memcpy(server_addr_store, servers.serverAddress, kNatNetIpv4AddrStrLenMax);
            std::memcpy(local_addr_store, servers.localAddress, kNatNetIpv4AddrStrLenMax);
            std::string multicast_addr = get_parameter("multicast_address").as_string();
            std::strcpy(multicast_addr_store, multicast_addr.c_str());

            connection_params.connectionType = servers.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
            connection_params.serverCommandPort = servers.serverCommandPort;
            connection_params.serverDataPort = servers.serverDescription.ConnectionDataPort;
            connection_params.serverAddress = server_addr_store;
            connection_params.localAddress = local_addr_store;
            connection_params.multicastAddress = multicast_addr_store;
        } else {
            // legacy server
            std::memcpy(server_addr_store, servers.serverAddress, kNatNetIpv4AddrStrLenMax);
            std::memcpy(local_addr_store, servers.localAddress, kNatNetIpv4AddrStrLenMax);

            connection_params.connectionType = ConnectionType_Multicast;
            connection_params.serverCommandPort = servers.serverCommandPort;
            connection_params.serverDataPort = 0;
            connection_params.serverAddress = server_addr_store;
            connection_params.localAddress = local_addr_store;
            connection_params.multicastAddress = NULL;
        }
        RCLCPP_INFO(get_logger(), "Automatic discovery found:");
    } else {
        std::string server_addr = get_parameter("server_address").as_string();
        std::strcpy(server_addr_store, server_addr.c_str());
        std::string local_addr = get_parameter("local_address").as_string();
        std::strcpy(local_addr_store, local_addr.c_str());
        std::string multicast_addr = get_parameter("multicast_address").as_string();
        std::strcpy(multicast_addr_store, multicast_addr.c_str());

        int connection_type_int = static_cast<int>(get_parameter("connection_type").as_int());
        connection_params.connectionType = static_cast<ConnectionType>(connection_type_int);
        connection_params.serverCommandPort = static_cast<uint16_t>(get_parameter("server_command_port").as_int());
        connection_params.serverDataPort = static_cast<uint16_t>(get_parameter("server_data_port").as_int());
        connection_params.serverAddress = server_addr_store;
        connection_params.localAddress = local_addr_store;
        connection_params.multicastAddress = multicast_addr_store;

        RCLCPP_INFO(get_logger(), "Manually set to:");
    }

    RCLCPP_INFO(get_logger(), "\tConnectionType: %s", connection_params.connectionType == 0 ? "Multicast" : "Unicast");
    RCLCPP_INFO(get_logger(), "\tCommand Port: %d", connection_params.serverCommandPort);
    RCLCPP_INFO(get_logger(), "\tData Port: %d", connection_params.serverDataPort);
    RCLCPP_INFO(get_logger(), "\tServer Address: %s", connection_params.serverAddress ? connection_params.serverAddress : "[IS NULL]");
    RCLCPP_INFO(get_logger(), "\tLocal Address: %s", connection_params.localAddress ? connection_params.localAddress : "[IS NULL]");
    RCLCPP_INFO(get_logger(), "\tMulticast Address: %s", connection_params.multicastAddress ? connection_params.multicastAddress : "[IS NULL]");
    
    client = std::make_unique<NatNetClient>();
    client->SetFrameReceivedCallback(received_callback, this);
    NatNet_SetLogCallback(log_callback);

    std::string cur_node_prefix = name + "/";
    frame_publisher = create_publisher<optitrack_msgs::msg::MocapFrameData>(cur_node_prefix + Names::NatNetPublisher::mocap_msg_name, 10);
    connect_service = create_service<optitrack_msgs::srv::Connect>(cur_node_prefix + Names::NatNetPublisher::connect_srv_name,
                                                                   [&](const optitrack_msgs::srv::Connect::Request::SharedPtr, optitrack_msgs::srv::Connect::Response::SharedPtr)
                                                                   { connect(); });
    disconnect_service = create_service<optitrack_msgs::srv::Disconnect>(cur_node_prefix + Names::NatNetPublisher::disconnect_srv_name,
                                                                         [&](const optitrack_msgs::srv::Disconnect::Request::SharedPtr, optitrack_msgs::srv::Disconnect::Response::SharedPtr)
                                                                         { disconnect(); });
    reset_service = create_service<optitrack_msgs::srv::Reset>(cur_node_prefix + Names::NatNetPublisher::reset_srv_name,
                                                               [&](const optitrack_msgs::srv::Reset::Request::SharedPtr, optitrack_msgs::srv::Reset::Response::SharedPtr)
                                                               { reset(); });

    connect();
}

NatNetPublisher::~NatNetPublisher(){
    disconnect();
}

bool NatNetPublisher::connect() {
    if (!client) {
        return false;
    }
    int ret = client->Connect(connection_params);
    if (ret != ErrorCode_OK) {
        RCLCPP_WARN(get_logger(), "Unable to connect to server. Error code: %d.", ret);
        return false;
    }
    RCLCPP_INFO(get_logger(), "Connected");
    get_server_desc();
    return true;
}

void NatNetPublisher::disconnect() {
    if (client) {
        RCLCPP_INFO(get_logger(), "Disonnected");
        client->Disconnect();
    }
}

bool NatNetPublisher::reset() {
    disconnect();
    return connect();
}

void NatNetPublisher::get_server_desc() {
    memset(&server_desc, 0, sizeof(server_desc));
    ErrorCode ret;
    void* response;
    int n_bytes;

    ret = client->GetServerDescription(&server_desc);
    if (ret == ErrorCode_OK && server_desc.HostPresent) {
        RCLCPP_INFO(get_logger(), "Server application info:");
        RCLCPP_INFO(get_logger(), "\tApplication: %s (ver. %d.%d.%d.%d)", server_desc.szHostApp, server_desc.HostAppVersion[0],
                    server_desc.HostAppVersion[1], server_desc.HostAppVersion[2], server_desc.HostAppVersion[3]);
        RCLCPP_INFO(get_logger(), "\tNatNet Version: %d.%d.%d.%d", server_desc.NatNetVersion[0], server_desc.NatNetVersion[1],
                    server_desc.NatNetVersion[2], server_desc.NatNetVersion[3]);
        RCLCPP_INFO(get_logger(), "\tClient IP:%s", connection_params.localAddress);
        RCLCPP_INFO(get_logger(), "\tServer IP:%s", connection_params.serverAddress);
        RCLCPP_INFO(get_logger(), "\tServer Name:%s", server_desc.szHostComputerName);
        RCLCPP_INFO(get_logger(), "\tClock Frequency:%d", server_desc.HighResClockFrequency);
    } else {
        RCLCPP_WARN(get_logger(), "Unable to connect to server.");
    }

    ret = client->SendMessageAndWait("FrameRate", &response, &n_bytes);
    if (ret == ErrorCode_OK) {
        float frame_rate = *((float*)response);
        RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f", frame_rate);
    } else {
        RCLCPP_WARN(get_logger(), "Error getting frame rate.");
    }

    ret = client->SendMessageAndWait("AnalogSamplesPerMocapFrame", &response, &n_bytes);
    if (ret == ErrorCode_OK) {
        int samples_per_frame = *((int*)response);
        RCLCPP_INFO(get_logger(), "Analog Samples Per Mocap Frame : %d", samples_per_frame);
    } else {
        RCLCPP_WARN(get_logger(), "Error getting Analog frame rate.");
    }
}

void NATNET_CALLCONV NatNetPublisher::received_callback(sFrameOfMocapData* frame, void* user_data) {
    NatNetPublisher* natnet_publisher = (NatNetPublisher*)user_data;

    rclcpp::Time recieved_ros_time = natnet_publisher->now();
    std::chrono::nanoseconds camera_mid_exposure_dur(static_cast<int64_t>(natnet_publisher->client->SecondsSinceHostTimestamp(frame->CameraMidExposureTimestamp) * 1e9));

    auto message = optitrack_msgs::msg::MocapFrameData();
    message.id = frame->iFrame;

    for(int i = 0; i < frame->nMarkerSets; i++){
        message.mocap.emplace_back();
        auto& mocap = message.mocap.back();
        auto& cur_mocap = frame->MocapData[i];
        mocap.name = cur_mocap.szName;
        for(int j = 0; j < cur_mocap.nMarkers; j++){
            mocap.markers.emplace_back();
            auto& marker = mocap.markers.back();
            auto& cur_marker = cur_mocap.Markers[j];
            marker.x = cur_marker[0];
            marker.y = cur_marker[1];
            marker.z = cur_marker[2];
        }
    }
    
    for(int i = 0; i < frame->nOtherMarkers; i++){
        message.other_markers.emplace_back();
        auto& other_marker = message.other_markers.back();
        auto& cur_marker = frame->OtherMarkers[i];
        other_marker.x = cur_marker[0];
        other_marker.y = cur_marker[1];
        other_marker.z = cur_marker[2];
    }

    for(int i = 0; i < frame->nRigidBodies; i++){
        message.rigidbodies.emplace_back();
        auto& rigidbody = message.rigidbodies.back();
        auto& cur_rigidbody = frame->RigidBodies[i];
        rigidbody.id = cur_rigidbody.ID;
        rigidbody.x = cur_rigidbody.x;
        rigidbody.y = cur_rigidbody.y;
        rigidbody.z = cur_rigidbody.z;
        rigidbody.qw = cur_rigidbody.qw;
        rigidbody.qx = cur_rigidbody.qx;
        rigidbody.qy = cur_rigidbody.qy;
        rigidbody.qz = cur_rigidbody.qz;
        rigidbody.mean_error = cur_rigidbody.MeanError;
        rigidbody.params = cur_rigidbody.params;
    }

    for(int i = 0; i < frame->nSkeletons; i++){
        message.skeletons.emplace_back();
        auto& skeleton = message.skeletons.back();
        auto& cur_skeleton = frame->Skeletons[i];
        skeleton.id = cur_skeleton.skeletonID;
        for(int j = 0; j < cur_skeleton.nRigidBodies; j++){
            skeleton.rigidbodies.emplace_back();
            auto& rigidbody = skeleton.rigidbodies.back();
            auto& cur_rigidbody = cur_skeleton.RigidBodyData[j];
            rigidbody.id = cur_rigidbody.ID;
            rigidbody.x = cur_rigidbody.x;
            rigidbody.y = cur_rigidbody.y;
            rigidbody.z = cur_rigidbody.z;
            rigidbody.qw = cur_rigidbody.qw;
            rigidbody.qx = cur_rigidbody.qx;
            rigidbody.qy = cur_rigidbody.qy;
            rigidbody.qz = cur_rigidbody.qz;
            rigidbody.mean_error = cur_rigidbody.MeanError;
            rigidbody.params = cur_rigidbody.params;
        }
    }

    for(int i = 0; i < frame->nLabeledMarkers; i++){
        message.labelled_markers.emplace_back();
        auto& labelled_marker = message.labelled_markers.back();
        auto& cur_labelled_marker = frame->LabeledMarkers[i];
        labelled_marker.id = cur_labelled_marker.ID;
        labelled_marker.x = cur_labelled_marker.x;
        labelled_marker.y = cur_labelled_marker.y;
        labelled_marker.z = cur_labelled_marker.z;
        labelled_marker.size = cur_labelled_marker.size;
        labelled_marker.params = cur_labelled_marker.params;
        labelled_marker.residual = cur_labelled_marker.residual;
    }

    for(int i = 0; i < frame->nForcePlates; i++){
        message.force_plates.emplace_back();
        auto& force_plate = message.force_plates.back();
        auto& cur_force_plate = frame->ForcePlates[i];
        force_plate.id = cur_force_plate.ID;
        for(int j = 0; j < cur_force_plate.nChannels; j++){
            force_plate.channels.emplace_back();
            auto& channel = force_plate.channels.back();
            auto& cur_channel = cur_force_plate.ChannelData[j];
            for (int k = 0; k < cur_channel.nFrames; k++){
                channel.values.emplace_back();
                auto& value = channel.values.back();
                auto& cur_value = cur_channel.Values[k];
                value = cur_value;
            }
        }
        force_plate.params = cur_force_plate.params;
    }

    for(int i = 0; i < frame->nDevices; i++){
        message.devices.emplace_back();
        auto& device = message.devices.back();
        auto& cur_device = frame->Devices[i];
        device.id = cur_device.ID;
        for(int j = 0; j < cur_device.nChannels; j++){
            device.channels.emplace_back();
            auto& channel = device.channels.back();
            auto& cur_channel = cur_device.ChannelData[j];
            for (int k = 0; k < cur_channel.nFrames; k++){
                channel.values.emplace_back();
                auto& value = channel.values.back();
                auto& cur_value = cur_channel.Values[k];
                value = cur_value;
            }
        }
        device.params = cur_device.params;
    }

    // message.timecode = frame->Timecode;
    // message.timecode_subframe = frame->TimecodeSubframe;
    // message.software_timestamp = frame->fTimestamp;
    // message.camera_mid_exposure_timestamp = frame->CameraMidExposureTimestamp;
    // message.camera_data_received_timestamp = frame->CameraDataReceivedTimestamp;
    // message.transmit_timestamp = frame->TransmitTimestamp;

    rclcpp::Time camera_mid_exposure_time = recieved_ros_time - camera_mid_exposure_dur;
    message.camera_mid_exposure_time.sec = camera_mid_exposure_time.nanoseconds() / 1000000000;
    message.camera_mid_exposure_time.nanosec = camera_mid_exposure_time.nanoseconds() % 1000000000;
    rclcpp::Time camera_data_received_time = camera_mid_exposure_time + std::chrono::nanoseconds(static_cast<int64_t>(frame->CameraDataReceivedTimestamp - frame->CameraMidExposureTimestamp) * (1000000000 / natnet_publisher->server_desc.HighResClockFrequency));
    message.camera_data_received_time.sec = camera_data_received_time.nanoseconds() / 1000000000;
    message.camera_data_received_time.nanosec = camera_data_received_time.nanoseconds() % 1000000000;
    rclcpp::Time transmit_time = camera_mid_exposure_time + std::chrono::nanoseconds(static_cast<int64_t>(frame->TransmitTimestamp - frame->CameraMidExposureTimestamp) * (1000000000 / natnet_publisher->server_desc.HighResClockFrequency));
    message.transmit_time.sec = transmit_time.nanoseconds() / 1000000000;
    message.transmit_time.nanosec = transmit_time.nanoseconds() % 1000000000;
    message.params = frame->params;
    
    natnet_publisher->frame_publisher->publish(message);
}

void NATNET_CALLCONV NatNetPublisher::log_callback(Verbosity verbosity, const char* message) {
    static auto logger = rclcpp::get_logger("NatNet");

    switch (verbosity) {
    case Verbosity_Debug:
        RCLCPP_DEBUG(logger, message);
        break;
    case Verbosity_Info:
        RCLCPP_INFO(logger, message);
        break;
    case Verbosity_Warning:
        RCLCPP_WARN(logger, message);
        break;
    case Verbosity_Error:
        RCLCPP_ERROR(logger, message);
        break;
    default:
        RCLCPP_INFO(logger, message);
        break;
    }
}
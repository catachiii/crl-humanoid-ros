#pragma once

namespace Names{
    namespace NatNetPublisher {
        static constexpr const char *mocap_msg_name = "mocap_frame";
        static constexpr const char *connect_srv_name = "connect";
        static constexpr const char *disconnect_srv_name = "disconnect";
        static constexpr const char *reset_srv_name = "reset";
    } // namespace NatNetPublisher

    namespace Dispatcher {
        static constexpr const char *rigidbody_prefix = "rigidbodies";
    }
} // namespace Names
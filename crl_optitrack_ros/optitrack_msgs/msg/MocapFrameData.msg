int32 id
optitrack_msgs/MarkerSetData[] mocap
optitrack_msgs/MarkerData[] other_markers
optitrack_msgs/RigidbodyData[] rigidbodies
optitrack_msgs/SkeletonData[] skeletons
optitrack_msgs/LabelledMarkerData[] labelled_markers
optitrack_msgs/ForcePlateData[] force_plates
optitrack_msgs/DeviceData[] devices

# uint32 timecode
# uint32 timecode_subframe
# float64 software_timestamp
# uint64 camera_mid_exposure_timestamp
# uint64 camera_data_received_timestamp
# uint64 transmit_timestamp

builtin_interfaces/Time camera_mid_exposure_time
builtin_interfaces/Time camera_data_received_time
builtin_interfaces/Time transmit_time

int16 params

# image_transport_py attempt (does not run on ROS 2 Jazzy)

These two scripts use `image_transport_py`, the Python bindings for `image_transport`.
That package is **not available for Jazzy**: there is no apt package
(`ros-jazzy-image-transport-py`) and the `jazzy` branch of `ros-perception/image_common`
does not contain it. They are kept here so the work can be picked up if the bindings are
ever released for Jazzy (or if you move to a distro that ships them).

The working implementation is the C++ package in `ros_ws/src/video_streaming_cpp`.

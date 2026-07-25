=========
CHANGELOG
=========

All notable changes to the ip_camera_ros2 project will be documented in this file.

[Unreleased]
============

Fixed
-----
- Fixed a crash in ``IpCameraRos2::capture_ipcam_image()`` when the configured crop ROI
  (``offset_x``, ``offset_y``, ``image_width``, ``image_height``) fell outside the actual
  frame bounds; the ROI is now clamped with ``cv::Rect`` intersection and the frame is
  skipped with a throttled warning instead of letting OpenCV throw.
- Fixed an integer division by zero in ``IpCameraRos2``'s constructor when ``frame_rate``
  was set to ``0``; the parameter is now validated and clamped to a minimum of ``1``.
- Fixed a potential out-of-bounds write in ``IpCameraRos2::create_cam_info_msg()`` by
  using ``std::copy_n`` bounded by the destination ``std::array`` size, and by only
  building/publishing ``CameraInfo`` when the ``correct_cam_info_`` calibration-validity
  flag (previously computed but never read) is ``true``.
- Fixed ``package.xml`` missing the ``image_transport`` and OpenCV (``libopencv-dev``)
  dependencies, which are required by ``CMakeLists.txt``/the C++ sources but were absent,
  breaking ``rosdep install`` on a clean checkout.
- Fixed published ``CameraInfo.height``/``CameraInfo.width`` being set from
  ``image_height``/``image_width`` (``-1`` when unset, wrapping to ``4294967295`` as
  ``uint32``) instead of the dimensions of the actually published frame.

Changed
-------
- Removed the unused ``geometry_msgs`` dependency and moved ``tf2``/``tf2_ros`` to
  ``exec_depend`` in ``package.xml`` since they are only used at runtime by the launch
  files, never compiled against. Added ``exec_depend`` entries for
  ``robot_state_publisher``, ``xacro`` and ``launch_ros``.
- Removed the no-op ``target_link_libraries(${PROJECT_NAME})`` call from
  ``CMakeLists.txt`` and replaced the trivial ``IpCameraRos2`` destructor definition
  with ``= default``.
- Synchronized ``README.md`` with the actual parameter defaults and documented the
  ``buffer_size`` parameter, which was previously missing from the docs. Corrected the
  Ubuntu version tested with ROS 2 Jazzy (24.04, not 22.04).
- Lowered the default ``buffer_size`` from 30 (code)/50 (``config/params.yaml``) to 2:
  the consumer always keeps only the latest frame and discards the rest, so a deep
  buffer never smoothed anything and only inflated worst-case memory usage.
- Refactored ``IpCameraRos2::update_params()`` to use new ``declare_and_get<T>()``/
  ``declare_and_get_array()`` helpers, removing repeated declare/get/log boilerplate and
  fixing copy-pasted parameter descriptions (e.g. ``image_height``/``image_width`` no
  longer describe a "Depth image").
- Extracted the crop/resize transform (``ip_camera_ros2::apply_crop_or_resize()`` in
  ``image_ops.hpp``/``.cpp``), the ``CameraInfo`` construction and calibration validation
  (``ip_camera_ros2::build_camera_info()``/``is_calibration_valid()`` in
  ``camera_info_builder.hpp``/``.cpp``), and the exponential backoff delay computation
  (``RTSPCapturer::next_retry_delay()``) out of ``IpCameraRos2``/``RTSPCapturer`` and into
  small, ROS-node-independent, unit-testable functions.

Performance
-----------
- Added a short interruptible sleep in ``RTSPCapturer::run()`` on transient
  ``grab()``/empty-frame failures to avoid a busy-wait loop pegging a CPU core.
- Publish images via the ``sensor_msgs::msg::Image::SharedPtr`` overload of
  ``image_transport::Publisher::publish()`` instead of dereferencing and passing by
  value, avoiding an extra copy per frame and enabling zero-copy intra-process delivery.

Added
-----
- Added a ``test/`` gtest suite (``ament_add_gtest``): ``test_image_ops`` covers ROI
  clamping and the crop/resize transform (including the [C1] out-of-bounds case),
  ``test_camera_info_builder`` covers calibration validation and ``CameraInfo``
  construction (including the [C3] oversized-vector case), ``test_rtsp_capturer_backoff``
  covers the exponential backoff sequence and its cap, and ``test_frame_buffer`` covers
  the new ``FrameBuffer`` class (below).

Architecture
------------
- Introduced ``ip_camera_ros2::FrameBuffer`` (``frame_buffer.hpp``/``.cpp``), replacing
  the raw ``std::deque<cv::Mat>``/``std::mutex`` pair that used to be public fields on
  ``IpCameraRos2`` purely so ``main()`` could wire up ``RTSPCapturer`` externally.
- ``IpCameraRos2`` now creates and owns its ``RTSPCapturer`` and producer thread
  internally (started in its constructor, stopped in its destructor); ``main.cpp`` is
  reduced to ``rclcpp::init`` + construct node + ``spin`` + ``rclcpp::shutdown``.
- ``RTSPCapturer`` takes a ``FrameBuffer &`` instead of a raw ``deque``/``mutex``/
  ``buffer_size`` triple.
- ``RTSPCapturer::stop()`` no longer calls ``cv::VideoCapture::release()`` from outside
  the producer thread; it only flips the ``running_`` atomic flag, and the producer
  thread releases the stream itself once it observes it. A ``grab()`` blocked on the
  network now unblocks via the existing 5 s connection timeout instead of a
  cross-thread ``release()``, which was technically a data race on ``cv::VideoCapture``.
- Migrated image/CameraInfo publishing to ``image_transport::CameraPublisher`` (used
  whenever ``enable_cam_info`` is on and calibration is valid), which publishes both
  atomically under the standard ``<image_topic>/camera_info`` sibling naming that
  calibration/rectification tools expect. The separately-configurable
  ``cam_info_topic`` parameter is removed as a result.
- Converted ``IpCameraRos2`` from ``rclcpp::Node`` to ``rclcpp_lifecycle::LifecycleNode``:
  parameters are read and publishers created in ``on_configure()``; the RTSP stream is
  only connected (``RTSPCapturer`` + producer thread created) in ``on_activate()`` and
  disconnected in ``on_deactivate()``; ``on_cleanup()``/``on_shutdown()`` release the
  remaining state. This lets the node be started without connecting, and orchestrated
  by external tooling (``ros2 lifecycle set``, a lifecycle manager, Nav2-style
  bringup), addressing [M3] from the analysis.

  As a consequence, ``image_transport`` is no longer used: it does not support
  ``LifecycleNode`` on ROS 2 Jazzy (``ros-perception/image_common#108``, only fixed for
  Rolling via ``#352`` in November 2025). Image and CameraInfo are now published via
  plain ``rclcpp_lifecycle::LifecyclePublisher``, on raw topics rather than through
  image_transport's compressed/theora transport negotiation; the standard
  ``<image_topic>/camera_info`` sibling naming is preserved via the new
  ``ip_camera_ros2::derive_camera_info_topic()`` (``topic_utils.hpp``/``.cpp``).
- Both launch files now launch ``ip_camera_ros2`` as a ``launch_ros.actions.LifecycleNode``
  and add an ``autostart`` launch argument (default ``true``) that automatically
  configures and activates it on startup, matching the previous non-lifecycle behaviour
  by default; pass ``autostart:=false`` to drive the lifecycle manually instead.

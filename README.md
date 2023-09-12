## Doppler-only Single-scan 3D Vehicle Odometry
This package estimates the motion of a vehicle based on the radial velocity measurements from a Doppler-capable sensor. 

## Launch parameters
* ``verbose``: whether the node should output execution information to console or not.
* ``input_topic``: the ``sensor_msg::PointCloud2`` topic to subscribe to. It should have the following ``float32`` fields in the same order: ``x``, ``y``, ``z``, ``range``, ``elevation``, ``azimuth``, ``power``, ``doppler``.
* ``tf_from_frame`` and ``tf_to_frame``: the tf frames where the ``geometry_msgs::TransformStamped`` and ``nav_msg::Odometry`` messages should point to. ``from`` becomes the ``tf.header.frame_id``, and ``to`` becomes ``tf.child_frame_id``.
* ``initial_pose``: starting pose of the vehicle, in the format of a flattened 4x4 matrix (row-major).
* ``sensor_pose``: pose of the sensor on the vehicle, in the format of position (3: x, y, z) and quaternion (4: x, y, z, w).
* ``calib_rot``: calibrated rotation to fix the sensor orientation, as a quaternion (4: x, y, z).
* ``sensor_to_icr_x``: distance between the sensor and the ICR location in meters.
* ``min_points``: minimum number of points for the node to work.
* ``use_power``: whether the power of the signal of each point should be used as weight.
* ``threshold_mode``: method to discard outliers and obtain the sensor velocity from the points of the scene. Possible values:
  * ``ransac``: default, uses RANSAC with the threshold value of ``threshold_value``.
  * ``percent``: uses Trimmed Iterative Reweighted Least Squares (IRLS), threshold is set to allow X percent of points as inliers, where is the value of ``threshold_value``.
  * ``mad``: uses Trimmed IRLS, threshold is set to the median of the residuals plus X times the mad, where X is the value of ``threshold_value``.
  * ``fixed``: uses Trimmed IRLS, threshold is the value of ``threshold_value``.
* ``threshold_value``: value used to discard outliers. See ``threshold_mode`` to see its application.
* ``max_linear_vel``: threshold applied to the linear velocity to be considered valid. Higher values are discarded [m/s].
* ``max_ang_vel``: threshold applied to the angular velocity to be considered valid. Higher values are discarded [rad/s].
* ``velocity_file``: file name where the estimated velocity and covariance of the sensor in each frame should be written. Leave empty so it does not write the file.
* ``odometry_file``: file name where the estimated odometry should be written. The format is timestamp (seconds, nanoseconds), position (x, y, z), orientation quaternion (x, y, z, w). Leave empty so it does not write the file.
* ``filtered_points_topic``: topic name where the filtered point cloud should be published, outliers are discarded.
  

## Reference
* TODO

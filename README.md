## Doppler-only Single-scan 3D Vehicle Odometry

This ROS node estimates the 3D odometry of a vehicle using the radial velocities provided by a Doppler-capable sensor. First, the velocity of the sensor is estimated from the sensed radial velocities from the scene. Then, the kinematic model of the vehicle provides the relation between the obtained sensor velocity and the vehicle velocity (both linear and angular). The algorithm is based on the following assumptions:

* Majority of the observed points belong to static objects.
* Sensor is calibrated so that the X, Y, and Z axes point forward, left, and upwards, respectively.
* Vehicle moves without roll rotation (around X axis).
* ICR related to yaw rotation is consistently found in a line parallel to the Y axis (Ackermann, skid-steer and differential drive fulfill this hypothesis).
* Vertical curvature of the ground is constant in the relatively small distance between wheel axes.

## Launch parameters

Please launch the node using a launch file to correctly set the different parameters:

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
  
## Dataset
The associated dataset can be found [here](https://zenodo.org/record/8346769).

## Reference
The paper titled 'Doppler-only Single-scan 3D Vehicle Odometry' is under review for publication.

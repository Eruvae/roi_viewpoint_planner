# roi_viewpoint_planner

Plan viewpoints to detect and scan regions of interest

## Related packages

[roi_viewpoint_planner_msgs](https://github.com/Eruvae/roi_viewpoint_planner_msgs): Messages and dynamic reconfigure options for the planner.

[rqt_roi_viewpoint_planner](https://github.com/Eruvae/rqt_roi_viewpoint_planner): [RQT](http://wiki.ros.org/rqt) plugin to control the planner.

[octomap_vpp](https://github.com/Eruvae/octomap_vpp): Extends the [octomap framework](http://octomap.github.io/) with octrees used by the planner.

[octomap_vpp_rviz_plugin](https://github.com/Eruvae/octomap_vpp_rviz_plugin): Visualization plugin for custom octrees in rviz.

[pointcloud_roi](https://github.com/Eruvae/pointcloud_roi): Provides nodelets to generate pointclouds with ROI to be inserted in the planning octree.

[pointcloud_roi_msgs](https://github.com/Eruvae/pointcloud_roi_msgs): Messages for pointclouds with ROI.

[ur_with_cam_gazebo](https://github.com/Eruvae/ur_with_cam_gazebo): Launch gazebo simulations of robot arm with camera in rooms with capsicum plants.

## Usage

For running experiments with a simulated arm, start a simulated environment as described in [ur_with_cam_gazebo](https://github.com/Eruvae/ur_with_cam_gazebo).

The planner node itself can be started with:
```Shell
  rosrun roi_viewpoint_planner planner_node
```
Some static and dynamic parameter are available to configure the planner. They are described in the tables below. The dynamic parameters can be configured at runtime using our [RQT plugin](https://github.com/Eruvae/rqt_roi_viewpoint_planner). This plugin can also be used to activate the planner (which is in idle by default), to save, load and reset the stored octree, to confirm plan requests if required, to randomize plant positions (if using our simulated environments), and to start the evaluation of the planner.

### Static Parameters

| Parameter                      | Description                                                                          | Default                   |
|--------------------------------|--------------------------------------------------------------------------------------|---------------------------|
| tree_resolution                | Resolution of planning octree (in m)                                                 | 0.01                      |
| workspace_tree                 | File path for workspace octree (Specifying region for valid viewpoints)              | [ur_with_cam_gazebo](https://github.com/Eruvae/ur_with_cam_gazebo)/workspace_trees/static/workspace_map.ot |
| sampling_tree                  | File path for sampling octree (Specifying region for viewpoint targets)              | [ur_with_cam_gazebo](https://github.com/Eruvae/ur_with_cam_gazebo)/workspace_trees/static/inflated_ws_tree.ot |
| map_frame                      | TF frame for which the planning octree is generated                                  | world                     |
| ws_frame                       | TF frame of workspace and sampling octree                                                         | arm_base_link             |
| update_planning_tree           | Subscribe to pointclouds with ROI to update octree (necessary for planner operation, can be turned off e.g. to load and evaluate saved octree) | True                     |
| initialize_evaluator           | Initialize evaluator to compare results with groundtruth                             | True                      |
| initial_joint_values           | Robot is moved to this joint configuration at start                                  | None (Robot is not moved) |

### Dynamic Parameters

| Parameter                      | Description                                                                              | Default              |
|--------------------------------|------------------------------------------------------------------------------------------|----------------------|
| mode                           | Select the planner mode (See modes table)                                                | IDLE                 |
| activate_execution             | If activated, planner moves arm to best suitable sampled viewpoint                       | True                 |
| require_execution_confirmation | If activated, each planning step execution has to be confirmed by the user (via rqt)     | False                |
| sensor_min_range               | Minimum range for the sensor                                                             | 0.3                  |
| sensor_max_range               | Maximum range for the sensor                                                             | 0.5                  |
| insert_scan_if_not_moved       | Update octree even if camera position has not changed since last viewpoint               | True                 |
| insert_scan_while_moving       | Update octree while camera is moving to target position                                  | False                |
| wait_for_scan                  | Planner only plans next viewpoint if scan has been inserted since last viewpoint         | False                |
| publish_planning_state         | Publish whether the robot is moving and if scans have been inserted since last viewpoint | True                 |
| planner                        | Select the planner used for motion planning (See planner table)                          | RRTConnect           |
| planning_time                  | Choose maximum motion planning time (in seconds)                                         | 5                    |
| use_cartesian_motion           | Try to move to viewpoints on cartesian path                                              | False                |
| compute_ik_when_sampling       | If true, IK is checked already during sampling, otherwise only during motion planning    | False                |
| velocity_scaling               | Scaling factor for maximum joint velocity                                                | 1.0                  |
| record_map_updates             | Store map updates in rosbag                                                              | False                |
| record_viewpoints              | Store planned viewpoints in rosbag                                                       | False                |
| activate_move_to_see           | Use move to see suggestions if available                                                 | False                |
| move_to_see_exclusive          | Only use move to see for planning                                                        | False                |
| m2s_delta_thresh               | Minimum delta for move to see                                                            | 0.5                  |
| m2s_max_steps                  | Maximum move to see steps before switching                                               | 3                    |
| publish_cluster_visualization  | Publish a visualization of the detected clusters with the map                            | False                |
| minimum_cluster_size           | Minimum size for cluster visualization                                                   | 10                   |
| cluster_neighborhood           | Neighborhood region for cluster visualization (NB_6, NB_18 or NB_26)                     | NB_26                |
| auto_roi_sampling              | Select the ROI sampling method for automatic sampling (planner modes 3-5)                | SAMPLE_ROI_CONTOURS  |
| auto_expl_sampling             | Select the exploration sampling method for automatic sampling (planner modes 6-8)        | SAMPLE_CONTOURS      |
| roi_max_samples                | Select max samples for ROI sampling                                                      | 100                  |
| roi_util                       | Select utility for ROI sampling (See utility table)                                      | ROI_VICINITY_UTILITY |
| expl_max_samples               | Select max samples for exploration sampling                                              | 100                  |
| expl_util                      | Select utility for exploration sampling (See utility table)                              | ROI_VICINITY_UTILITY |

### Planner Modes

| Modes                   | Description                                               |
| ------------------------|-----------------------------------------------------------|
| IDLE (0)                | Do nothing                                                |
| MAP_ONLY (1)            | Generate map, but do not plan viewpoints                  |
| SAMPLE_AUTOMATIC (2)    | Automatically select sampling algorithm                   |
| SAMPLE_ROI_CONTOURS (3) | Sample viewpoints at ROI contours                         |
| SAMPLE_ROI_ADJACENT (4) | Sample viewpoints at border close to ROIs                 |
| SAMPLE_ROI_CENTERS (5)  | Sample viewpoints around ROI centers                      |
| SAMPLE_EXPLORATION (6)  | Sample viewpoint pointing from workspace to sampling tree |
| SAMPLE_CONTOURS (7)     | Sample viewpoints at object contours                      |
| SAMPLE_BORDER (8)       | Sample viewpoints at border to unknown space              |

### Motions Planners

| Planner    | Description                                                 |
| -----------|-------------------------------------------------------------|
| SBL        | Single-query Bi-directional Lazy collision checking planner |
| EST        | Expansive Space Trees                                       |
| LBKPIECE   | Lazy Bi-directional KPIECE                                  |
| BKPIECE    | Bi-directional KPIECE                                       |
| KPIECE     | Kinematic Planning by Interior-Exterior Cell Exploration    |
| RRT        | Rapidly-exploring Random Trees                              |
| RRTConnect | RRT Connect                                                 |
| RRTstar    | RRT*                                                        |
| TRRT       | Transition-based RRT                                        |
| PRM        | Probabilistic Roadmap Method                                |
| PRMstar    | PRM*                                                        |

### Viewpoint utilities

| Utility               | Description                               |
| ----------------------|-------------------------------------------|
| SINGLE_RAY_UTILITY    | Evaluate single ray from origin to target |
| MULTI_RAY_UTILITY     | Evaluate multiple rays in camera FOV      |
| ROI_VICINITY_UTILITY  | Evaluate ROI vicinity around target       |
| ROI_OCCLUSION_UTILITY | Evaluate occusion of ROI                  |

## Papers

We have published two papers for this work:

[Viewpoint Planning for Fruit Size and Position Estimation](https://arxiv.org/pdf/2011.00275.pdf), accepted at IROS 2021

```bibtex
@inproceedings{zaenker2020viewpoint,
	title={Viewpoint Planning for Fruit Size and Position Estimation},
	author={Zaenker, Tobias and Smitt, Claus and McCool, Chris and Bennewitz, Maren},
	booktitle={Proc.~of the IEEE/RSJ Intl.~Conf.~on Intelligent Robots and Systems (IROS)},
	year={2021}
}
```
[Combining Local and Global Viewpoint Planning for Fruit Coverage](https://arxiv.org/pdf/2108.08114.pdf), accepted at ECMR 2021

```bibtex
@inproceedings{zaenker2020ecmr,
	title={Combining Local and Global Viewpoint Planning for Fruit Coverage},
	author={Zaenker, Tobias and Lehnert, Chris and McCool, Chris and Bennewitz, Maren},
	booktitle={Proc.~of the Europ.~Conf.~on Mobile Robotics (ECMR)},
	year={2021}
}
```

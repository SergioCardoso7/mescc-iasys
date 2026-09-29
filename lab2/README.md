# LAB 2 Deliverables

Followed indications and solved exercises from [lab2_pdf](./IASYS-LAB2-2026.pdf)

## Node/interface table for the main nodes

| Node                 | Subscribes to                                      | Publishes                                                                                            | Other interfaces                                          |
| -------------------- | -------------------------------------------------- | ---------------------------------------------------------------------------------------------------- | --------------------------------------------------------- |
| `/world`             | `/parameter_events`                                | `/map`, `/world_markers`, `/parameter_events`, `/rosout`                                             | Parameter management services                             |
| ---                  | ---                                                | ---                                                                                                  | ---                                                       |
| `/vehicle_simulator` | `/cmd_vel`, `/parameter_events`                    | `/odom`, `/tf`, `/trajectory`, `/vehicle_marker`, `/vehicle_markers`, `/parameter_events`, `/rosout` | `/reset_vehicle` (Trigger); parameter management services |
| ---                  | ---                                                | ---                                                                                                  | ---                                                       |
| `/simple_planner`    | `/goal_pose`, `/map`, `/odom`, `/parameter_events` | `/path`, `/goal_marker`, `/planner_markers`, `/parameter_events`, `/rosout`                          | Parameter management services                             |
| ---                  | ---                                                | ---                                                                                                  | ---                                                       |
| `/simple_controller` | `/odom`, `/path`, `/parameter_events`              | `/cmd_vel`, `/parameter_events`, `/rosout`                                                           | Parameter management services                             |

## RQT_GRAPH

![rqt_graph](./functional_path_rqt_graph.png)

## Calling /reset_vehicle and changing linear speed at runtime

### Calling /reset_vehicle

![call_reset_vehicle](./reset_vehicle_service_call.png)

### Changing linear speed at runtime

![change_linear_speed_runtime](./changing_linear_speed_at_runtime.png)

### Publishing to /monitor/distance_from_start

![publishing_monitor_distance_from_start](./publishing_to_monitor_distance_from_start.png)

### /reset_statistics_service call from ROS2 CLI

![reset_statistics_call](./reseting_statistics_call.png)

### RCLCPP_WARN

![rclcpp_warn](./producing_custom_RCLCPP_WARN.png)

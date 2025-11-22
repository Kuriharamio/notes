# 行为树导航

> https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html
>
> BT Navigator（行为树导航器）模块实现了 NavigateToPose、NavigateThroughPoses 和其他任务接口它是一种基于行为树的导航实现，旨在实现导航任务的灵活性，并提供一种轻松指定复杂机器人行为（包括恢复）的方法

###  bt_navigator 参数

1. **use_sim_time**
   
   - **中文名称**：使用仿真时间
   - **用途**：指定是否使用仿真时间
   - **配置方法**：布尔值，设置为 `true` 或 `false`
   
2. **global_frame**
   - **中文名称**：全局坐标系
   - **用途**：指定全局坐标系的名称
   - **配置方法**：字符串，例如 `map`

3. **robot_base_frame**
   - **中文名称**：机器人基坐标系
   - **用途**：指定机器人基坐标系的名称
   - **配置方法**：字符串，例如 `base_link`

4. **transform_tolerance**
   - **中文名称**：变换容差
   - **用途**：指定坐标变换的容差
   - **配置方法**：浮点数，例如 `0.1` 秒

5. **default_nav_to_pose_bt_xml**
   - **中文名称**：默认导航到姿态行为树XML
   - **用途**：指定默认导航到姿态行为树的XML文件路径
   - **配置方法**：字符串，例如 `replace/with/path/to/bt.xml` 或 `$(find-pkg-share my_package)/behavior_tree/my_nav_to_pose_bt.xml`

6. **default_nav_through_poses_bt_xml**
   - **中文名称**：默认导航通过姿态行为树XML
   - **用途**：指定默认导航通过姿态行为树的XML文件路径
   - **配置方法**：字符串，例如 `replace/with/path/to/bt.xml` 或 `$(find-pkg-share my_package)/behavior_tree/my_nav_through_poses_bt.xml`

7. **always_reload_bt_xml**
   - **中文名称**：始终重新加载行为树XML
   - **用途**：指定是否始终重新加载行为树XML文件
   - **配置方法**：布尔值，设置为 `true` 或 `false`

8. **goal_blackboard_id**
   - **中文名称**：目标黑板ID
   - **用途**：指定目标在黑板中的ID
   - **配置方法**：字符串，例如 `goal`

9. **goals_blackboard_id**
   - **中文名称**：目标列表黑板ID
   - **用途**：指定目标列表在黑板中的ID
   - **配置方法**：字符串，例如 `goals`

10. **path_blackboard_id**
    - **中文名称**：路径黑板ID
    - **用途**：指定路径在黑板中的ID
    - **配置方法**：字符串，例如 `path`

11. **navigators**
    - **中文名称**：导航器
    - **用途**：指定要使用的导航器列表
    - **配置方法**：字符串列表，例如 `['navigate_to_pose', 'navigate_through_poses']`

12. **navigate_to_pose**
    - **中文名称**：导航到姿态
    - **用途**：指定导航到姿态的插件
    - **配置方法**：字符串，例如 `"nav2_bt_navigator::NavigateToPoseNavigator"`

13. **navigate_through_poses**
    - **中文名称**：导航通过姿态
    - **用途**：指定导航通过姿态的插件
    - **配置方法**：字符串，例如 `"nav2_bt_navigator::NavigateThroughPosesNavigator"`

14. **plugin_lib_names**
    - **中文名称**：插件库名称
    - **用途**：指定行为树节点插件库的名称列表
    - **配置方法**：字符串列表，例如：
      ```yaml
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      ```

15. **error_code_names**
    - **中文名称**：错误代码名称
    - **用途**：指定错误代码的名称列表
    - **配置方法**：字符串列表，例如：
      ```yaml
      - compute_path_error_code
      - follow_path_error_code
      ```

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    default_nav_to_pose_bt_xml: replace/with/path/to/bt.xml # or $(find-pkg-share my_package)/behavior_tree/my_nav_to_pose_bt.xml
    default_nav_through_poses_bt_xml: replace/with/path/to/bt.xml # or $(find-pkg-share my_package)/behavior_tree/my_nav_through_poses_bt.xml
    always_reload_bt_xml: false
    goal_blackboard_id: goal
    goals_blackboard_id: goals
    path_blackboard_id: path
    navigators: ['navigate_to_pose', 'navigate_through_poses']
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator" # In Iron and older versions, "/" was used instead of "::"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator" # In Iron and older versions, "/" was used instead of "::"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
    error_code_names:
      - compute_path_error_code
      - follow_path_error_code
      # - smoother_error_code, navigate_to_pose_error_code, navigate_through_poses_error_code, etc
```

# 行为树XML及插件

https://docs.nav2.org/configuration/packages/configuring-bt-xml.html

[nav2_behavior_tree](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&u=https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree)包提供了几个预先注册的导航特定节点，可以包含在行为树中

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
              <GoalUpdated/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ReactiveFallback name="FollowPathRecoveryFallback">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </ReactiveFallback>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.15" backup_speed="0.025"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

## Action Plugins[¶](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html#action-plugins)

- [Wait](https://docs.nav2.org/configuration/packages/bt-plugins/actions/Wait.html)
- [Spin](https://docs.nav2.org/configuration/packages/bt-plugins/actions/Spin.html)
- [BackUp](https://docs.nav2.org/configuration/packages/bt-plugins/actions/BackUp.html)
- [DriveOnHeading](https://docs.nav2.org/configuration/packages/bt-plugins/actions/DriveOnHeading.html)
- [AssistedTeleop](https://docs.nav2.org/configuration/packages/bt-plugins/actions/AssistedTeleop.html)
- [ComputePathToPose](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ComputePathToPose.html)
- [FollowPath](https://docs.nav2.org/configuration/packages/bt-plugins/actions/FollowPath.html)
- [NavigateToPose](https://docs.nav2.org/configuration/packages/bt-plugins/actions/NavigateToPose.html)
- [ClearEntireCostmap](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ClearEntireCostmap.html)
- [ClearCostmapExceptRegion](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ClearCostmapExceptRegion.html)
- [ClearCostmapAroundRobot](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ClearCostmapAroundRobot.html)
- [ReinitializeGlobalLocalization](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ReinitializeGlobalLocalization.html)
- [TruncatePath](https://docs.nav2.org/configuration/packages/bt-plugins/actions/TruncatePath.html)
- [TruncatePathLocal](https://docs.nav2.org/configuration/packages/bt-plugins/actions/TruncatePathLocal.html)
- [PlannerSelector](https://docs.nav2.org/configuration/packages/bt-plugins/actions/PlannerSelector.html)
- [ControllerSelector](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ControllerSelector.html)
- [SmootherSelector](https://docs.nav2.org/configuration/packages/bt-plugins/actions/SmootherSelector.html)
- [GoalCheckerSelector](https://docs.nav2.org/configuration/packages/bt-plugins/actions/GoalCheckerSelector.html)
- [ProgressCheckerSelector](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ProgressCheckerSelector.html)
- [NavigateThroughPoses](https://docs.nav2.org/configuration/packages/bt-plugins/actions/NavigateThroughPoses.html)
- [ComputePathThroughPoses](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ComputePathThroughPoses.html)
- [ComputeCoveragePath](https://docs.nav2.org/configuration/packages/bt-plugins/actions/ComputeCoveragePath.html)
- [CancelCoverage](https://docs.nav2.org/configuration/packages/bt-plugins/actions/CancelCoverage.html)
- [RemovePassedGoals](https://docs.nav2.org/configuration/packages/bt-plugins/actions/RemovePassedGoals.html)
- [CancelControl](https://docs.nav2.org/configuration/packages/bt-plugins/actions/CancelControl.html)
- [CancelBackUp](https://docs.nav2.org/configuration/packages/bt-plugins/actions/CancelBackUp.html)
- [CancelSpin](https://docs.nav2.org/configuration/packages/bt-plugins/actions/CancelSpin.html)
- [CancelWait](https://docs.nav2.org/configuration/packages/bt-plugins/actions/CancelWait.html)
- [CancelDriveOnHeading](https://docs.nav2.org/configuration/packages/bt-plugins/actions/CancelDriveOnHeading.html)
- [CancelAssistedTeleop](https://docs.nav2.org/configuration/packages/bt-plugins/actions/CancelAssistedTeleop.html)
- [SmoothPath](https://docs.nav2.org/configuration/packages/bt-plugins/actions/Smooth.html)
- [GetPoseFromPath](https://docs.nav2.org/configuration/packages/bt-plugins/actions/GetPoseFromPath.html)

## Condition Plugins[¶](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html#condition-plugins)

- [GoalReached](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/GoalReached.html)
- [TransformAvailable](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/TransformAvailable.html)
- [DistanceTraveled](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/DistanceTraveled.html)
- [GoalUpdated](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/GoalUpdated.html)
- [GloballyUpdatedGoal](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/GloballyUpdatedGoal.html)
- [InitialPoseReceived](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/InitialPoseReceived.html)
- [IsStuck](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/IsStuck.html)
- [TimeExpired](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/TimeExpired.html)
- [IsBatteryLow](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/IsBatteryLow.html)
- [IsPathValid](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/IsPathValid.html)
- [PathExpiringTimer](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/PathExpiringTimer.html)
- [AreErrorCodesPresent](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/AreErrorCodesPresent.html)
- [WouldAControllerRecoveryHelp](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/WouldAControllerRecoveryHelp.html)
- [WouldAPlannerRecoveryHelp](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/WouldAPlannerRecoveryHelp.html)
- [WouldASmootherRecoveryHelp](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/WouldASmootherRecoveryHelp.html)
- [IsBatteryCharging](https://docs.nav2.org/configuration/packages/bt-plugins/conditions/IsBatteryCharging.html)

## Control Plugins[¶](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html#control-plugins)

- [PipelineSequence](https://docs.nav2.org/configuration/packages/bt-plugins/controls/PipelineSequence.html)
- [RoundRobin](https://docs.nav2.org/configuration/packages/bt-plugins/controls/RoundRobin.html)
- [RecoveryNode](https://docs.nav2.org/configuration/packages/bt-plugins/controls/RecoveryNode.html)

## Decorator Plugins[¶](https://docs.nav2.org/configuration/packages/configuring-bt-xml.html#decorator-plugins)

- [RateController](https://docs.nav2.org/configuration/packages/bt-plugins/decorators/RateController.html)
- [DistanceController](https://docs.nav2.org/configuration/packages/bt-plugins/decorators/DistanceController.html)
- [SpeedController](https://docs.nav2.org/configuration/packages/bt-plugins/decorators/SpeedController.html)
- [GoalUpdater](https://docs.nav2.org/configuration/packages/bt-plugins/decorators/GoalUpdater.html)
- [PathLongerOnApproach](https://docs.nav2.org/configuration/packages/bt-plugins/decorators/PathLongerOnApproach.html)
- [SingleTrigger](https://docs.nav2.org/configuration/packages/bt-plugins/decorators/SingleTrigger.html)



# Costmap 2D

https://docs.nav2.org/configuration/packages/configuring-costmaps.html

Costmap 2D 包实现了基于 2D 网格的 costmap，用于环境表示和许多传感器处理插件（AI 输出、深度传感器障碍物缓冲、语义信息等）它用于规划器和控制器服务器，用于创建用于检查碰撞或需要绕过的高成本区域的空间

### global_costmap 参数

1. **footprint_padding**
   - **中文名称**：足迹填充
   - **用途**：在计算机器人足迹时添加的额外填充量
   - **配置方法**：浮点数，例如 `0.03`

2. **update_frequency**
   - **中文名称**：更新频率
   - **用途**：全局代价图更新的频率（每秒更新次数）
   - **配置方法**：浮点数，例如 `1.0`

3. **publish_frequency**
   - **中文名称**：发布频率
   - **用途**：全局代价图发布的频率（每秒发布次数）
   - **配置方法**：浮点数，例如 `1.0`

4. **global_frame**
   - **中文名称**：全局坐标系
   - **用途**：全局代价图使用的坐标系
   - **配置方法**：字符串，常用值为 `map`

5. **robot_base_frame**
   - **中文名称**：机器人基座坐标系
   - **用途**：代价图中机器人位置计算所基于的坐标系
   - **配置方法**：字符串，常用值为 `base_link`

6. **use_sim_time**
   - **中文名称**：使用模拟时间
   - **用途**：是否使用模拟时间（仿真环境中常用）
   - **配置方法**：布尔值，常用值为 `True`

7. **robot_radius**
   - **中文名称**：机器人半径
   - **用途**：在没有设置机器人足迹的情况下，使用的机器人半径
   - **配置方法**：浮点数，例如 `0.22`

8. **resolution**
   - **中文名称**：分辨率
   - **用途**：代价图的分辨率，单位是米/像素
   - **配置方法**：浮点数，例如 `0.05`

9. **plugins**
   - **中文名称**：插件
   - **用途**：使用的代价图层插件列表
   - **配置方法**：字符串列表，常用值包括 `["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]`

### obstacle_layer 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定代价图层使用的插件
   - **配置方法**：字符串，常用值为 `"nav2_costmap_2d::ObstacleLayer"`

2. **observation_sources**
   - **中文名称**：观测源
   - **用途**：指定障碍物层观测数据的来源
   - **配置方法**：字符串，例如 `scan`

### voxel_layer 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定体素层使用的插件
   - **配置方法**：字符串，常用值为 `"nav2_costmap_2d::VoxelLayer"`

2. **observation_sources**
   - **中文名称**：观测源
   - **用途**：指定体素层观测数据的来源
   - **配置方法**：字符串，例如 `pointcloud`

### static_layer 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定静态层使用的插件
   - **配置方法**：字符串，常用值为 `"nav2_costmap_2d::StaticLayer"`

### inflation_layer 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定膨胀层使用的插件
   - **配置方法**：字符串，常用值为 `"nav2_costmap_2d::InflationLayer"`

### local_costmap 参数

1. **update_frequency**
   - **中文名称**：更新频率
   - **用途**：局部代价图更新的频率（每秒更新次数）
   - **配置方法**：浮点数，例如 `5.0`
2. **publish_frequency**
   - **中文名称**：发布频率
   - **用途**：局部代价图发布的频率（每秒发布次数）
   - **配置方法**：浮点数，例如 `2.0`
3. **global_frame**
   - **中文名称**：全局坐标系
   - **用途**：局部代价图使用的坐标系
   - **配置方法**：字符串，常用值为 `odom`
4. **rolling_window**
   - **中文名称**：滚动窗口
   - **用途**：是否使用滚动窗口模式
   - **配置方法**：布尔值，常用值为 `true`

## Plugin Parameters[¶](https://docs.nav2.org/configuration/packages/configuring-costmaps.html#plugin-parameters)

- [Static Layer Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/static.html)
- [Inflation Layer Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html)
- [Obstacle Layer Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html)
- [Voxel Layer Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html)
- [Range Sensor Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/range.html)
- [Denoise Layer Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/denoise.html)

```yaml
local_costmap:
  ros__parameters:
    plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
    voxel_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
```



## Costmap Filters Parameters[¶](https://docs.nav2.org/configuration/packages/configuring-costmaps.html#costmap-filters-parameters)

- [Keepout Filter Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/keepout_filter.html)
- [Speed Filter Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/speed_filter.html)
- [Binary Filter Parameters](https://docs.nav2.org/configuration/packages/costmap-plugins/binary_filter.html)

```yaml
local_costmap:
  ros__parameters:
    filters: ["keepout_filter", "speed_filter"]
    keepout_filter:
      plugin: "nav2_costmap_2d::KeepoutFilter"
    speed_filter:
      plugin: "nav2_costmap_2d::SpeedFilter"
```



```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      footprint_padding: 0.03
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.117 # radius set and used, so no footprint points
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        footprint_clearing_enabled: true
        max_obstacle_height: 2.0
        combination_method: 1
        scan:
          topic: /scan
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          inf_is_valid: false
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        footprint_clearing_enabled: true
        max_obstacle_height: 2.0
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        unknown_threshold: 15
        mark_threshold: 0
        observation_sources: pointcloud
        combination_method: 1
        pointcloud:  # no frame set, uses frame from message
          topic: /intel_realsense_r200_depth/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        enabled: true
        subscribe_to_updates: true
        transform_tolerance: 0.1
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.13
        cost_scaling_factor: 1.0
        inflate_unknown: false
        inflate_around_unknown: true
      always_send_full_costmap: True


local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
```

# 生命周期管理器

https://docs.nav2.org/configuration/packages/configuring-lifecycle.html

生命周期管理器模块实现了以确定性方式处理堆栈生命周期转换状态的方法它将接收一组有序节点，逐个转换为配置和激活状态以运行堆栈然后，它将以相反的顺序将堆栈降至最终状态它还将与服务器创建绑定连接，以确保它们仍然处于启动状态，并在任何节点无响应或崩溃时关闭所有节点

### lifecycle_manager 参数

1. **autostart**
   - **中文名称**：自动启动
   - **用途**：指定是否在启动时自动启动所有管理的节点
   - **配置方法**：布尔值，常用值为 `true`

2. **node_names**
   - **中文名称**：节点名称
   - **用途**：指定生命周期管理器管理的节点名称列表
   - **配置方法**：字符串列表，常用值包括 `['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower']`

3. **bond_timeout**
   - **中文名称**：绑定超时
   - **用途**：指定节点绑定（bond）的超时时间，超过此时间未建立绑定则认为节点失效
   - **配置方法**：浮点数，常用值为 `4.0` 秒

4. **attempt_respawn_reconnection**
   - **中文名称**：尝试重新连接
   - **用途**：指定是否尝试在节点失效后重新建立连接
   - **配置方法**：布尔值，常用值为 `true`

5. **bond_respawn_max_duration**
   - **中文名称**：绑定重新启动最大时长
   - **用途**：指定尝试重新启动节点的最大时长
   - **配置方法**：浮点数，常用值为 `10.0` 秒

这些参数的配置确保了生命周期管理器能够有效地监控和管理导航堆栈中的各个节点，确保系统的稳定性和可靠性在实际应用中，根据网络状况和节点性能，可能需要调整这些参数以优化系统行为

```yaml
lifecycle_manager:
  ros__parameters:
    autostart: true
    node_names: ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower']
    bond_timeout: 4.0
    attempt_respawn_reconnection: true
    bond_respawn_max_duration: 10.0
```



# 规划服务器

## 总体

https://docs.nav2.org/configuration/packages/configuring-planner-server.html

规划器服务器实现用于处理堆栈的规划器请求的服务器，并托管插件实现的映射它将接收目标和要使用的规划器插件名称，并调用适当的插件来计算到达目标的路径它还托管全局成本图

### planner_server 参数

1. **expected_planner_frequency**
   - **中文名称**：预期规划器频率
   - **用途**：指定规划器预期的运行频率（每秒规划次数）
   - **配置方法**：浮点数，常用值为 `20.0`

2. **planner_plugins**
   - **中文名称**：规划器插件
   - **用途**：指定使用的规划器插件名称列表
   - **配置方法**：字符串列表，常用值包括 `['GridBased']`

### GridBased 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的规划器插件
   - **配置方法**：字符串，常用值为 `'nav2_navfn_planner::NavfnPlanner'`在Iron及更早版本中，使用的是 `'nav2_navfn_planner/NavfnPlanner'`

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ['GridBased']
    GridBased:
      plugin: 'nav2_navfn_planner::NavfnPlanner' # In Iron and older versions, "/" was used instead of "::"
```

## NavFn规划器

https://docs.nav2.org/configuration/packages/configuring-navfn.html

### GridBased 插件参数

1. **use_astar**
   - **中文名称**：使用A*算法
   - **用途**：指定是否使用A*算法进行路径规划A*算法是一种启发式搜索算法，能够找到从起点到终点的有效路径
   - **配置方法**：布尔值，常用值为`True`

2. **allow_unknown**
   - **中文名称**：允许穿越未知区域
   - **用途**：指定规划器是否允许路径通过未知区域在某些情况下，允许路径规划穿越未知区域可能有助于找到目标路径，但这也可能带来风险
   - **配置方法**：布尔值，常用值为`True`

3. **tolerance**
   - **中文名称**：容忍度
   - **用途**：指定到达目标点的容忍度，以米为单位这意味着在规划路径时，如果最终路径与目标点的距离小于或等于容忍度，则认为已成功到达目标
   - **配置方法**：浮点数，例如`1.0`

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ['GridBased']
    GridBased:
      plugin: 'nav2_navfn_planner::NavfnPlanner' # In Iron and older versions, "/" was used instead of "::"
      use_astar: True
      allow_unknown: True
      tolerance: 1.0
```

## Smac 2D规划器

https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html

### planner_server 参数

1. **planner_plugins**
   - **中文名称**：规划器插件
   - **用途**：指定使用的规划器插件名称列表
   - **配置方法**：字符串列表，常用值包括 `["GridBased"]`

2. **use_sim_time**
   - **中文名称**：使用仿真时间
   - **用途**：指定是否使用仿真时间在仿真环境中，使用仿真时间可以同步所有节点的时钟
   - **配置方法**：布尔值，常用值为 `True`

### GridBased 插件参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的规划器插件
   - **配置方法**：字符串，常用值为 `"nav2_smac_planner::SmacPlanner2D"`在Iron及更早版本中，使用的是 `"nav2_smac_planner/SmacPlanner2D"`

2. **tolerance**
   - **中文名称**：容忍度
   - **用途**：指定到达目标点的容忍度，以米为单位
   - **配置方法**：浮点数，常用值为 `0.125`

3. **downsample_costmap**
   - **中文名称**：降采样代价地图
   - **用途**：指定是否对代价地图进行降采样
   - **配置方法**：布尔值，常用值为 `false`

4. **downsampling_factor**
   - **中文名称**：降采样因子
   - **用途**：指定代价地图的降采样因子
   - **配置方法**：整数，常用值为 `1`

5. **allow_unknown**
   - **中文名称**：允许穿越未知区域
   - **用途**：指定规划器是否允许路径通过未知区域
   - **配置方法**：布尔值，常用值为 `true`

6. **max_iterations**
   - **中文名称**：最大迭代次数
   - **用途**：指定规划器在搜索路径时的最大迭代次数
   - **配置方法**：整数，常用值为 `1000000`

7. **max_on_approach_iterations**
   - **中文名称**：接近目标最大迭代次数
   - **用途**：指定规划器在接近目标时的最大迭代次数
   - **配置方法**：整数，常用值为 `1000`

8. **max_planning_time**
   - **中文名称**：最大规划时间
   - **用途**：指定规划器的最大规划时间，以秒为单位
   - **配置方法**：浮点数，常用值为 `2.0`

9. **cost_travel_multiplier**
   - **中文名称**：代价旅行乘数
   - **用途**：指定搜索时应用于高代价区域的代价乘数
   - **配置方法**：浮点数，常用值为 `2.0`

10. **use_final_approach_orientation**
    - **中文名称**：使用最终接近方向
    - **用途**：指定是否在接近目标时使用最终路径姿态
    - **配置方法**：布尔值，常用值为 `false`

### smoother 参数

1. **max_iterations**
   - **中文名称**：最大迭代次数
   - **用途**：指定平滑器的最大迭代次数
   - **配置方法**：整数，常用值为 `1000`

2. **w_smooth**
   - **中文名称**：平滑权重
   - **用途**：指定平滑器的平滑权重
   - **配置方法**：浮点数，常用值为 `0.3`

3. **w_data**
   - **中文名称**：数据权重
   - **用途**：指定平滑器的数据权重
   - **配置方法**：浮点数，常用值为 `0.2`

4. **tolerance**
   - **中文名称**：容忍度
   - **用途**：指定平滑器的容忍度
   - **配置方法**：浮点数，常用值为 `1.0e-10`

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    use_sim_time: True

    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D" # In Iron and older versions, "/" was used instead of "::"
      tolerance: 0.125                      # tolerance for planning if unable to reach exact pose, in meters
      downsample_costmap: false             # whether or not to downsample the map
      downsampling_factor: 1                # multiplier for the resolution of the costmap layer (e.g. 2 on a 5cm costmap would be 10cm)
      allow_unknown: true                   # allow traveling in unknown space
      max_iterations: 1000000               # maximum total iterations to search for before failing (in case unreachable), set to -1 to disable
      max_on_approach_iterations: 1000      # maximum number of iterations to attempt to reach goal once in tolerance
      max_planning_time: 2.0                # max time in s for planner to plan, smooth
      cost_travel_multiplier: 2.0           # Cost multiplier to apply to search to steer away from high cost areas. Larger values will place in the center of aisles more exactly (if non-`FREE` cost potential field exists) but take slightly longer to compute. To optimize for speed, a value of 1.0 is reasonable. A reasonable tradeoff value is 2.0. A value of 0.0 effective disables steering away from obstacles and acts like a naive binary search A*.
      use_final_approach_orientation: false # Whether to set the final path pose at the goal's orientation to the requested orientation (false) or in line with the approach angle so the robot doesn't rotate to heading (true)
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
```

## Smac Hybrid-A*规划器

https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html

### planner_server 参数

1. **planner_plugins**
   - **中文名称**：规划器插件
   - **用途**：指定要使用的路径规划器插件列表
   - **配置方法**：设置为包含规划器插件名称的字符串列表，例如 `["GridBased"]`

2. **use_sim_time**
   - **中文名称**：使用仿真时间
   - **用途**：指定是否使用仿真时间
   - **配置方法**：布尔值，设置为 `True` 或 `False`

### GridBased 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定要使用的路径规划器插件的具体实现
   - **配置方法**：设置为插件的完整命名空间和类名，例如 `"nav2_smac_planner::SmacPlannerHybrid"`

2. **downsample_costmap**
   - **中文名称**：下采样代价地图
   - **用途**：指定是否对代价地图进行下采样
   - **配置方法**：布尔值，设置为 `true` 或 `false`

3. **downsampling_factor**
   - **中文名称**：下采样因子
   - **用途**：指定代价地图的下采样因子
   - **配置方法**：整数，例如 `1` 表示不进行下采样

4. **tolerance**
   - **中文名称**：容差
   - **用途**：指定目标位置的距离容差
   - **配置方法**：浮点数，例如 `0.25`

5. **allow_unknown**
   - **中文名称**：允许未知空间
   - **用途**：指定是否允许在未知空间中移动
   - **配置方法**：布尔值，设置为 `true` 或 `false`

6. **max_iterations**
   - **中文名称**：最大迭代次数
   - **用途**：指定路径规划的最大迭代次数
   - **配置方法**：整数，例如 `1000000`，设置为 `-1` 表示无限制

7. **max_on_approach_iterations**
   - **中文名称**：接近最大迭代次数
   - **用途**：指定在接近目标时的最大迭代次数
   - **配置方法**：整数，例如 `1000`

8. **max_planning_time**
   - **中文名称**：最大规划时间
   - **用途**：指定路径规划的最大时间
   - **配置方法**：浮点数，例如 `5.0` 秒

9. **motion_model_for_search**
   - **中文名称**：搜索运动模型
   - **用途**：指定用于搜索的运动模型
   - **配置方法**：字符串，例如 `"DUBIN"`

10. **angle_quantization_bins**
    - **中文名称**：角度量化箱数
    - **用途**：指定搜索时的角度量化箱数
    - **配置方法**：整数，例如 `72`

11. **analytic_expansion_ratio**
    - **中文名称**：解析扩展比率
    - **用途**：指定解析扩展的比率
    - **配置方法**：浮点数，例如 `3.5`

12. **analytic_expansion_max_length**
    - **中文名称**：解析扩展最大长度
    - **用途**：指定解析扩展的最大长度
    - **配置方法**：浮点数，例如 `3.0` 米

13. **analytic_expansion_max_cost**
    - **中文名称**：解析扩展最大成本
    - **用途**：指定解析扩展的最大成本
    - **配置方法**：浮点数，例如 `200.0`

14. **analytic_expansion_max_cost_override**
    - **中文名称**：解析扩展最大成本覆盖
    - **用途**：指定是否覆盖解析扩展的最大成本设置
    - **配置方法**：布尔值，设置为 `true` 或 `false`

15. **minimum_turning_radius**
    - **中文名称**：最小转弯半径
    - **用途**：指定路径或车辆的最小转弯半径
    - **配置方法**：浮点数，例如 `0.40` 米

16. **reverse_penalty**
    - **中文名称**：倒车惩罚
    - **用途**：指定倒车时的惩罚
    - **配置方法**：浮点数，例如 `2.0`

17. **change_penalty**
    - **中文名称**：方向改变惩罚
    - **用途**：指定方向改变时的惩罚
    - **配置方法**：浮点数，例如 `0.0`

18. **non_straight_penalty**
    - **中文名称**：非直线惩罚
    - **用途**：指定非直线移动时的惩罚
    - **配置方法**：浮点数，例如 `1.2`

19. **cost_penalty**
    - **中文名称**：成本惩罚
    - **用途**：指定高成本区域的惩罚
    - **配置方法**：浮点数，例如 `2.0`

20. **retrospective_penalty**
    - **中文名称**：回顾惩罚
    - **用途**：指定回顾惩罚
    - **配置方法**：浮点数，例如 `0.015`

21. **lookup_table_size**
    - **中文名称**：查找表大小
    - **用途**：指定Dubin/Reeds-Sheep距离窗口缓存的大小
    - **配置方法**：浮点数，例如 `20.0` 米

22. **cache_obstacle_heuristic**
    - **中文名称**：缓存障碍物启发式
    - **用途**：指定是否缓存障碍物地图动态编程距离扩展启发式
    - **配置方法**：布尔值，设置为 `true` 或 `false`

23. **debug_visualizations**
    - **中文名称**：调试可视化
    - **用途**：指定是否启用调试可视化
    - **配置方法**：布尔值，设置为 `true` 或 `false`

24. **use_quadratic_cost_penalty**
    - **中文名称**：使用二次成本惩罚
    - **用途**：指定是否使用二次成本惩罚
    - **配置方法**：布尔值，设置为 `true` 或 `false`

25. **downsample_obstacle_heuristic**
    - **中文名称**：下采样障碍物启发式
    - **用途**：指定是否对障碍物启发式进行下采样
    - **配置方法**：布尔值，设置为 `true` 或 `false`

26. **allow_primitive_interpolation**
    - **中文名称**：允许原始插值
    - **用途**：指定是否允许原始插值
    - **配置方法**：布尔值，设置为 `true` 或 `false`

27. **smooth_path**
    - **中文名称**：平滑路径
    - **用途**：指定是否对路径进行简单快速的平滑后处理
    - **配置方法**：布尔值，设置为 `true` 或 `false`

### smoother 参数

1. **max_iterations**
   - **中文名称**：最大迭代次数
   - **用途**：指定平滑器的最大迭代次数
   - **配置方法**：整数，例如 `1000`
2. **w_smooth**
   - **中文名称**：平滑权重
   - **用途**：指定平滑权重
   - **配置方法**：浮点数，例如 `0.3`
3. **w_data**
   - **中文名称**：数据权重
   - **用途**：指定数据权重
   - **配置方法**：浮点数，例如 `0.2`
4. **tolerance**
   - **中文名称**：容差
   - **用途**：指定平滑器的容差
   - **配置方法**：浮点数，例如 `1.0e-10`
5. **do_refinement**
   - **中文名称**：进行细化
   - **用途**：指定是否进行细化
   - **配置方法**：布尔值，设置为 `true` 或 `false`
6. **refinement_num**
   - **中文名称**：细化次数
   - **用途**：指定细化的次数
   - **配置方法**：整数，例如 `2`

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    use_sim_time: False

    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid" # In Iron and older versions, "/" was used instead of "::"
      downsample_costmap: false           # whether or not to downsample the map
      downsampling_factor: 1              # multiplier for the resolution of the costmap layer (e.g. 2 on a 5cm costmap would be 10cm)
      tolerance: 0.25                     # dist-to-goal heuristic cost (distance) for valid tolerance endpoints if exact goal cannot be found.
      allow_unknown: true                 # allow traveling in unknown space
      max_iterations: 1000000             # maximum total iterations to search for before failing (in case unreachable), set to -1 to disable
      max_on_approach_iterations: 1000    # Maximum number of iterations after within tolerances to continue to try to find exact solution
      max_planning_time: 10.0              # max time in s for planner to plan, smooth
      motion_model_for_search: "DUBIN"    # Hybrid-A* Dubin, Redds-Shepp
      angle_quantization_bins: 72         # Number of angle bins for search
      analytic_expansion_ratio: 3.5       # The ratio to attempt analytic expansions during search for final approach.
      analytic_expansion_max_length: 3.0  # For Hybrid/Lattice nodes: The maximum length of the analytic expansion to be considered valid to prevent unsafe shortcutting
      analytic_expansion_max_cost: 200.0  # The maximum single cost for any part of an analytic expansion to contain and be valid, except when necessary on approach to goal
      analytic_expansion_max_cost_override: false  #  Whether or not to override the maximum cost setting if within critical distance to goal (ie probably required)
      minimum_turning_radius: 0.40        # minimum turning radius in m of path / vehicle
      reverse_penalty: 2.0                # Penalty to apply if motion is reversing, must be => 1
      change_penalty: 0.0                 # Penalty to apply if motion is changing directions (L to R), must be >= 0
      non_straight_penalty: 1.2           # Penalty to apply if motion is non-straight, must be => 1
      cost_penalty: 2.0                   # Penalty to apply to higher cost areas when adding into the obstacle map dynamic programming distance expansion heuristic. This drives the robot more towards the center of passages. A value between 1.3 - 3.5 is reasonable.
      retrospective_penalty: 0.015
      lookup_table_size: 20.0             # Size of the dubin/reeds-sheep distance window to cache, in meters.
      cache_obstacle_heuristic: false     # Cache the obstacle map dynamic programming distance expansion heuristic between subsiquent replannings of the same goal location. Dramatically speeds up replanning performance (40x) if costmap is largely static.
      debug_visualizations: false         # For Hybrid nodes: Whether to publish expansions on the /expansions topic as an array of poses (the orientation has no meaning) and the path's footprints on the /planned_footprints topic. WARNING: heavy to compute and to display, for debug only as it degrades the performance.
      use_quadratic_cost_penalty: False
      downsample_obstacle_heuristic: True
      allow_primitive_interpolation: False
      smooth_path: True                   # If true, does a simple and quick smoothing post-processing to the path

      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
        do_refinement: true
        refinement_num: 2
```

## Smac State Lattice 规划器

https://docs.nav2.org/configuration/packages/smac/configuring-smac-lattice.html

### planner_server 参数

1. **planner_plugins**
   - **中文名称**：规划器插件
   - **用途**：指定使用的规划器插件名称列表
   - **配置方法**：字符串列表，常用值包括 `["GridBased"]`

2. **use_sim_time**
   - **中文名称**：使用仿真时间
   - **用途**：指定是否使用仿真时间在仿真环境中，使用仿真时间可以同步所有节点的时钟
   - **配置方法**：布尔值，常用值为 `True`

### GridBased 插件参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的规划器插件
   - **配置方法**：字符串，常用值为 `"nav2_smac_planner::SmacPlannerLattice"`在Iron及更早版本中，使用的是 `"nav2_smac_planner/SmacPlannerLattice"`

2. **allow_unknown**
   - **中文名称**：允许穿越未知区域
   - **用途**：指定规划器是否允许路径通过未知区域
   - **配置方法**：布尔值，常用值为 `true`

3. **tolerance**
   - **中文名称**：容忍度
   - **用途**：指定到达目标点的容忍度，以米为单位
   - **配置方法**：浮点数，常用值为 `0.25`

4. **max_iterations**
   - **中文名称**：最大迭代次数
   - **用途**：指定规划器在搜索路径时的最大迭代次数
   - **配置方法**：整数，常用值为 `1000000`

5. **max_on_approach_iterations**
   - **中文名称**：接近目标最大迭代次数
   - **用途**：指定规划器在接近目标时的最大迭代次数
   - **配置方法**：整数，常用值为 `1000`

6. **max_planning_time**
   - **中文名称**：最大规划时间
   - **用途**：指定规划器的最大规划时间，以秒为单位
   - **配置方法**：浮点数，常用值为 `5.0`

7. **analytic_expansion_ratio**
   - **中文名称**：分析扩展比率
   - **用途**：指定在搜索过程中尝试分析扩展的比率
   - **配置方法**：浮点数，常用值为 `3.5`

8. **analytic_expansion_max_length**
   - **中文名称**：分析扩展最大长度
   - **用途**：指定分析扩展的最大长度，以防止不安全的捷径
   - **配置方法**：浮点数，常用值为 `3.0`

9. **analytic_expansion_max_cost**
   - **中文名称**：分析扩展最大代价
   - **用途**：指定分析扩展中任何部分的单个最大代价
   - **配置方法**：浮点数，常用值为 `200.0`

10. **analytic_expansion_max_cost_override**
    - **中文名称**：分析扩展最大代价覆盖
    - **用途**：指定是否在接近目标的关键距离内覆盖最大代价设置
    - **配置方法**：布尔值，常用值为 `false`

11. **reverse_penalty**
    - **中文名称**：反向惩罚
    - **用途**：指定如果运动方向反转时应用的惩罚
    - **配置方法**：浮点数，常用值为 `2.0`

12. **change_penalty**
    - **中文名称**：方向改变惩罚
    - **用途**：指定如果运动方向改变时应用的惩罚
    - **配置方法**：浮点数，常用值为 `0.05`

13. **non_straight_penalty**
    - **中文名称**：非直线惩罚
    - **用途**：指定如果运动方向非直线时应用的惩罚
    - **配置方法**：浮点数，常用值为 `1.05`

14. **cost_penalty**
    - **中文名称**：代价惩罚
    - **用途**：指定在障碍物地图动态规划距离扩展启发式中应用于高代价区域的惩罚
    - **配置方法**：浮点数，常用值为 `2.0`

15. **rotation_penalty**
    - **中文名称**：旋转惩罚
    - **用途**：指定应用于原地旋转的惩罚
    - **配置方法**：浮点数，常用值为 `5.0`

16. **retrospective_penalty**
    - **中文名称**：回顾惩罚
    - **用途**：指定应用于回顾性分析的惩罚
    - **配置方法**：浮点数，常用值为 `0.015`

17. **lattice_filepath**
    - **中文名称**：状态格网文件路径
    - **用途**：指定状态格网图的文件路径
    - **配置方法**：字符串，常用值为 `""`

18. **lookup_table_size**
    - **中文名称**：查找表大小
    - **用途**：指定Dubin/Reeds-Sheep距离窗口缓存的大小，以米为单位
    - **配置方法**：浮点数，常用值为 `20.0`

19. **cache_obstacle_heuristic**
    - **中文名称**：缓存障碍物启发式
    - **用途**：指定是否在后续重规划中缓存障碍物地图动态规划距离扩展启发式
    - **配置方法**：布尔值，常用值为 `false`

20. **allow_reverse_expansion**
    - **中文名称**：允许反向扩展
    - **用途**：指定是否允许机器人使用基本单元在当前机器人方向的镜像相反方向（反向）进行扩展
    - **配置方法**：布尔值，常用值为 `false`

21. **smooth_path**
    - **中文名称**：平滑路径
    - **用途**：指定是否对路径进行简单的快速平滑后处理
    - **配置方法**：布尔值，常用值为 `True`

### smoother 参数

1. **max_iterations**
   - **中文名称**：最大迭代次数
   - **用途**：指定平滑器的最大迭代次数
   - **配置方法**：整数，常用值为 `1000`

2. **w_smooth**
   - **中文名称**：平滑权重
   - **用途**：指定平滑器的平滑权重
   - **配置方法**：浮点数，常用值为 `0.3`

3. **w_data**
   - **中文名称**：数据权重
   - **用途**：指定平滑器的数据权重
   - **配置方法**：浮点数，常用值为 `0.2`

4. **tolerance**
   - **中文名称**：容忍度
   - **用途**：指定平滑器的容忍度
   - **配置方法**：浮点数，常用值为 `1.0e-10`

5. **do_refinement**
   - **中文名称**：进行细化
   - **用途**：指定是否进行路径细化
   - **配置方法**：布尔值，常用值为 `true`

6. **refinement_num**
   - **中文名称**：细化次数
   - **用途**：指定路径细化的次数
   - **配置方法**：整数，常用值为 `2`

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    use_sim_time: True

    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerLattice" # In Iron and older versions, "/" was used instead of "::"
      allow_unknown: true                 # Allow traveling in unknown space
      tolerance: 0.25                     # dist-to-goal heuristic cost (distance) for valid tolerance endpoints if exact goal cannot be found.
      max_iterations: 1000000             # Maximum total iterations to search for before failing (in case unreachable), set to -1 to disable
      max_on_approach_iterations: 1000    # Maximum number of iterations after within tolerances to continue to try to find exact solution
      max_planning_time: 5.0              # Max time in s for planner to plan, smooth
      analytic_expansion_ratio: 3.5       # The ratio to attempt analytic expansions during search for final approach.
      analytic_expansion_max_length: 3.0  # For Hybrid/Lattice nodes The maximum length of the analytic expansion to be considered valid to prevent unsafe shortcutting
      analytic_expansion_max_cost: 200.0  # The maximum single cost for any part of an analytic expansion to contain and be valid, except when necessary on approach to goal
      analytic_expansion_max_cost_override: false  #  Whether or not to override the maximum cost setting if within critical distance to goal (ie probably required)
      reverse_penalty: 2.0                # Penalty to apply if motion is reversing, must be => 1
      change_penalty: 0.05                # Penalty to apply if motion is changing directions (L to R), must be >= 0
      non_straight_penalty: 1.05          # Penalty to apply if motion is non-straight, must be => 1
      cost_penalty: 2.0                   # Penalty to apply to higher cost areas when adding into the obstacle map dynamic programming distance expansion heuristic. This drives the robot more towards the center of passages. A value between 1.3 - 3.5 is reasonable.
      rotation_penalty: 5.0               # Penalty to apply to in-place rotations, if minimum control set contains them
      retrospective_penalty: 0.015
      lattice_filepath: ""                # The filepath to the state lattice graph
      lookup_table_size: 20.0             # Size of the dubin/reeds-sheep distance window to cache, in meters.
      cache_obstacle_heuristic: false     # Cache the obstacle map dynamic programming distance expansion heuristic between subsiquent replannings of the same goal location. Dramatically speeds up replanning performance (40x) if costmap is largely static.
      allow_reverse_expansion: false      # If true, allows the robot to use the primitives to expand in the mirrored opposite direction of the current robot's orientation (to reverse).
      smooth_path: True                   # If true, does a simple and quick smoothing post-processing to the path
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
        do_refinement: true
        refinement_num: 2
```

## Theta Star 规划器

https://docs.nav2.org/configuration/packages/configuring-thetastar.html

### planner_server 参数

1. **expected_planner_frequency**
   - **中文名称**：预期规划器频率
   - **用途**：指定规划器的预期运行频率，以赫兹（Hz）为单位
   - **配置方法**：浮点数，常用值为 `20.0`

2. **use_sim_time**
   - **中文名称**：使用仿真时间
   - **用途**：指定是否使用仿真时间在仿真环境中，使用仿真时间可以同步所有节点的时钟
   - **配置方法**：布尔值，常用值为 `True`

3. **planner_plugins**
   - **中文名称**：规划器插件
   - **用途**：指定使用的规划器插件名称列表
   - **配置方法**：字符串列表，常用值包括 `["GridBased"]`

### GridBased 插件参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的规划器插件
   - **配置方法**：字符串，常用值为 `"nav2_theta_star_planner::ThetaStarPlanner"`在Iron及更早版本中，使用的是 `"nav2_theta_star_planner/ThetaStarPlanner"`

2. **how_many_corners**
   - **中文名称**：角点数量
   - **用途**：指定在路径规划中考虑的角点数量
   - **配置方法**：整数，常用值为 `8`

3. **w_euc_cost**
   - **中文名称**：欧氏距离代价权重
   - **用途**：指定欧氏距离代价的权重
   - **配置方法**：浮点数，常用值为 `1.0`

4. **w_traversal_cost**
   - **中文名称**：遍历代价权重
   - **用途**：指定遍历代价的权重
   - **配置方法**：浮点数，常用值为 `2.0`

5. **w_heuristic_cost**
   - **中文名称**：启发式代价权重
   - **用途**：指定启发式代价的权重
   - **配置方法**：浮点数，常用值为 `1.0`

**注意**：

​	请仔细阅读此 repo 链接上提供的 README 文件，以更好地了解如何调整此规划器此规划器还要求您调整 costmap 的 cost_scaling_factor 参数，以获得良好的结果

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_theta_star_planner::ThetaStarPlanner" # In Iron and older versions, "/" was used instead of "::"
      how_many_corners: 8
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
      w_heuristic_cost: 1.0
```

# 控制服务器

## 总体

https://docs.nav2.org/configuration/packages/configuring-controller-server.html

控制器服务器实现用于处理堆栈控制器请求的服务器，并托管插件实现的映射它将接收控制器、进度检查器和目标检查器的路径和插件名称，并调用相应的插件它还托管本地成本图

### controller_server 参数

1. **use_sim_time**
   - **中文名称**：使用仿真时间
   - **用途**：指定是否使用仿真时间在仿真环境中，使用仿真时间可以同步所有节点的时钟
   - **配置方法**：布尔值，常用值为 `True`

2. **controller_frequency**
   - **中文名称**：控制器频率
   - **用途**：指定控制器的运行频率，以赫兹（Hz）为单位
   - **配置方法**：浮点数，常用值为 `20.0`

3. **min_x_velocity_threshold**
   - **中文名称**：最小X速度阈值
   - **用途**：指定X轴方向的最小速度阈值
   - **配置方法**：浮点数，常用值为 `0.001`

4. **min_y_velocity_threshold**
   - **中文名称**：最小Y速度阈值
   - **用途**：指定Y轴方向的最小速度阈值
   - **配置方法**：浮点数，常用值为 `0.5`

5. **min_theta_velocity_threshold**
   - **中文名称**：最小角速度阈值
   - **用途**：指定角速度的最小阈值
   - **配置方法**：浮点数，常用值为 `0.001`

6. **failure_tolerance**
   - **中文名称**：失败容忍度
   - **用途**：指定控制器在失败前的容忍度
   - **配置方法**：浮点数，常用值为 `0.3`

7. **odom_topic**
   - **中文名称**：里程计话题
   - **用途**：指定里程计数据的话题名称
   - **配置方法**：字符串，常用值为 `"odom"`

8. **progress_checker_plugins**
   - **中文名称**：进度检查器插件
   - **用途**：指定进度检查器插件名称列表
   - **配置方法**：字符串列表，常用值包括 `["progress_checker"]`

9. **goal_checker_plugins**
   - **中文名称**：目标检查器插件
   - **用途**：指定目标检查器插件名称列表
   - **配置方法**：字符串列表，常用值包括 `["goal_checker"]`

10. **controller_plugins**
    - **中文名称**：控制器插件
    - **用途**：指定控制器插件名称列表
    - **配置方法**：字符串列表，常用值包括 `["FollowPath"]`

### progress_checker 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的进度检查器插件
   - **配置方法**：字符串，常用值为 `"nav2_controller::SimpleProgressChecker"`

2. **required_movement_radius**
   - **中文名称**：所需移动半径
   - **用途**：指定所需移动的半径，以米为单位
   - **配置方法**：浮点数，常用值为 `0.5`

3. **movement_time_allowance**
   - **中文名称**：移动时间容差
   - **用途**：指定移动时间的容差，以秒为单位
   - **配置方法**：浮点数，常用值为 `10.0`

### goal_checker 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的目标检查器插件
   - **配置方法**：字符串，常用值为 `"nav2_controller::SimpleGoalChecker"`
2. **xy_goal_tolerance**
   - **中文名称**：XY目标容忍度
   - **用途**：指定XY平面上的目标容忍度，以米为单位
   - **配置方法**：浮点数，常用值为 `0.25`
3. **yaw_goal_tolerance**
   - **中文名称**：偏航目标容忍度
   - **用途**：指定偏航角的目标容忍度，以弧度为单位
   - **配置方法**：浮点数，常用值为 `0.25`
4. **stateful**
   - **中文名称**：状态性
   - **用途**：指定目标检查器是否具有状态性
   - **配置方法**：布尔值，常用值为 `True`



### Provided Plugins[¶](https://docs.nav2.org/configuration/packages/configuring-controller-server.html#provided-plugins)

> The plugins listed below are inside the `nav2_controller` namespace.

- [SimpleProgressChecker](https://docs.nav2.org/configuration/packages/nav2_controller-plugins/simple_progress_checker.html)
- [PoseProgressChecker](https://docs.nav2.org/configuration/packages/nav2_controller-plugins/pose_progress_checker.html)
- [SimpleGoalChecker](https://docs.nav2.org/configuration/packages/nav2_controller-plugins/simple_goal_checker.html)
- [StoppedGoalChecker](https://docs.nav2.org/configuration/packages/nav2_controller-plugins/stopped_goal_checker.html)



## FollowPath 插件参数

#### DWB

https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的路径跟随控制器插件
   - **配置方法**：字符串，常用值为 `"dwb_core::DWBLocalPlanner"`

2. **debug_trajectory_details**
   - **中文名称**：调试轨迹详细信息
   - **用途**：指定是否启用调试轨迹的详细信息
   - **配置方法**：布尔值，常用值为 `True`

3. **min_vel_x**
   - **中文名称**：最小X速度
   - **用途**：指定X轴方向的最小速度
   - **配置方法**：浮点数，常用值为 `0.0`

4. **min_vel_y**
   - **中文名称**：最小Y速度
   - **用途**：指定Y轴方向的最小速度
   - **配置方法**：浮点数，常用值为 `0.0`

5. **max_vel_x**
   - **中文名称**：最大X速度
   - **用途**：指定X轴方向的最大速度
   - **配置方法**：浮点数，常用值为 `0.26`

6. **max_vel_y**
   - **中文名称**：最大Y速度
   - **用途**：指定Y轴方向的最大速度
   - **配置方法**：浮点数，常用值为 `0.0`

7. **max_vel_theta**
   - **中文名称**：最大角速度
   - **用途**：指定角速度的最大值
   - **配置方法**：浮点数，常用值为 `1.0`

8. **min_speed_xy**
   - **中文名称**：最小XY速度
   - **用途**：指定XY平面上的最小速度
   - **配置方法**：浮点数，常用值为 `0.0`

9. **max_speed_xy**
   - **中文名称**：最大XY速度
   - **用途**：指定XY平面上的最大速度
   - **配置方法**：浮点数，常用值为 `0.26`

10. **min_speed_theta**
    - **中文名称**：最小角速度
    - **用途**：指定最小角速度
    - **配置方法**：浮点数，常用值为 `0.0`

11. **acc_lim_x**
    - **中文名称**：X方向加速度限制
    - **用途**：指定X轴方向的加速度限制
    - **配置方法**：浮点数，常用值为 `2.5`

12. **acc_lim_y**
    - **中文名称**：Y方向加速度限制
    - **用途**：指定Y轴方向的加速度限制
    - **配置方法**：浮点数，常用值为 `0.0`

13. **acc_lim_theta**
    - **中文名称**：角加速度限制
    - **用途**：指定角加速度的限制
    - **配置方法**：浮点数，常用值为 `3.2`

14. **decel_lim_x**
    - **中文名称**：X方向减速度限制
    - **用途**：指定X轴方向的减速度限制
    - **配置方法**：浮点数，常用值为 `-2.5`

15. **decel_lim_y**
    - **中文名称**：Y方向减速度限制
    - **用途**：指定Y轴方向的减速度限制
    - **配置方法**：浮点数，常用值为 `0.0`

16. **decel_lim_theta**
    - **中文名称**：角减速度限制
    - **用途**：指定角减速度的限制
    - **配置方法**：浮点数，常用值为 `-3.2`

17. **vx_samples**
    - **中文名称**：X方向速度采样
    - **用途**：指定在轨迹生成中X轴方向速度的采样数
    - **配置方法**：整数，常用值为 `20`

18. **vy_samples**
    - **中文名称**：Y方向速度采样
    - **用途**：指定在轨迹生成中Y轴方向速度的采样数
    - **配置方法**：整数，常用值为 `5`

19. **vtheta_samples**
    - **中文名称**：角速度采样
    - **用途**：指定在轨迹生成中角速度的采样数
    - **配置方法**：整数，常用值为 `20`

20. **sim_time**
    - **中文名称**：仿真时间
    - **用途**：指定轨迹生成的仿真时间
    - **配置方法**：浮点数，常用值为 `1.7`

21. **linear_granularity**
    - **中文名称**：线性粒度
    - **用途**：指定轨迹生成中的线性粒度
    - **配置方法**：浮点数，常用值为 `0.05`

22. **angular_granularity**
    - **中文名称**：角度粒度
    - **用途**：指定轨迹生成中的角度粒度
    - **配置方法**：浮点数，常用值为 `0.025`

23. **transform_tolerance**
    - **中文名称**：转换容忍度
    - **用途**：指定变换的容忍度
    - **配置方法**：浮点数，常用值为 `0.2`

24. **xy_goal_tolerance**
    - **中文名称**：XY目标容忍度
    - **用途**：指定XY平面上的目标容忍度
    - **配置方法**：浮点数，常用值为 `0.25`

25. **trans_stopped_velocity**
    - **中文名称**：平移停止速度
    - **用途**：指定平移停止时的速度
    - **配置方法**：浮点数，常用值为 `0.25`

26. **short_circuit_trajectory_evaluation**
    - **中文名称**：短路轨迹评估
    - **用途**：指定是否启用短路轨迹评估，以减少计算量
    - **配置方法**：布尔值，常用值为 `True`

27. **stateful**
    - **中文名称**：状态性
    - **用途**：指定控制器是否具有状态性
    - **配置方法**：布尔值，常用值为 `True`

##### critics 参数

1. **critics**
   - **中文名称**：批评器
   - **用途**：指定用于路径规划的批评器插件名称列表
   - **配置方法**：字符串列表，常用值包括 `["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]`

2. **BaseObstacle.scale**
   - **中文名称**：基本障碍物比例
   - **用途**：指定基本障碍物批评器的比例
   - **配置方法**：浮点数，常用值为 `0.02`

3. **PathAlign.scale**
   - **中文名称**：路径对齐比例
   - **用途**：指定路径对齐批评器的比例
   - **配置方法**：浮点数，常用值为 `32.0`

4. **GoalAlign.scale**
   - **中文名称**：目标对齐比例
   - **用途**：指定目标对齐批评器的比例
   - **配置方法**：浮点数，常用值为 `24.0`

5. **PathAlign.forward_point_distance**
   - **中文名称**：路径对齐前向点距离
   - **用途**：指定路径对齐批评器的前向点距离
   - **配置方法**：浮点数，常用值为 `0.1`

6. **GoalAlign.forward_point_distance**
   - **中文名称**：目标对齐前向点距离
   - **用途**：指定目标对齐批评器的前向点距离
   - **配置方法**：浮点数，常用值为 `0.1`

7. **PathDist.scale**
   - **中文名称**：路径距离比例
   - **用途**：指定路径距离批评器的比例
   - **配置方法**：浮点数，常用值为 `32.0`

8. **GoalDist.scale**
   - **中文名称**：目标距离比例
   - **用途**：指定目标距离批评器的比例
   - **配置方法**：浮点数，常用值为 `24.0`

9. **RotateToGoal.scale**
   - **中文名称**：旋转到目标比例
   - **用途**：指定旋转到目标批评器的比例
   - **配置方法**：浮点数，常用值为 `32.0`

10. **RotateToGoal.slowing_factor**
    - **中文名称**：旋转到目标减速因子
    - **用途**：指定旋转到目标批评器的减速因子
    - **配置方法**：浮点数，常用值为 `5.0`

11. **RotateToGoal.lookahead_time**
    - **中文名称**：旋转到目标前瞻时间
    - **用途**：指定旋转到目标批评器的前瞻时间
    - **配置方法**：浮点数，常用值为 `-1.0`



##### Controller[¶](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html#controller)

- [DWB Controller](https://docs.nav2.org/configuration/packages/dwb-params/controller.html)
- [XYTheta Iterator](https://docs.nav2.org/configuration/packages/dwb-params/iterator.html)
- [Kinematic Parameters](https://docs.nav2.org/configuration/packages/dwb-params/kinematic.html)
- [Publisher](https://docs.nav2.org/configuration/packages/dwb-params/visualization.html)

##### Plugins[¶](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html#plugins)

The plugins listed below are inside the `dwb_plugins` namespace.

- [LimitedAccelGenerator](https://docs.nav2.org/configuration/packages/dwb-plugins/limited_accel_generator.html)
- [StandardTrajectoryGenerator](https://docs.nav2.org/configuration/packages/dwb-plugins/standard_traj_generator.html)

##### Trajectory Critics[¶](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html#trajectory-critics)

The trajectory critics listed below are inside the `dwb_critics` namespace.

- [BaseObstacleCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/base_obstacle.html)
- [GoalAlignCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/goal_align.html)
- [GoalDistCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/goal_dist.html)
- [ObstacleFootprintCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/obstacle_footprint.html)
- [OscillationCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/oscillation.html)
- [PathAlignCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/path_align.html)
- [PathDistCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/path_dist.html)
- [PreferForwardCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/prefer_forward.html)
- [RotateToGoalCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/rotate_to_goal.html)
- [TwirlingCritic](https://docs.nav2.org/configuration/packages/trajectory_critics/twirling.html)



```yaml
controller_server:
  ros__parameters:
    # controller server parameters (see Controller Server for more info)
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    # DWB controller parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```



#### Regulated Pure Pursuit

https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html

受控的 Pure Pursuit 控制器实现了 Pure Pursuit  控制器的变体，专门针对服务/工业机器人的需求它通过路径曲率调节线性速度，以帮助减少盲角高速时的超调，从而使操作更加安全它还比 Pure  Pursuit  的任何其他变体更好地遵循路径它还具有启发式方法，可以在接近其他障碍物时减速，以便您可以在附近发生潜在碰撞时自动减慢机器人的速度它还实现了自适应前瞻点功能，可以通过速度进行缩放，从而在更大范围的平移速度下实现更稳定的行为

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的路径跟随控制器插件
   - **配置方法**：字符串，常用值为 `"nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"`

2. **desired_linear_vel**
   - **中文名称**：期望线速度
   - **用途**：指定期望的线速度
   - **配置方法**：浮点数，常用值为 `0.5`

3. **lookahead_dist**
   - **中文名称**：前瞻距离
   - **用途**：指定前瞻距离
   - **配置方法**：浮点数，常用值为 `0.6`

4. **min_lookahead_dist**
   - **中文名称**：最小前瞻距离
   - **用途**：指定最小前瞻距离
   - **配置方法**：浮点数，常用值为 `0.3`

5. **max_lookahead_dist**
   - **中文名称**：最大前瞻距离
   - **用途**：指定最大前瞻距离
   - **配置方法**：浮点数，常用值为 `0.9`

6. **lookahead_time**
   - **中文名称**：前瞻时间
   - **用途**：指定前瞻时间
   - **配置方法**：浮点数，常用值为 `1.5`

7. **rotate_to_heading_angular_vel**
   - **中文名称**：旋转到航向角速度
   - **用途**：指定旋转到航向的角速度
   - **配置方法**：浮点数，常用值为 `1.8`

8. **transform_tolerance**
   - **中文名称**：转换容忍度
   - **用途**：指定变换的容忍度
   - **配置方法**：浮点数，常用值为 `0.1`

9. **use_velocity_scaled_lookahead_dist**
   - **中文名称**：使用速度缩放前瞻距离
   - **用途**：指定是否使用速度缩放的前瞻距离
   - **配置方法**：布尔值，常用值为 `false`

10. **min_approach_linear_velocity**
    - **中文名称**：最小接近线速度
    - **用途**：指定最小接近线速度
    - **配置方法**：浮点数，常用值为 `0.05`

11. **approach_velocity_scaling_dist**
    - **中文名称**：接近速度缩放距离
    - **用途**：指定接近速度缩放距离
    - **配置方法**：浮点数，常用值为 `0.6`

12. **use_collision_detection**
    - **中文名称**：使用碰撞检测
    - **用途**：指定是否使用碰撞检测
    - **配置方法**：布尔值，常用值为 `true`

13. **max_allowed_time_to_collision_up_to_carrot**
    - **中文名称**：最大允许碰撞时间至胡萝卜点
    - **用途**：指定最大允许碰撞时间至胡萝卜点
    - **配置方法**：浮点数，常用值为 `1.0`

14. **use_regulated_linear_velocity_scaling**
    - **中文名称**：使用调节线速度缩放
    - **用途**：指定是否使用调节线速度缩放
    - **配置方法**：布尔值，常用值为 `true`

15. **use_fixed_curvature_lookahead**
    - **中文名称**：使用固定曲率前瞻
    - **用途**：指定是否使用固定曲率前瞻
    - **配置方法**：布尔值，常用值为 `false`

16. **curvature_lookahead_dist**
    - **中文名称**：曲率前瞻距离
    - **用途**：指定曲率前瞻距离
    - **配置方法**：浮点数，常用值为 `0.25`

17. **use_cost_regulated_linear_velocity_scaling**
    - **中文名称**：使用成本调节线速度缩放
    - **用途**：指定是否使用成本调节线速度缩放
    - **配置方法**：布尔值，常用值为 `false`

18. **regulated_linear_scaling_min_radius**
    - **中文名称**：调节线速度缩放最小半径
    - **用途**：指定调节线速度缩放的最小半径
    - **配置方法**：浮点数，常用值为 `0.9`

19. **regulated_linear_scaling_min_speed**
    - **中文名称**：调节线速度缩放最小速度
    - **用途**：指定调节线速度缩放的最小速度
    - **配置方法**：浮点数，常用值为 `0.25`

20. **use_rotate_to_heading**
    - **中文名称**：使用旋转到航向
    - **用途**：指定是否使用旋转到航向
    - **配置方法**：布尔值，常用值为 `true`

21. **allow_reversing**
    - **中文名称**：允许反向
    - **用途**：指定是否允许反向
    - **配置方法**：布尔值，常用值为 `false`

22. **rotate_to_heading_min_angle**
    - **中文名称**：旋转到航向最小角度
    - **用途**：指定旋转到航向的最小角度
    - **配置方法**：浮点数，常用值为 `0.785`

23. **max_angular_accel**
    - **中文名称**：最大角加速度
    - **用途**：指定最大角加速度
    - **配置方法**：浮点数，常用值为 `3.2`

24. **max_robot_pose_search_dist**
    - **中文名称**：最大机器人位姿搜索距离
    - **用途**：指定最大机器人位姿搜索距离
    - **配置方法**：浮点数，常用值为 `10.0`

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_fixed_curvature_lookahead: false
      curvature_lookahead_dist: 0.25
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      max_robot_pose_search_dist: 10.0
```

#### 模型预测路径积分控制器（MPPI）

https://docs.nav2.org/configuration/packages/configuring-mppic.html

MPPI 控制器实现了[模型预测路径积分控制器](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&u=https://ieeexplore.ieee.org/document/7487277)新的 Nav2 MPPI 控制器是一种预测控制器 - TEB 和纯路径跟踪 MPC 控制器的后继者它使用基于采样的方法来选择最佳轨迹，并在连续迭代之间进行优化它包含基于插件的目标函数，用于针对各种行为和行为属性进行定制和扩展

它目前适用于差动、全向和阿克曼机器人经测量，该控制器在中等英特尔处理器（第 4 代 i5）上以 50+ Hz 运行

MPPI 算法是 MPC  的变体，它使用迭代方法为机器人找到控制速度使用前一个时间步骤的最佳控制解决方案和机器人的当前状态，应用一组来自高斯分布的随机采样扰动这些噪声控制被正向模拟以在机器人的运动模型中生成一组轨迹接下来，使用一组基于插件的批评函数对这些轨迹进行评分，以找到批次中的最佳轨迹输出分数用于通过软最大函数设置最佳控制然后重复此过程多次并返回收敛解决方案然后将此解决方案用作下一个时间步骤的初始控制的基础

这项工作的一个强大成果是能够利用不需要凸也不需要可微的目标函数，从而为设计者的行为提供更大的灵活性

##### 用户须知[¶](https://docs-nav2-org.translate.goog/configuration/packages/configuring-mppic.html?_x_tr_sl=auto&_x_tr_tl=zh-CN&_x_tr_hl=zh-CN#notes-to-users)

###### 智慧箴言[¶](https://docs-nav2-org.translate.goog/configuration/packages/configuring-mppic.html?_x_tr_sl=auto&_x_tr_tl=zh-CN&_x_tr_hl=zh-CN#general-words-of-wisdom)

该`model_dt`参数通常应设置为控制频率的持续时间因此，如果您的控制频率是 20hz，则该参数应为`0.05`但是，您也可以将其设置得更低，**但不能设置得更大**

使用可视化轨迹`visualize`会使用计算资源来退出可视化轨迹，因此会减慢计算时间不建议在部署使用期间将此参数设置为`true`，但它是调整系统时有用的调试工具，但请谨慎使用以 30 hz 的频率可视化 56 个点的 2000 个批次*很多*

您可能想要开始更改的最常见参数是速度曲线（`vx_max`如果是完整的`vx_min`，，`wz_max`和`vy_max`）和`motion_model`与您的车辆相对应的明智的做法是考虑`prune_distance`路径规划的与您的最大速度和预测范围的比例唯一可能需要针对您的特定设置进行调整的更深层次的参数是障碍物批评者，`repulsion_weight`因为对此的调整与您的膨胀层半径成比例`repulsion_weight`由于惩罚形成（例如），较高的半径应该对应于减小如果这个惩罚太高，机器人从非成本空间进入成本空间时会显著变慢，或者在狭窄的走廊中抖动值得注意的是，但可能没有必要更改，如果，障碍物批评者可能会使用完整的足迹信息，但这会增加计算成本`inflation_radius - min_dist_to_obstacle``consider_footprint = true`

否则，这些参数已经由您友好的邻居导航员预先仔细调整，为您提供了一个不错的起点，希望您只需要稍微重新调整一下（如果有的话）即可实现您所需的特定行为改变成本地图参数或最大速度是最需要注意的操作，如下所述：

###### 预测范围、成本图大小和偏移[¶](https://docs-nav2-org.translate.goog/configuration/packages/configuring-mppic.html?_x_tr_sl=auto&_x_tr_tl=zh-CN&_x_tr_hl=zh-CN#prediction-horizon-costmap-sizing-and-offsets)

由于这是一个预测规划器，因此最大速度、预测时间和代价地图大小之间存在一些关系，用户在调整应用程序时应牢记这些关系如果将控制器服务器代价地图的大小设置为 3.0 米，则意味着以机器人为中心，机器人两侧的信息量为 1.5 米当最大速度 () 下的预测范围 ( ) 大于此范围时，您的机器人的最大速度和行为将受到代价地图限制的人为限制例如，如果您以 0.5 米/秒的最大速度预测前进 3 秒（60 步，每步 0.05 秒），则所需的最小**代价**地图半径为 1.5 米 - 或总宽度为 3 米`time_steps * model_dt``vx_max`

这同样适用于路径跟随和从最远点对齐偏移在同一个例子中，如果我们可以考虑的最远点已经在代价地图的边缘，那么进一步的偏移将被阈值化，因为它们不可用因此，在选择这些参数时，重要的是确保理论偏移可以存在于所选代价地图设置中，并使用最大预测范围和所需速度将路径跟随器 +  目标批评者中要考虑的阈值设置为与预测范围相同，可以确保它们之间有干净的交接，因为路径跟随器一旦达到作为其标记的最终目标姿势，就会尝试稍微减速

路径跟随批评者无法驱动大于该速度在滚动代价地图上可用路径上的可投影距离的速度路径对齐批评者offset_from_furthest表示轨迹在跟踪路径时经过的路径点数如果将其设置得过低（例如  5），则当机器人只是试图开始路径跟踪时，它可能会触发，从而导致在开始任务时出现一些次优行为和局部最小值如果相对于路径分辨率和代价地图大小将其设置得过高（例如 50），则批评者可能永远不会触发或仅在全速时触发这里的平衡是明智的将此值选择为投影的最大速度距离的 ~30% 是好的（例如，如果规划器每  2.5 厘米产生一个点，则 60 个点可以放在 1.5 米的局部代价地图半径上如果最大速度为 0.5 米/秒，预测时间为 3 秒，那么 20  个点代表在预测范围内投影到路径上的最大速度的 33%）如有疑问，这是一个很好的基准`prediction_horizon_s * max_speed / path_resolution / 3.0`

###### 障碍物、膨胀层和路径跟随[¶](https://docs-nav2-org.translate.goog/configuration/packages/configuring-mppic.html?_x_tr_sl=auto&_x_tr_tl=zh-CN&_x_tr_hl=zh-CN#obstacle-inflation-layer-and-path-following)

成本地图配置和障碍物批评者配置之间也存在关系如果障碍物批评者没有很好地调整成本地图参数（膨胀半径、比例），它可能会导致机器人在尝试以稍低的成本采取有限的低成本轨迹以换取急促的运动时出现明显摆动它还可能在自由空间中执行尴尬的动作，以尝试在 0  成本的小口袋中最大化时间，而不是更自然的运动，这涉及移动到一些低成本区域最后，如果增益设置得比路径跟随得分高得多，以鼓励机器人沿着路径移动，那么当从自由的 0 成本空间开始时，它通常可能拒绝进入成本空间这是因为留在自由空间的批评者成本比进入成本较低的空间以换取任务进展更具吸引力

因此，应谨慎选择障碍物评论家的权重以及代价地图膨胀半径和比例，以便机器人不会出现此类问题我（史蒂夫，您友好的邻居导航员）对此进行调整的方法是首先创建与膨胀层参数相结合的适当障碍物评论家行为值得注意的是，障碍物评论家将成本转换为与障碍物的距离，因此膨胀中成本分布的性质并不十分重要但是，膨胀半径和比例将定义分布末端的成本，其中自由空间与半径内的最低成本值相遇因此，应考虑在超过该阈值时测试质量行为

随着您增加或减少障碍物的权重，您可能会注意到上述行为（例如，无法克服自由到非自由的阈值）要克服这些问题，请增加 FollowPath  批评者成本，以增加轨迹规划器继续朝着目标前进的愿望但请确保不要超出这个范围，保持平衡理想的结果是大致在空间中心平稳运动，而不会与障碍物发生显着的近距离相互作用它不应该完美地遵循路径，输出速度也不应该参差不齐地摆动

一旦您调整了避障行为并将其与适当的路径跟随惩罚相匹配，就可以调整路径对齐批评者以与路径对齐如果您设计了精确路径对齐行为，则可以跳过障碍批评者步骤，因为高度调整系统以跟随路径将降低其偏离避开障碍物的能力（尽管它会减速并停止）将障碍批评者的批评者权重调整得较高将完成避免近距离碰撞的工作，但排斥权重对您来说基本上是不必要的对于其他想要更多动态行为的人来说，慢慢降低障碍批评者的权重以给路径对齐批评者更多的工作空间*可能会*有所帮助如果您的路径是使用成本感知规划器（如 Nav2  提供的所有规划器）生成的，并且提供的路径足够远离障碍物以满足您的要求，那么使用路径对齐批评者略微减少障碍批评者的影响将对您有好处不过度加权路径对齐批评器将允许机器人偏离路径以绕过场景中的动态障碍物或路径规划期间之前未考虑的其他障碍物对于您的应用程序而言，最佳行为是主观的，但事实证明，MPPI 可以成为精确的路径跟踪器和/或非常流畅地避开动态障碍物提供的默认值通常处于平衡初始权衡的正确状态

##### controller_server 参数

1. **controller_frequency**
   - **中文名称**：控制器频率
   - **用途**：指定控制器的运行频率，以赫兹（Hz）为单位
   - **配置方法**：浮点数，常用值为 `30.0`

##### FollowPath 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的路径跟随控制器插件
   - **配置方法**：字符串，常用值为 `"nav2_mppi_controller::MPPIController"`

2. **time_steps**
   - **中文名称**：时间步数
   - **用途**：指定MPPI控制器的时间步数
   - **配置方法**：整数，常用值为 `56`

3. **model_dt**
   - **中文名称**：模型时间步长
   - **用途**：指定模型的时间步长
   - **配置方法**：浮点数，常用值为 `0.05`

4. **batch_size**
   - **中文名称**：批量大小
   - **用途**：指定MPPI控制器的批量大小
   - **配置方法**：整数，常用值为 `2000`

5. **vx_std**
   - **中文名称**：X方向速度标准差
   - **用途**：指定X方向速度的标准差
   - **配置方法**：浮点数，常用值为 `0.2`

6. **vy_std**
   - **中文名称**：Y方向速度标准差
   - **用途**：指定Y方向速度的标准差
   - **配置方法**：浮点数，常用值为 `0.2`

7. **wz_std**
   - **中文名称**：角速度标准差
   - **用途**：指定角速度的标准差
   - **配置方法**：浮点数，常用值为 `0.4`

8. **vx_max**
   - **中文名称**：最大X方向速度
   - **用途**：指定X方向的最大速度
   - **配置方法**：浮点数，常用值为 `0.5`

9. **vx_min**
   - **中文名称**：最小X方向速度
   - **用途**：指定X方向的最小速度
   - **配置方法**：浮点数，常用值为 `-0.35`

10. **vy_max**
    - **中文名称**：最大Y方向速度
    - **用途**：指定Y方向的最大速度
    - **配置方法**：浮点数，常用值为 `0.5`

11. **wz_max**
    - **中文名称**：最大角速度
    - **用途**：指定最大角速度
    - **配置方法**：浮点数，常用值为 `1.9`

12. **ax_max**
    - **中文名称**：最大X方向加速度
    - **用途**：指定X方向的最大加速度
    - **配置方法**：浮点数，常用值为 `3.0`

13. **ax_min**
    - **中文名称**：最小X方向加速度
    - **用途**：指定X方向的最小加速度
    - **配置方法**：浮点数，常用值为 `-3.0`

14. **ay_max**
    - **中文名称**：最大Y方向加速度
    - **用途**：指定Y方向的最大加速度
    - **配置方法**：浮点数，常用值为 `3.0`

15. **az_max**
    - **中文名称**：最大角加速度
    - **用途**：指定最大角加速度
    - **配置方法**：浮点数，常用值为 `3.5`

16. **iteration_count**
    - **中文名称**：迭代次数
    - **用途**：指定MPPI控制器的迭代次数
    - **配置方法**：整数，常用值为 `1`

17. **prune_distance**
    - **中文名称**：修剪距离
    - **用途**：指定修剪距离
    - **配置方法**：浮点数，常用值为 `1.7`

18. **transform_tolerance**
    - **中文名称**：转换容忍度
    - **用途**：指定变换的容忍度
    - **配置方法**：浮点数，常用值为 `0.1`

19. **temperature**
    - **中文名称**：温度
    - **用途**：指定MPPI控制器的温度参数
    - **配置方法**：浮点数，常用值为 `0.3`

20. **gamma**
    - **中文名称**：伽马
    - **用途**：指定MPPI控制器的伽马参数
    - **配置方法**：浮点数，常用值为 `0.015`

21. **motion_model**
    - **中文名称**：运动模型
    - **用途**：指定使用的运动模型
    - **配置方法**：字符串，常用值为 `"DiffDrive"`

22. **visualize**
    - **中文名称**：可视化
    - **用途**：指定是否启用可视化
    - **配置方法**：布尔值，常用值为 `false`

23. **reset_period**
    - **中文名称**：重置周期
    - **用途**：指定重置周期
    - **配置方法**：浮点数，常用值为 `1.0`

24. **regenerate_noises**
    - **中文名称**：重新生成噪声
    - **用途**：指定是否重新生成噪声
    - **配置方法**：布尔值，常用值为 `false`

##### TrajectoryVisualizer 参数

1. **trajectory_step**
   - **中文名称**：轨迹步长
   - **用途**：指定轨迹步长
   - **配置方法**：整数，常用值为 `5`

2. **time_step**
   - **中文名称**：时间步长
   - **用途**：指定时间步长
   - **配置方法**：整数，常用值为 `3`

##### AckermannConstraints 参数

1. **min_turning_r**
   - **中文名称**：最小转弯半径
   - **用途**：指定最小转弯半径
   - **配置方法**：浮点数，常用值为 `0.2`

##### critics 参数

1. **critics**
   - **中文名称**：批评器
   - **用途**：指定用于路径规划的批评器插件名称列表
   - **配置方法**：字符串列表，常用值包括 `["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]`

2. **ConstraintCritic**
   - **enabled**
     - **中文名称**：启用
     - **用途**：指定是否启用约束批评器
     - **配置方法**：布尔值，常用值为 `true`
   - **cost_power**
     - **中文名称**：成本幂
     - **用途**：指定成本幂
     - **配置方法**：整数，常用值为 `1`
   - **cost_weight**
     - **中文名称**：成本权重
     - **用途**：指定成本权重
     - **配置方法**：浮点数，常用值为 `4.0`

3. **GoalCritic**
   - **enabled**
     - **中文名称**：启用
     - **用途**：指定是否启用目标批评器
     - **配置方法**：布尔值，常用值为 `true`
   - **cost_power**
     - **中文名称**：成本幂
     - **用途**：指定成本幂
     - **配置方法**：整数，常用值为 `1`
   - **cost_weight**
     - **中文名称**：成本权重
     - **用途**：指定成本权重
     - **配置方法**：浮点数，常用值为 `5.0`
   - **threshold_to_consider**
     - **中文名称**：考虑阈值
     - **用途**：指定考虑阈值
     - **配置方法**：浮点数，常用值为 `1.4`

4. **GoalAngleCritic**
   - **enabled**
     - **中文名称**：启用
     - **用途**：指定是否启用目标角度批评器
     - **配置方法**：布尔值，常用值为 `true`
   - **cost_power**
     - **中文名称**：成本幂
     - **用途**：指定成本幂
     - **配置方法**：整数，常用值为 `1`
   - **cost_weight**
     - **中文名称**：成本权重
     - **用途**：指定成本权重
     - **配置方法**：浮点数，常用值为 `3.0`
   - **threshold_to_consider**
     - **中文名称**：考虑阈值
     - **用途**：指定考虑阈值
     - **配置方法**：浮点数，常用值为 `0.5`

5. **PreferForwardCritic**
   - **enabled**
     - **中文名称**：启用
     - **用途**：指定是否启用偏好前向批评器
     - **配置方法**：布尔值，常用值为 `true`
   - **cost_power**
     - **中文名称**：成本幂
     - **用途**：指定成本幂
     - **配置方法**：整数，常用值为 `1`
   - **cost_weight**
     - **中文名称**：成本权重
     - **用途**：指定成本权重
     - **配置方法**：浮点数，常用值为 `5.0`
   - **threshold_to_consider**
     - **中文名称**：考虑阈值
     - **用途**：指定考虑阈值
     - **配置方法**：浮点数，常用值为 `0.5`

6. **CostCritic**
   - **enabled**
     - **中文名称**：启用
     - **用途**：指定是否启用成本批评器
     - **配置方法**：布尔值，常用值为 `true`
   - **cost_power**
     - **中文名称**：成本幂
     - **用途**：指定成本幂
     - **配置方法**：整数，常用值为 `1`
   - **cost_weight**
     - **中文名称**：成本权重
     - **用途**：指定成本权重
     - **配置方法**：浮点数，常用值为 `3.81`
   - **critical_cost**
     - **中文名称**：关键成本
     - **用途**：指定关键成本
     - **配置方法**：浮点数，常用值为 `300.0`
   - **consider_footprint**
     - **中文名称**：考虑足迹
     - **用途**：指定是否考虑足迹
     - **配置方法**：布尔值，常用值为 `true`
   - **collision_cost**
     - **中文名称**：碰撞成本
     - **用途**：指定碰撞成本
     - **配置方法**：浮点数，常用值为 `1000000.0`
   - **near_goal_distance**
     - **中文名称**：接近目标距离
     - **用途**：指定接近目标距离
     - **配置方法**：浮点数，常用值为 `1.0`
   - **trajectory_point_step**
     - **中文名称**：轨迹点步长
     - **用途**：指定轨迹点步长
     - **配置方法**：整数，常用值为 `2`

7. **PathAlignCritic**
   - **enabled**
     - **中文名称**：启用
     - **用途**：指定是否启用路径对齐批评器
     - **配置方法**：布尔值，常用值为 `true`
   - **cost_power**
     - **中文名称**：成本幂
     - **用途**：指定成本幂
     - **配置方法**：整数，常用值为 `1`
   - **cost_weight**
     - **中文名称**：成本权重
     - **用途**：指定成本权重
     - **配置方法**：浮点数，常用值为 `14.0`
   - **max_path_occupancy_ratio**
     - **中文名称**：最大路径占用比
     - **用途**：指定最大路径占用比
     - **配置方法**：浮点数，常用值为 `0.05`
   - **trajectory_point_step**
     - **中文名称**：轨迹点步长
     - **用途**：指定轨迹点步长
     - **配置方法**：整数，常用值为 `4`
   - **threshold_to_consider**
     - **中文名称**：考虑阈值
     - **用途**：指定考虑阈值
     - **配置方法**：浮点数，常用值为 `0.5`
   - **offset_from_furthest**
     - **中文名称**：从最远点偏移
     - **用途**：指定从最远点的偏移
     - **配置方法**：整数，常用值为 `20`
   - **use_path_orientations**
     - **中文名称**：使用路径方向
     - **用途**：指定是否使用路径方向
     - **配置方法**：布尔值，常用值为 `false`

8. **PathFollowCritic**
   - **enabled**
     - **中文名称**：启用
     - **用途**：指定是否启用路径跟随批评器
     - **配置方法**：布尔值，常用值为 `true`
   - **cost_power**
     - **中文名称**：成本幂
     - **用途**：指定成本幂
     - **配置方法**：整数，常用值为 `1`
   - **cost_weight**
     - **中文名称**：成本权重
     - **用途**：指定成本权重
     - **配置方法**：浮点数，常用值为 `5.0`
   - **offset_from_furthest**
     - **中文名称**：从最远点偏移
     - **用途**：指定从最远点的偏移
     - **配置方法**：整数，常用值为 `5`
   - **threshold_to_consider**
     - **中文名称**：考虑阈值
     - **用途**：指定考虑阈值
     - **配置方法**：浮点数，常用值为 `1.4`

9. **PathAngleCritic**
   - **enabled**
     - **中文名称**：启用
     - **用途**：指定是否启用路径角度批评器
     - **配置方法**：布尔值，常用值为 `true`
   - **cost_power**
     - **中文名称**：成本幂
     - **用途**：指定成本幂
     - **配置方法**：整数，常用值为 `1`
   - **cost_weight**
     - **中文名称**：成本权重
     - **用途**：指定成本权重
     - **配置方法**：浮点数，常用值为 `2.0`
   - **offset_from_furthest**
     - **中文名称**：从最远点偏移
     - **用途**：指定从最远点的偏移
     - **配置方法**：整数，常用值为 `4`
   - **threshold_to_consider**
     - **中文名称**：考虑阈值
     - **用途**：指定考虑阈值
     - **配置方法**：浮点数，常用值为 `0.5`
   - **max_angle_to_furthest**
     - **中文名称**：最大角度到最远点
     - **用途**：指定最大角度到最远点
     - **配置方法**：浮点数，常用值为 `1.0`
   - **mode**
     - **中文名称**：模式
     - **用途**：指定模式
     - **配置方法**：整数，常用值为 `0`



```yaml
controller_server:
  ros__parameters:
    controller_frequency: 30.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.5
      wz_max: 1.9
      ax_max: 3.0
      ax_min: -3.0
      ay_max: 3.0
      az_max: 3.5
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: false
      reset_period: 1.0 # (only in Humble)
      regenerate_noises: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      AckermannConstraints:
        min_turning_r: 0.2
      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
      # ObstaclesCritic:
      #   enabled: true
      #   cost_power: 1
      #   repulsion_weight: 1.5
      #   critical_weight: 20.0
      #   consider_footprint: false
      #   collision_cost: 10000.0
      #   collision_margin_distance: 0.1
      #   near_goal_distance: 0.5
      #   inflation_radius: 0.55 # (only in Humble)
      #   cost_scaling_factor: 10.0 # (only in Humble)
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        critical_cost: 300.0
        consider_footprint: true
        collision_cost: 1000000.0
        near_goal_distance: 1.0
        trajectory_point_step: 2
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0
      # VelocityDeadbandCritic:
      #   enabled: true
      #   cost_power: 1
      #   cost_weight: 35.0
      #   deadband_velocities: [0.05, 0.05, 0.05]
      # TwirlingCritic:
      #   enabled: true
      #   twirling_cost_power: 1
      #   twirling_cost_weight: 10.0
```



#### 旋转垫片控制器（RotationShimController）

https://docs.nav2.org/configuration/packages/configuring-rotation-shim-controller.html

将`nav2_rotation_shim_controller`检查机器人与新接收路径之间的粗略航向差异如果在阈值范围内，它将把请求传递给`primary_controller`执行任务如果超出阈值，该控制器将让机器人原地旋转，朝着该路径航向一旦在公差范围内，它将把控制执行从旋转垫片控制器传递到主控制器插件此时，机器人的主插件将接管控制，以便顺利交接任务

当`rotate_to_goal_heading`参数设置为 true 时，此控制器还能够在达到目标检查器的 XY 目标公差时收回对机器人的控制在这种情况下，机器人将朝目标方向旋转，直到目标检查器验证目标并结束当前导航任务

最`RotationShimController`适合：

- 可以原地旋转的机器人，如差速机器人、全向机器人等
- 当开始跟踪与机器人当前航向有明显不同航向的新路径时，或者当调整控制器以执行其任务使紧密旋转变得困难时，倾向于在原地旋转
- 使用非运动学上可行的规划器，例如 NavFn、Theta* 或 Smac 2D（可行的规划器，例如 Smac Hybrid-A* 和 State Lattice 将从机器人的实际起始航向开始搜索，不需要旋转，因为它们的路径由物理约束保证可驾驶）

###### FollowPath 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的路径跟随控制器插件
   - **配置方法**：字符串，常用值为 `"nav2_rotation_shim_controller::RotationShimController"`

2. **primary_controller**
   - **中文名称**：主要控制器
   - **用途**：指定主要控制器插件
   - **配置方法**：字符串，常用值为 `"nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"`

3. **angular_dist_threshold**
   - **中文名称**：角度距离阈值
   - **用途**：指定角度距离阈值，以弧度为单位
   - **配置方法**：浮点数，常用值为 `0.785`

4. **forward_sampling_distance**
   - **中文名称**：前向采样距离
   - **用途**：指定前向采样距离，以米为单位
   - **配置方法**：浮点数，常用值为 `0.5`

5. **rotate_to_heading_angular_vel**
   - **中文名称**：旋转到航向角速度
   - **用途**：指定旋转到航向的角速度，以弧度/秒为单位
   - **配置方法**：浮点数，常用值为 `1.8`

6. **max_angular_accel**
   - **中文名称**：最大角加速度
   - **用途**：指定最大角加速度，以弧度/秒²为单位
   - **配置方法**：浮点数，常用值为 `3.2`

7. **simulate_ahead_time**
   - **中文名称**：模拟前瞻时间
   - **用途**：指定模拟前瞻时间，以秒为单位
   - **配置方法**：浮点数，常用值为 `1.0`

8. **rotate_to_goal_heading**
   - **中文名称**：旋转到目标航向
   - **用途**：指定是否旋转到目标航向
   - **配置方法**：布尔值，常用值为 `false`

###### 主要控制器参数（Primary Controller Params）

这些参数是针对主要控制器`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`的配置

1. **desired_linear_vel**
   - **中文名称**：期望线速度
   - **用途**：指定期望的线速度
   - **配置方法**：浮点数，常用值为 `0.5`

2. **lookahead_dist**
   - **中文名称**：前瞻距离
   - **用途**：指定前瞻距离
   - **配置方法**：浮点数，常用值为 `0.6`

3. **min_lookahead_dist**
   - **中文名称**：最小前瞻距离
   - **用途**：指定最小前瞻距离
   - **配置方法**：浮点数，常用值为 `0.3`

4. **max_lookahead_dist**
   - **中文名称**：最大前瞻距离
   - **用途**：指定最大前瞻距离
   - **配置方法**：浮点数，常用值为 `0.9`

5. **lookahead_time**
   - **中文名称**：前瞻时间
   - **用途**：指定前瞻时间
   - **配置方法**：浮点数，常用值为 `1.5`

6. **transform_tolerance**
   - **中文名称**：转换容忍度
   - **用途**：指定变换的容忍度
   - **配置方法**：浮点数，常用值为 `0.1`

7. **use_velocity_scaled_lookahead_dist**
   - **中文名称**：使用速度缩放前瞻距离
   - **用途**：指定是否使用速度缩放的前瞻距离
   - **配置方法**：布尔值，常用值为 `false`

8. **min_approach_linear_velocity**
   - **中文名称**：最小接近线速度
   - **用途**：指定最小接近线速度
   - **配置方法**：浮点数，常用值为 `0.05`

9. **approach_velocity_scaling_dist**
   - **中文名称**：接近速度缩放距离
   - **用途**：指定接近速度缩放距离
   - **配置方法**：浮点数，常用值为 `0.6`

10. **use_collision_detection**
    - **中文名称**：使用碰撞检测
    - **用途**：指定是否使用碰撞检测
    - **配置方法**：布尔值，常用值为 `true`

11. **max_allowed_time_to_collision_up_to_carrot**
    - **中文名称**：最大允许碰撞时间至胡萝卜点
    - **用途**：指定最大允许碰撞时间至胡萝卜点
    - **配置方法**：浮点数，常用值为 `1.0`

12. **use_regulated_linear_velocity_scaling**
    - **中文名称**：使用调节线速度缩放
    - **用途**：指定是否使用调节线速度缩放
    - **配置方法**：布尔值，常用值为 `true`

13. **use_fixed_curvature_lookahead**
    - **中文名称**：使用固定曲率前瞻
    - **用途**：指定是否使用固定曲率前瞻
    - **配置方法**：布尔值，常用值为 `false`

14. **curvature_lookahead_dist**
    - **中文名称**：曲率前瞻距离
    - **用途**：指定曲率前瞻距离
    - **配置方法**：浮点数，常用值为 `0.25`

15. **use_cost_regulated_linear_velocity_scaling**
    - **中文名称**：使用成本调节线速度缩放
    - **用途**：指定是否使用成本调节线速度缩放
    - **配置方法**：布尔值，常用值为 `false`

16. **regulated_linear_scaling_min_radius**
    - **中文名称**：调节线速度缩放最小半径
    - **用途**：指定调节线速度缩放的最小半径
    - **配置方法**：浮点数，常用值为 `0.9`

17. **regulated_linear_scaling_min_speed**
    - **中文名称**：调节线速度缩放最小速度
    - **用途**：指定调节线速度缩放的最小速度
    - **配置方法**：浮点数，常用值为 `0.25`

18. **use_rotate_to_heading**
    - **中文名称**：使用旋转到航向
    - **用途**：指定是否使用旋转到航向
    - **配置方法**：布尔值，常用值为 `true`

19. **allow_reversing**
    - **中文名称**：允许反向
    - **用途**：指定是否允许反向
    - **配置方法**：布尔值，常用值为 `false`

20. **rotate_to_heading_min_angle**
    - **中文名称**：旋转到航向最小角度
    - **用途**：指定旋转到航向的最小角度
    - **配置方法**：浮点数，常用值为 `0.785`

21. **max_angular_accel**
    - **中文名称**：最大角加速度
    - **用途**：指定最大角加速度
    - **配置方法**：浮点数，常用值为 `3.2`

22. **max_robot_pose_search_dist**
    - **中文名称**：最大机器人位姿搜索距离
    - **用途**：指定最大机器人位姿搜索距离
    - **配置方法**：浮点数，常用值为 `10.0`

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      angular_dist_threshold: 0.785
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.8
      max_angular_accel: 3.2
      simulate_ahead_time: 1.0
      rotate_to_goal_heading: false

      # Primary controller params can be placed here below
      # ...
```

#### 优雅控制器（GracefulController）

https://docs.nav2.org/configuration/packages/configuring-graceful-motion-controller.html

优雅控制器实现了基于 Jong Jin Park 和 Benjamin Kuipers 在“2D 环境中差速轮式移动机器人优雅运动的平滑控制律”(ICRA 2011) 中的工作的控制器在此实现中，motion_target设置在距离机器人一定距离处，该距离呈指数稳定，以生成机器人要遵循的平滑轨迹

###### FollowPath 参数

1. **plugin**
   - **中文名称**：插件
   - **用途**：指定具体使用的路径跟随控制器插件
   - **配置方法**：字符串，常用值为 `"nav2_graceful_controller::GracefulController"`

2. **transform_tolerance**
   - **中文名称**：转换容忍度
   - **用途**：指定变换的容忍度
   - **配置方法**：浮点数，常用值为 `0.1`

3. **motion_target_dist**
   - **中文名称**：运动目标距离
   - **用途**：指定运动目标距离
   - **配置方法**：浮点数，常用值为 `0.6`

4. **initial_rotation**
   - **中文名称**：初始旋转
   - **用途**：指定是否进行初始旋转
   - **配置方法**：布尔值，常用值为 `true`

5. **initial_rotation_min_angle**
   - **中文名称**：初始旋转最小角度
   - **用途**：指定初始旋转的最小角度
   - **配置方法**：浮点数，常用值为 `0.75`

6. **final_rotation**
   - **中文名称**：最终旋转
   - **用途**：指定是否进行最终旋转
   - **配置方法**：布尔值，常用值为 `true`

7. **allow_backward**
   - **中文名称**：允许反向
   - **用途**：指定是否允许反向
   - **配置方法**：布尔值，常用值为 `false`

8. **k_phi**
   - **中文名称**：k_phi 增益
   - **用途**：指定 k_phi 增益
   - **配置方法**：浮点数，常用值为 `3.0`

9. **k_delta**
   - **中文名称**：k_delta 增益
   - **用途**：指定 k_delta 增益
   - **配置方法**：浮点数，常用值为 `2.0`

10. **beta**
    - **中文名称**：beta 参数
    - **用途**：指定 beta 参数
    - **配置方法**：浮点数，常用值为 `0.4`

11. **lambda**
    - **中文名称**：lambda 参数
    - **用途**：指定 lambda 参数
    - **配置方法**：浮点数，常用值为 `2.0`

12. **v_linear_min**
    - **中文名称**：最小线速度
    - **用途**：指定最小线速度
    - **配置方法**：浮点数，常用值为 `0.1`

13. **v_linear_max**
    - **中文名称**：最大线速度
    - **用途**：指定最大线速度
    - **配置方法**：浮点数，常用值为 `1.0`

14. **v_angular_max**
    - **中文名称**：最大角速度
    - **用途**：指定最大角速度
    - **配置方法**：浮点数，常用值为 `5.0`

15. **slowdown_radius**
    - **中文名称**：减速半径
    - **用途**：指定减速半径
    - **配置方法**：浮点数，常用值为 `1.5`

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    FollowPath:
      plugin: nav2_graceful_controller::GracefulController
      transform_tolerance: 0.1
      motion_target_dist: 0.6
      initial_rotation: true
      initial_rotation_min_angle: 0.75
      final_rotation: true
      allow_backward: false
      k_phi: 3.0
      k_delta: 2.0
      beta: 0.4
      lambda: 2.0
      v_linear_min: 0.1
      v_linear_max: 1.0
      v_angular_max: 5.0
      slowdown_radius: 1.5
```



# Map Server / Saver

https://docs.nav2.org/configuration/packages/configuring-map-server.html

地图服务器实现了处理堆栈的地图加载请求的服务器，并托管地图主题它还实现了地图保存器服务器，该服务器将在后台运行并根据服务请求保存地图还有一个类似于 ROS 1 的地图保存器 CLI，用于保存单个地图

### map_server 参数

1. **yaml_filename**
   - **中文名称**：YAML文件名
   - **用途**：指定地图配置文件的名称
   - **配置方法**：字符串，常用值为 `"turtlebot3_world.yaml"`

2. **topic_name**
   - **中文名称**：话题名称
   - **用途**：指定发布地图的话题名称
   - **配置方法**：字符串，常用值为 `"map"`

3. **frame_id**
   - **中文名称**：帧ID
   - **用途**：指定地图的坐标系ID
   - **配置方法**：字符串，常用值为 `"map"`

### map_saver 参数

1. **save_map_timeout**
   - **中文名称**：保存地图超时时间
   - **用途**：指定保存地图的超时时间，以秒为单位
   - **配置方法**：浮点数，常用值为 `5.0`

2. **free_thresh_default**
   - **中文名称**：自由阈值默认值
   - **用途**：指定自由区域的阈值默认值
   - **配置方法**：浮点数，常用值为 `0.25`

3. **occupied_thresh_default**
   - **中文名称**：占用阈值默认值
   - **用途**：指定占用区域的阈值默认值
   - **配置方法**：浮点数，常用值为 `0.65`

### costmap_filter_info_server 参数

1. **type**
   - **中文名称**：类型
   - **用途**：指定过滤器信息的类型
   - **配置方法**：整数，常用值为 `1`

2. **filter_info_topic**
   - **中文名称**：过滤器信息话题
   - **用途**：指定过滤器信息的话题名称
   - **配置方法**：字符串，常用值为 `"costmap_filter_info"`

3. **mask_topic**
   - **中文名称**：掩码话题
   - **用途**：指定掩码的话题名称
   - **配置方法**：字符串，常用值为 `"filter_mask"`

4. **base**
   - **中文名称**：基础值
   - **用途**：指定基础值
   - **配置方法**：浮点数，常用值为 `0.0`

5. **multiplier**
   - **中文名称**：乘数
   - **用途**：指定乘数
   - **配置方法**：浮点数，常用值为 `0.25`

```yaml
map_server:
  ros__parameters:
    yaml_filename: "turtlebot3_world.yaml"
    topic_name: "map"
    frame_id: "map"

map_saver:
  ros__parameters:
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

costmap_filter_info_server:
  ros__parameters:
    type: 1
    filter_info_topic: "costmap_filter_info"
    mask_topic: "filter_mask"
    base: 0.0
    multiplier: 0.25
```



# AMCL

https://docs.nav2.org/configuration/packages/configuring-amcl.html

AMCL 实现了获取静态地图并使用自适应蒙特卡罗定位器在其中定位机器人的服务器

### AMCL 参数

1. **alpha1 - alpha5**
   - **中文名称**：运动模型噪声参数
   - **用途**：控制速度和旋转运动的噪声
   - **配置方法**：浮点数，常用值约为 `0.2`

2. **base_frame_id**
   - **中文名称**：基础帧ID
   - **用途**：机器人的基础坐标系
   - **配置方法**：字符串，常用值为 `"base_footprint"`

3. **beam_skip_distance, beam_skip_error_threshold, beam_skip_threshold**
   - **中文名称**：激光束跳过相关参数
   - **用途**：优化激光扫描处理，通过跳过某些激光束来减少计算量
   - **配置方法**：浮点数

4. **do_beamskip**
   - **中文名称**：是否进行激光束跳过
   - **用途**：控制是否启用激光束跳过优化
   - **配置方法**：布尔值，常用值为 `false`

5. **global_frame_id**
   - **中文名称**：全局帧ID
   - **用途**：指定AMCL定位结果的全局坐标系
   - **配置方法**：字符串，常用值为 `"map"`

6. **lambda_short, laser_likelihood_max_dist, laser_max_range, laser_min_range**
   - **中文名称**：激光模型相关参数
   - **用途**：控制激光传感器的模型参数
   - **配置方法**：浮点数

7. **laser_model_type**
   - **中文名称**：激光模型类型
   - **用途**：指定使用的激光模型
   - **配置方法**：字符串，常用值为 `"likelihood_field"`

8. **max_particles, min_particles**
   - **中文名称**：粒子数
   - **用途**：控制粒子滤波器的粒子数范围
   - **配置方法**：整数

9. **odom_frame_id**
   - **中文名称**：里程计帧ID
   - **用途**：指定里程计的坐标系
   - **配置方法**：字符串，常用值为 `"odom"`

10. **pf_err, pf_z**
    - **中文名称**：粒子滤波器参数
    - **用途**：控制粒子滤波器的错误和权重
    - **配置方法**：浮点数

11. **recovery_alpha_fast, recovery_alpha_slow**
    - **中文名称**：恢复因子
    - **用途**：控制快速和慢速恢复的速率
    - **配置方法**：浮点数

12. **resample_interval**
    - **中文名称**：重采样间隔
    - **用途**：控制重采样粒子的频率
    - **配置方法**：整数，常用值为 `1`

13. **robot_model_type**
    - **中文名称**：机器人模型类型
    - **用途**：指定使用的机器人运动模型
    - **配置方法**：字符串，常用值为 `"nav2_amcl::DifferentialMotionModel"`

14. **save_pose_rate**
    - **中文名称**：保存姿态频率
    - **用途**：控制保存机器人姿态的频率
    - **配置方法**：浮点数，常用值为 `0.5`

15. **sigma_hit, z_hit, z_max, z_rand, z_short**
    - **中文名称**：激光击中模型参数
    - **用途**：控制激光击中模型的各种参数
    - **配置方法**：浮点数

16. **tf_broadcast**
    - **中文名称**：是否广播变换
    - **用途**：控制是否将定位结果通过TF广播
    - **配置方法**：布尔值，常用值为 `true`

17. **transform_tolerance**
    - **中文名称**：变换容忍度
    - **用途**：控制变换相关操作的容忍度
    - **配置方法**：浮点数，常用值为 `1.0`

18. **update_min_a, update_min_d**
    - **中文名称**：更新最小角度和距离
    - **用途**：控制触发滤波器更新的最小移动和旋转
    - **配置方法**：浮点数

19. **scan_topic, map_topic**
    - **中文名称**：扫描和地图话题
    - **用途**：指定接收激光扫描和地图数据的话题
    - **配置方法**：字符串，常用值分别为 `"scan"` 和 `"map"`

20. **set_initial_pose, always_reset_initial_pose, first_map_only**
    - **中文名称**：初始姿态设置相关参数
    - **用途**：控制是否在启动时设置初始姿态，是否总是重置初始姿态，以及是否只接受第一张地图
    - **配置方法**：布尔值

21. **initial_pose**
    - **中文名称**：初始姿态
    - **用途**：指定机器人的初始姿态
    - **配置方法**：包含 `x`, `y`, `z`, `yaw` 的字典

```yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
    map_topic: map
    set_initial_pose: false
    always_reset_initial_pose: false
    first_map_only: false
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```





```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 1.0

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    # 'default_nav_through_poses_bt_xml' and 'default_nav_to_pose_bt_xml' are use defaults:
    # nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml
    # nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery.xml
    # They can be set here or via a RewrittenYaml remap from a parent launch file to Nav2.
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_updated_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    controller_frequency: 30.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.5
      wz_max: 1.9
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      AckermannConstraints:
        min_turning_r: 0.2
      critics: ["ConstraintCritic", "ObstaclesCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
      ObstaclesCritic:
        enabled: true
        cost_power: 1
        repulsion_weight: 1.5
        critical_weight: 20.0
        consider_footprint: false
        collision_cost: 10000.0
        collision_margin_distance: 0.1
        near_goal_distance: 0.5
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 3
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        forward_preference: true
      # TwirlingCritic:
      #   enabled: true
      #   twirling_cost_power: 1
      #   twirling_cost_weight: 10.0

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "turtlebot3_world.yaml"

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_theta_star_planner/ThetaStarPlanner" # In Iron and older versions, "/" was used instead of "::"
      how_many_corners: 8
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
      w_heuristic_cost: 1.0

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200

```


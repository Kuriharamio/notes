# ROS2

## 编程小点

1. 不同功能包之间相互依赖时，只需在 `package.xml ` 中添加对应的依赖关系，就会自动先编译被依赖的功能包

2. 用得到的 `C++` 特性

   1. `auto` 关键字

   2. 智能指针

   3. `Lambda` 表达式

     ```cpp
     [capture list] (parameters) -> return_type {function body};
     ```

     > `capture list` 表示捕获列表，可以用来捕获外部变量
     >
     > `parameters` 表示参数列表
     >
     > `return_type` 表示返回类型
     >
     > `function body` 表示函数体

     ```cpp
     int main()
     {
         auto add = [] (int a, int b) -> int { return a + b; };
         int sum = add(3, 5);
         auto print_sum = [sum] () -> void { std::cout << "3 + 5 = " << sum << std::endl; };
         print_sum();
         return 0;
     }
     
     rclcpp::Service<Patrol>::SharedPtr patrol_server_ = this->create_service<Patrol>(
     	"patrol",
         [&] (const std::shared_ptr<Patrol::Request> request, std::shared_ptr<Patrol::Response> response) -> void{
             // code process
         });
     ```

   4. `std::function` 函数包装器

     > 用于存储任意可调用对象（函数、函数指针、`Lambda` 表达式等），并提供统一的调用接口
     >
     > `std::bind` 可以将一个成员函数变成一个 `std::function` 对象，将对象的成员函数和对象绑定在一起，并使用占位符预留传递函数的参数

     ```cpp
     #include <iostream>
     #include <functional>
     
     void save_with_free_fun(const std::string &file_name)
     {
     	std::cout << "调用了自由函数，保存：" << file_name << std::endl;
     }
     
     class FileSave
     {
         public:
         	void save_with_member_fun(const std::string &file_name)
             {
                 std::cout << "调用了成员方法，保存：" << file_name << std::endl;
             };
     };
         
     int main()
     {
         FileSave file_save;
         auto save_with_lambda_fun = [] (const std::string &file_name) -> void
         {
             std::cout << "调用了 Lambda 函数， 保存：" << file_name << std::endl;
         };
         
         std::function<void(const std::string &)> save1 = save_with_free_fun;
         std::function<void(const std::string &)> save2 = save_with_lambda_fun;
         std::function<void(const std::string &)> save3 = std::bind(&FileSave::save_with_member_fun, &file_save, std::placeholders::_1);
         
         save1("file.txt");
         save2("file.txt");
         save3("file.txt");
         
         return 0;
     }
     ```

   5. 多线程与回调函数

   ```cpp
   #include <thread>
   #include <rclcpp/rclcpp.hpp>
   #include <rclcpp/executors/single_threaded_executor.hpp>
   
   class LearnExecutorNode : public rclcpp::Node {
   public:
     LearnExecutorNode() : Node("learn_executor") {
       publisher_ = this->create_publisher<std_msgs::msg::String>("string_topic", 10);
       timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LearnExecutorNode::timer_callback, this));
       // 互斥回调组
       service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);  
       service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
           "add_two_ints",
           std::bind(&LearnExecutorNode::add_two_ints_callback, this, std::placeholders::_1, std::placeholders::_2),
           rmw_qos_profile_services_default, 
           service_callback_group_);
     }
   private:
     void timer_callback() {}
     void add_two_ints_callback(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                                std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {}
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
     rclcpp::TimerBase::SharedPtr timer_;
     rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
     rclcpp::CallbackGroup::SharedPtr service_callback_group_;
   };
   int main(int argc, char *argv)
   {
       rclcpp::init(argc, argv);
       auto node = std::make_shared<Node>();
       size_t N = 3; // 线程数量
       auto options = rclcpp::ExecutorOpetions();
       auto executor = rclcpp::executors::MultiThreadedExecutor(option, N);
       executor.add_node(node);
       executor.spin();
       rclcpp::shutdown();
       return 0;
   }
   ```

   - 线程执行器
     - `SingleThreadedExecutor`：单线程
     - `MultiThreadedExecutor`：多线程
   - 回调函数组
     - `MutuallyExclusiveCallbackGroup()`：互斥回调组（默认所有回调在一个互斥回调组中）
     - `ReentrantCallbackGroup()`：可重入回调组

3. 提高编程效率

   1. ```cpp
      '使用时间单位的字面量，可以在代码中使用 s 和 ms 表示时间'
      using namespace std::chrono_literals
         
      timer_ = this->create_wall_timer(100ms, std::bind(&Class::timer_callback, this));
      ```

   2. ```cpp
      using geometry_msgs::msg::Pose
          
      rclcpp::Publishewr<Pose>::SharedPtr pub_;
      ```

4. 当一个类不继承 `Node` 时，可以使用 `RCLCPP_INFO(rclcpp::get_logger("class_name"), "info");` 进行日志输出

5. 4种时间

   ```cpp
   auto t = rclcpp::Clock().now();
   RCLCPP_INFO(this->get_logger(), "[rclcpp::Clock().now()] sec:%lf nano:%ld", t.seconds(), t.nanoseconds());
   
   auto t1 = std::chrono::system_clock::now(); 
   time_t tt = std::chrono::system_clock::to_time_t ( t1 );
   RCLCPP_INFO(this->get_logger(), "[std::chrono::system_clock::now()] sec:%ld", tt);
   
   std::chrono::steady_clock::time_point td = std::chrono::steady_clock::now(); 
   std::chrono::steady_clock::duration dtn = td.time_since_epoch();
   double secs = dtn.count() * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
   RCLCPP_INFO(this->get_logger(), "[std::chrono::steady_clock::now()] sec:%lf", secs);
   
   auto t2 = this->get_clock()->now();
   RCLCPP_INFO(this->get_logger(), "[get_clock()->now()] sec:%lf nano:%ld", t2.seconds(), t2.nanoseconds());
   
   auto t3 = this->now();
   RCLCPP_INFO(this->get_logger(), "[this->now()] sec:%lf nano:%ld", t3.seconds(), t3.nanoseconds());
   ```

   - 确切时间：

     ```cpp
     time = seconds + nanoseconds / 1e9
     ```



## c_cpp_properties.json

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**",
                "/usr/include/**",
                "/usr/local/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/g++",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "compileCommands": "${workspaceFolder}/build/compile_commands.json",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```




## CMakeLists.txt

- 最外层 `CMakeLists.txt`

  ```cmake
  # 指定最低版本
  cmake_minimum_required(VERSION 3.22)
  
  # 工程名字
  project(project_name)
  
  # C/C++标准
  if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
  
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
  
  # 设置预编译指令，生成compile_commands.json文件
  # 然后让c_cpp_properties.json读取这个文件，防止预编译找不到cmake中定义的宏而报错
  # c_cpp_properties.json中添加
  # "compileCommands": "${workspaceFolder}/build/compile_commands.json"
  set(CMAKE_EXPORT_COMPLIE_COMMANDS ON)
  
  # 设置包名，让子目录节点安装的时候使用
  set(XXX_PACKAGE_NAME ${PROJECT_NAME})
  
  # 设置统一包含的头文件
  include_directories(
    /usr/local/include/eigen-3.4.0
    src/folder/include
    # .etc
  )
  
  # 为特定目标设置包含的头文件
  target_include_directories(
  
  )
  
  # 查找依赖
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  # .etc
  
  # 添加子目录，以查找子目录下的CMakeLists.txt，相当于入口
  add_subdirectory(src/folder1)
  add_subdirectory(src/folder2)
  # .etc
  
  # 安装辅助目录
  install(DIRECTORY 
    config launch urdf rviz
    DESTINATION share/${PROJECT_NAME})
  
  ament_package()
  ```

- 子目录 `CMakeLists.txt`

  ```cmake
  cmake_minimum_required(VERSION 3.22)
  project(project_child_name)
  
  # 设置依赖
  set(DEPENDS 
    "rclcpp"
    "geometry_msgs"
    # .etc
  )
  
  # 添加动态库
  add_library(project_child_name SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/lib1/lib1.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib2/lib2.cpp
  )
  
  set(LIB_TARGETS 
    base
  )
  
  # 添加节点
  add_executable(node 
    ${CMAKE_CURRENT_SOURCE_DIR}/node.cpp
  )
  
  set(NODE_TARGETS 
    node
  )
  
  set(PROJECT_TARGETS 
    base 
    node
  )
  
  # 添加依赖
  foreach(target ${PROJECT_TARGETS})
    ament_target_dependencies(${target}
      ${DEPENDS}
    )
  endforeach()
  
  target_link_libraries(${NODE_TARGETS} 
    lib1
    lib_out_1
  )
  
  target_link_libraries(${LIB_TARGETS} 
    lib_out_1
  )
  
  # 安装动态库、节点和相关目录
  install(TARGETS 
    ${LIB_TARGETS} 
    LIBRARY DESTINATION lib)
  
  install(TARGETS 
    ${NODE_TARGETS} 
    DESTINATION lib/${PLANNING_PACKAGE_NAME}) 
  ```

  

- 可用变量

  ```cmake
  # 工程名字
  ${PROJECT_NAME}
  # 工程根目录
  ${PROJECT_SOURCE_DIR}
  # build 目录
  ${PROJECT_BINARY_DIR}
  # 当前CMakeLists.txt文件所在目录
  ${CMAKE_CURRENT_SOURCE_DIR}
  # target编译目录，可使用ADD_SUBDIRECTORY来修改此变量
  ${CMAKE_CURRENT_BINARY_DIR}
  ```

  

## 自定义接口编写

类型

- 话题接口格式：`xxx.msg`

  ```
  int64 num
  ```

- 服务接口格式：`xxx.srv`

  ```
  int64 a
  int64 b
  ---
  int64 sum
  ```

- 动作接口格式：`xxx.action`

  ```
  int32 order
  ---
  int32[] sequence
  ---
  int32[] partial_sequence
  ```

例子：`Addressook.msg`

```
uint8 PHONE_TYPE_HOME=0
uint8 PHONE_TYPE_WORK=1
uint8 PHONE_TYPE_MOBILE=2

string first_name
string last_name
string phone_number
uint8 phone_type
```

```cmake
find_package(rosidl_default_generators REQUIRED)
#声明消息列表：
set(msg_files
  "msg/AddressBook.msg"
)
#生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
#确保导出消息运行时依赖项
ament_export_dependencies(rosidl_default_runtime)
```

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

其它

```cmake
#可以用set来整齐地列出所有接口
set(msg_files
  "msg/Message1.msg"
  "msg/Message2.msg"
  # etc
  )

set(srv_files
  "srv/Service1.srv"
  "srv/Service2.srv"
   # etc
  )
#并立即生成所有列表
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

#使用同一包中生成的消息时，需要以下cmake码链接
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(publish_address_book ${cpp_typesupport_target})
```

导入接口的三个步骤：

> `ament_cmake`类型功能包导入消息接口分为三步：
>
> 1. 在`CMakeLists.txt`中导入，具体是先`find_packages`再`ament_target_dependencies`。
> 2. 在`packages.xml`中导入，具体是添加`depend`标签并将消息接口写入。
> 3. 在代码中导入，C++中是`#include"消息功能包/xxx/xxx.hpp"`。

```cmake
# CMakeLists.txt
# 这里我们一次性把服务端和客户端对example_interfaces的依赖都加上
find_package(example_interfaces REQUIRED)

add_executable(service_client_01 src/service_client_01.cpp)
ament_target_dependencies(service_client_01 rclcpp example_interfaces)

add_executable(service_server_01 src/service_server_01.cpp)
ament_target_dependencies(service_server_01 rclcpp example_interfaces)
```

```xml
<!--packages.xml-->
<depend>example_interfaces</depend>
```

```cpp
//代码
#include "example_interfaces/srv/add_two_ints.hpp"
```



## 参数代码编写

```cpp
#include "rcl_interfaces/msg/set_parameters_result.hpp"
using SetParameterResult = rcl_interfacess::msg::SetParametersResult;

class ParamLoader : pubnlic rclcpp::Node{
    public:
    	ParamLoader() : Node("ParamLoader"){
            // 声明参数
            this->declare_parameter("p", 1.0);
            this->get_parameter("p", p_);
            
            // 添加参数设置回调
            parameters_callback_handle_ = this->add_on_set_parameters_callback(
            [&] (const std::vector<rclcpp::Parameter> &params) -> SetParametersResult {
                for(auto param: params){
                    if(param.get_name() == "p"){
                        p_ = param.as_double();
                    }else if(){
                        
                    }// .etc
                }
                auto result = SetParameterResult();
                result.successful = true;
                return result;
            });
        }
    private:
    	double p_;
    	OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};
```

- 修改其它节点的参数

  ```cpp
  #include "rcl_interfaces/msg/parammeter.hpp"
  #include "rcl_interfaces/msg/parammeter_value.hpp"
  #include "rcl_interfaces/msg/parammeter_type.hpp"
  #include "rcl_interfaces/msg/set_parammeters.hpp"
  
  using SetP = rcl_interfaces::srv::SetParameters;
  
  class PatrolClient : public rclcpp::Node{
      std::shared_ptr<SetP::Response> call_set_parameters(rcl_interfaces::msg::Parameter &parameter)
      {
          auto param_client = this->create_client<SetP>("/turtle_controller/set_parameters"); // /节点/set_parameters
          while(!param_client->wait_for_service(std::chrono::seconds(1)))
          {
              if(!rclcpp::ok())
              {
                  RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
                  return nullptr;
              }
              RCLCPP_INFO(this->get_logger(), "等待参数设置服务端上线中...");
          }
          
          auto request = std::make_shared<SetP::Request>();
          request->parameters.push_back(parameter);
          
          auto future = param_client->async_client->async_send_request(request);
          rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
          
          auto response = future.get();
          return response;
      }
      
      void update_server_param_p(double p)
      {
          auto param = rcl_interfaces::msg::Parameter();
          param.name = "p";
          
          auto param_value = rcl_interfaces::msg::ParameterValue();
          param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          param_value.double_value = p;
          param.value = param_value;
          
          auto response = call_set_parameters(param);
          if(response == nullptr){
              RCLCPP_ERROR(this->get_logger(), "参数修改失败");
              return;
          }else{
              for(auto result : response->results){
                  if(result.successful){
                      RCLCPP_INFO(this->get_logger(), "参数 p 已修改为 %f", p);
                  }else{
                      RCLCPP_WARN(this->get_logger(), "参数 p 修改失败，原因：%s", result.reason.c_str());
                  }
              }
          }
      }
  }
  
  ```

  

## 话题代码编写

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class SingleDogNode: public rclcpp::Node
{
    public:
        SingleDogNode(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(),"大家好，我是单身狗%s.", name.c_str());
            sub_novel = this->create_subscription<std_msgs::msg::String>("sexy_girl", 10, std::bind(&SingleDogNode::topic_callback, this, _1));
            pub_money = this->create_publisher<std_msgs::msg::UInt32>("sexy_girl_money", 10);
        }
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_novel;
        rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_money;
        void topic_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            std_msgs::msg::UInt32 money;
            money.data = 10;
            pub_money->publish(money);
            RCLCPP_INFO(this->get_logger(), "已阅：'%s'", msg->data.c_str());
        };

};      

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleDogNode>("wang2");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

```



## 服务代码编写

- ```cpp
  #include "example_interfaces/srv/add_two_ints.hpp"
  #include "rclcpp/rclcpp.hpp"
  /**********************************
  1、创建节点
  2、节点内创建服务，该服务调用一个回调函数
  3、编写回调函数
  **********************************/
  class ServiceServer01 : public rclcpp::Node {
  public:
    ServiceServer01(std::string name) : Node(name) {
      RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
      // 创建服务
      add_ints_server_ =
        this->create_service<example_interfaces::srv::AddTwoInts>(
          "add_two_ints_srv",
          std::bind(&ServiceServer01::handle_add_two_ints, this, std::placeholders::_1, std::placeholders::_2));
    }
  
  private:
    // 声明一个服务
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr add_ints_server_;
  
    // 收到请求的处理函数
    void handle_add_two_ints(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
      					   std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) 
    {
      RCLCPP_INFO(this->get_logger(), "收到a: %ld b: %ld", request->a, request->b);
      response->sum = request->a + request->b;
    };
  };
  ```

- ```cpp
  #include "example_interfaces/srv/add_two_ints.hpp"
  /**********************************
  1、创建节点
  2、节点内创建客户端
  3、编写请求服务函数
  **********************************/
  class ServiceClient01 : public rclcpp::Node {
  public:
    // 构造函数,有一个参数为节点名称
    ServiceClient01(std::string name) : Node(name) {
      RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
      // 创建客户端
      client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints_srv");
    }
  
    // 可以按需调用该函数
    void send_request(int a, int b) 
    {
      RCLCPP_INFO(this->get_logger(), "计算%d+%d", a, b);
  
      // 1.等待服务端上线
      while (!client_->wait_for_service(std::chrono::seconds(1))) {
        //等待时检测rclcpp的状态
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
      }
  
      // 2.构造请求的
      auto request =
        std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
      request->a = a;
      request->b = b;
  
      // 3.发送异步请求，然后等待返回，返回时调用回调函数
      client_->async_send_request(request, std::bind(&ServiceClient01::result_callback_, this, std::placeholders::_1));
    };
  
  private:
    // 声明客户端
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  
    void result_callback_(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future)
    {
      auto response = result_future.get();
      RCLCPP_INFO(this->get_logger(), "计算结果：%ld", response->sum);
    }
  };
  ```



## 动作代码编写



## launch 工具

- 动作：执行进程、输出日志、一段命令、其它 `launch`、组合、定时启动
- 条件：根据条件决定是否启动
- 替换：替换参数

```python
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.coonditions import IfCondition
from launch_ros.substitutions import FindPackageShare

# 入口函数，launch工具运行时，会在py中搜索名称为generate_launch_description的函数来获取对启动内容的描述
def generate_launch_description():
    package_name = 'package_name'
    urdf_name = "urdf_name.urdf"
    rviz_name = "rviz_name.rviz"
    
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    rviz_config_path = os.path.join(pkg_share, f'rviz/{rviz_name}')
    
    
    
    # 使用 DeclarelaunchArgument 动作声明参数
    # 可在启动时指定参数： ros2 launch package_name launch.py launch_arg1:=3.0
    action_declare_arg_arg1 = launch.actions.DeclarelaunchArgument('launch_arg1', default_value='2.0', description='一个double变量')
    action_declare_arg_arg2 = launch.actions.DeclarelaunchArgument('launch_arg2', default_value='False', description='一个bool变量')
    action_declare_arg_mode_path = launch.actions.DeclarelaunchArgument(name='model', default_value=str(urdf_model_path), description='urdf路径')
    
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['cat', launch.substitutions.launchConfiguration('model')]), 
        value_type=str
    )
    
    # 使用 IncludeLaunchDescription 动作 include 其它 launch.py
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
        	[get_package_share_directory(package_name), "/launch", "launch.py"]
        ),
        # 替换原有参数
        launch_arguments={
            'use_sim_time' : use_sim_time
        }.items()
    )
    
    # 使用 ExecuteProcess 动作在命令行执行命令，并附带执行条件
    action_executeprocess = launch.actions.ExecuteProcess(
        condition=IfCondition(action_declare_arg_arg2)
    	cmd=['ros2', 'service', 'call', '/service', 'service/srv/service_type', '{request_arg1: value, request_arg2: value}']
    )
    
    # 使用 LogInfo 动作输出日志
    action_log_info = launch.actions.LogInfo(msg='输出了日志。')
    
    
    # 使用 Node 启动节点
    action_node_1= launch_ros.actions.Node(
    	package=package_name,
        executable='node1_name',
        output='screen', # screen、log、both
        # 使用替换组件进行参数替换
        parameters=[{'arg1': launch.substitutions.launchConfiguration('launch_arg1', default='2.0')}]
    )
    
    action_node_2 = launch_ros.actions.Node(
    	package=package_name
        executable='node2_name',
        output='screen', # screen、log、both
    )
    
    # 发布机器人 urdf 模型
    # 状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_model_path}]
    )
	# 关节发布节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
    )
    
    # 启动 rviz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        argument=['-d', rviz_config_path]
        output='screen',
    )
    
    # 启动 gazebo
    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', "robot_name",  '-file', urdf_model_path ], 
        output='screen',
    )
    
    # 利用定时器动作实现一次启动日志输出和进程执行，并用 GroupAction 封装成组合
    acton_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=2.0, actions=[action_log_info]),
        launch.actions.TimerAction(period=3.0, actions=[action_executeprocess]),
        launch.actions.TimerAction(period=2.0, actions=[action_node_1]),
        launch.actions.TimerAction(period=2.0, actions=[action_node_2]),
    ])
    
    launch_description = launch.LaunchDescription([
        action_include_launch, 
        action_group,
    ])
    
    return launch_description
```

```python
import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
```



## tf2 工具

- `CMackeLists.txt` 依赖

  ```cmake
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  ```

- 发布静态变换

  ```bash
  ros2 run tf2_ros static_transform_publisher --x 0.1 --y 0.0 --z 0.2 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id link_2 --child-frame-id link_1
  ```

  ```cpp
  #include <memory>
  #include "geometry_msgs/msg/transform_stamped.hpp"
  #include "rclcpp/rclcpp"
  #include "tf2/LinearMath/Quaterion.h" // tf2::Quaternion
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"	// 消息类型转换
  #include "tf2_ros/static_transform_broadcaster.h"
  
  class StaticTFBroadcaster() : Node public rclcpp::Node{
      public:
      	StaticTFBroadcaster() : Node("tf_broadcaster_node"){
              tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
              this->publish_tf();
          }
      
      	void publish_tf(){
              geometry_msgs::msg::TransformStamped transform;
              transform.header.stamp = this->get_clock()->now();
              transform.header.frame_id = "frame_father";
              transform.child_fram_id = "frame_child";
              transform.transform.translation.x = 5.0;
              transform.transform.translation.y = 3.0;
              transform.transform.translation.x = 0.0;
              tf2::Quaternion quat;
              quat.setRPY(0, 0, 60 * M_PI / 180.0f);
              transform.transform.rotation = tf2::toMsg(quat);
              tf_broadcaster_->sendTransform(transform);
          }
      private:
      	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  };
  ```

- 发布动态变换

  ```cpp
  #include <memory>
  #include "geometry_msgs/msg/transform_stamped.hpp"
  #include "rclcpp/rclcpp"
  #include "tf2/LinearMath/Quaterion.h" // tf2::Quaternion
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"	// 消息类型转换
  #include "tf2_ros/transform_broadcaster.h"
  #incldue <chrono>
  using namespace std::chrono_literals;
  
  class DynamicTFBroadcaster() : Node public rclcpp::Node{
      public:
      	DynamicTFBroadcaster() : Node("tf_broadcaster_node"){
              tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
              timer_ = create_wall_timer(10ms, std::bind(&DynamicTFBroadcaster::publishTransform, this));
          }
      
      	void publishTransform(){
              geometry_msgs::msg::TransformStamped transform;
              transform.header.stamp = this->get_clock()->now();
              transform.header.frame_id = "frame_father";
              transform.child_fram_id = "frame_child";
              transform.transform.translation.x = 5.0;
              transform.transform.translation.y = 3.0;
              transform.transform.translation.x = 0.0;
              tf2::Quaternion quat;
              quat.setRPY(0, 0, 60 * M_PI / 180.0f);
              transform.transform.rotation = tf2::toMsg(quat);
              tf_broadcaster_->sendTransform(transform);
          }
      private:
      	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
      	rclcpp::TimerBase::SharedPtr timer_;
  };
  ```

- 监听坐标变换

  ```cpp
  #include <memory>
  #include "geometry_msgs/msg/transform_stamped.hpp"
  #include "rclcpp/rclcpp"
  #include "tf2/LinearMath/Quaterion.h" 	// tf2::Quaternion
  #include "tf2/utils.h"					// tf2::getEulerYPR
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"	// 消息类型转换
  #include "tf2_ros/buffer.h"				// TF 缓冲类 Buffer
  #include "tf2_ros/transform_listener.h"	// TF 坐标监听器
  #include "tf2_ros/transform_broadcaster.h"
  #incldue <chrono>
  using namespace std::chrono_literals;
  
  class TFListener : public rlcpp::Node{
      public:
      	TFListener() : Node("tf_listener"){
              tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
              tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
              timer_ = this->create_wall_timer(5s, std::bind(&TFListener::getTransform, this));
          }
      
      	void getTransform(){
              try{
                  const auto transform = tf_buffer_->lookupTransform("link_1", "link_2", 
                                                                     this->get_clock()->now(), 
                                                                     rclcpp::Duration::from_seconds(1.0f));
                  const auto &translation = transform.transform.translation;
                  const auto &rotation = transform.transform.rotation;
                  double yaw, pitch, roll;
                  tf2::getEulerYPR(rotation, yaw, pitch, roll);                
              }catch(tf2::TransformException &ex){
                  RCLCPP_WARN(this->get_logger(), "y异常： %s", ex.what());
              }
          }
      
      private:
      	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
      	rclcpp::TimerBase::SharedPtr timer_;
  };
  ```

  

- 可视化

  ```bash
  sudo apt install ros-humble-python-mrpt
  sudo apt install mrpt-apps
  3d-rotation-converter
  ```

  或 rviz2

- 计算变换关系

  ```bash
  ros2 run tf2_ros tf2_echo link_1 link_2
  ```

  - 输出

    ```bash
    At time 0.0
    - Translation: [-0.100, 0.000, -0.200]
    - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
    - Rotation: in RPY (radian) [0.000, -0.000, 0.000]
    - Rotation: in RPY (degree) [0.000, -0.000, 0.000]
    - Matrix:
      1.000  0.000  0.000 -0.100
      0.000  1.000  0.000  0.000
      0.000  0.000  1.000 -0.200
      0.000  0.000  0.000  1.000
    
    ```

- 查看所有坐标系的连接关系

  ```bash
  ros2 run tf2_tools view_frames
  ```

  - 会在当前目录生成 `pdf`、`GV` 格式文件

- 查看 tf tree

  ```bash
  ros2 run rqt_tf_tree rqt_tf_tree --force-discover
  ```




## `QoS` 配置

- 参数

  - 历史记录 `History`
    - 仅保留最新 `Keep last`：仅存储最多 N 个样本，可通过历史队列深度 `Depth` 来配置
    - 全部保留`Keep all`：存储所有样本，受底层中间件的资源限制配置的影响
  - 历史队列深度 `Depth`
    - 队列大小 `Queue size`：仅当选用 `Keep last` 时生效
  - 可靠性 `Reliability`
    - 尽力而为 `Best effort`：尝试传递，但如果网络不稳定可能会丢失
    - 可靠传递 `Reliable`：保证样本被传递，丢失会重传
  - 持久性 `Durability`
    - 瞬态本地 `Transient local`：发布者负责为后续加入的订阅者保留数据
    - 易失性 `Volatile`：不保留任何数据
  - 截止时间 `Deadline`
    - 持续时间 `Duration`：消息需要在截止时间之前被接收，否则会被丢弃
  - 寿命 `Lifespan`
    - 持续时间 `Duration`：超过寿命的信息会被丢弃
  - 租约持续时间 `Liveliness Lease Duration`
  - 活跃度 `Liveliness`
    - 自动 `Automatic`：一个话题是否活跃由 ROS 的 rmw 层自动检测和报告，每次报告在下一个租约持续时间内都将视其为活跃状态
    - 按主题手动 `Manual by topic`：话题每发布一次或手动声明一次，则在下一个租约持续时间内将视其为活跃状态

- 默认配置

  ```cpp
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
  
  static const rmw_qos_profile_t rmw_qos_profile_parameters =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1000,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
  
  static const rmw_qos_profile_t rmw_qos_profile_default =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
  
  static const rmw_qos_profile_t rmw_qos_profile_services_default =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
  
  static const rmw_qos_profile_t rmw_qos_profile_parameter_events =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1000,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
  
  static const rmw_qos_profile_t rmw_qos_profile_system_default =
  {
    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
  
  static const rmw_qos_profile_t rmw_qos_profile_unknown =
  {
    RMW_QOS_POLICY_HISTORY_UNKNOWN,
    RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_RELIABILITY_UNKNOWN,
    RMW_QOS_POLICY_DURABILITY_UNKNOWN,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_UNKNOWN,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
  ```

  - 现成类

    ```cpp
    /**
     * Clock QoS class
     *    - History: Keep last,
     *    - Depth: 1,
     *    - Reliability: Best effort,
     *    - Durability: Volatile,
     *    - Deadline: Default,
     *    - Lifespan: Default,
     *    - Liveliness: System default,
     *    - Liveliness lease duration: default,
     *    - avoid ros namespace conventions: false
     */
    class RCLCPP_PUBLIC ClockQoS : public QoS
    {
    public:
      explicit
      ClockQoS(
        const QoSInitialization & qos_initialization = KeepLast(1));
    };
    
    /**
     * Sensor Data QoS class
     *    - History: Keep last,
     *    - Depth: 5,
     *    - Reliability: Best effort,
     *    - Durability: Volatile,
     *    - Deadline: Default,
     *    - Lifespan: Default,
     *    - Liveliness: System default,
     *    - Liveliness lease duration: default,
     *    - avoid ros namespace conventions: false
     */
    class RCLCPP_PUBLIC SensorDataQoS : public QoS
    {
    public:
      explicit
      SensorDataQoS(
        const QoSInitialization & qos_initialization = (
          QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)
      ));
    };
    
    /**
     * Parameters QoS class
     *    - History: Keep last,
     *    - Depth: 1000,
     *    - Reliability: Reliable,
     *    - Durability: Volatile,
     *    - Deadline: Default,
     *    - Lifespan: Default,
     *    - Liveliness: System default,
     *    - Liveliness lease duration: default,
     *    - Avoid ros namespace conventions: false
     */
    class RCLCPP_PUBLIC ParametersQoS : public QoS
    {
    public:
      explicit
      ParametersQoS(
        const QoSInitialization & qos_initialization = (
          QoSInitialization::from_rmw(rmw_qos_profile_parameters)
      ));
    };
    
    /**
     * Services QoS class
     *    - History: Keep last,
     *    - Depth: 10,
     *    - Reliability: Reliable,
     *    - Durability: Volatile,
     *    - Deadline: Default,
     *    - Lifespan: Default,
     *    - Liveliness: System default,
     *    - Liveliness lease duration: default,
     *    - Avoid ros namespace conventions: false
     */
    class RCLCPP_PUBLIC ServicesQoS : public QoS
    {
    public:
      explicit
      ServicesQoS(
        const QoSInitialization & qos_initialization = (
          QoSInitialization::from_rmw(rmw_qos_profile_services_default)
      ));
    };
    
    /**
     * Parameter events QoS class
     *    - History: Keep last,
     *    - Depth: 1000,
     *    - Reliability: Reliable,
     *    - Durability: Volatile,
     *    - Deadline: Default,
     *    - Lifespan: Default,
     *    - Liveliness: System default,
     *    - Liveliness lease duration: default,
     *    - Avoid ros namespace conventions: false
     */
    class RCLCPP_PUBLIC ParameterEventsQoS : public QoS
    {
    public:
      explicit
      ParameterEventsQoS(
        const QoSInitialization & qos_initialization = (
          QoSInitialization::from_rmw(rmw_qos_profile_parameter_events)
      ));
    };
    
    /**
     * Rosout QoS class
     *    - History: Keep last,
     *    - Depth: 1000,
     *    - Reliability: Reliable,
     *    - Durability: TRANSIENT_LOCAL,
     *    - Deadline: Default,
     *    - Lifespan: {10, 0},
     *    - Liveliness: System default,
     *    - Liveliness lease duration: default,
     *    - Avoid ros namespace conventions: false
     */
    class RCLCPP_PUBLIC RosoutQoS : public QoS
    {
    public:
      explicit
      RosoutQoS(
        const QoSInitialization & rosout_qos_initialization = (
          QoSInitialization::from_rmw(rcl_qos_profile_rosout_default)
      ));
    };
    
    /**
     * System defaults QoS class
     *    - History: System default,
     *    - Depth: System default,
     *    - Reliability: System default,
     *    - Durability: System default,
     *    - Deadline: Default,
     *    - Lifespan: Default,
     *    - Liveliness: System default,
     *    - Liveliness lease duration: System default,
     *    - Avoid ros namespace conventions: false
     */
    class RCLCPP_PUBLIC SystemDefaultsQoS : public QoS
    {
    public:
      explicit
      SystemDefaultsQoS(
        const QoSInitialization & qos_initialization = (
          QoSInitialization::from_rmw(rmw_qos_profile_system_default)
      ));
    };
    
    }  // namespace rclcpp
    ```

- 兼容性

  - 可靠性

    |    发布者     |    订阅者     | 兼容性 |
    | :-----------: | :-----------: | :----: |
    | `Best effort` | `Best effort` |   是   |
    | `Best effort` |  `Reliable`   |   否   |
    |  `Reliable`   | `Best effort` |   是   |
    |  `Reliable`   |  `Reliable`   |   是   |

  - 持久性

    |      发布者       |      订阅者       | 兼容性 |         结果         |
    | :---------------: | :---------------: | :----: | :------------------: |
    |    `Volatile`     |    `Volatile`     |   是   |    仅适用于新消息    |
    |    `Volatile`     | `Transient local` |   否   |        无通信        |
    | `Transient local` |    `Volatile`     |   是   |    仅适用于新消息    |
    | `Transient local` | `Transient local` |   是   | 适用于新消息和旧消息 |

  - 活跃度

    |      发布者       |      订阅者       | 兼容性 |
    | :---------------: | :---------------: | :----: |
    |    `Automatic`    |    `Automatic`    |   是   |
    |    `Automatic`    | `Manual by topic` |   否   |
    | `Manual by topic` |    `Automatic`    |   是   |
    | `Manual by topic` | `Manual by topic` |   是   |

- 配置方法

  ```cpp
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
  ```

  - 自定义

    ```
    rclcpp::QoS qos_profile(10); 										// 队列深度
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); 	// 可靠性策略
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); 	// 持久性策略
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST); 				// 历史记录策略
    qos_profile.deadline(rclcpp::Duration(1.0)); 						// 截止时间
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos_profile);
    ```



## 生命周期节点

## 组件

## 消息过滤器

## DDS

## ROS2_Control

## Navigation2

- 保存地图

  ```bash
  sudo apt install ros-$ROS_DISTRO-nav2-map-server
  ros2 run nav2_map_server map_saver_cli - room
  ```

- 自定义插件

  - 规划器
  - 控制器

## SLAM

- `slam_toolbox`

  ```bash
  sudo apt install ros-$ROS_DISTRO-slam-toolbox
  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
  ```

  - 输入话题:`/scan`、`odom` to `base`

- `cartographer_ros`

- `rtabmap_slam`



## 命令

### 1、节点相关

1. #### 运行节点

  - ```bash
    ros2 run <package_name> <executable_name> #包名 可执行文件名
    ```

2. #### 查看节点列表：

  - ```bash
    ros2 node list
    ```

3. #### 查看节点信息：

  - ```bash
    ros2 node info <node_name>
    ```

4. #### 重映射节点名称：

  - ```bash
    ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
    ```

    

### 2、功能包相关

1. #### **安装获取功能包：**

  - ```bash
    sudo apt install ros-<version'''humble'''>-package_name	#自动放在/opt/ROS/humble/
    ```

  - 若使用手动编译，则需手动source工作空间的install目录

2. #### **创建功能包：**

  - ```bash
    ros2 pkg create <package-name> --build-type {cmake,ament_cmake,ament_python} --dependencies <依赖名字>
    #ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_srvcli --dependencies rclcpp example_interfaces
    #ros2 pkg create example_interfaces_rclcpp --build-type ament_cmake --dependencies rclcpp example_ros2_interfaces --destination-directory src --node-name example_interfaces_robot_01 
    ```

3. #### **列出可执行文件：**

  - ```bash
    ros2 pkg executables #列出所有
    ```

  - ```bash
    ros2 pkg executables turtlesim	#列出某个功能包
    ```

4. #### **列出所有包：**

  - ```bash
    ros2 pkg list
    ```

5. #### **输出某个包所在路径的前缀：**

  - ```bash
    ros2 pkg prefix <package-name>
    ```

6. #### **列出功能包的清单描述文件：**

  - ```
    ros2 pkg xml <package-name>
    ```



### 3、Colcon

1. #### **编译工程:**

  - ```bash
    colcon build
    ```

2. #### **只编译一个功能包：**

  - ```bash
    colcon build --packages-select YOUR_PKG_NAME 
    ```

3. #### **不编译测试单元：**

  - ```bash
    colcon build --packages-select YOUR_PKG_NAME  --cmake-args -DBUILD_TESTING=0
    ```

4. #### **运行编译的包的测试：**

  - ```bash
    colcon test
    ```

5. #### **允许通过更改src下的部分文件来改变install：**

  - ```bash
    colcon build --symlink-install
    ```



### 4、接口相关：

- ### 查看包下所有接口：

  - ```bash
    ros2 interface package village_interfaces
    ```

- ### 查看内容：

  - ```
    ros2 interface show village_interfaces/msg/Novel 
    ```

- ### 显示属性：

  - ```
    ros2 interface proto village_interfaces/msg/Novel 
    ```



### 5、服务相关：

- ### 查看服务列表：

  - ```bash
    ros2 service list
    ```

- ### 手动调用服务：

  - ```bash
    ros2 service call <service.name> + <data>
    #ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5,b: 10}"
    ```

- ### 查看服务接口类型：

  - ```bash
    ros2 service type <service.name>
    #ros2 service type /add_two_ints
    ```

- ### 查找使用某一接口服务：

  - ```bash
    ros2 service find <service.type>
    #ros2 service find example_interfaces/srv/AddTwoInts
    ```

- ### 重映射服务：

  - ```bash
    ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
    #还可用于node topic
    #ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle  节点
    ```



### 6、参数相关

- ### 查看参数：

  - ```cmd
    ros2 param list
    ```

- ### 获得参数：

  - ```cmd
    ros2 param get <node.name> <param.name>
    ```

- ### 修改参数：

  - ```bash
    ros2 param set <node.name> <param.name> <value>
    ```

- ### 保存参数：

  - ```cmd
    ros2 param dump <node.name> > <file>
    ```

- ### 加载参数：

  - ```cmd
    ros2 param load <node.name> <file>
    # ros2 param load  /turtlesim ./turtlesim.yaml
    ```

- ### 获取参数描述：

  ```cmd
  ros2 param describe /turtlesim background_r
  ```

  

- ### 打开节点时使用参数：

  - ```bash
    ros2 run <package_name> <node_name> --ros-args --params-file <file_name>
    # ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
    # 设置某参数：
    # ros2 run example_parameters_rclcpp parameters_basic --ros-args -p rcl_log_level:=10
    ```




### 7、行为相关



- ### 查看action：

  - ```bash
    ros2 action list
    ros2 action list -t #可查看类型
    ```

- ### 查看action信息：

  - ```bash
    ros2 interface show <type.name>
    ```

- ### 查看action客户端和服务端的数量和名字：

  - ```
    ros2 action info <action.name>
    ```

- ### 发送action请求：

  - ```bash
    ros2 action send_goal <action.name> <type.name> data(yaml)
    #ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.6}" --feedback(可实时反馈)
    ```



### 8、rosbag2

- ### 记录话题：

  - ```bash
    ros2 bag record <topic.name>
    ros2 bag record <topic1.name> <topic2.name> <topic3.name> #多个
    ros2 bag record -a #所有
    ros2 bag record -o <file.name> <topic.name> #指定文件名
    ```

- ### 查看记录信息：

  - ```bash
    ros2 bag info bag-file
    ```

- ### 播放数据：

  - ```bash
    ros2 bag play xxx.db3
    #同时使用 ros2 toopic echo <topic.name> 来查看播放的数据
    #播放选项：-r 5 五倍速 -l 循环播放 --topics <topic.name> 播放当
    ```




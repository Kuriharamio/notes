# ROS 2

## 1、节点相关

1. ### 运行节点

  - ```bash
    ros2 run <package_name> <executable_name> #包名 可执行文件名
    ```

2. ### 查看节点列表：

  - ```bash
    ros2 node list
    ```

3. ### 查看节点信息：

  - ```bash
    ros2 node info <node_name>
    ```

4. ### 重映射节点名称：

  - ```bash
    ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
    ```

    

## 2、功能包相关

1. ### **安装获取功能包：**

  - ```bash
    sudo apt install ros-<version'''humble'''>-package_name	#自动放在/opt/ROS/humble/
    ```

  - 若使用手动编译，则需手动source工作空间的install目录

2. ### **创建功能包：**

  - ```bash
    ros2 pkg create <package-name> --build-type {cmake,ament_cmake,ament_python} --dependencies <依赖名字>
    #ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_srvcli --dependencies rclcpp example_interfaces
    #ros2 pkg create example_interfaces_rclcpp --build-type ament_cmake --dependencies rclcpp example_ros2_interfaces --destination-directory src --node-name example_interfaces_robot_01 
    ```

3. ### **列出可执行文件：**

  - ```bash
    ros2 pkg executables #列出所有
    ```

  - ```bash
    ros2 pkg executables turtlesim	#列出某个功能包
    ```

4. ### **列出所有包：**

  - ```bash
    ros2 pkg list
    ```

5. ### **输出某个包所在路径的前缀：**

  - ```bash
    ros2 pkg prefix <package-name>
    ```

6. ### **列出功能包的清单描述文件：**

  - ```
    ros2 pkg xml <package-name>
    ```

## 3、Colcon

1. ### **编译工程:**

  - ```bash
    colcon build
    ```

2. ### **只编译一个功能包：**

  - ```bash
    colcon build --packages-select YOUR_PKG_NAME 
    ```

3. ### **不编译测试单元：**

  - ```bash
    colcon build --packages-select YOUR_PKG_NAME  --cmake-args -DBUILD_TESTING=0
    ```

4. ### **运行编译的包的测试：**

  - ```bash
    colcon test
    ```

5. ### **允许通过更改src下的部分文件来改变install：**

  - ```bash
    colcon build --symlink-install
    ```

## 4、运行一个自己编的节点

```bash
source install/setup.bash	
#打开一个终端使用 cd colcon_test 进入我们刚刚创建的工作空间，先source 一下资源
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
#运行一个订杂志节点，你将看不到任何打印，因为没有发布者
source install/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
#打开一个新的终端，先source，再运行一个发行杂志节点
```

1. ### 编写Python节点：

  - ```python
    import rclpy
    from rclpy.node import Node
    
    class WriterNode(Node):
        def __init__(self,name):
            super().__init__(name)
            self.get_logger().info("大家好，我是作家%s." % name)
    
    def main(args=None):
        '''
        入口函数
            1.导入库文件
            2.初始化客户端库
            3.新建节点列表
            4.spin循环节点
            5.关闭客户端库
        '''
        rclpy.init(args=args)  
        li4_node = WriterNode("li4")  #新建节点 Node里的参数为节点名字
        #li4_node.get_logger().info("大家好，我是作家li4")  pop思想
        rclpy.spin(li4_node)
        rclpy.shutdown()
    ```

2. ### 编写C++节点：

  - ```cpp
    #include "rclcpp/rclcpp.hpp"
    
    class SingleDogNode: public rclcpp::Node
    {
    private:
        /* data */
    public:
        SingleDogNode(std::string name):Node(name)
        {
            RCLCPP_INFO(this->get_logger(),"大家好，我是单身狗%s.",name.c_str());
        }
    };
    
    
    int main(int argc, char ** argv)
    {
        rclcpp::init(argc,argv);
        auto node = std::make_shared<SingleDogNode>("wang2");
    
        //RCLCPP_INFO(node->get_logger(),"大家好，我是单身狗wang2.");
        
        rclcpp::spin(node);
        rclcpp::shutdown();
    }
    
    
    
    ```

## 5、RQT工具：

- ### **观察数据走向：**

  - ```bash
    rqt_graph
    ```

- ### **查找服务**：

  - ```bash
    rqt --force-discover
    ```

    

## 6、话题相关：

- ### **查看指令：**

    - ```bash
      ros2 topic  -h	
      ```


- ### **返回系统中当前活动的所有主题的列表：**

   - ```bash
     ros2 topic list
     ```


- ### 增加消息类型：

  - ```bash
    ros2 topic list -t
    ```

- ### 打印实时话题内容：

  - ```bash
    ros2 topic echo <topic.name> #/chatter
    ```

- ### 查看主题信息：

  - ```bash
    ros2 topic info <topic.name> #/chatter
    ```

- ### 查看消息类型：

  - ```bash
    ros2 interface show std_msgs/msg/String #由上个指令知消息是std_msgs/msg/String
    ```

- ### 手动发布命令：

  - ```bash
    ros2 topic pub <topic.name> std_msgs/msg/String 'data:"123"'
    #ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
    #对于带有header时间戳的消息：
    #1、std_msgs/msg/Header 可以将header设置为auto来填写该stamp
    ros2 topic pub /pose geometry_msgs/msg/PoseStamped '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'
    #2、仅具有type字段builtin_interfaces/msg/Time 
    ros2 topic pub /reference sensor_msgs/msg/TimeReference '{header: "auto", time_ref: "now", source: "dumy"}'
    ```
    
    

## 7、 编写话题发布者与订阅者

- ### Python：

  - ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, UInt32
    
    class WriterNode(Node):
        def __init__(self, name):
            super().__init__(name)
            self.get_logger().info("大家好，我是作家%s." % name)
            self.pub_novel = self.create_publisher(String, "sexy_girl", 10)
            self.rcv_money = self.create_subscription(UInt32, "sexy_girl_money", self.recv_money_callback, 10)
            self.account = 80
            self.cnt = 0
            timer_period = 5
            self.timer = self.create_timer(timer_period, self.timer_callback)
        
        def timer_callback(self):
            msg = String()
            msg.data = '第%d回：潋滟湖 %d 次偶遇胡艳娘' % (self.cnt,self.cnt)
            self.pub_novel.publish(msg)
            self.get_logger().info('li4: 我发布了艳娘传奇："%s"' % msg.data) 
            self.cnt+=1
    
        def recv_money_callback(self,money):
            self.account += money.data
            self.get_logger().info("li4: 我收到%d稿费，账户有%d元" % (money.data, self.account))
    
    def main(args = None):
        rclpy.init(args=args)
        node = WriterNode("li4")
        rclpy.spin(node)
        rclpy.shutdown()
    
    ```
  
- ### C++:

  - ```c++
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


## 8、接口相关：

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

- ### 自定义消息：

  - ```cmake
    find_package(geometry_msgs REQUIRED) #自定义消息
    find_package(rosidl_default_generators REQUIRED)
    #rosidl_default_generators 是 ROS 2 中用于生成消息和服务代码的工具。它可以根据 IDL（接口定义语言）文件生成对应的 C++ 和 Python 代码，使得用户可以方便地在 ROS 2 系统中使用自定义的消息和服务。
    rosidl_generate_interfaces(${PROJECT_NAME}
      #rosidl_generate_interfaces 中的第一个参数（库名称）必须与 ${PROJECT_NAME} 匹配
      "msg/Num.msg"
      "msg/Sphere.msg"
      "srv/AddThreeInts.srv"
      DEPENDENCIES geometry_msgs  
    )
    ```

  - ```xml
    <depend>geometry_msgs</depend>
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

- ### 自定义接口：

  - 类型
  
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
  
  - ```c
    //例子：Addressook.msg
    uint8 PHONE_TYPE_HOME=0
    uint8 PHONE_TYPE_WORK=1
    uint8 PHONE_TYPE_MOBILE=2
    
    string first_name
    string last_name
    string phone_number
    uint8 phone_type
    ```
  
    
  
  - ```cmake
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
  
    - ```cmake
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
      rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} rosidl_typesupport_cpp)
      
      target_link_libraries(publish_address_book "${cpp_typesupport_target}")
      ```
  
  - ```xml
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
  
- ### **导入接口的三个步骤**：

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

## 9、服务相关：

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

- ### **简单服务端**

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
            std::bind(&ServiceServer01::handle_add_two_ints, this,
                      std::placeholders::_1, std::placeholders::_2));
      }
    
    private:
      // 声明一个服务
      rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr
        add_ints_server_;
    
      // 收到请求的处理函数
      void handle_add_two_ints(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
        RCLCPP_INFO(this->get_logger(), "收到a: %ld b: %ld", request->a,
                    request->b);
        response->sum = request->a + request->b;
      };
    };
    ```

- ### 简单客户端

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
    
      void send_request(int a, int b) {
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
        client_->async_send_request(
          request, std::bind(&ServiceClient01::result_callback_, this,
                             std::placeholders::_1));
      };
    
    private:
      // 声明客户端
      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    
      void result_callback_(
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture
          result_future) {
        auto response = result_future.get();
        RCLCPP_INFO(this->get_logger(), "计算结果：%ld", response->sum);
      }
    };
    ```

  - [3.1.1 create_client](https://fishros.com/d2lros2/#/humble/chapt3/get_started/5.服务之RCLCPP实现?id=_311-create_client)

    ![image-20220606224628001](https://fishros.com/d2lros2/humble/chapt3/get_started/5.服务之RCLCPP实现/imgs/image-20220606224628001.png)

    参数加上ServiceT（接口类型），一共有四个，都是老熟人了，就不介绍了。

    #### [3.1.2 async_send_request](https://fishros.com/d2lros2/#/humble/chapt3/get_started/5.服务之RCLCPP实现?id=_312-async_send_request)

    接着我们来看看发送请求的API，[地址](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Client.html#a62e48edd618bcb73538bfdc3ee3d5e63)

    ![image-20220606224824684](https://fishros.com/d2lros2/humble/chapt3/get_started/5.服务之RCLCPP实现/imgs/image-20220606224824684.png)

    我们这里要用的是这个函数`async_send_request()`同时传入两个参数

    - request，请求的消息，这里用于放a，b两个数。
    - CallBack，回调函数，异步接收服务器的返回的函数。

    > 至于为什么ROS2中那么多回调函数，以及用回调函数的好处，小鱼这里就不解释了，不清楚的小伙伴可以看看基础篇的内容。

    #### [3.1.3 wait_for_service](https://fishros.com/d2lros2/#/humble/chapt3/get_started/5.服务之RCLCPP实现?id=_313-wait_for_service)

    这个函数是用于等待服务上线的，这个函数并不在rclcpp::Client中定义，而是在其[父类](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1ClientBase.html#a8b4b432338a460ceb26a7fa6ddd59e1d)中定义的。

    ![image-20220606225526647](https://fishros.com/d2lros2/humble/chapt3/get_started/5.服务之RCLCPP实现/imgs/image-20220606225526647.png)

    上面是继承图，在其父类中有这个函数的解释。

    ![image-20220606225411304](https://fishros.com/d2lros2/humble/chapt3/get_started/5.服务之RCLCPP实现/imgs/image-20220606225411304.png)

    参数就一个，等待的时间，返回值是bool类型的，上线了就是true，不上线就是false。

    之所以会用的这个函数的原因是，再发送请求之前保证服务端启动了，避免发送一个请求出去而无人响应的尴尬局面。

    #### [3.1.4 回调函数](https://fishros.com/d2lros2/#/humble/chapt3/get_started/5.服务之RCLCPP实现?id=_313-wait_for_service)

    回调函数`void result_callback_(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future)`

    函数的参数是客户端`AddTwoInts`类型的`SharedFuture`对象，这个对象的定义如下

    ![image-20220606230244527](https://fishros.com/d2lros2/humble/chapt3/get_started/5.服务之RCLCPP实现/imgs/image-20220606230244527.png)

    可以看到其又是利用C++11的新特性`std::shared_future`创建的`SharedResponse`类模板。

    类模板 `std::shared_future` 提供访问异步操作结果的机制，类似 [std::future](https://www.apiref.com/cpp-zh/cpp/thread/future.html) ，除了允许多个线程等候同一共享状态。

    我们具体看看[std::shared_future的API](https://www.apiref.com/cpp-zh/cpp/thread/shared_future.html)

    ![image-20220606231019702](https://fishros.com/d2lros2/humble/chapt3/get_started/5.服务之RCLCPP实现/imgs/image-20220606231019702.png)

    可以看到使用`get`函数即可获取结果。

## 10、参数相关

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

- ### 为参数设置描述符

  - ```c++
    // ...
    
    class MinimalParam : public rclcpp::Node
    {
    public:
      MinimalParam(）: Node("minimal_param_node")
      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter is mine!";
    
        this->declare_parameter("my_parameter", "world", param_desc);
    
        timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalParam::timer_callback, this));
      }
    ```

- ### 简单参数设置代码

  - ```cpp
    #include <chrono>
    #include "rclcpp/rclcpp.hpp"
    /*
        # declare_parameter            声明和初始化一个参数
        # describe_parameter(name)  通过参数名字获取参数的描述
        # get_parameter                通过参数名字获取一个参数
        # set_parameter                设置参数的值
    */
    class ParametersBasicNode : public rclcpp::Node {
     public:
      // 构造函数,有一个参数为节点名称
      explicit ParametersBasicNode(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
          
        this->declare_parameter("rcl_log_level", 0);     /*声明参数*/
        this->get_parameter("rcl_log_level", log_level); /*获取参数*/
          
        /*设置日志级别*/
        this->get_logger().set_level((rclcpp::Logger::Level)log_level);
          
        using namespace std::literals::chrono_literals;
        timer_ = this->create_wall_timer(
            500ms, std::bind(&ParametersBasicNode::timer_callback, this));
      }
    
     private:
      int log_level;
      rclcpp::TimerBase::SharedPtr timer_;
    
      void timer_callback() {
        this->get_parameter("rcl_log_level", log_level); /*获取参数*/
          
        /*设置日志级别*/
        this->get_logger().set_level((rclcpp::Logger::Level)log_level);
        std::cout<<"======================================================"<<std::endl;
        RCLCPP_DEBUG(this->get_logger(), "我是DEBUG级别的日志，我被打印出来了!");
        RCLCPP_INFO(this->get_logger(), "我是INFO级别的日志，我被打印出来了!");
        RCLCPP_WARN(this->get_logger(), "我是WARN级别的日志，我被打印出来了!");
        RCLCPP_ERROR(this->get_logger(), "我是ERROR级别的日志，我被打印出来了!");
        RCLCPP_FATAL(this->get_logger(), "我是FATAL级别的日志，我被打印出来了!");
      }
    };
    
    int main(int argc, char** argv) {
      rclcpp::init(argc, argv);
      /*创建对应节点的共享指针对象*/
      auto node = std::make_shared<ParametersBasicNode>("parameters_basic");
      /* 运行节点，并检测退出信号*/
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
    }
    ```

    


## 11、Action相关：

- ### Action的三大组成部分目标、反馈和结果。

  - 目标：即Action客户端告诉服务端要做什么，服务端针对该目标要有响应。解决了不能确认服务端接收并处理目标问题
  - 反馈：即Action服务端告诉客户端此时做的进度如何（类似与工作汇报）。解决执行过程中没有反馈问题
  - 结果：即Action服务端最终告诉客户端其执行结果，结果最后返回，用于表示任务最终执行情况。

  > 参数是由服务构建出来了，而Action是由话题和服务共同构建出来的（一个Action = 三个服务+两个话题） 三个服务分别是：1.目标传递服务    2.结果传递服务    3.取消执行服务 两个话题：1.反馈话题（服务发布，客户端订阅）   2.状态话题（服务端发布，客户端订阅）

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


- ### 简单action示例

  - ```cpp
    //robot.h
    #ifndef EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
    #define EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
    #include "rclcpp/rclcpp.hpp"
    #include "robot_control_interfaces/action/move_robot.hpp"
    
    class Robot {
     public:
      using MoveRobot = robot_control_interfaces::action::MoveRobot;
      Robot() = default;
      ~Robot() = default;
      float move_step(); /*移动一小步，请间隔500ms调用一次*/
      bool set_goal(float distance); /*移动一段距离*/
      float get_current_pose();
      int get_status();
      bool close_goal(); /*是否接近目标*/
      void stop_move();  /*停止移动*/
    
     private:
      float current_pose_ = 0.0;             /*声明当前位置*/
      float target_pose_ = 0.0;              /*目标距离*/
      float move_distance_ = 0.0;            /*目标距离*/
      std::atomic<bool> cancel_flag_{false}; /*取消标志*/
      int status_ = MoveRobot::Feedback::STATUS_STOP;
    };
    
    #endif  // EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
    ```

  - ```cpp
    //robot.cpp
    #include "example_action_rclcpp/robot.h"
    
    /*移动一小步，请间隔500ms调用一次*/
    float Robot::move_step() {
      int direct = move_distance_ / fabs(move_distance_);
      float step = direct * fabs(target_pose_ - current_pose_) *
                   0.1; /* 每一步移动当前到目标距离的1/10*/
      current_pose_ += step;
      std::cout << "移动了：" << step << "当前位置：" << current_pose_ << std::endl;
      return current_pose_;
    }
    
    /*移动一段距离*/
    bool Robot::set_goal(float distance) {
      move_distance_ = distance;
      target_pose_ += move_distance_;
    
      /* 当目标距离和当前距离大于0.01同意向目标移动 */
      if (close_goal()) {
        status_ = MoveRobot::Feedback::STATUS_STOP;
        return false;
      }
      status_ = MoveRobot::Feedback::STATUS_MOVEING;
      return true;
    }
    
    float Robot::get_current_pose() { return current_pose_; }
    int Robot::get_status() { return status_; }
    /*是否接近目标*/
    bool Robot::close_goal() { return fabs(target_pose_ - current_pose_) < 0.01; }
    void Robot::stop_move() {
      status_ = MoveRobot::Feedback::STATUS_STOP;
    } /*停止移动*/
    ```

  - ```cpp
    class ActionRobot01 : public rclcpp::Node {
     public:
      using MoveRobot = robot_control_interfaces::action::MoveRobot;
      using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;
    
      explicit ActionRobot01(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    
        using namespace std::placeholders;  // NOLINT
    
        this->action_server_ = rclcpp_action::create_server<MoveRobot>(
            this, "move_robot",
            std::bind(&ActionRobot01::handle_goal, this, _1, _2),
            std::bind(&ActionRobot01::handle_cancel, this, _1),
            std::bind(&ActionRobot01::handle_accepted, this, _1));
      }
    
     private:
      Robot robot;
      rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;
    
      rclcpp_action::GoalResponse handle_goal(
          const rclcpp_action::GoalUUID& uuid,
          std::shared_ptr<const MoveRobot::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with distance %f",
                    goal->distance);
        (void)uuid;
        if (fabs(goal->distance > 100)) {
          RCLCPP_WARN(this->get_logger(), "目标距离太远了，本机器人表示拒绝！");
          return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(),
                    "目标距离%f我可以走到，本机器人接受，准备出发！",
                    goal->distance);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
    
      rclcpp_action::CancelResponse handle_cancel(
          const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        robot.stop_move(); /*认可取消执行，让机器人停下来*/
        return rclcpp_action::CancelResponse::ACCEPT;
      }
    
      void execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "开始执行移动 %f 。。。", goal->distance);
    
        auto result = std::make_shared<MoveRobot::Result>();
        rclcpp::Rate rate = rclcpp::Rate(2);
        robot.set_goal(goal->distance);
        while (rclcpp::ok() && !robot.close_goal()) {
          robot.move_step();
          auto feedback = std::make_shared<MoveRobot::Feedback>();
          feedback->pose = robot.get_current_pose();
          feedback->status = robot.get_status();
          goal_handle->publish_feedback(feedback);
          /*检测任务是否被取消*/
          if (goal_handle->is_canceling()) {
            result->pose = robot.get_current_pose();
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
            return;
          }
          RCLCPP_INFO(this->get_logger(), "Publish Feedback"); /*Publish feedback*/
          rate.sleep();
        }
    
        result->pose = robot.get_current_pose();
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
      }
    
      void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
        using std::placeholders::_1;
        std::thread{std::bind(&ActionRobot01::execute_move, this, _1), goal_handle}
            .detach();
      }
    };
    
    ```

    Action使用了三个回调函数，分别用于处理收到目标、收到停止、确认接受执行。

    - handle_goal，收到目标，反馈是否可以执行该目标，可以则返回`ACCEPT_AND_EXECUTE`,不可以则返回`REJECT`
    - handle_cancel，收到取消运行请求，可以则返回`ACCEPT`，不可以返回`REJECT`。
    - handle_accepted，处理接受请求，当handle_goal中对移动请求`ACCEPT`后则进入到这里进行执行，这里我们是单独开了个线程进行执行`execute_move`函数，目的是避免阻塞主线程。

  - ```cpp
    class ActionControl01 : public rclcpp::Node {
     public:
      using MoveRobot = robot_control_interfaces::action::MoveRobot;
      using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;
    
      explicit ActionControl01(
          std::string name,
          const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
          : Node(name, node_options) {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    
        this->client_ptr_ =
            rclcpp_action::create_client<MoveRobot>(this, "move_robot");
    
        this->timer_ =
            this->create_wall_timer(std::chrono::milliseconds(500),
                                    std::bind(&ActionControl01::send_goal, this));
      }
    
      void send_goal() {
        using namespace std::placeholders;
    
        this->timer_->cancel();
    
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
          RCLCPP_ERROR(this->get_logger(),
                       "Action server not available after waiting");
          rclcpp::shutdown();
          return;
        }
    
        auto goal_msg = MoveRobot::Goal();
        goal_msg.distance = 10;
    
        RCLCPP_INFO(this->get_logger(), "Sending goal");
    
        auto send_goal_options =
            rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ActionControl01::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ActionControl01::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&ActionControl01::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
      }
    
     private:
      rclcpp_action::Client<MoveRobot>::SharedPtr client_ptr_;
      rclcpp::TimerBase::SharedPtr timer_;
    
      void goal_response_callback(GoalHandleMoveRobot::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "Goal accepted by server, waiting for result");
        }
      }
    
      void feedback_callback(
          GoalHandleMoveRobot::SharedPtr,
          const std::shared_ptr<const MoveRobot::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback current pose:%f", feedback->pose);
      }
    
      void result_callback(const GoalHandleMoveRobot::WrappedResult& result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
    
        RCLCPP_INFO(this->get_logger(), "Result received: %f", result.result->pose);
        // rclcpp::shutdown();
      }
    };  // class ActionControl01
    
    ```

    

## 12、rosbag2

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


## 13、广播坐标：

- 静态：

  - ```python
    #导入rclpy和Node
    import rclpy
    from rclpy.node import Node
    #导入TF帧：对应的消息接口为 geometry_msgs.msg 下的 TransformStamped
    from geometry_msgs.msg import TransformStamped
    #从 tf2_ros 包中导入静态坐标变换广播器
    from tf2_ros import StaticTransformBroadcaster
    #初始化节点
    rclpy.init()
    node = Node('transform_node')
    #构建静态广播发布器
    static_tf_pub = StaticTransformBroadcaster(node)
    #构造TF帧以及其要发布的消息
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    
    t.header.frame_id = 'B' #parent-name：基坐标系B
    t.child_frame_id = 'C'	#child-name：相机坐标系C
    
    #平移关系，单位m
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 3.0
    #旋转关系，四元数形式，我们需要将欧拉角的形式转换成四元数
    #http://quaternions.online/ 坐标转换工具
    t.transform.rotation.x = 1.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 0.0
    
    #发布坐标关系
    static_tf_pub.sendTransform(t)
    #使用命令行监听坐标关系：ros2 run tf2_ros tf2_echo B C
    ```

- 广播发布器：

  - ```python
    #导入rclpy和Node
    import rclpy
    from rclpy.node import Node
    #导入TF帧：对应的消息接口为 geometry_msgs.msg 下的 TransformStamped
    from geometry_msgs.msg import TransformStamped
    #从 tf2_ros 包中导入坐标变换广播器
    from tf2_ros import TransformBroadcaster
    #初始化节点
    rclpy.init()
    node = Node('transform_node2')
    #构建广播发布器
    tf_pub = TransformBroadcaster(node)
    #构造TF帧以及其要发布的消息
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    
    t.header.frame_id = 'C' #parent-name：相机坐标系C
    t.child_frame_id = 'P'	#child-name：工件坐标系P
    
    #平移关系，单位m
    t.transform.translation.x = 2.0
    t.transform.translation.y = 1.0
    t.transform.translation.z = 2.0
    #旋转关系，四元数形式，我们需要将欧拉角的形式转换成四元数
    #http://quaternions.online/ 坐标转换工具
    t.transform.rotation.x = 1.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 0.0
    
    #以10Hz发布坐标关系
    def send_transform():
        t.header.stamp = node.get_clock().now().to_msg()
        tf_pub.sendTransform(t)
    node.create_timer(0.1,send_transform)
    rclpy.spin(node)
    #使用命令行监听坐标关系：ros2 run tf2_ros tf2_echo C P
    ```

- 坐标变换监听：

  - ```python
    #导入rclpy和Node
    import rclpy
    from rclpy.node import Node
    #从tf2_ros中导入坐标变换监听器
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener
    #初始化节点
    rclpy.init()
    node = Node('transform_node3')
    #构建监听器
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    #监听回调函数
    def transform_callback():
        try:
            now = rclpy.time.Time()
            trans = tf_buffer.lookup_transform('B','P', now)
            print('trans',trans)
        except TransformException as ex:
            print(f'Could not transform B to P: {ex}')
    node.create_timer(1, transform_callback)
    rclpy.spin(node)
    
    ```


## 14、检查依赖：

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

## 15、创建插件：

```bash
#1
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --dependencies pluginlib --node-name area_node polygon_base
#2
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --dependencies polygon_base pluginlib --library-name polygon_plugins polygon_plugins
```

```xml
<library path="polygon_plugins">
  <class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
    <description>This is a square plugin.</description>
  </class>
  <class type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
    <description>This is a triangle plugin.</description>
  </class>
</library>
```



## 16、Launch

### [2.2 使用Python编写Launch](https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.启动管理工具-Launch?id=_22-使用python编写launch)

我们的目标是编写一个launch文件，最后使用launch指令，同时启动服务端和客户端节点。

#### [2.2.1 创建功能包和launch文件](https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.启动管理工具-Launch?id=_221-创建功能包和launch文件)

创建文件夹和功能包，接着touch一个launch文件，后缀为`.py`。

```
mkdir -p chapt5/chapt5_ws/src
cd chapt5/chapt5_ws/src
ros2 pkg create robot_startup --build-type ament_cmake --destination-directory src
mkdir -p src/robot_startup/launch
touch src/robot_startup/launch/example_action.launch.py
```

#### [2.2.2 启动多个节点的示例](https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.启动管理工具-Launch?id=_222-启动多个节点的示例)

我们需要导入两个库，一个叫做LaunchDescription，用于对launch文件内容进行描述，一个是Node，用于声明节点所在的位置。

> 注意这里要定一个名字叫做`generate_launch_description`的函数，ROS2会对该函数名字做识别。

```python
# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    action_robot_01 = Node(
        package="example_action_rclcpp",
        executable="action_robot_01"
    )
    action_control_01 = Node(
        package="example_action_rclcpp",
        executable="action_control_01"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [action_robot_01, action_control_01])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
```

#### [2.2.3 将launch文件拷贝到安装目录](https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.启动管理工具-Launch?id=_223-将launch文件拷贝到安装目录)

如果你编写完成后直接编译你会发现install目录下根本没有你编写的launch文件，后续launch自然也找不到这个launch文件。

因为我们用的是`ament_cmake`类型功能包，所以这里要使用cmake命令进行文件的拷贝

```cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
```

如果是`ament_python`功能包版

```python
from setuptools import setup
from glob import glob
import os

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
)
```

### [3 添加参数&修改命名空间](https://fishros.com/d2lros2/#/humble/chapt5/get_started/1.启动管理工具-Launch?id=_3-添加参数amp修改命名空间)

接着我们尝试使用launch运行参数节点，并通过launch传递参数，和给节点以不同的命名空间。

```python
# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    parameters_basic1 = Node(
        package="example_parameters_rclcpp",
        namespace="rclcpp",
        executable="parameters_basic",
        parameters=[{'rcl_log_level': 40}]
    )
    parameters_basic2 = Node(
        package="example_parameters_rclpy",
        namespace="rclpy",
        executable="parameters_basic",
        parameters=[{'rcl_log_level': 50}]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [parameters_basic1, parameters_basic2])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
```

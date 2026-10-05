# nav_lecture4小作业：Qos_debugger
这道题的目标就是让你快速上手、理解什么是ros。抛开复杂的概念，ros本质上完成的任务就是便利的进程间通信。比如，我有两个进程，一个进程发布雷达数据，另一个进程接收。使用ros就可以方便的完成通讯。你可以搜索以下，发送信息有哪些类型，分别有何特点。特别注意，不同的信息传输方式有不同的质量要求。你不会允许送的外卖没到你手上，但是一个电话过来，也许漏接了也无所谓，可能只是个诈骗。ros2也是这样。重点关注这一点会对这道题有所帮助

> 环境要求：ROS2 Humble

## 包结构

```
src/
  nav_hw_interfaces/     # 接口包：只放 .msg，无业务代码
    msg/SensorData.msg   # 可以打开.msg文件查看接口详细内容
  qos_debugger/          # 业务节点包
    src/qos_debugger_pub.cpp   # 发布 /SensorData
    src/qos_debugger_sub.cpp   # 订阅 /SensorData
```

## 编译

```bash
cd lecture4/homework
colcon build 
source install/setup.bash
```


## 任务一：实现pub和sub的通信

```bash
ros2 -h     //有忘记的命令就输入-h去查询用法
```

**现象**：启动pub和sub节点后sub节点订阅不到任何消息
提示：如果两个节点不能通过话题通信，我们应该如何区查看话题的详细信息（有没有相关的命令）
任务一仅修复qos_debugger_pub.cpp的一处或几处代码即可完成


---

## 任务二：为什么收到的消息会丢包？/(ㄒoㄒ)/~~

第一问找到问题并修改代码后，记得重新
```colcon build```
```source install/setup.bash```
**现象**：sub会打印黄色的warning输出告诉你丢包的序列，每秒还会打印出丢包率

提示：
有没有什么命令可以查看节点的配置(ros2 param -h)
可以通过修复qos_debugger_sub.cpp中的一处或几处代码解决该问题（可能会有多种解决方法）

## 任务三：把收到的消息的帧率计算并打印出来（放在定时器回调函数中每秒打印一次即可）
补全qos_debugger_sub.cpp即可



在下面按顺序完成三个任务，要求把用到的命令放入代码块中并讲解命令，每一问最好加入自己的理解



## 任务一完整流程（从“收不到消息”到定位问题）
### 步骤1：确认节点都启动了
```bash
ros2 node list
```
**命令讲解**：列出当前所有正在运行的ROS2节点。
输出应该看到：
```
/sensor_publisher
/sensor_subscriber
```
证明pub和sub两个进程都正常启动了，没有崩溃退出。

### 步骤2：确认话题存在
```bash
ros2 topic list
```
**命令讲解**：列出当前系统中所有已存在的话题。
输出里要能看到 `/sensor_data`，说明发布者成功创建了话题，不是话题名写错了。

### 🔴 步骤3（任务一核心排查命令）：查看话题QoS详情
```bash
ros2 topic info /sensor_data --verbose
```
**命令讲解**：
- `ros2 topic info 话题名`：查看话题基础信息（有几个发布者、几个订阅者）
- `--verbose`：开启详细模式，打印**发布者和订阅者双方完整的QoS配置**，包括可靠性策略、队列策略，这是定位QoS不兼容问题的核心工具。

#### 最初看到的输出：
```
Publisher QoS:
  Reliability: BEST_EFFORT       # 发布者：尽力传输
  History (Depth): UNKNOWN

Subscriber QoS:
  Reliability: RELIABLE           # 订阅者：要求可靠传输
  History (Depth): UNKNOWN
```
定位问题：Pub是`BEST_EFFORT`，Sub是`RELIABLE`，QoS可靠性不兼容，所以无法正常通信，sub完全收不到消息。

### 步骤4：修复pub代码
把发布端默认可靠性从`best_effort`改成`reliable`，和订阅者保持一致，重新编译运行，两端QoS匹配，就可以正常收到消息。

任务一完成






# 任务二：分析丢包现象，理解KeepLast队列深度depth
> 实验目标：通过`ros2 param`相关命令查看节点参数，观察丢包现象，理解`depth`仅能缓冲瞬时峰值，无法根治持续过载。
> 原理回顾：`callback_delay_ms`模拟回调处理耗时；当消息到达速率持续大于回调处理速率（ρ>1），消息会在`KeepLast(depth)`队列持续堆积，队列满后丢弃最早消息，触发黄色丢包警告。

## 🔴 核心命令（ros2 param）
```bash
ros2 param list /sensor_subscriber
```
命令讲解：`ros2 param list`用于查看指定节点暴露的所有参数名称。这里查看`sensor_subscriber`节点，可以看到`depth`、`callback_delay_ms`、`reliability`这三个我们定义的参数，确认节点成功加载了代码声明的参数。

```bash
ros2 param get /sensor_subscriber depth
```
命令讲解：`ros2 param get /节点名 参数名`，读取节点当前该参数的**运行时值**。用来查看当前订阅节点正在使用的队列深度depth，验证代码里写的默认参数是否生效。

```bash
ros2 param get /sensor_subscriber callback_delay_ms
```
命令讲解：读取`callback_delay_ms`的当前值，确认回调休眠参数是否为代码设置的默认值。

## 辅助验证命令
```bash
ros2 topic info /sensor_data --verbose
```
命令讲解：查看话题的发布者、订阅者QoS详细信息，核对订阅端KeepLast队列配置。

## 核心现象
当 `callback_delay_ms=30`，发布端 100Hz 发消息：
消息到达速率（100 条 / 秒）＞回调处理速率（≈33 条 / 秒），属于**持续过载**。消息不断堆积在`KeepLast(depth)`队列，队列满之后，新来消息会丢弃队列里最早的消息，打印黄色丢包警告。
哪怕把`depth`调到 100，仅仅是缓冲区变大，可以多存一些消息，**无法改变 “来的比处理的快” 这个根本矛盾**，一段时间之后依旧会丢包。

## 两种解决思路（区分「临时缓解」和「根治」）

### 方案 1：临时缓解（治标，不推荐作为最终方案）
调大`depth`。
原理：扩大消息等待队列的缓冲区，可以容纳更多积压消息，**推迟丢包出现的时间**。
缺点：只要消息持续过载，队列终究会填满，丢包迟早发生，不能解决根本问题。
> 适用场景：只用来应对短暂的瞬时消息峰值，不能处理长期过载。

### 方案 2：根治丢包
降低`callback_delay_ms`，把它设置为 0。
原理：`callback_delay_ms`是回调内部模拟的消息处理耗时。设置为 0，`sleep`不再执行，回调处理消息速度极快，**消息处理速率 > 消息到达速率（100Hz）**。消息几乎不需要排队，队列不会持续堆积。此时`depth`只需要很小的值（例如 4,甚至调为0都行），就可以吸收系统调度带来的微小抖动，全程不会丢包。

## 个人理解
1. `rclcpp::KeepLast(depth)`的depth是订阅端等待回调处理的消息队列最大长度。回调繁忙时新消息排队；队列存满，新来消息丢弃最旧消息。
3. 排队论：消息到达速率持续大于处理速率（ρ>1），队列长度会持续上涨。无论depth设置多大有限值，缓冲区最终都会被填满。增大depth只能推迟丢包发生的时间，治标不治本。想要彻底消除过载丢包，要减少回调内部阻塞`callback_delay_ms`，提升消息处理速度,或者是多线程同时处理消息。`depth`只用来应对短时间消息峰值。

任务二完成





# 任务三：统计并打印消息接收帧率

> 
> 实验目标：在report定时器回调函数中，利用`std::chrono::steady_clock`记录时间点，结合消息接收计数，计算并输出消息接收帧率，评估消息接收速度。
> 原理回顾：帧率 = 这段时间收到的消息数量 / 两次统计之间真实的时间间隔。使用`steady_clock`，不受系统时间校正影响，可以获取稳定的时间差，相比假定定时器严格1秒，计算结果更加准确。

## 代码修改部分（修改report函数）

```
void report()
{
  const uint64_t total = static_cast<uint64_t>(received_count_) + lost_count_;
  const double loss_rate = (total == 0) ? 0.0 : 100.0 * lost_count_ / total;
  RCLCPP_INFO(
      this->get_logger(),
      "累计: 收到 %u 条, 丢失 %u 条, 丢包率 %.2f%%",
      received_count_, lost_count_, loss_rate);

  // 任务3新增帧率计算代码
  auto now = std::chrono::steady_clock::now();
  double delta_sec = std::chrono::duration<double>(now - last_report_time_).count();
  uint32_t msg_in_period = received_count_ - last_received_count_;
  double fps = static_cast<double>(msg_in_period) / delta_sec;

  RCLCPP_INFO(
      this->get_logger(),
      "接收帧率：%.2f Hz",
      fps);

  last_received_count_ = received_count_;
  last_report_time_ = now;
}
```

类内已经预先定义好两个成员变量，无需新增：

```
uint32_t last_received_count_{0};
std::chrono::steady_clock::time_point last_report_time_{std::chrono::steady_clock::now()};
```

## 个人理解

1. `steady_clock`是稳定时钟，只会单向递增，不受系统时间、NTP校时干扰，适合用来计算时间间隔。
2. `last_report_time_`保存上一次统计的时间点，`last_received_count_`保存上一次统计时收到的消息总数。每次进入report，获取当前时间`now`，算出两次report之间真实时间差，再用消息增量除以时间差得到真实帧率。
3. 不直接假定定时器间隔固定为1秒。操作系统调度存在微小抖动，采用时间戳计算，鲁棒性更强。
4. 实验现象：发布端100Hz发送消息，在无丢包的情况下，打印帧率稳定在100Hz附近；如果系统过载丢包，帧率会低于100Hz。

## 实验现象总结

- `callback_delay_ms=0`：无丢包，接收帧率稳定接近100.00 Hz。
- `callback_delay_ms=30`：持续丢包，接收帧率明显低于100Hz。

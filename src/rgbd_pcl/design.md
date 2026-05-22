
# 问题定义：
求出能量棒的法线，供下一步进行顶部圆识别；

# 可行性分析：
不行也得行

# 需求分析：
时间、空间复杂度不限（比较快会好一点）；

精度要求： 不高，角度在±5°之间，距离差在±3cm之间就可；方向不需要，让操作手来选

# 总体设计
使用增量模型，一个个模块开发，开发好再进行下一个
## 类的设计（类图）
基类：
* MidCylinder，用于定义能量棒中间圆柱体。包含：
  * 属性：法线、顶点、status(用于记录目前是到哪一阶段)；
  * 方法：updateNormalLine()（可以通过输入不同的参数来决定如何更新，到达哪一个阶段。），
* SideBar，用于定义能量棒侧灯条。
  * 属性：一个轮廓，记录侧灯条的轮廓点；侧灯条的法向量；计算的中心法向量。
  * x方法：SideBarDetect()，计算侧灯条并且返回，可以给MidCylinder用。
* Donut，用于定义能量棒顶部圆环
  * 属性：一个轮廓，记录顶部圆环的轮廓点；圆环的法向量；
  * x方法：DountDetect()，计算圆环并且返回一个类，可以给MidCylinder用。
* Square，用于定义能量棒底部方形
  * 属性：一个轮廓，记录底部方形的轮廓点；方形的法向量；
  * x1方法：SquareDetect()，计算方形并且返回，可以给MidCylinder用。

## 时序图设计（流程图）
控制发信息开深度相机-初始化一个MidCylinder，试图识别能量棒侧面灯条获得圆柱体法线-更新状态-控制
将机械臂移动并拍摄顶部圆环、识别顶部圆环获得法线-更新状态-利用顶部圆环和底部方形更新法线-结束
## 数据流图+数据字典




## 整个系统的状态转换图
## 整个系统的模块结构图


# notes:

接收服务-启动节点-先两个相机都识别一次同心圆，没有结果就从头开始：
节点中接收图像进callback-识别侧灯条计算法线-更新MidCylinder状态-
发法线给控制-控制挪好机械臂之后发一个信息给我-接收到信息之后更新状态，开始计算顶部圆环法线-
计算法线之后实时更新MidCylinder状态，发到topic上-控制挪位置直到法线满足某个指向-
满足指向后更新状态-五点计算最终法线-发最终法线到topic上-结束

可以在每一个callback前面写一个判断，当前状态是否需要进行计算，当前状态需要进行哪种计算。不需要直接return



问题定义-可行性分析-需求分析-总体设计-详细设计-编码-测试


状态：
* 未识别
* 正相机尝试识别同心圆
* 正相机尝试识别侧灯条
* 正相机识别到侧灯条
* 侧相机正在识别顶部圆环
* 侧相机已识别顶部圆环
* moving？
* 侧相机正在识别底部方形
* 侧相机已识别底部方形

# Q
* 图像怎么传进来？
```
给两个相机话题写两个callback，
callback里判断当前状态，执行不同代码。
注意：可能需要绑定深度图像和rgb图像的callback。

1 xxxxx写全局变量，法线、参考点、参考系、全部写全局变量，
其余的东西全部放到能量棒类中，包括三个特征的类

每一个状态写一个服务的callback，每到一个状态让控制call一次服务，这边只负责做识别和结算
图像、相机信息在每个服务callback里订阅
法线写在每个特征类里
注意：这个只能识别一帧图像，不能连续识别图像
五点pnp：在底部方形的函数参数中加一个圆
注意：每次call服务之前要清空一次状态；服务返回值可能要加一个bool值

2 图像callback放在能量棒类中，将图像转存到能量棒类中；
每个特征类写一个set_image函数（公有，接收对应的图像类型），将能量棒类里的图像传入每个特征类自己的图像指针；
3 每个特征类自己写一个callback接收函数，再和一个状态话题的callback绑定，仅当两个都满足时才接收图像进callback。
························
4 使用类的继承，各个特征类继承能量棒类，能量棒类中声明一个static图像变量，
同时将法线、状态等变量也声明为static。
其实这个是类似全局变量的思想
每个特征类的callback直接使用这些static变量。
每个特征类单独开一个cpp，各自跑一个节点。
  能量棒节点：接收服务，管理控制交接过来的状态，收发图像等话题
  特征类节点：使用static接收图像状态等，进行实际识别和位姿解算，传法线到能量棒节点。
  
5 改进：开一个基类，放图像、状态、法线等static变量，能量棒类和特征类都继承这个基类。
可以避免能量棒类的自用函数被特征类继承的问题。


```
* 怎么触发每一个特征的callback？ 每个特征不需要callback，通过服务改变状态来触发特征类的函数。v
  * 用for循环来识别多帧？
  * 服务callback里写图像callback，每次识别一帧
* 各个类之间的关系是怎样的？v
* 在哪里触发服务？v
* 在哪里调用动态调参？v 
  * 自瞄识别的做法：一开始先从参数服务器读入参数变量，之后动态调参第一次会把参数变量覆盖到动态调参的config，之后则是用config覆盖参数变量。
  * 
* 如何传出状态？ 
* 如何在同心圆和底部方形之间传数据？ 
```

```
```
  特征类中想办法传出、
  在能量棒类中对法线进行判断合法后进行更新 --negative，因为重投影之类的需要写在特征类中
```

控制call服务->设置为callback中tag为true->放开topic接受图像传到img里面->大类调用函数进行识别解算获取法线，return到服务上->改tag为false


#include <memory>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace vision_exchanger {

// 需要复用 nh 的业务类：通过构造函数注入
class HsvModule {
public:
HsvModule(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh) {
// 示例：使用同一个 nh 创建 pub/sub/参数等
// pub_ = nh_.advertise<std_msgs::String>("topic", 1);
// pnh_.param("some_param", some_param_, 0);
}

private:
ros::NodeHandle nh_;
ros::NodeHandle pnh_;
// ros::Publisher pub_;
// ros::Subscriber sub_;
};

// Nodelet：在 onInit() 获取句柄并注入
class AssemblyPost : public nodelet::Nodelet {
public:
void onInit() override {
ros::NodeHandle nh = getNodeHandle();
ros::NodeHandle pnh = getPrivateNodeHandle();

        // 构造/重置成员对象，把 nh 注入进去
        module_ = std::make_unique<HsvModule>(nh, pnh);

        // 如果你自己也要长期用 nh/pnh，可以存成成员（可选）
        nh_ = nh;
        pnh_ = pnh;
    }

private:
ros::NodeHandle nh_;
ros::NodeHandle pnh_;
std::unique_ptr<HsvModule> module_;
};

} // namespace vision_exchanger


在 nodelet 里不要自己 ros::NodeHandle nh; 全局建句柄，而是在 onInit() 用 getNodeHandle() / getPrivateNodeHandle() 获取，然后把它们通过构造函数注入到成员对象里（成员对象保存一份 ros::NodeHandle 拷贝即可，共享同一底层上下文）。
下面给一个最小可用骨架：AssemblyPost 在 onInit() 获取 nh / pnh，并用它们构造业务模块成员。

std::unique_ptr<Tracker> tracker_;
tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_distance, tracking_threshold, lost_threshold,
tracker_nh);
这种写法就是：声明了一个智能指针，随后用 std::make_unique 创建对象并用后面的参数初始化，并赋值给智能指针。
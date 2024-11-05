为什么后端比前端优化器好？
为什么loam不能做回环检测？

# 多传感器融合

缺点就是现有的通用的视觉 slam 技术依赖图像的纹理来进行特征点的提取，没有纹理或者黑夜图像就很难被很好的利用。

图像中缺乏3d 信息，通常建模 slam 问题需要同时优化位姿和地图点。另一方面，单目图像缺乏尺度信息。

视觉SLAM： 常见的方式是一个视觉特征点前端（当然还有基于直接法的前端，如DSO），通过光流或者描述子建立不同帧特征点之间的关联，后端根据前端特征关联的结果和其他传感器数据进行融合，根据融合的方式分为基于优化的后端（ORBSLAM2、3, VINS-MONO，VINS-FUSION）以及基于滤波的后端（MSCKF），视觉通常会提供一个重投影误差作为约束或者更新量。

激光 slam 方向：目前性能最好使用最广的激光 slam 方案是基于 LOAM 的系列方案，LOAM 主要是为多线激光雷达设计的 lidar 定位和建图的方案，当然，由于现在其他一些 lidar 硬件的推出，一些 LOAM 的改进版本也是适当推出，如（Livox LOAM）。

基于 LOAM 方案通常前端是对当前帧激光雷达提取特征（通常是面特征和线特征），通常后端结合其他传感器信息给当前帧到地图中的匹配提供一个良好的初值（激光slam 中最重要的事情就是给 scan matching 提供一个更准确的 init guess）


1、激光雷达需要运动补偿，我们需要短期内可靠的运动观测源，IMU 以及轮速就可以被充分利用

2、激光雷达匹配本质上是一个优化问题，需要提供一个很好的初始值，和 1 一样，也是需要可靠的短期运动观测源，紧耦合的 IMU 融合或者轮速也是非常好的处理方式

3、激光雷达频率不高，为了提高频率，我们需要高频的其他传感器以获得高频输出，此时 IMU 和轮速又可以成为备选

4、里程记会有累计漂移的问题，全局观测是解决里程记该问题的非常好的方式，GPS作为常见的全局定位传感器，提供了修正累计漂移的功能

#### 1. LOAM & A-LOAM

![Screenshot from 2024-04-10 02-41-09](https://github.com/countsp/SLAM-learning/assets/102967883/842c58b1-b7d9-4520-9263-fa1e3c02bd27)

非常经典的激光里程记和建图方案，也是其他 LOAM 方案的鼻祖，LOAM 只基于激光雷达（可选 IMU），通过把 SLAM 拆分成一个**高频低精**的scan-to-scan前端(帧间里程计)以及一个**低频高精**的scan-to-map后端,来实现 lidar 里程记的实时性。

#### 2.LeGO-LOAM

![Screenshot from 2024-04-10 02-42-53](https://github.com/countsp/SLAM-learning/assets/102967883/8bbbc636-e3c5-4ce4-8c29-9484dbe62582)

在原有 LOAM 基础上，在前端增加了**地面点提取**，并且根据嵌入式平台的计算特点，将前端做的**更加轻量级**，以便于在算力有限的平台上可以实时运行，后端将其使用slam 中关键帧的定义进行重构，同时增加**回环检测和回环位姿图优化**，使得地图的全局一致性更优。

1、利用车载激光大多水平安装的特征，提取出地面点

2、使用聚类算法，使得前端特征更为干净

3、前端使用两步优化方法，减少运算负载，使其在嵌入式平台上也能运行

4、后端引入关键帧概念，同时加入了回环检测

 #### 3.LIO-SAM
 ![Screenshot from 2024-04-10 02-44-28](https://github.com/countsp/SLAM-learning/assets/102967883/4fb7b2ce-ac9f-40aa-8196-62e1f7d3b0ee)
 
在 LOAM 基础上采用紧耦合的 imu 使用方式，放弃了帧间里程记作为前端里程记，而使用紧耦合后 imu 的预测值作为前端里程记，后端沿用 LeGO-LOAM，同时加入了对GPS信号的适配，使得其全局一致性更出色。

1、由于其支持手持设备，因此没有对地面点进行特殊处理

2、紧耦合的 lidar+IMU 融合模块，使得其充分利用 IMU 的数据，对快速旋转等场景有着更好的鲁棒性

3、融合 GPS，使得全局地图可以在没有回环的情况下有着更好的全局一致性

4、易于扩展的框架，方便我们将其他传感器融合进来

总体来说，这三种框架随着时间顺序都是在前有基础进行的改造，因此，都是在吸收
现在框架基础上进行的改进

#### 4.LVI-SAM
![Screenshot from 2024-04-10 02-47-17](https://github.com/countsp/SLAM-learning/assets/102967883/abecebd9-1398-48eb-add9-209a0bc1cd0d)

LVI-SAM 是一种视觉、激光、IMU 三种传感器紧耦合的里程计框架，他是由两个独立的里程计（VINS-Mono 以及 LIO-SAM）融合而成，视觉里程计给激光里程计提供高频先验位姿，回环检测，而激光里程计给视觉里程计提供初始化的先验以及特征点深度信息。

1、加入视觉里程计部分，构成了紧耦合的视觉-激光-惯性里程计

2、利用视觉、激光互补的特征，取长补短，推进整体系统的精度和鲁棒性



---

* **[LOAM/A-LOAM](#LOAM/A-LOAM)**
* **[LeGO-LOAM](#LeGO-LOAM)**
* **[LIO-SAM](#LIO-SAM)**
* **[VINS-Mono](#VINS-Mono)**
* **[LVI-SAM](#LVI-SAM)**



## LOAM/A-LOAM

**地图构成：**

在原始 LOAM 中，使用的是基于珊格的地图存储方式。具体来说，将整个地图分成 21×21×11 个珊格，每个珊格是一个边长 50m 的正方体，当地图逐渐累加时，珊格之外的部分就被舍弃。

如果当前位姿远离的珊格覆盖范围，则地图也就没有意义了，因此，珊格地图也需要随着当前位姿动态调整，从而保证我们可以从珊格地图中取出离当前位姿比较近的点云来进行 scan-to-map 算法，借以获得最优位姿估计。

scan-to-map 算法使用5×5×3的栅格局部地图与scan匹配。

**前端**

在前端里程记部分，我们通过当前帧的线特征和面特征分别和上一帧的线特征和面特征进行匹配，构建约束，然后进行优化求解。

**线特征的提取**

通过 kdtree 在地图中找到 5 个最近的线特征，为了判断他们是否符合线特征的特性，我们需要对其进行特征值分解，通常来说，当上述 5 个点都在一条直线上时，他们只有一个主方向，也就是特征值是一个大特征值，以及两个小特征值，最大特征值对应的特征向量就对应着直线的方向向量。

方法为：特征值分解

**面特征的提取**

同样首先通过 kdtree 在地图中找到最近的面特征，原则上面特征也可以使用特征值分解的方式，选出最小特征值对应的特征向量及平面的法向量，不过代码里选用的是平面拟合的方式：

我们知道平面方程为𝐴𝑥 + 𝐵𝑦 + 𝐶𝑧 + 𝐷 = 0，考虑到等式的形式，可以进一步写成，𝐴𝑥 + 𝐵𝑦 + 𝐶𝑧 + 1 = 0，也就是三个未知数，五个方程，写成矩阵的形式就是一个5×3大小的矩阵，求出结果之后，我们还需要对结果进行校验，来观察其是否符合平面约束，具体就是分别求出 5 个点到求出平面的距离，如果太远，则说明该平面拟合不成功。

方法为：求解超定方程

**流程：**

1.选择面点、角点。16线Velodyne一圈1800点等分为4等分，每一份最多2角点、四个平面点。

![Screenshot from 2024-04-10 03-28-05](https://github.com/countsp/SLAM-learning/assets/102967883/92e2436f-eb7a-4ae3-a2e4-9d46de53e081)

通过同一个平行线上邻近两点到雷达的距离差来判定角点和面点。面点的c小，角点的c大。在将一段范围内点的c排序后找到最大的角点。


2.删除异常点(LIO-SAM中实现，A-LOAM无)

![Screenshot from 2024-04-10 03-35-35](https://github.com/countsp/SLAM-learning/assets/102967883/dc8b31fe-41f7-49c2-a199-56f1d5aeda50)

(a）中的B是面点，但是与射线平行，删去

LIO-SAM中用当前点与左右点的相对距离差判定: 如果diff > 0.02 * cloudInfo.pointRange[i]，即两点距离比较大，则可能是平行的点，则当前点设置为无效点，不做特征计算。

```
float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
cloudNeighborPicked[i] = 1;
```
(b)中的A断层了，不稳定，删去

LIO-SAM中用相邻两点的绝对距离差判定: 如果depth1 - depth2 > 0.3 ， 则depth 1容易被遮挡，则之前五个点设置为无效点,不做特征计算。

```


if(depth1 - depth2 > 0.3)
{
cloudNeighborPicked[i-5]=1;
cloudNeighborPicked[i-4]=1;
cloudNeighborPicked[i-3]=1;
cloudNeighborPicked[i-2]=1;
cloudNeighborPicked[i-1]=1;
cloudNeighborPicked[i]=1;
}
```
3. 特征提取

传统方法（通过曲率计算平坦/曲率较大）

```
for (int i = 5; i < cloudSize - 5; i++)
    { 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
```
4. 特征点匹配

将线点和面点放在KD-tree中，使用KD-tree寻找两帧之间的匹配对

![Screenshot from 2024-04-10 03-44-05](https://github.com/countsp/SLAM-learning/assets/102967883/84439891-a796-44d7-b871-8d3e41925b7c)

匹配线寻找：i是当前帧k+1帧线点的一部分，找到上一帧（k帧）的线做匹配：在上一帧k帧找离i最近的线点j，再在j周边找最近的线点l（j与l不同扫描线），jl构成线，i与线jl构成残差。

匹配面寻找：i是当前帧k+1帧面点的一部分，找到上一帧（k帧）离i最近的面点j，同一扫描线找j的最近面点l，与i不同扫描线上最近面点m，jlm构成面，i与jlm构成残差。

![Screenshot from 2024-04-10 04-18-17](https://github.com/countsp/SLAM-learning/assets/102967883/fae738d8-3758-468a-8aa4-a605cf13e817)

![Screenshot from 2024-04-10 04-21-34](https://github.com/countsp/SLAM-learning/assets/102967883/a50800ff-1cc4-4c4b-abfd-50aa58ec4cde)

点到线距离计算：k帧中有点j,l;k+1帧中有点i

点到面距离计算：k帧中有点j,m,l;k+1帧中有点i

每个特征点进行特征匹配，通过寻找上一帧的对应点（如边缘点和平面点），构建残差，然后将这些残差项放入Ceres中进行优化。

5.运动估计

![Screenshot from 2024-04-10 04-55-07](https://github.com/countsp/SLAM-learning/assets/102967883/d8136bb7-0ab2-45d0-993d-22e5635a3882)

线性插值法

![Screenshot from 2024-04-10 05-01-12](https://github.com/countsp/SLAM-learning/assets/102967883/7e8289b0-c285-472a-b9dc-0cc573e90077)

![Screenshot from 2024-04-10 05-02-53](https://github.com/countsp/SLAM-learning/assets/102967883/a820a630-c7a1-4620-8d02-cb525c664da2)

通过残差最小化估计运动

论文中使用LM，代码用高斯牛顿法

6.运动补偿

运动补偿的目的就是把所有的点云补偿到某一个时刻，这样就可以把本身在过去100ms 内收集的点云统一到一个时间点上去

比如一种的做法是补偿到起始时刻
```
𝑃𝑠𝑡𝑎𝑟𝑡 = 𝑇_𝑠𝑡𝑎𝑟𝑡_𝑐𝑢𝑟𝑟𝑒𝑛𝑡 ∗ 𝑃𝑐𝑢𝑟𝑟𝑒𝑛𝑡
```
因此运动补偿需要知道每个点该时刻对应的位姿𝑇_𝑠𝑡𝑎𝑟𝑡_𝑐𝑢𝑟𝑟𝑒𝑛𝑡,通常有几种做法

1、如果有高频里程记，可以比较方便的获取每个点相对起始扫描时刻的位姿

2、如果有 imu，可以方便的求出每个点相对起始点的旋转

3、(A-LOAM使用)如果没有其他传感器，可以使用匀速模型假设，使用上一个帧间里程记的结果作为当前两帧之间的运动，同时假设当前帧也是匀速运动，也可以估计出每个点相对起始时刻的位姿

```
Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr); //slerp: Spherical linear interpolation 插值
Eigen::Vector3d t_point_last = s * t_last_curr;
```
7.建图(后端)
 
同样找线点与面点，特征点数量为前端的10倍，同样用KD-tree与栅格地图找匹配对，方法为3D KD-tree

对协方差矩阵进行分解，分解为特征值和特征向量。如果是线点，一个特征值明显大于其余两个。如果面点，两个特征值比较大，小的特征值对应特征向量是面的法向量。

8.ceres 优化
```
ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);   //定义核函数
ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization(); //旋转不满足一般意义的加法，使用ceres自带的工具
ceres::Problem::Options problem_options;

ceres::Problem problem(problem_options);

problem.AddParameterBlock(para_q, 4, q_parameterization); // 待优化变量为para_q数组的前四个变量，与para_t数组的前三个变量
problem.AddParameterBlock(para_t, 3);
```

---

## LeGO-LOAM
![Screenshot from 2024-04-16 03-11-12](https://github.com/countsp/SLAM-learning/assets/102967883/ff46158b-24aa-4110-96ff-cfdedee72365)

要求lidar水平放置

**前端**

1. 对**地面点进行分类和提取**，避免一些边缘点的提取

2. 应用了一个简单的点云聚类算法，剔除了一些可能的 outlier 
  
3. 两步迭代求解前端帧间里程记，不影响精度的情况下减轻计算负载，保障了嵌入式平台的实时性

**后端**

1. 使用 slam 中关键帧的概念对后端部分进行了重构，将点云与关键帧的地图相匹配，而不是最近的栅格地图，则可以做回环。
   
2. 引入回环检测和位姿图优化概念，使得地图的全局一致性更好。用当前帧与历史帧ICP匹配、gtsam优化。


**地面分离**

![Screenshot from 2024-04-16 05-18-33](https://github.com/countsp/SLAM-learning/assets/102967883/ae9915ed-5a68-4dd3-b1c0-d93b7404e3ea)

如上图，相邻的两个扫描线束的同一列打在地面上如 AB 点所示，他们的垂直高度差ℎ = |𝑧0 − 𝑧1|，水平距离差𝑑 = √(𝑥0 − 𝑥1)2 + (𝑦0 − 𝑦1)2，计算垂直高度差和水平高度差的角度，𝜃 = 𝑎𝑡𝑎𝑛2(ℎ, 𝑑)，理想情况下，𝜃应该接近 0,考虑到一方面激光雷达安装也无法做到绝对水平，另一方面，地面也不是绝对水平，因此，这个角度会略微大于 0,考虑到作者实际在草坪之类的场景下运动，因此这个值被设置成10度，小于10度则考虑为地面点。

```
angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

if (abs(angle - sensorMountAngle) <= 10){
    groundMat.at<int8_t>(i,j) = 1;
    groundMat.at<int8_t>(i+1,j) = 1;
```

**广度有限BFS**
在 LeGO-LOAM 中，BFS 算法用于实现一个简单的点云聚类功能。

广度优先遍历（BFS）和深度优先遍历（DFS）同属于两种经典的图遍历的算法。

![Screenshot from 2024-04-17 01-27-55](https://github.com/countsp/SLAM-learning/assets/102967883/7a05f70c-3505-47e8-af59-8623a8940c05)

具体到广度优先遍历算法来说，首先从某个节点出发，一层一层地遍历，下一层必须等到上一层节点全部遍历完成之后才会开始遍历。

比如在上面这个无向图中，如果我们从 A 节点开始遍历，那么首先访问和 A 节点相邻的节点，这里就是 S、B、D，然后在访问和 S、B、D 相邻的其他节点，这里就是 C，因此，遍历的顺序是 A->S->B->D->C;如果我们从 S 开始遍历，则顺序就是 S->A->B->C->D；可以看到，不同的起始点对应的遍历顺序是不同的。

BFS 算法适用于图数据结构，为了把单帧 lidar 点云运用上 BFS 算法，首先需要将其建模成一个图模型，一个很简单有效的办法就是将其投影到一个平面图上，以velodyne-16 为例，我们将其投影到一个 16×1800 大小的图上（这里 16 是一共有 16跟线束，1800 是因为水平分辨率是 0.2 度，一个扫描周期有 1800 个点）如图

![Screenshot from 2024-04-17 01-38-52](https://github.com/countsp/SLAM-learning/assets/102967883/142fa0ca-8cbe-4025-b137-31e3dce1595a)

分别判断近邻和自身距离是否足够近，如果angle 越大（大于60度）则认为两点越可能是同一个聚类物体上的点，则打上同样的 label。

![Screenshot from 2024-04-17 01-40-14](https://github.com/countsp/SLAM-learning/assets/102967883/1b892e10-0a3c-4b59-aef1-739ff7584ad3)

如果angle 越大（大于60度）则认为两点越可能是同一个聚类物体上的点

```
angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

if (angle > segmentTheta){
...
...

```

超过30个同一个label则视为有效聚类

```
bool feasibleSegment = false;
if (allPushedIndSize >= 30)
    feasibleSegment = true;
```

考虑到树干等纵向物体被扫描时可能点少（激光雷达纵向分辨率差），另有一个x方向阈值，如果跨x方向像素多，也可以算有效聚类。

```
 else if (allPushedIndSize >= segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;
```

**两步优化的帧间里程记**
和原始 LOAM（或者 A-LOAM）一样，通过前后两帧点云来估计两帧之间的运动，从而累加得到前端里程记的输出，和上述方法使用线面约束同时优化六自由度帧间位姿不同，LeGO-LOAM 的前端分成两个步骤，每个步骤估计三自由度的变量。

1. 利用地面点优化

地面点更符合面特征的性质，因此，地面点的优化问题就使用点到面的约束来构建，同时我们注意到，地面点之间的约束对 x，y 和 yaw 这三个自由度是不能观的。

当这三个自由度的值发生变化时，点到面的残差不会发生显著变化，所以，地面点之间的优化只会对 pitch，roll 以及 z 进行约束和优化。

calculateTransformationSurf()中

```
transformCur[0] += matX.at<float>(0, 0);  // pitch
transformCur[2] += matX.at<float>(1, 0);  // roll
transformCur[4] += matX.at<float>(2, 0);  // z
```
2.利用角点优化

第一部优化完 pitch、roll 以及 z 之后，我们仍需对另外三个自由度的变量进行估计，此时，我们选用提取的角点进行优化，由于多线激光雷达提取的角点通常是垂直的边缘特征，因此，这些特征对 x、y 以及 yaw 有着比较好的能观性，通过角点的优化结合上地面点的结果可以得到六自由度的帧间优化结果。

calculateTransformationCorner()中

```
transformCur[1] += matX.at<float>(0, 0); // yaw
transformCur[3] += matX.at<float>(1, 0); // x
transformCur[5] += matX.at<float>(2, 0); // y
```
---

## LIO-SAM

在LeGO-LOAM的基础上新增了对IMU和GPS的紧耦合，LIO-SAM在一些不好的场景下表现要更鲁棒，回环处的漂移也更小。

采用一个因子图对位姿进行优化，引入四个因子（IMU预积分因子、lidar里程计、GPS 因子、回环因子），对机器人状态（位姿、速度、IMU偏置）进行优化。


![Screenshot from 2024-04-10 02-44-28](https://github.com/countsp/SLAM-learning/assets/102967883/4fb7b2ce-ac9f-40aa-8196-62e1f7d3b0ee)

![image](https://github.com/countsp/SLAM-learning/assets/102967883/fa4966cc-08fd-4fe5-8190-f75c1340354b)

![Screenshot from 2024-06-06 20-40-36](https://github.com/countsp/SLAM-learning/assets/102967883/d581ce56-b5c0-4262-a598-900fca5b2623)

1. 激光点云运动畸变矫正
   
   利用当前帧起止时刻之间的imu数据，imu里程计数据计算预积分，得到每一点时刻的激光点位姿，从而变换到初始时刻激光点坐标系下实现矫正。

2. 特征提取
   
   对经过矫正的点云计算曲率，提取角点面点特征（单条扫描线划分6段，每段取20个角点）

3. 特征匹配

   提取局部关键帧map的特征点，与当前帧特征点进行scan-to-map匹配（laserCloudInfoHandler-cornerOptimization）

   使用kd-tree在局部角点map中查找当前角点相邻的5个角点；LM位姿（LMOptimizaion）迭代30次 估计当前点云相对map的位姿；transformUpdate()

4. 因子图优化
   
   添加激光里程计因子、GPS因子、闭环因子，执行因子图优化，更新所有关键帧位姿。

5. 闭环检测

   在历史关键帧中找候选闭环匹配帧，执行scan-to-map匹配，得到位姿变换，构建闭环因子，加入到因子图中一并优化。   

   
**imu紧耦合作用**

1. imu对点云做运动补偿（去畸变）2. 给lidar里程计提供初值。3.优化的结果反过来矫正imu的偏移

imu预积分使用gt-sam实现

```
imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
```


### imageProjection.cpp 点云去畸变（对旋转，不对平移）

**主要功能**

    imageProjection的主要功能是订阅原始点云数据和imu数据，根据高频的imu信息对点云成像时雷达的位移和旋转造成的畸变进行校正
    
    同时，在发布去畸变点云的时候加入IMU输出的角度和IMU里程计（imuPreintegration）的角度和位姿作为该帧的初始位姿，作为图优化的初始估计
    
    并且，要对点云的Range进行计算，同时记录每个点的行列，以便在特征提取中被使用

**主要流程**

        cachePointCloud(); //点云缓存到cloudQueue中

        deskewInfo(); //获取运动补偿所需信息,imu补偿旋转，odom补偿平移
        
        projectPointCloud(); // 创建矩阵cv::Mat保存点云，对点云遍历，在接收到一帧点云后，将纵横线数取出，将distance放入矩阵的range中，补偿过后点云保存到fullCloud中

        cloudExtraction();//提出有效点

        publishClouds();//发布点云

        resetParameters();

#### deskewInfo()功能

确保imu数据覆盖这帧点云,并执行

1.imuDeskewInfo();  //计算每个时刻的姿态角，方便后续查找对应每个点云时间的值

```
imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
imuTime[imuPointerCur] = currentImuTime;

```

2.odomDeskewInfo();  // 1.记录起始时刻对应的odom姿态，便于后端位姿估计  2.计算起始与结束相对运动 transBt
```
cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
```

```
Eigen::Affine3f transBt = transBegin.inverse() * transEnd;
```

#### projectPointCloud()功能

创建矩阵cv::Mat保存点云，对点云遍历，在接收到一帧点云后，将纵横线数取出，将distance放入矩阵的range中，补偿过后点云保存到fullCloud中

```
//取出scan线数
int rowIdn = laserCloudIn->points[i].ring
columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;

//线性运动补偿
thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

//放入range矩阵中
rangeMat.at<float>(rowIdn, columnIdn) = range;
```

其中deskewPoint为去畸变函数，调用findRotation()角度插值，平移不插值

```
int imuPointerBack = imuPointerFront - 1;
double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
*rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
*rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
*rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
```

```
// 新点为 R * p + t ，把点补偿到第一个点对应时刻的位姿
newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
newPoint.intensity = point->intensity;
```

 1. 通过imuPreintegration的imu积分，提供良好初值
 2. cv::Mat对点云预处理，投影到cv::Mat中
 3. 点云旋转部分运动补偿，用的是线性插值





#### featureExtraction.cpp

**主要流程**

接收到从imageProjection中发布出的一个去畸变点云信息cloudInfo(自定义格式)

对每个点计算曲率。计算时是计算周围点的平均距离用来作为曲率的替代

标记遮挡点和与激光平行的点，后续这些点不能采纳为特征点

特征提取。分别做角点（曲率大）和平面点（曲率小）特征点提取

整合信息，发布完整数据包

**目的：**

1.提取角、面特征并发布（和A-LOAM基本一样）





####  mapOptimization.cpp


**目的**

1.角、面点云配准，得到位姿

2.lidar里程计、GPS 因子、回环因子 加入因子图，做位姿优化

**主要流程**

###### 位姿初始化updateInitialGuess

读取特征提取模块的线特征和平面特征，以及由IMU预积分提供的初始位姿。

线特征和平面特征由特征提取模块featureExtraction发布，

    第一帧做初始化：直接使用IMU输出roll、pitch、yaw作为当前帧的初始位姿。
    后续帧初始化：
        如果IMU里程计可用（已经融合了激光里程计结果），使用其作为6D位姿初始化
        如果IMU里程计不可用但IMU原始角度可用，则在上一帧的位姿上叠加IMU角度的变化作为初始化

        
##### 构建局部地图extractSurroundingKeyFrames
挑选局部地图关键帧的方法很巧妙，每个关键帧的位置(x,y,z)坐标本质上也是一堆3D点，因此可以使用KD树搜索距离当前关键帧最近的点云，同时又对这些3D点位置进行降采样，3D点的稀疏性保证了对应的关键帧位置不会挨得太近。选出关键帧后就提取对应的点云，转到同一个坐标系然后融合为局部地图。

    对所有关键帧3D位姿构建KD树
    使用最后一个关键帧位姿作为索引，从KD树中找到指定半径范围内的其他关键帧
    对找出的关键帧数量做降采样，避免关键帧位姿太过靠近
    加上时间上相邻的关键帧
    对所有挑选出的关键帧数量再做一次降采样，避免位置过近
    将挑选出的关键帧点云转换到odom坐标系。（这里使用一个map缓存坐标变换后的点云，避免重复计算）
    对局部地图的角点、平面点点云做降采样，并融合到一个局部地图中

    
##### 点云降采样downsampleCurrentScan

##### map-2-scan并进行位姿优化scan2MapOptimization

分别进行角点和平面点匹配，找到候选点后整合在一起，然后进行匹配优化。

基于角点的匹配优化

    将每个角点从雷达坐标系转换到map坐标系
    基于KD树搜索得到局部地图中距离当前角点最近的5个点
    5个点中，判断距离最远的点和当前点的距离，如果距离小于1m，认为该点有效
    基于5个点距离中心的协方差矩阵和特征值判断5点是否构成一条直线
        对协方差矩阵进行特征值分解
        如果最大的特征值要远大于第二个特征值，则认为则5个点能够构成一条直线
    计算点到直线的距离，如果距离小于阈值将该点作为优化目标点，计算当前帧角点到直线的距离、垂线的单位向量，存储为角点参数

基于平面点的匹配优化

    采用上述方法寻找5个平面点
    对5点进行平面拟合，使用最小二乘求解超定方程Ax+By+Cz+1=0
    计算点到平面的距离，如果距离小于阈值将该点作为优化目标点，将当前帧平面点到平面的距离、垂线的单位向量，存储为平面点参数

整合候选匹配点信息

    将cornerOptimization和surfOptimization两个函数计算出来的边缘点、平面点到局部地图的距离、法向量集合在一起

基于高斯牛顿法进行迭代优化

    基于点到直线、点到平面的距离构建残差方程，基于高斯牛顿法迭代求解使得残差最小时对应的位姿
    
    求解过程是将雅格比矩阵的形式写出，然后计算高斯牛顿法的hessian矩阵，然后对Hessian进行QR分解计算特征向量来更新位姿，在达到最大迭代次数之前如果收敛则优化成功。
    
    最后用imu原始RPY数据与scan-to-map优化后的位姿进行加权融合（只加权融合roll、pitch角），更新当前帧位姿的roll、pitch，约束z坐标

##### 创建关键帧saveKeyFramesAndFactor



在地图优化模块，scan-2-map并没有使用因子图，只是简单的点云配准问题，通过解析解的方式迭代求解局部地图匹配位姿。只有在创建关键帧时才会将新的关键帧加入到因子图，并进行因子图优化位姿。该因子图如下所示：

![image](https://github.com/countsp/SLAM-learning/assets/102967883/63caebbd-de41-41f8-a348-11fb0cdb7481)

    变量：每个关键帧的位姿
    因子：激光里程计因子、GPS因子、回环检测因子
        虽然论文中也添加了IMU因子，但是在实现过程中并没有使用


##### 回环校正correctPoses

里程计和位姿发布
实则是使用之前因子图优化后的位姿，对位姿序列相应关键帧的位姿进行替换









##### imuPreintegration.cpp

**主要流程**

LIO-SAM是通过激光里程计矫正IMU的累计误差，然后对IMU原始数据进行连续积分得到关于IMU的里程计。

    IMU原始数据的旋转积分用于去除点云的旋转畸变
    
    IMU里程计的平移信息用于去除点云的运动畸变
    
    IMU里程计的位姿信息和对应点云一起被封装，为map-2-scan提供初值

**目的**

1.IMU预积分因子、lidar里程计 加入因子图，做位姿优化，以imu频率发出紧耦合的lidar里程计，给地图优化提供好的初值

2.估计imu偏置







##### 因子图

在 slam 的后端优化问题中，我们通常会通过一些传感器的观测，比如视觉特征点，IMU 预积分量，Lidar 面点和边缘点的约束去构建一个优化问题，求解状态量（如位姿、速度等），这个时候我们考虑一个问题，当给这个系统新增一个约束时，我们就会重新对所有的约束对状态的优化问题进行求解，当图优化模型增大时，显然进行一次优化的时间也会增加很多，一方面实时性遭遇了挑战，另一方面，很久之前的状态似乎也没有继续更新的必要。为了解决这个问题，一种方式是使用滑动窗口来控制优化问题的规模，通常来讲滑动窗口需要好处理边缘化的问题，另一方面，我们可以使用因子图的模型来解决这个问题。

Kaess 等科研人员提出 iSAM，即增量平滑和建图，使其可以自动增量处理大规模优化问题，具体来说，其内部使用一种基于概率的贝叶斯树，使得每次给因子图增加一个约束时，其会根据贝叶斯树的连接关系，调整和当前结点“关系比较密切”的结点，如此，既保障了优化问题的求解精度，也使得耗时不会随着优化问题的增大而增大。关于因子图优化理论可以参考 iSAM，iSAM2 相关论文等文献。

**因子图中一些概念**

**变量结点**：类似 g2O 中的顶点或者 ceres 中的参数块，代表需要被优化的变量

```
gtsam::Pose3 // 表示六自由度位姿
gtsam::Vector3 // 表示三自由度速度
gtsam::imuBias::ConstantBias // 表示 IMU 零偏
```

**因子结点**：类似 g2O 中的边或者 ceres 中的 cost function，代表约束，如预积分约束、位姿先验约束、帧间位姿约束等

```
gtsam::PriorFactor<T> //先验因子，表示对某个状态量 T 的一个先验估计，约束某个状态变量的状态不会离该先验值过远
gtsam::ImuFactor //imu 因子，通过 IMU 预积分量构造出 IMU 因子，即 IMU 约束
gtsam::BetweenFactor //状态量间的约束，约束相邻两状态量之间的差值不会距离该约束过远
```

**GT-SAM 关于 IMU 预积分相关接口**

预积分相关参数，IMU 的噪声，重力方向等参数
```
gtsam::PreintegrationParams
```

预积分相关的计算
```
gtsam::PreintegratedImuMeasurements
```

相关接口

(1) resetIntegrationAndSetBias

将预积分量复位，也就是说清空刚刚的预积分量，重新开始一个新的预积分量，因为预积分的计算依赖一个初始的 IMU 零偏，因此，在复位之后需要输入零偏值，所以这里复位和重设零偏在一个接口里。

(2) integrateMeasurement

输入 IMU 的测量值，其内部会自动实现预积分量的更新以及协方差矩阵的更新

(3) deltaTij

预积分量跨越的时间长度

(4) predict

预积分量可以计算出两帧之间的相对位置、速度、姿态的变化量，那结合上一帧的状态量就可以计算出下一关键帧根据预积分结果的推算值

**代码：**

首先要添加约束、状态：

```
            // 添加约束 initial pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);

            prevVel_ = gtsam::Vector3(0, 0, 0);
            graphFactors.add(priorVel);

            prevBias_ = gtsam::imuBias::ConstantBias();
            graphFactors.add(priorBias);

            
            // add values 添加状态量
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            
            // optimize once
            optimizer.update(graphFactors, graphValues);

            // update到优化器后清零
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
```


## VINS-Mono
![Screenshot from 2024-06-12 21-32-07](https://github.com/countsp/SLAM-learning/assets/102967883/248fcb0c-9d6b-469c-86e3-5aece4ff20d6)

单目slam中尺度是模糊的，一般VIO框架使用IMU来提供尺度上的观测和约束。

视觉框架通常是在图像中提取特征点，以及通过光流或者描述子匹配（特征点周边像素的特征（明暗））的方式建立不同帧特征点之间的关联，通过三⾓化的方式恢复特征点的 3D 位置，随后在优化问题的建模中，通常会把各个关键帧的位姿以及特征点3D位置同时优化



### VINS-Mono 是由几个关键的模块组成：

**视觉前端**
    
    特征点的提取，同时通过光流追踪的方式建立两个连续视觉帧之间特征点之间的对应关系
    
**IMU 预积分**
    
    通过将两个连续视觉帧之间的 IMU 数据进行积分，形成两个视觉帧之间的预积分约束，对两帧之间的位姿，速度零偏等状态量形成约束
    
**视觉惯性联合初始化**
    
    在整个系统初始化之前，视觉的尺度以及各个关键帧的状态是未知的，因此 VINS 使用了一个松耦合求解 的方式，计算出初始的相机和 IMU 的外参，视觉尺度，各个关键帧的状态以及重力方向等
    
**基于滑动窗口的优化**
    
    系统初始化之后，系统就进入了滑窗优化求解，为了同时保证计算精度和计算效率，VINS 采取了滑动窗口的方式，及只优化最近的十个关键帧的状态，更早的关键帧将被移出滑窗，同时以先验的方式继续影 响着滑窗内状态的优化，在整个优化建模中，主要通过重投影约束和预积分约束两种视觉和惯性的约束进行优化求解。
    
**回环检测和位姿图优化**
    
    通过**视觉词袋**的方式进行回环帧检测，同时通过 PNP 的方式进行几何校验，一旦认为是成功的回环之后 就会触发位姿优化，使得全局位姿一致性更佳

### 视觉里程计 VS 激光里程计

LIO-SAM 和 VINS-Mono 作为两款不同的 slam 系统，本质上都是实现位姿估计和地图构建的目的，他俩相比有一些共同和不同的地方

**相同：**

他们拥有着类似的框架，前端数据特征提取，后端位姿优化，同时加上各自的回环检测模块和位姿图优化模块以实现更高精度的建图和定位。

**不同：**
然而，他们的主传感器不同，视觉 slam 更多是在图像上提取一些⾓点为基础，激光slam 的特征则更多是 几何上的一些曲率独特的点。

同时建立特征关联的方式也不同，视觉更多通过描述子匹配或者光流追踪，而激光更多是做的最近邻搜索来建立对应关系。

同时由于视觉缺乏尺度，因此需要一个鲁棒的初始化过程来恢复尺度，同时由于尺度的保持需要依赖加速度计的观测，使用IMU 的方式耦合度更高，同时在匀速直线运动情况下可能会尺度发散，而激光雷达 可以直接获取尺度信息，因此不需要过分复杂的初始化以及运动场景的依赖。

视觉的场景识别能力更强，视觉的 DBOW 模型可以在没有位姿先验的情况下来搜索相似度较高的图⽚作为回环候选帧，而一般的激光雷达可能很难借助类似方式实现（当然 scan context 或许也是一种方式）


## LVI-SAM

由两个子模块构成，激光惯性融合框架 LIO-SAM，以及视觉惯性融合框架 VINS-Mono

视觉给激光提供更鲁棒的里程计，和回环

LVI-SAM运行时依赖 相机数据、激光雷达数据、IMU数据、传感器外参

激光建图模块的初始位姿由vins-mono里程计信息提供

视觉特征点深度既可以通过视觉特征点三角化也可以通过激光雷达投影计算

回环检测通过DBOW实现

激光里程计帮助视觉恢复尺度，LIO-SAM帮助视觉恢复重力方向，帮助视觉计算出初始位姿

## 算法中工程化技巧总结

在上述一些框架中，我们可以看到一些工程话的技巧，比如

1、LeGO-LOAM 前端，对地面点的提取，利用地面点的一些性质对 roll，pitch 以及 z 进行一些约束和优化

2、通过 BFS 算法对前端特征进行过滤，使得更干净的特征留存了下来

3、后端滑动窗口的引入，使得构建局部地图更加方便，也更加方便实现纯里程记功能

4、对 GPS 的融合的谨慎的使用方式，使得既引入了全局观测，又不会对当前里程记的平滑性产生负面影响

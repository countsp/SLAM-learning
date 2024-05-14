看icp配准
为什么后端比前端优化器好？
为什么loam不能做回环检测？
# 多传感器融合

缺点就是现有的通用的视觉 slam 技术依赖图像的纹理来进行特征点的提取，没有纹理或者黑夜图像就很难被很好的利用。

图像中缺乏3d 信息，通常建模 slam 问题需要同时优化位姿和地图点。另一方面，单目图像缺乏尺度信息。

视觉SLAM： 常见的方式是一个视觉特征点前端（当然还有基于直接法的前端，如DSO），通过光流或者描述子建立不同帧特征点之间的关联，后端根据前端特征关联的结果和其他传感器数据进行融合，根据融合的方式分为基于优化的后端（ORBSLAM2、3, VINS-MONO，VINS-FUSION）以及基于滤波的后端（MSCKF），视觉通常会提供一个重投影误差作为约束或者更新量。

激光 slam 方向：目前性能最好使用最广的激光 slam 方案是基于 LOAM 的系列方案，LOAM 主要是为多线激光雷达设计的 lidar 定位和建图的方案，当然，由于现在其他一些 lidar 硬件的推出，一些 LOAM 的改进版本也是适当推出，如（Livox LOAM）。

基于 LOAM 方案通常前端是对当前帧激光雷达提取特征（通常是面特征和线特征），通常后端结合其他传感器信息给当前帧到地图中的匹配提供一个良好的初值（激光slam 中最重要的事情就是给 scan matching 提供一个更准确的 init guess）

#### 1. LOAM & A-LOAM

![Screenshot from 2024-04-10 02-41-09](https://github.com/countsp/SLAM-learning/assets/102967883/842c58b1-b7d9-4520-9263-fa1e3c02bd27)

非常经典的激光里程记和建图方案，也是其他 LOAM 方案的鼻祖，LOAM 只基于激光雷达（可选 IMU），通过把 SLAM 拆分成一个**高频低精**的scan-to-scan前端(帧间里程计)以及一个**低频高精**的scan-to-map后端,来实现 lidar 里程记的实时性。

#### 2.LeGO-LOAM

![Screenshot from 2024-04-10 02-42-53](https://github.com/countsp/SLAM-learning/assets/102967883/8bbbc636-e3c5-4ce4-8c29-9484dbe62582)

在原有 LOAM 基础上，在前端增加了**地面点提取**，并且根据嵌入式平台的计算特点，将前端做的**更加轻量级**，以便于在算力有限的平台上可以实时运行，后端将其使用slam 中关键帧的定义进行重构，同时增加**回环检测和回环位姿图优化**，使得地图的全局一致性更优。

 #### 3.LIO-SAM
 ![Screenshot from 2024-04-10 02-44-28](https://github.com/countsp/SLAM-learning/assets/102967883/4fb7b2ce-ac9f-40aa-8196-62e1f7d3b0ee)
 
在 LOAM 基础上采用紧耦合的 imu 使用方式，放弃了帧间里程记作为前端里程记，而使用紧耦合后 imu 的预测值作为前端里程记，后端沿用 LeGO-LOAM，同时加入了对GPS信号的适配，使得其全局一致性更出色。

#### 4.LVI-SAM
![Screenshot from 2024-04-10 02-47-17](https://github.com/countsp/SLAM-learning/assets/102967883/abecebd9-1398-48eb-add9-209a0bc1cd0d)

LVI-SAM 是一种视觉、激光、IMU 三种传感器紧耦合的里程计框架，他是由两个独立的里程计（VINS-Mono 以及 LIO-SAM）融合而成，视觉里程计给激光里程计提供高频先验位姿，回环检测，而激光里程计给视觉里程计提供初始化的先验以及特征点深度信息。


---
* **[LOAM/A-LOAM](#LOAM/A-LOAM)**

  
#### LOAM/A-LOAM

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

通过同一个线上邻近两点的距离差来判定角点和面点。面点的c小，角点的c大。


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

使用KD-tree寻找两帧之间的匹配对

![Screenshot from 2024-04-10 03-44-05](https://github.com/countsp/SLAM-learning/assets/102967883/84439891-a796-44d7-b871-8d3e41925b7c)

匹配线寻找：i是k+1帧边缘点的一部分，找到上一帧离i最近的线点j，j与i不同扫描线。

匹配面寻找：找到上一帧离i最近的面点j，同一扫描线找j最近面点l，与i不同扫描线上最近面点m

![Screenshot from 2024-04-10 04-18-17](https://github.com/countsp/SLAM-learning/assets/102967883/fae738d8-3758-468a-8aa4-a605cf13e817)

![Screenshot from 2024-04-10 04-21-34](https://github.com/countsp/SLAM-learning/assets/102967883/a50800ff-1cc4-4c4b-abfd-50aa58ec4cde)

点到线距离计算：k帧中有点j,l;k+1帧中有点i

点到面距离计算：k帧中有点j,m,l;k+1帧中有点i


4.运动估计

![Screenshot from 2024-04-10 04-55-07](https://github.com/countsp/SLAM-learning/assets/102967883/d8136bb7-0ab2-45d0-993d-22e5635a3882)

线性插值法

![Screenshot from 2024-04-10 05-01-12](https://github.com/countsp/SLAM-learning/assets/102967883/7e8289b0-c285-472a-b9dc-0cc573e90077)

![Screenshot from 2024-04-10 05-02-53](https://github.com/countsp/SLAM-learning/assets/102967883/a820a630-c7a1-4620-8d02-cb525c664da2)

通过残差最小化估计运动

论文中使用LM，代码用高斯牛顿法

5.运动补偿

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
6.建图(后端)

栅格匹配找匹配对，方法为3D KD-tree

对协方差矩阵进行分解，分解为特征值和特征向量。如果是线点，一个特征值明显大于其余两个。如果面点，两个特征值比较大，小的特征值对应特征向量是面的法向量。

---

#### LeGO-LOAM （代码部分沿用LOAM）
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

#### LIO-SAM

imu对点云做补偿去畸变,必须要有imu

因子图优化问题，有四个因子（IMU预积分因子、lidar里程计、GPS 因子、回环因子），对机器人状态（位姿、速度、IMU偏置）进行优化。

imu紧耦合： 1. imu对点云做运动补偿（去畸变）2. 给lidar里程计提供初值。3.优化的结果反过来矫正imu的偏移

 ![Screenshot from 2024-04-10 02-44-28](https://github.com/countsp/SLAM-learning/assets/102967883/4fb7b2ce-ac9f-40aa-8196-62e1f7d3b0ee)

**imageProjection.cpp**

 1. 通过imuPreintegration的imu积分，提供良好初值
 2. cv::Mat对点云预处理，投影到cv::Mat中
 3. 点云旋转部分运动补偿，用的是线性插值

**featureExtraction.cpp**

1.提取角、面特征并发布

**mapOptimization.cpp**

1.角、面点云配准，得到位姿
2.lidar里程计、GPS 因子、回环因子 加入因子图，做位姿优化


**imuPreintegration.cpp**

1.IMU预积分因子、lidar里程计 加入因子图，做位姿优化

2.估计imu偏置

**因子图**

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

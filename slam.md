# 多传感器融合

缺点就是现有的通用的视觉 slam 技术依赖图像的纹理来进行特征点的提取，没有纹理或者黑夜图像就很难被很好的利用。

图像中缺乏3d 信息，通常建模 slam 问题需要同时优化位姿和地图点。另一方面，单目图像缺乏尺度信息。

视觉SLAM： 常见的方式是一个视觉特征点前端（当然还有基于直接法的前端，如DSO），通过光流或者描述子建立不同帧特征点之间的关联，后端根据前端特征关联的结果和其他传感器数据进行融合，根据融合的方式分为基于优化的后端（ORBSLAM2、3, VINS-MONO，VINS-FUSION）以及基于滤波的后端（MSCKF），视觉通常会提供一个重投影误差作为约束或者更新量。

激光 slam 方向：目前性能最好使用最广的激光 slam 方案是基于 LOAM 的系列方案，LOAM 主要是为多线激光雷达设计的 lidar 定位和建图的方案，当然，由于现在其他一些 lidar 硬件的推出，一些 LOAM 的改进版本也是适当推出，如（Livox LOAM）。

基于 LOAM 方案通常前端是对当前帧激光雷达提取特征（通常是面特征和线特征），通常后端结合其他传感器信息给当前帧到地图中的匹配提供一个良好的初值（激光slam 中最重要的事情就是给 scan matching 提供一个更准确的 init guess）

#### 1.A-LOAM

![Screenshot from 2024-04-10 02-41-09](https://github.com/countsp/SLAM-learning/assets/102967883/842c58b1-b7d9-4520-9263-fa1e3c02bd27)

非常经典的激光里程记和建图方案，也是其他 LOAM 方案的鼻祖，LOAM 只基于激光雷达（可选 IMU），通过把 SLAM 拆分成一个**高频低精的前端**以及一个**低频高精的后端**来实现 lidar 里程记的实时性。

#### 2.LeGO-LOAM

![Screenshot from 2024-04-10 02-42-53](https://github.com/countsp/SLAM-learning/assets/102967883/8bbbc636-e3c5-4ce4-8c29-9484dbe62582)

在原有 LOAM 基础上，在前端增加了地面点提取，并且根据嵌入式平台的计算特点，将前端做的更加轻量级，以便于在算力有限的平台上可以实时运行，后端将其使用slam 中关键帧的定义进行重构，同时增加回环检测和回环位姿图优化，使得地图的全局一致性更优。

 #### 3.LIO-SAM
 ![Screenshot from 2024-04-10 02-44-28](https://github.com/countsp/SLAM-learning/assets/102967883/4fb7b2ce-ac9f-40aa-8196-62e1f7d3b0ee)
 
在 LOAM 基础上采用紧耦合的 imu 使用方式，放弃了帧间里程记作为前端里程记，而使用紧耦合后 imu 的预测值作为前端里程记，后端沿用 LeGO-LOAM，同时加入了对GPS信号的适配，使得其全局一致性更出色。

#### 4.LVI-SAM
![Screenshot from 2024-04-10 02-47-17](https://github.com/countsp/SLAM-learning/assets/102967883/abecebd9-1398-48eb-add9-209a0bc1cd0d)

LVI-SAM 是一种视觉、激光、IMU 三种传感器紧耦合的里程计框架，他是由两个独立的里程计（VINS-Mono 以及 LIO-SAM）融合而成，视觉里程计给激光里程计提供高频先验位姿，回环检测，而激光里程计给视觉里程计提供初始化的先验以及特征点深度信息。


* **[LOAM/A-LOAM](#LOAM/A-LOAM)**
* **[A-LOAM](#A-LOAM)**

---

#### LOAM/A-LOAM

1.选择面点、角点。16线Velodyne一圈1800点等分为4等分，每一份最多2角点、四个平面点。

![Screenshot from 2024-04-10 03-28-05](https://github.com/countsp/SLAM-learning/assets/102967883/92e2436f-eb7a-4ae3-a2e4-9d46de53e081)

通过同一个线上邻近两点的距离差来判定角点和面点。面点的c小，角点的c大。

前端特征提取：传统方法（通过曲率计算平坦/曲率较大）

```
for (int i = 5; i < cloudSize - 5; i++)
    { 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
```

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

使用KD-tree寻找两帧之间的匹配对

![Screenshot from 2024-04-10 03-44-05](https://github.com/countsp/SLAM-learning/assets/102967883/84439891-a796-44d7-b871-8d3e41925b7c)

匹配线寻找：i是k+1帧边缘点的一部分，找到上一帧离i最近的线点j，j与i不同扫描线。

匹配面寻找：找到上一帧离i最近的面点j，同一扫描线找j最近面点l，与i不同扫描线上最近面点m

![Screenshot from 2024-04-10 04-18-17](https://github.com/countsp/SLAM-learning/assets/102967883/fae738d8-3758-468a-8aa4-a605cf13e817)

![Screenshot from 2024-04-10 04-21-34](https://github.com/countsp/SLAM-learning/assets/102967883/a50800ff-1cc4-4c4b-abfd-50aa58ec4cde)

点到线距离计算：k帧中有点j,l;k+1帧中有点i

点到面距离计算：k帧中有点j,m,l;k+1帧中有点i


3.运动估计

![Screenshot from 2024-04-10 04-55-07](https://github.com/countsp/SLAM-learning/assets/102967883/d8136bb7-0ab2-45d0-993d-22e5635a3882)

线性插值法

![Screenshot from 2024-04-10 05-01-12](https://github.com/countsp/SLAM-learning/assets/102967883/7e8289b0-c285-472a-b9dc-0cc573e90077)

![Screenshot from 2024-04-10 05-02-53](https://github.com/countsp/SLAM-learning/assets/102967883/a820a630-c7a1-4620-8d02-cb525c664da2)

通过残差最小化估计运动

论文中使用LM，代码用高斯牛顿法

4.建图

栅格匹配找匹配对，方法为3D KD-tree

对协方差矩阵进行分解，分解为特征值和特征向量。如果是线点，一个特征值明显大于其余两个。如果面点，两个特征值比较大，小的特征值对应特征向量是面的法向量。

---

#### A-LOAM

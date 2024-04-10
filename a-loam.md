# 多传感器融合

[A-LOAM]("A-LOAM")

缺点就是现有的通用的视觉 slam 技术依赖图像的纹理来进行特征点的提取，没有纹理或者黑夜图像就很难被很好的利用。

图像中缺乏3d 信息，通常建模 slam 问题需要同时优化位姿和地图点。

另一方面，单目图像缺乏尺度信息。

视觉SLAM： 常见的方式是一个视觉特征点前端（当然还有基于直接法的前端，如DSO），通过光流或者描述子建立不同帧特征点之间的关联，后端根据前端特征关联的结果和其他传感器数据进行融合，根据融合的方式分为基于优化的后端（ORBSLAM2、3, VINS-MONO，VINS-FUSION）以及基于滤波的后端（MSCKF），视觉通常会提供一个重投影误差作为约束或者更新量。

激光 slam 方向：目前性能最好使用最广的激光 slam 方案是基于 LOAM 的系列方案，LOAM 主要是为多线激光雷达设计的 lidar 定位和建图的方案，当然，由于现在其他一些 lidar 硬件的推出，一些 LOAM 的改进版本也是适当推出，如（Livox LOAM）。

基于 LOAM 方案通常前端是对当前帧激光雷达提取特征（通常是面特征和线特征），通常后端结合其他传感器信息给当前帧到地图中的匹配提供一个良好的初值（激光slam 中最重要的事情就是给 scan matching 提供一个更准确的 init guess）

#### A-LOAM

![Screenshot from 2024-04-10 02-41-09](https://github.com/countsp/SLAM-learning/assets/102967883/842c58b1-b7d9-4520-9263-fa1e3c02bd27)

非常经典的激光里程记和建图方案，也是其他 LOAM 方案的鼻祖，LOAM 只基于激光雷达（可选 IMU），通过把 SLAM 拆分成一个高频低精的前端以及一个低频高精的后端来实现 lidar 里程记的实时性。

#### LeGO-LOAM

![Screenshot from 2024-04-10 02-42-53](https://github.com/countsp/SLAM-learning/assets/102967883/8bbbc636-e3c5-4ce4-8c29-9484dbe62582)

在原有 LOAM 基础上，在前端增加了地面点提取，并且根据嵌入式平台的计算特点，将前端做的更加轻量级，以便于在算力有限的平台上可以实时运行，后端将其使用slam 中关键帧的定义进行重构，同时增加回环检测和回环位姿图优化，使得地图的全局一致性更优。

 #### LIO-SAM
 ![Screenshot from 2024-04-10 02-44-28](https://github.com/countsp/SLAM-learning/assets/102967883/4fb7b2ce-ac9f-40aa-8196-62e1f7d3b0ee)
 
在 LOAM 基础上采用紧耦合的 imu 使用方式，放弃了帧间里程记作为前端里程记，而使用紧耦合后 imu 的预测值作为前端里程记，后端沿用 LeGO-LOAM，同时加入了对GPS信号的适配，使得其全局一致性更出色。

#### LVI-SAM
![Screenshot from 2024-04-10 02-47-17](https://github.com/countsp/SLAM-learning/assets/102967883/abecebd9-1398-48eb-add9-209a0bc1cd0d)

LVI-SAM 是一种视觉、激光、IMU 三种传感器紧耦合的里程计框架，他是由两个独立的里程计（VINS-Mono 以及 LIO-SAM）融合而成，视觉里程计给激光里程计提供高频先验位姿，回环检测，而激光里程计给视觉里程计提供初始化的先验以及特征点深度信息。



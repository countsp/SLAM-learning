# KF
### 估计一个在一维直线上匀速运动的物体的位置和速度。该滤波器考虑到了系统的噪声和测量的噪声。
### 卡尔曼滤波器参数解释

1. **初始状态估计**（`x̂₀`）：
   - 初始状态可能为 `[0, 0]^T`，表示物体开始时的位置为0，速度也为0。这是滤波器在开始运行之前对系统状态的初始估计。

2. **初始误差协方差**（`P₀`）：
   - 这是一个2x2的矩阵，通常设置为对角矩阵，例如 `[[1000, 0], [0, 1000]]`，表示对初始状态估计的不确定性很大。

3. **状态转移矩阵**（`A`）：
   - 对于匀速模型，状态转移矩阵可以设为 `[[1, Δt], [0, 1]]`，其中 `Δt` 是时间步长。这反映了物体位置的更新依赖于前一位置和速度的线性组合。

4. **控制输入矩阵**（`B`）（如果有控制输入的话）：
   - 在匀速运动例子中，如果没有外部控制影响，这个矩阵和控制向量通常不被使用。

5. **观测矩阵**（`H`）：
   - 如果我们只能测量位置，观测矩阵可以是 `[1, 0]`，表示观测值只包含位置信息。在gnss-imu融合中是定值。

6. **过程噪声协方差**（`Q`）：
   - 描述由于模型不精确或外部扰动引起的预测误差。例如，可以设为 `[[0.25*Δt^4, 0.5*Δt^3], [0.5*Δt^3, Δt^2]] * process_variance`，以反映位置和速度的预测不确定性。

7. **测量噪声协方差**（`R`）：
   - 如果测量设备在测量位置时有固定的噪声标准差，例如2米，那么 `R` 可以设为 `[[4]]`（即方差为4）。

8. **卡尔曼增益**（`K`）：
   - 这个参数不是提前设定的，而是在每个时间步骤中基于当前的预测误差协方差和测量噪声协方差动态计算得出的。


这些参数共同工作，使卡尔曼滤波器能够有效地对系统状态进行估计和更新，即使在存在噪声的情况下也能提供较为准确的动态跟踪。

### 输入输出

#### 输入
1. **当前状态估计（`x_{k-1}`）**：
   - 这是前一时间步的状态估计，代表滤波器对系统状态（如位置、速度等）的最新理解。

2. **当前误差协方差矩阵（`P_{k-1}`）**：
   - 这是与当前状态估计相关的误差协方差矩阵，描述了估计的不确定性。

3. **状态转移矩阵（`A`）**：
   - 该矩阵模拟系统状态随时间如何演变，是预测下一状态的基础。

4. **过程噪声协方差矩阵（`Q`）**：
   - 表示系统模型中的内在不确定性或噪声，影响预测的准确性。

#### 输出
1. **预测状态估计（`x_k^-`）**：
   - 这是基于模型动态及先前状态估计计算出的下一时间步的状态预测。

2. **预测误差协方差矩阵（`P_k^-`）**：
   - 这是预测状态的更新误差协方差，反映了预测的不确定性。

### 处理流程

先预测（x,p），预测出的值是输出，再更新参数

### 卡尔曼滤波器的预测和更新步骤

![Screenshot from 2024-09-04 11-37-17](https://github.com/user-attachments/assets/dde62c0f-fcb0-45c8-8429-b47dae67ad3f)


#### 代码
```
import numpy as np

class KalmanFilter:
    def __init__(self, delta_t, sigma_process, sigma_measurement, initial_state):
        self.delta_t = delta_t  # 时间步长
        self.sigma_process = sigma_process  # 过程噪声的标准差
        self.sigma_measurement = sigma_measurement  # 测量噪声的标准差
        self.x_hat = np.array(initial_state)  # 状态估计 [位置, 速度]
        self.P = np.eye(2)  # 误差协方差矩阵

        # 状态转移矩阵 A
        self.A = np.array([[1, delta_t],
                           [0, 1]])

        # 观测矩阵 H
        self.H = np.array([[1, 0]])

        # 过程噪声协方差矩阵 Q
        self.Q = np.array([[0.25 * delta_t**4, 0.5 * delta_t**3],
                           [0.5 * delta_t**3, delta_t**2]]) * sigma_process**2

        # 测量噪声协方差矩阵 R
        self.R = np.array([[sigma_measurement**2]])

    def predict(self):
        # 状态预测
        self.x_hat = np.dot(self.A, self.x_hat)
        # 误差协方差预测
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

    def update(self, z):
        # 计算卡尔曼增益 K
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))

        # 更新估计
        y = z - np.dot(self.H, self.x_hat)
        self.x_hat += np.dot(K, y)

        # 更新误差协方差
        I = np.eye(self.H.shape[1])
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

# 创建卡尔曼滤波器实例
kf = KalmanFilter(delta_t=1.0, sigma_process=1.0, sigma_measurement=2.0, initial_state=[0, 0])

# 模拟测量数据
measurements = [np.random.normal(loc=position, scale=2.0) for position in np.linspace(0, 10, 11)]

# 应用卡尔曼滤波器
for z in measurements:
    kf.predict()
    kf.update(z)
    print(f"Predicted state: {kf.x_hat}")

```
# EKF

### 匀加速运动的预测

### 代码
```
import numpy as np

class KalmanFilter:
    def __init__(self, delta_t, sigma_process, sigma_measurement, initial_state):
        self.delta_t = delta_t  # 时间步长
        self.sigma_process = sigma_process  # 过程噪声的标准差
        self.sigma_measurement = sigma_measurement  # 测量噪声的标准差
        self.x_hat = np.array(initial_state)  # 状态估计 [位置, 速度, 加速度]
        self.P = np.eye(3)  # 误差协方差矩阵

        # 状态转移矩阵 A
        self.A = np.array([[1, delta_t, 0.5 * delta_t**2],
                           [0, 1, delta_t],
                           [0, 0, 1]])

        # 观测矩阵 H
        self.H = np.array([[1, 0, 0]])

        # 过程噪声协方差矩阵 Q
        self.Q = np.array([[delta_t**5 / 20, delta_t**4 / 8, delta_t**3 / 6],
                           [delta_t**4 / 8, delta_t**3 / 3, delta_t**2 / 2],
                           [delta_t**3 / 6, delta_t**2 / 2, delta_t]]) * sigma_process**2

        # 测量噪声协方差矩阵 R
        self.R = np.array([[sigma_measurement**2]])

    def predict(self):
        # 状态预测
        self.x_hat = np.dot(self.A, self.x_hat)
        # 误差协方差预测
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

    def update(self, z):
        # 计算卡尔曼增益 K
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))

        # 更新估计
        y = z - np.dot(self.H, self.x_hat)
        self.x_hat += np.dot(K, y)

        # 更新误差协方差
        I = np.eye(self.H.shape[1])
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

# 创建卡尔曼滤波器实例
kf = KalmanFilter(delta_t=0.1, sigma_process=0.1, sigma_measurement=10.0, initial_state=[0, 0, 0])

# 模拟测量数据
measurements = [0, 0.5, 2.0, 4.5, 8.0]  # 假设这是位置的测量值

# 应用卡尔曼滤波器
for z in measurements:
    kf.predict()
    kf.update(z)
    print(f"Predicted state: {kf.x_hat}")


```
## 思路
1.找到状态转移函数

状态转移函数 f(x) 描述了在系统中，状态 x 如何从一个时刻 k−1 演变到下一个时刻 k

2.找到状态x

状态向量 x 是系统的核心变量，它包含了对系统状态的完整描述。在匀加速运动中，状态向量通常包括位置、速度和加速度 x=[s,v,a]。

3.求状态转移函数对状态 x 的雅可比矩阵

计算状态转移函数 f(x) 对状态 x 的偏导数

4.找到观测函数 h(x)

观测函数 h(x) 描述了系统的测量值 z 如何从状态向量 x 中得到。对于非线性系统，观测函数也可以是非线性的。

5.求观测函数对状态 x 的雅可比矩阵

6.初始状态估计 x0 和初始协方差矩阵 P0

7.定义过程噪声和测量噪声的协方差矩阵


# ESKF

和EKF差不多，只是在误差协方差 P 上稍微修改了。

```
def eskf(x, P, z, dt, R, Q):
    # 预测步骤
    F = F_jacobian(x, dt)
    x_pred = f(x, dt)
    P_pred = F @ P @ F.T + Q  # 预测协方差矩阵

    # 更新步骤
    H = H_jacobian(x)
    y = z - h(x_pred)  # 观测残差
    S = H @ P_pred @ H.T + R  # 观测预测协方差
    K = P_pred @ H.T @ np.linalg.inv(S)  # 卡尔曼增益
    x_new = x_pred + K @ y  # 更新后的状态估计

    # 通过增强误差状态进行调整
    P_new = (np.eye(len(x)) - K @ H) @ P_pred  # 更新后的协方差矩阵
    P_new = np.maximum(P_new, 1e-6 * np.eye(len(x)))  # 增强稳定性（避免协方差矩阵变得过小） ********************

    return x_new, P_new


def ekf(x, P, z, dt, R, Q):
    # 预测步骤
    F = F_jacobian(x, dt)
    x_pred = f(x, dt)
    P_pred = F @ P @ F.T + Q  # 预测协方差矩阵

    # 更新步骤
    H = H_jacobian(x)
    y = z - h(x_pred)  # 观测残差
    S = H @ P_pred @ H.T + R  # 观测预测协方差
    K = P_pred @ H.T @ np.linalg.inv(S)  # 卡尔曼增益
    x_new = x_pred + K @ y  # 更新后的状态估计
    P_new = (np.eye(len(x)) - K @ H) @ P_pred  # 更新后的协方差矩阵************************************

    return x_new, P_new

```

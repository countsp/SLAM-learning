# 如何计算得到 U 的值（SVD 分解详细步骤）

在奇异值分解 (SVD) 中，矩阵 \( U \) 是通过本质矩阵 \( E \) 的 **左奇异向量** 计算得到的，这些向量是矩阵 \( E E^T \) 的特征向量。

---

## 1. 计算 \( E E^T \)

根据定义：
\[
E E^T =
\begin{bmatrix}
0 & -1 & 0 \\
1 &  0 & -2 \\
0 &  2 &  0
\end{bmatrix}
\begin{bmatrix}
0 &  1 &  0 \\
-1 &  0 &  2 \\
0 & -2 &  0
\end{bmatrix}
\]

计算结果为：
\[
E E^T =
\begin{bmatrix}
1 & 0 & -2 \\
0 & 5 &  0 \\
-2 & 0 &  4
\end{bmatrix}
\]

---

## 2. 求解特征值和特征向量

### 特征值

特征值 \( \lambda \) 满足以下特征方程：
\[
\det(E E^T - \lambda I) = 0
\]

对于 \( E E^T \)：
\[
E E^T - \lambda I =
\begin{bmatrix}
1 - \lambda & 0 & -2 \\
0 & 5 - \lambda & 0 \\
-2 & 0 & 4 - \lambda
\end{bmatrix}
\]

解得特征值：
- \( \lambda_1 = 5 \)
- \( \lambda_2 = 2.236 \)
- \( \lambda_3 = 0 \)

---

### 特征向量

根据每个特征值 \( \lambda_i \)，求解线性方程组 \( (E E^T - \lambda_i I) \mathbf{u} = 0 \) 来计算特征向量：

#### 对 \( \lambda_1 = 5 \)
解得特征向量：
\[
\mathbf{u_1} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}
\]

#### 对 \( \lambda_2 = 2.236 \)
解得特征向量：
\[
\mathbf{u_2} = \begin{bmatrix} -0.447 \\ 0 \\ 0.894 \end{bmatrix}
\]

#### 对 \( \lambda_3 = 0 \)
解得特征向量：
\[
\mathbf{u_3} = \begin{bmatrix} -0.894 \\ 0 \\ -0.447 \end{bmatrix}
\]

---

## 3. 构造矩阵 \( U \)

将特征向量按列排列，构造正交矩阵 \( U \)：
\[
U =
\begin{bmatrix}
0 & -0.447 & -0.894 \\
1 & 0 & 0 \\
0 & 0.894 & -0.447
\end{bmatrix}
\]

---

## 4. 验证正交性

矩阵 \( U \) 应满足正交矩阵的性质：
\[
U^T U = I
\]

通过数值验证可以确认上述矩阵 \( U \) 是正交矩阵。

---

## 总结

1. 计算 \( E E^T \)。
2. 求解 \( E E^T \) 的特征值和特征向量。
3. 将特征向量按列排列，构造 \( U \) 矩阵。

这是 \( U \) 的完整计算过程。如需进一步验证或扩展，请随时讨论！

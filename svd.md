![Screenshot from 2025-01-24 13-33-31](https://github.com/user-attachments/assets/20192ebc-fa70-4986-a168-9e14613fd2ca)


![Screenshot from 2025-01-24 13-33-51](https://github.com/user-attachments/assets/f29dc664-2373-4ffd-8261-a94ae8229478)


在对极几何中 ， 通过本质矩阵E 计算 R 和 t

根据本质矩阵的特性，调整奇异值为 Σ=diag(1,1,0)Σ=diag(1,1,0)。

定义辅助矩阵 W 和 Z：
W=[0，−1，0，
  1， 0， 0，
  ，0 ，0 ，1],
  
Z=[0， 1， 0，
  −1， 0， 0，
  0，  0， 0]


计算旋转矩阵 R1,R2 和平移向量 t：
R1=UWVT,R2=UWTVT,t=u3


其中 u3 是 U 的第三列。

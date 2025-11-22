# Lec6 Eigenvalue and Eigenvector

## 一、特征值与特征向量定义

### 1. 几何变换示例

- **剪切变换**： 
  $$
   T \begin{pmatrix} x \\ y \end{pmatrix} = \begin{bmatrix} x + my \\ y \end{bmatrix} 
  $$
  特征向量 $\begin{pmatrix} 1 \\ 0 \end{pmatrix}$，特征值 $\lambda = 1$
  
- **反射变换**（关于 $y = \frac{1}{2}x\\$）：
  特征向量 $b_1$（$\lambda=1$），$b_2$（$\lambda=-1$）

- **旋转矩阵**：
  $$
   M(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} 
  $$
  实特征值仅当 $\theta = 0^\circ$ 或 $180^\circ$

### 2. 不变子空间 (Invariant Subspace)

- **定义**：子空间 $U \subseteq V$ 满足 $u \in U \implies Tu \in U$

- **例子**：  
  $$
  \{0\}, V, \text{null }T, \text{range }T\qquad 均为不变子空间
  $$
  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601211946575.png" alt="image-20250601211946575" style="zoom:67%;" />
  
- **微分算子**：  
  
  $ Tp = p' $ 在多项式空间 $ P_n(\mathbb{R}) $ 上
  
  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601211959186.png" alt="image-20250601211959186" style="zoom:67%;" />

---

## 二、特征值理论

### 1. 核心定义
- **特征值**：$\lambda \in \mathbb{F}$ 满足 $ Tv = \lambda v $（$v \neq 0$）

- **特征向量**：$v$ 是 $(T - \lambda I)v = 0$ 的非零解

- **等价条件**（有限维）：  

  (a) $\lambda$ 是特征值  

  (b) $T - \lambda I$ 不可逆  

  (c) $\det(T - \lambda I) = 0$

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601212059533.png" alt="image-20250601212059533" style="zoom:67%;" />

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601223255809.png" alt="image-20250601223255809" style="zoom:67%;" />

- **特征空间**

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601231440354.png" alt="image-20250601231440354" style="zoom:67%;" />

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601231521710.png" alt="image-20250601231521710" style="zoom:67%;" />

### 2. 存在性与性质

- **复向量空间**：算子必有特征值（代数基本定理）

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601224617021.png" alt="image-20250601224617021" style="zoom:67%;" />

- **线性无关性**：不同特征值的特征向量线性无关

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601224636545.png" alt="image-20250601224636545" style="zoom:67%;" />

- **特征值数量**：$\leq \dim V$（有限维空间）

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601224624767.png" alt="image-20250601224624767" style="zoom:67%;" />

---

## 三、矩阵表示与对角化

### 1. 矩阵表示

- 基 $\{v_1, \dots, v_n\}$ 下算子 $T$ 的矩阵：  

$$
M(T) = \begin{pmatrix} A_{11} & \cdots & A_{1n} \\ \vdots & \ddots & \vdots \\ A_{n1} & \cdots & A_{nn} \end{pmatrix}, \quad Tv_k = \sum_{j=1}^n A_{jk} v_j
$$

### 2. 上三角矩阵

- **条件**（基 $\{v_1, \dots, v_n\}$）：  
  $$
  Tv_j \in \text{span}(v_1, \dots, v_j)  \iff M(T) 上三角  \iff \text{span}(v_1, \dots, v_j) 不变
  $$
  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601225422896.png" alt="image-20250601225422896" style="zoom:67%;" />

- **复向量空间**：算子总存在上三角矩阵表示

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601225705530.png" alt="image-20250601225705530" style="zoom:67%;" />

- 假设 $T∈L(V)$ （*T* 是向量空间 *V* 上的线性变换），且 T 在某个基下对应的矩阵是上三角矩阵。那么，T 是可逆的当且仅当这个上三角矩阵的对角线上的所有元素都不为零。

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601230524358.png" alt="image-20250601230524358" style="zoom:67%;" />

- 假设 $T∈L(V)$ （*T* 是向量空间 *V* 上的线性变换），且 T 在某个基下对应的矩阵是上三角矩阵。那么，T 的所有特征值恰好是该上三角矩阵对角线上的元素。

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601230826506.png" alt="image-20250601230826506" style="zoom:67%;" />

### 3. 对角化 (Diagonalizability)

- **定义**：存在基使 $M(T)$ 为对角阵

<img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601231635616.png" alt="image-20250601231635616" style="zoom:67%;" />

- **等价条件**：  

  (a) $V$ 有特征向量基  

  (b) $ V = E(\lambda_1, T) \oplus \cdots \oplus E(\lambda_m, T) $  

  (c) $\dim V = \sum \dim E(\lambda_j, T)$

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601232003910.png" alt="image-20250601232003910" style="zoom:67%;" />

- **充分条件**：有 $\dim V$ 个互异特征值

  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601232255875.png" alt="image-20250601232255875" style="zoom:67%;" />

#### 例子：
- **可对角化**：  
  $$
  T(x,y,z) = (2x+y, 5y+3z, 8z)
  $$
  特征值 $\lambda=2,5,8$，基 $\{(1,0,0), (1,3,0), (1,6,6)\}$

  

- **不可对角化**：  
  
  $ T(w,z) = (z,0) $ 仅有一个特征值 $\lambda=0$，且 $\dim E(0,T)=1 < 2$

---

## 四、微分方程系统

### 1. 一阶系统 $\dot{Y} = AY$

- **解**：$ Y = \sum c_k e^{\lambda_k t} x_k $（若 $A$ 可对角化）
- **复数特征值**（$\lambda = a \pm bi$）：实解为 $\operatorname{Re}(e^{\lambda t}x)$ 和 $\operatorname{Im}(e^{\lambda t}x)$

<img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601233554461.png" alt="image-20250601233554461" style="zoom:67%;" />

<img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601234137604.png" alt="image-20250601234137604" style="zoom:67%;" />

<img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250601234149246.png" alt="image-20250601234149246" style="zoom:67%;" />

### 2. 高阶系统

### **将二阶系统转化为一阶系统**

#### **原系统**

给定一个二阶微分方程系统：
$$
\mathbf{Y}'' = A_1 \mathbf{Y} + A_2 \mathbf{Y}'
$$
其中，$ \mathbf{Y} $ 是一个 $ n $-维向量，$ A_1 $ 和 $ A_2 $ 是 $ n \times n $ 矩阵。

#### **转化为一阶系统**
通过引入新的变量，将二阶系统转化为一阶系统：
1. 定义新的变量：
   $$
   \mathbf{Y}_1 = \mathbf{Y} = (y_1, y_2, \ldots, y_n)^T
   $$
   $$
   \mathbf{Y}_2 = \mathbf{Y}' = (y_{n+1}, y_{n+2}, \ldots, y_{2n})^T
   $$
   其中，$ y_{n+1} = y_1', y_{n+2} = y_2', \ldots, y_{2n} = y_n' $。

2. 建立新的微分方程：
   $$
   \mathbf{Y}_1' = O \mathbf{Y}_1 + I \mathbf{Y}_2
   $$
   $$
   \mathbf{Y}_2' = A_1 \mathbf{Y}_1 + A_2 \mathbf{Y}_2
   $$
   其中，$ O $ 是零矩阵，$ I $ 是单位矩阵。

3. 将方程组合成一个 $ 2n \times 2n $ 的一阶系统：
   $$
   \begin{bmatrix}
   \mathbf{Y}_1' \\
   \mathbf{Y}_2'
   \end{bmatrix}
   =
   \begin{bmatrix}
   O & I \\
   A_1 & A_2
   \end{bmatrix}
   \begin{bmatrix}
   \mathbf{Y}_1 \\
   \mathbf{Y}_2
   \end{bmatrix}
   $$

### **将 m 阶系统转化为一阶系统**

#### **原系统**
对于一个 m 阶微分方程系统：
$$
\mathbf{Y}^{(m)} = A_1 \mathbf{Y} + A_2 \mathbf{Y}' + \ldots + A_m \mathbf{Y}^{(m-1)}
$$
其中，每个 $ A_i $ 是 $ n \times n $ 矩阵。

#### **转化为一阶系统**

通过引入新的变量：
$$
\mathbf{Y}_1 = \mathbf{Y}, \mathbf{Y}_2 = \mathbf{Y}', \ldots, \mathbf{Y}_m = \mathbf{Y}^{(m-1)}
$$
建立新的微分方程：
$$
\mathbf{Y}_1' = \mathbf{Y}_2
$$
$$
\mathbf{Y}_2' = \mathbf{Y}_3
$$
$$
\vdots
$$
$$
\mathbf{Y}_{m-1}' = \mathbf{Y}_m
$$
$$
\mathbf{Y}_m' = A_1 \mathbf{Y}_1 + A_2 \mathbf{Y}_2 + \ldots + A_m \mathbf{Y}_m
$$
将方程组合成一个 $ mn \times mn $ 的一阶系统：
$$
\begin{bmatrix}
\mathbf{Y}_1' \\
\mathbf{Y}_2' \\
\vdots \\
\mathbf{Y}_m'
\end{bmatrix}
=
\begin{bmatrix}
O & I & O & \ldots & O \\
O & O & I & \ldots & O \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
A_1 & A_2 & A_3 & \ldots & A_m
\end{bmatrix}
\begin{bmatrix}
\mathbf{Y}_1 \\
\mathbf{Y}_2 \\
\vdots \\
\mathbf{Y}_m
\end{bmatrix}
$$

### **振动系统的应用**

#### **系统描述**

考虑一个由两个质量 $ m_1 $ 和 $ m_2 $ 和三个弹簧组成的振动系统，弹簧常数均为 $ k $。系统的运动方程为：
$$
m_1 x_1''(t) = -k x_1 + k (x_2 - x_1)
$$
$$
m_2 x_2''(t) = -k (x_2 - x_1) - k x_2
$$
化简后得到：
$$
x_1'' = -\frac{k}{m_1} (2 x_1 - x_2)
$$
$$
x_2'' = -\frac{k}{m_2} (-x_1 + 2 x_2)
$$

#### **设定参数**
假设 $ m_1 = m_2 = 1 $ 和 $ k = 1 $，则系统方程变为：
$$
\mathbf{X}'' = A \mathbf{X}
$$
其中：
$$
A = \begin{bmatrix}
-2 & 1 \\
1 & -2
\end{bmatrix}
$$

#### **特征值和特征向量**
矩阵 $ A $ 的特征值为 $ \lambda_1 = -1 $ 和 $ \lambda_2 = -3 $。对应的特征向量为：
- $ \lambda_1 = -1 $ 对应的特征向量 $ \mathbf{v}_1 = (1, 1)^T $
- $ \lambda_2 = -3 $ 对应的特征向量 $ \mathbf{v}_2 = (1, -1)^T $

#### **求解系统**

对于特征值 $ \lambda_1 = -1 $，其 m 阶根为 $ \sigma_1 = \pm i $。因此：
- $ e^{i t} \mathbf{v}_1 $ 和 $ e^{-i t} \mathbf{v}_1 $ 是系统的复数解。
- 利用欧拉公式，可以得到实数解：
  $$
  \frac{1}{2} (e^{i t} + e^{-i t}) \mathbf{v}_1 = \cos t \cdot \mathbf{v}_1
  $$
  $$
  \frac{1}{2i} (e^{i t} - e^{-i t}) \mathbf{v}_1 = \sin t \cdot \mathbf{v}_1
  $$

### **总结**
1. **高阶系统转化为一阶系统**：通过引入新的变量，将高阶微分方程转化为一阶微分方程组，便于求解。
2. **振动系统应用**：通过设定参数和求解特征值与特征向量，可以找到系统的通解。
3. **复数解转化为实数解**：利用欧拉公式，将复数形式的解转换为实数形式的解，便于物理实现和分析。

---

## 五、矩阵指数与 SVD

### 1. 矩阵指数 (Matrix Exponential)

<img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250602003555629.png" alt="image-20250602003555629" style="zoom:67%;" />

<img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250602003631605.png" alt="image-20250602003631605" style="zoom:67%;" />

- **定义**：$ e^A = \sum_{k=0}^{\infty} \frac{A^k}{k!} \\$

- **对角矩阵**：  
  $$
   e^D = \begin{bmatrix} e^{\lambda_1} & & \\ & \ddots & \\ & & e^{\lambda_n} \end{bmatrix}
  $$
  <img src="Lec6 Eigenvalue and Eigenvector.assets/image-20250602003527060.png" alt="image-20250602003527060" style="zoom:67%;" />

- **可对角化矩阵**：$ e^A = X e^D X^{-1} $

- **微分方程解**：$ Y' = AY $, $ Y(0) = Y_0 $ 的解为 $ Y = e^{tA} Y_0 $

### 2. 奇异值分解 (SVD)

- **定理**：任意 $m \times n$ 矩阵 $A$ 可分解为 
  $$
   A = U \Sigma V^T
  $$

  - $U$：$m \times m$ 正交矩阵（左奇异向量）
  - $V$：$n \times n$ 正交矩阵（右奇异向量）
  - $\Sigma$：$m \times n$ 对角阵（奇异值 $\sigma_1 \geq \cdots \geq \sigma_k > 0$）

- **性质**：  
  - $A^TA$ 和 $AA^T$ 的非零特征值相同  
  - $\text{rank}(A) = \text{rank}(\Sigma)$  
  - 最佳低秩逼近：保留前 $r$ 个奇异值

---

## 总结
1. **特征值/向量**：定义算子不变方向，用于解微分方程与对角化
2. **对角化**：依赖于特征空间直和分解 $\dim V = \sum \dim E(\lambda_j, T)$
3. **应用**：  
   - PageRank（幂迭代）  
   - 线性微分方程（解为特征基的线性组合）  
   - SVD（任意矩阵的广义对角化）
# Lec2: Vector Space (Linear Space)

## Vector Space Axioms (向量空间公理)

> ==满足以下 8 条公理即为向量空间==⭐⭐⭐

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626142341083.png" alt="image-20250626142341083" style="zoom:80%;" />

> 其他性质

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626142757429.png" alt="image-20250626142757429" style="zoom:80%;" />

## Subspace (子空间)

> ==满足以下 3 个性质的非空子集即为子空间== ⭐⭐⭐⭐⭐

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626143549996.png" alt="image-20250626143549996" style="zoom:80%;" />

> 对加法和数乘封闭
>
> 子空间也是向量空间
>
> ==**交也是子空间，并不一定是子空间**==⭐⭐⭐

## Sum of Subspaces (子空间的和)

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626144022973.png" alt="image-20250626144022973" style="zoom:80%;" />

> 为包含指定空间的最小子空间

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626144330468.png" alt="image-20250626144330468" style="zoom:80%;" />

> 直和（和的特殊情况）
>
> > - 普通的和中的任一向量可以由不同的 $u_i$ 组合而成
> > - 直和中的任一向量只能由==唯一一组== $u_i$ 组合而成
>
> 直和中的子空间之间是独立的

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626144754098.png" alt="image-20250626144754098" style="zoom:80%;" />

> ==判断一个和是否为直和==⭐⭐⭐

- ==方法1==⭐⭐⭐

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626145657855.png" alt="image-20250626145657855" style="zoom:80%;" />

- ==方法2==⭐⭐⭐

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626145737555.png" alt="image-20250626145737555" style="zoom:80%;" />

## The Span of a Set of Vectors (张成、生成空间)

> 线性组合

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626150108893.png" alt="image-20250626150108893" style="zoom:80%;" />

> ==为子空间==⭐⭐⭐

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626150158631.png" alt="image-20250626150158631" style="zoom:80%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626150208823.png" alt="image-20250626150208823" style="zoom:80%;" />

> 生成集合（基）

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626150312194.png" alt="image-20250626150312194" style="zoom:80%;" />

> ==证明两个空间相同：相互包含==⭐⭐⭐

## Linear Independence (线性无关、线性独立)

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626150420343.png" alt="image-20250626150420343" style="zoom:80%;" />

> 张成空间中的向量有唯一线性组合（线性无关的）

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626151139577.png" alt="image-20250626151139577" style="zoom:80%;" />

> 线性相关时，去掉一个多余的不影响张成空间，多余的这个也可以被其他的线性组合表示

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626151742852.png" alt="image-20250626151742852" style="zoom:80%;" />

## Basis and Dimension (基和维数)

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626152039203.png" alt="image-20250626152039203" style="zoom:80%;" />

> 标准基 $e_i$

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626152108897.png" alt="image-20250626152108897" style="zoom:80%;" />

> 空间中的向量被基唯一表示

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626152727692.png" alt="image-20250626152727692" style="zoom:80%;" />

> 空间的任意一组线性无关的向量最多包含 $n$ 个，多于 $n$ 个就线性相关

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626152821831.png" alt="image-20250626152821831" style="zoom:80%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626153238213.png" alt="image-20250626153238213" style="zoom:80%;" />

> 不同基的数量相同，取决于空间，定义为维数

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626154040541.png" alt="image-20250626154040541" style="zoom:80%;" />

> ==五个性质==⭐⭐⭐⭐⭐
>
> 线性无关的 $n$ 个向量就能当基，少了扩充，多了减少

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626155603053.png" alt="image-20250626155603053" style="zoom:80%;" />

> 存在一个空间与已知有限维子空间的直和为原空间

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626160052925.png" alt="image-20250626160052925" style="zoom:80%;" />

> ==可能要考的题==⭐⭐⭐⭐⭐

![image-20250626224547959](Lec2 Vector Space (Linear Space).assets/image-20250626224547959.png)





## Change of Basis (基变换)

> 坐标向量 $[v]_E$

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626160506944.png" alt="image-20250626160506944" style="zoom:80%;" />

> ==过渡矩阵（转移矩阵）==⭐⭐⭐⭐⭐
>
> 旧基到新基的表示
> $$
> [old\ basis]=[new\ basis]\cdot S\\
> [u]_{new\ basis}=S\cdot [u]_{old\ basis}
> $$

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626160751679.png" alt="image-20250626160751679" style="zoom:80%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626162532352.png" alt="image-20250626162532352" style="zoom: 67%;" />

> ==过渡矩阵通用情况算法==⭐⭐⭐⭐⭐

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626164417887.png" alt="image-20250626164417887" style="zoom: 60%;" />

> 过渡矩阵例题

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626162624569.png" alt="image-20250626162624569" style="zoom:80%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626163337976.png" alt="image-20250626163337976" style="zoom:80%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626164503338.png" alt="image-20250626164503338" style="zoom:80%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626230206490.png" alt="image-20250626230206490" style="zoom:80%;" />

## Four Subspaces of a Matrix A

> ==“也是毫无疑问必考题，一定要会算四个空间”==⭐⭐⭐⭐⭐

> Column Space of A (A的列空间)

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626165834490.png" alt="image-20250626165834490" style="zoom: 50%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626165559130.png" alt="image-20250626165559130" style="zoom: 50%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626165821523.png" alt="image-20250626165821523" style="zoom: 50%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626171147745.png" alt="image-20250626171147745" style="zoom: 60%;" />

> Nullspace of A (A的零空间)

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626171306186.png" alt="image-20250626171306186" style="zoom:80%;" />

> Row Space of A (A的行空间)

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626171531052.png" alt="image-20250626171531052" style="zoom:80%;" />

> Left Nullspace of A (A的左零空间)

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626171542374.png" alt="image-20250626171542374" style="zoom:80%;" />



## Rank

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173112471.png" alt="image-20250626173112471" style="zoom: 67%;" />

## Orthogonality of the Four Subspaces

> 向量正交

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173322780.png" alt="image-20250626173322780" style="zoom:80%;" />

> 空间正交

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173339184.png" alt="image-20250626173339184" style="zoom:80%;" />

> 正交补

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173354201.png" alt="image-20250626173354201" style="zoom:80%;" />

> 定理

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173426158.png" alt="image-20250626173426158" style="zoom:80%;" /> 

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173431733.png" alt="image-20250626173431733" style="zoom:80%;" /> 

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173440845.png" alt="image-20250626173440845" style="zoom:80%;" /> 

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173509393.png" alt="image-20250626173509393" style="zoom: 67%;" /> 

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173515511.png" alt="image-20250626173515511" style="zoom:67%;" /> 

## Fundamental Theorem of Linear Algebra

> ==“毫无疑问必考题”==⭐⭐⭐⭐⭐
>
> 包括 `Note 2.9.2`

### Part 1

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173205009.png" alt="image-20250626173205009" style="zoom:80%;" />

### Part 2

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173230275.png" alt="image-20250626173230275" style="zoom:80%;" />

<img src="Lec2 Vector Space (Linear Space).assets/image-20250626173547226.png" alt="image-20250626173547226" style="zoom:80%;" /> 
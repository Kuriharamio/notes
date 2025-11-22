# ompl规划器



## CForest: Coupled Forest of Random Engrafting Search Trees

### 核心思想
CForest是一个旨在提高单查询最短路径规划算法并行性框架。它不被视为一个独立的路径规划算法，而是一个支持任何随机树算法的工具，这些算法能够在任何配置空间中工作，从而实现几乎完全收敛到最优解的目标。CForest依赖于底层规划器设置的优化目标，并利用多个并行规划器来加速路径搜索。

### 算法原理
CForest通过结合多个随机树规划器的力量，利用并行计算的优势来加速路径规划的解决过程。它的工作流程按照以下步骤进行：
1. **初始化多个规划器实例**，这些实例可取用不同的算法，如RRT、PRM等。
2. 在每个规划器内执行路径搜索，同时共享状态和路径信息。
3. **同步规划器的结果**，以保证算法收敛到最佳解决方案。
4. 动态调整**搜索焦点**以优化规划性能。

### 优点
- **高效的资源利用**：通过同时运行多个规划器，充份利用多核处理器的计算能力。
- **几乎收敛到最优解**：由于依赖于具有良好收敛性质的随机树算法，CForest能够在多次运行中找到近似最优解。
- **灵活性**：支持多种随机树算法，易于与其他规划器集成。

### 缺点
- **复杂性**：设置和管理多个并行规划器增加了算法的复杂性。
- **同步开销**：多个规划器之间的状态共享可能引入一些同步开销，影响性能。
- **参数调整需求**：需要用户对各个参数进行合理配置，以达到最佳效果。

### 示例代码
```cpp
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/base/spaces/SE3StateSpace.h>

// 创建规划环境
ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

// 创建 CForest 规划器
ompl::geometric::CForest cforest(si);

// 设置默认线程数
cforest.setNumThreads(4); // 设置使用4个线程

// 设置焦点搜索选项
cforest.setFocusSearch(true); // 启用搜索焦点

// 添加具体的规划器实例（如 RRT规划器）
cforest.addPlannerInstances<ompl::geometric::RRT>(2); // 添加2个 RRT 规划器实例

// 清除之前的规划器实例
cforest.clearPlannerInstances(); // 清空当前所有的规划器实例
```

- 参数说明

  - `setNumThreads(unsigned int numThreads)`：
    - 设置当未指定规划器实例时，CForest使用的默认线程数。
    

  - `setFocusSearch(bool focus)`：
    - 控制在搜索过程中是否集中搜索。
    

  - `addPlannerInstances(std::size_t num)`：
    - 添加多个特定类型的规划器实例，利用并行计算来增强搜索能力。




## EST: Expansive Space Trees

### 核心思想

EST（Expansive Space Trees）是一种基于树的运动规划器，它通过测量已探索空间的密度来检测较少被探索的区域，从而偏向于探索那些密度最低的空间。这种策略有助于加快对复杂配置空间的有效路径规划，尤其是在广阔且未探索区域较多的环境中。

### 算法原理

EST的工作原理主要包括以下几个步骤：

1. **随机采样**：算法在配置空间中随机选取状态，目标是尝试到达这些状态。
2. **目标偏置**：在采样过程中，算法可能以一定概率直接选择目标状态，从而增加找到目标的机会。默认情况下，这个概率大约设定为0.05。
3. **扩展树结构**：通过向已知状态添加运动，算法构建一棵拓展树。每次扩展时，算法会考虑到与树中现有状态的距离，确保只添加在最大运动范围内的状态。
4. **邻域计算**：算法使用最近邻数据结构来管理树中的状态，通过计算邻域内的状态来判断下一步的扩展方向。

### 优点

- **高效空间探索**：通过偏向低密度区域，算法能够快速找到复杂配置空间中的路径。
- **灵活性**：适用范围广，能够处理高维度和非结构化空间。
- **全局搜索能力**：适当的目标偏置和扩展方法促使算法能够在全局范围内探索。

### 缺点

- **参数敏感性**：算法的性能高度依赖于目标偏置和最大运动范围的参数设置，选择不当可能导致效率低下。
- **复杂度**：在高维空间中，树的构建和维护可能变得复杂，尤其是随着状态数的增加。
- **计算开销**：每次扩展都需要邻域计算，这在状态数非常多时可能引入额外的成本。

### 示例代码

```cpp
#include <ompl/geometric/planners/EST.h>
#include <ompl/base/spaces/SE3StateSpace.h>

// 创建规划环境
ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

// 创建 EST 规划器
ompl::geometric::EST est(si);

// 设置最大运动范围
est.setRange(0.5); // 设置最大为0.5单位的运动范围

// 设置目标偏置概率
est.setGoalBias(0.05); // 设置目标状态选择概率为0.05

```

- 参数说明

  - `setRange(double distance)`：
    - 设置规划器使用的最大运动范围，影响路径扩展的有效性和运行时间。

  - `getRange() const`：
    - 获取当前设置的运动范围。

  - `setGoalBias(double goalBias)`：
    - 设置占据树中正在扩展向目标状态的概率。值通常应在0.0到1.0之间，一般建议设置为0.05。

  - `getGoalBias() const`：
    - 获取当前目标偏置的概率。



## BiEST: Bi-directional Expansive Space Trees

### 核心思想

BiEST（Bi-directional Expansive Space Trees）是一种双向树形运动规划器，旨在通过测量配置空间内已探索区域的密度来识别未被探索的区域，并且同时从起始点和目标点两方向进行扩展。这种方式可以有效减少找到路径所需的搜索时间，并提高规划效率。

### 算法原理

BiEST的基本工作原理包括以下步骤：

1. **双向扩展**：算法同时从起始点和目标点扩展两棵树，以更快地找到连接这两个点的路径。
2. **目标检测**：通过定期评估树中节点的密度，BiEST能够在低密度的区域中进行更多的扩展，从而提升路径规划的效率。
3. **邻域探索**：在每次扩展时，将通过最近邻数据结构来寻找邻域内的状态。
4. **连接优化**：在两棵树的扩展过程中，当树之间接触时，算法将尝试连接它们，以找到一条完整路径。

### 优点

- **高效性**：双向搜索策略显著减少了找到路径的时间，尤其是在复杂的配置空间中。
- **低密度区域优化**：通过偏向低密度区域扩展，算法能够更快找到路径。
- **灵活性**：能够适应多种环境配置，处理高维空间问题。

### 缺点

- **复杂性**：双向搜索的实现相对复杂，需要仔细管理两个树的连接和扩展。
- **参数敏感性**：算法的表现受到最大运动范围等参数的影响，需要根据具体情况调整。
- **可能的连接失败**：在某些情况下，连接两棵树的过程可能失败，从而导致规划失败。

### 示例代码

```cpp
#include <ompl/geometric/planners/BiEST.h>
#include <ompl/base/spaces/SE3StateSpace.h>

// 创建规划环境
ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

// 创建 BiEST 规划器
ompl::geometric::BiEST biest(si);

// 设置最大运动范围
biest.setRange(0.5); // 设置最大为0.5单位的运动范围
```

- 参数说明

  - `setRange(double distance)`：
    - 设置规划器扩展时使用的最大运动范围，影响路径扩展的有效性和运行时间。

  - `getRange() const`：
    - 获取当前设置的运动范围。



## ProjEST: Projection Expansive Space Trees

### 核心思想

ProjEST（Projection Expansive Space Trees）是一种基于树的运动规划器，它通过在状态空间的投影上施加网格来检测未被充分探索的区域，优先扩展这些区域。这种方法改进了传统的EST算法，使得树的扩展更具方向性，能够有效探索配置空间。

### 算法原理

ProjEST的基本步骤包括：

1. **网格化投影**：算法依赖于一个格子（grid）以组织和管理状态空间的探索，将状态空间投影到较低维度，并在这些投影上进行网格划分。
2. **状态选择**：在每次扩展中，ProjEST会选择未被充分探索的网格单元来进行状态采样和扩展。
3. **动态树扩展**：在扩展过程中，如果与目标状态连接的机会出现，算法会尝试将当前状态与目标状态连接，以构建完整路径。
4. **使用投影评估器**：通过设置投影评估器，用户可以自定义网格化操作，从而更好地适应特定的状态空间特征。

### 优点

- **有效的空间探索**：通过对探索空间的网格化，算法可以更有效地关注于较少探索的区域，加快路径寻找的速度。
- **灵活的投影选择**：用户可以通过设置不同的投影评估器来适应不同的应用场景。
- **良好的适应性**：能够处理高维复杂的配置空间，适应不同的动态场景。

### 缺点

- **依赖投影质量**：算法效果很大程度上依赖所选投影的质量，劣质的投影可能导致探索效率低下。
- **复杂的参数调节**：用户需要合理设置多个参数（例如，最大运动范围、投影评估器等）以达到最佳效果。
- **内存开销**：维护网格和其数据结构可能引入额外的内存开销，尤其在状态空间较大时。

### 示例代码

```cpp
#include <ompl/geometric/planners/ProjEST.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ProjectionEvaluator.h>

// 创建规划环境
ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

// 创建 ProjEST 规划器
ompl::geometric::ProjEST projEst(si);

// 设置最大运动范围
projEst.setRange(0.5); // 设置最大为0.5单位的运动范围

// 设置目标偏置概率
projEst.setGoalBias(0.05); // 设置目标状态选择概率为0.05

// 设置投影评估器
projEst.setProjectionEvaluator("my_custom_projection_name"); // 设置自定义投影评估器

// 进行规划设置
projEst.setup(); // 初始化规划器，准备执行规划
```

- 参数说明

  - `setRange(double distance)`：
    - 设置规划器扩展时使用的最大运动范围，影响路径扩展的有效性和运行时间。

  - `getRange() const`：
    - 获取当前设置的运动范围。

  - `setGoalBias(double goalBias)`：
    - 设置占据树中正在扩展向目标状态的概率。

  - `getGoalBias() const`：
    - 获取当前目标偏置的概率。

  - `setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)`：
    - 设置用于计算状态投影的评估器，允许自定义投影处理方式。

  - `setProjectionEvaluator(const std::string &name)`：
    - 根据名称设置投影评估器，使用已注册的评估器。



## BFMT*: Bidirectional Fast Marching Trees

### 核心思想

**BFMT***（Bidirectional Fast Marching Trees）是一种渐近最优的双向采样基于动态规划的运动规划算法，旨在解决高维复杂配置空间中的路径规划问题。该算法通过构建两棵从起始状态和目标状态分别向外扩展的树来寻找最短路径，从而能够以较高的效率找到解决方案。

### 算法原理

BFMT*的工作流程主要包含以下几个步骤：

1. **双向树的构建**：算法从起始状态和目标状态分别开始构建两棵树，同时在运动过程中不断扩展这两棵树。
2. **采样与连接**：在每次迭代中，从自由配置空间中随机采样若干状态，并尝试将这些状态连接到树中，以更新路径信息。
3. **成本计算**：每个状态的连接成本会被计算，并且每个连接便形成一个新的节点，进一步扩展到树中。
4. **最优性检查**：算法通过维护打开列表（Open list）和关闭列表（Closed list），确保只连接那些具有最低代价的路径，并不断更新当前代价。

### 优点

- **渐近最优**：BFMT*保证连接到最短路径，适合处理复杂的路径规划问题。
- **高效的空间利用**：能够在高维空间中进行有效的采样与搜索，减少不必要的计算。
- **灵活的策略选择**：用户可以选择不同的策略进行探索、终止等，例如选择是否使用K近邻策略等。

### 缺点

- **内存消耗**：使用缓存来存储碰撞检查结果可能导致较大的内存开销，特别是在节点数较多时。
- **依赖参数设置**：算法的性能在很大程度上依赖于各类参数（如采样数量、搜索半径等）的合理设置。
- **复杂性**：实现相对复杂，对使用者需要一定的理解和经验。

### 示例代码

```cpp
#include <ompl/geometric/planners/BFMT.h>
#include <ompl/base/spaces/SE3StateSpace.h>

// 创建规划环境
ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

// 创建 BFMT 规划器
ompl::geometric::BFMT bfmt(si);

// 设置样本数量
bfmt.setNumSamples(1000); // 设置样本数量为1000

// 设置半径乘数
bfmt.setRadiusMultiplier(1.1); // 设置半径乘数大于1以确保渐近收敛

// 启用缓存碰撞检查
bfmt.setCacheCC(true); // 启用碰撞检查缓存

// 进行规划设置
bfmt.setup(); // 初始化规划器，准备执行规划
```

- 参数说明

  - `setNumSamples(unsigned int numSamples)`：
    - 设置规划器在采样时要使用的状态数量，默认为1000。

  - `setRadiusMultiplier(double radiusMultiplier)`：
    - 设置进行最近邻搜索时的半径乘数，默认值为1.1，确保渐近收敛。

  - `setCacheCC(bool ccc)`：
    - 启用或禁用碰撞检查缓存，以提高性能和减少重复计算。

  - `setHeuristics(bool h)`：
    - 启用或禁用代价到达启发式计算，以便更精确地排序探索的节点。

    

## FMT*: Fast Marching Trees

### 核心思想

**FMT***（Fast Marching Trees）是一种渐近最优的采样基运动规划算法，旨在解决高维复杂配置空间中的路径规划问题。该算法通过使用延迟动态规划方法，对一组基于概率采样的状态进行递归，逐步扩展出一颗路径树，最终收敛到最短路径的解决方案。

### 算法原理

FMT*的主要工作流程包括以下几个关键步骤：

1. **采样状态**：从自由配置空间中随机采样多个状态。这些状态用于构建树，从而保证路径的多样性和覆盖性。
2. **邻接连接**：对于每个节点，算法会搜索与其距离在特定范围内的其他节点，并尝试将其连接到当前树中，以增加树的大小和覆盖范围。
3. **成本计算**：每个连接的路径会计算相应的成本，算法会选择成本最低的路径来进行扩展，从而形成最优解。
4. **收敛性**：随着样本数量的增加，FMT*保证最终能收敛到一条最短路径。

### 优点

- **渐近最优**：算法能保证最终找到最短路径，适用于复杂的高维路径规划问题。
- **高效性**：通过使用延迟更新和近似处理，FMT*能够在大多数情况下快速找到解。
- **灵活的参数配置**：用户可以调整多个参数（如样本数量、半径乘数），以优化算法性能。

### 缺点

- **内存需求**：引入的缓存机制可能增加内存需求，但随着样本增多，内存负担趋于O(n)。
- **对参数敏感**：算法的性能很大程度上取决于参数设置；不当的参数选择可能导致效率下降或无法收敛。
- **实现复杂度**：相较于其他基本算法，FMT*的实现较复杂，需一定的理解和调试能力。

### 示例代码

```cpp
#include <ompl/geometric/planners/FMT.h>
#include <ompl/base/spaces/SE3StateSpace.h>

// 创建规划环境
ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

// 创建 FMT 规划器
ompl::geometric::FMT fmt(si);

// 设置采样数量
fmt.setNumSamples(1000); // 设置样本数量为1000

// 设置半径乘数
fmt.setRadiusMultiplier(1.1); // 设置半径乘数以确保渐近收敛

// 启用碰撞检查缓存
fmt.setCacheCC(true); // 启用碰撞检查缓存
```

- 参数说明

  - `setNumSamples(unsigned int numSamples)`：
    - 设置规划器在一次规划中要使用的样本数量，默认值为1000。

  - `setRadiusMultiplier(double radiusMultiplier)`：
    - 设置用于最近邻搜索的半径乘数，建议值在1到5之间，默认值为1.1。

  - `setCacheCC(bool ccc)`：
    - 启用或禁用碰撞检查缓存，以提高算法性能。。

  - `setHeuristics(bool h)`：
    - 启用或禁用代价到达启发式计算。

  - `setExtendedFMT(bool e)`：
    - 启用或禁用扩展的 FMT* 策略，在没有找到解决方案时添加新样本。

    

## ABIT*: Advanced Batch Informed Trees

### 核心思想

**ABIT***（Advanced Batch Informed Trees）是一种几乎总是渐近最优的路径规划算法。该算法将路径规划问题视为两个子问题——近似和搜索。这种视角使其能够使用先进的图搜索技术，例如膨胀（inflation）和截断（truncation）。ABIT*是从BIT*算法演变而来，它结合了多种优化目标，能够处理多个起点和目标，以及使用按需采样等新技术，以适应无限状态空间。

### 算法原理

ABIT*的主要实现机制和步骤包括：

1. **近似和搜索**：算法同时处理路径的近似生成和有效搜索，通过动态调整参数来增强搜索效率。
2. **样本生成**：在找到初始解之前，算法优先进行探索，生成样本以扩大搜索范围。
3. **图的扩展**：使用膨胀和截断策略更新图结构，以提高搜索的准确性和效率。
4. **动态参数调整**：通过设置膨胀因子和截断因子的更新策略，算法能够适应不同的环境和优化目标。

### 优点

- **高效探索**：依靠按需生成样本的能力，ABIT*能够有效应对无限状态空间和复杂环境。
- **灵活性**：支持多种优化目标，包括路径长度、时间等，用户可以根据需要自定义目标。
- **先进的图搜索技术**：通过使用先进的图搜索策略，如延迟重连、膨胀和截断等，显著提高了探索的效率。

### 缺点

- **复杂性**：与其他简单的路径规划算法相比，ABIT*的实现和调试较为复杂，需要用户对算法原理有深入理解。
- **计算开销**：在处理大量样本或高维空间时，可能会引入较大的计算和内存开销，尤其是在图更新的过程中。

### 示例代码

```cpp
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/base/spaces/SE3StateSpace.h>

// 创建规划环境
ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

// 创建 ABIT* 规划器
ompl::geometric::ABITstar abitstar(si);

// 设置初始膨胀因子
abitstar.setInitialInflationFactor(1000000.0); // 并指定一个足够大的初始膨胀因子

// 设置膨胀因子更新参数
abitstar.setInflationScalingParameter(10.0); // 设置膨胀因子缩放参数

// 设置截断因子更新参数
abitstar.setTruncationScalingParameter(5.0); // 设置截断因子缩放参数

```

- 参数说明

  - `setInitialInflationFactor(double factor)`：
    - 设置初始的膨胀因子，用于首次搜索时的路径连接。
- `setInflationScalingParameter(double parameter)`：
    - 设置膨胀因子更新策略的缩放参数，影响路径连接的质量。
  - `setTruncationScalingParameter(double parameter)`：
  - 设置截断因子的更新策略参数，根据样本数量动态调整。



## LBKPIECE1: Lazy Bi-directional KPIECE with One Level of Discretization

### 核心思想

LBKPIECE1是一种树形规划器，利用离散化（通常为多个级别）来引导连续空间的探索。该实现采用简化方案，使用单一级别的离散化，即一个Grid。规划器优先在Grid的边界上进行探索，Grid的边界是通过计算在n维投影空间中邻接少于两个非对角邻居的Grid单元格来确定的。此外，该实现采用双向树探索与延迟碰撞检测相结合，从而更有效地进行路径规划。

### 算法原理

LBKPIECE1的工作原理基于以下步骤：

1. **初始化两个探索树**，一个是起始树，另一个是目标树。
2. 在每个树中探索时，优先考虑Grid的边界，以更高效地生成可能的路径。
3. 使用延迟碰撞检测：即在规划过程中不立即验证每条路径的有效性，而是当需要确认结果时，才进行碰撞检测。
4. 利用设定的投影评估器对状态进行有效的管理和状态的映射。

### 优点

- **高效探索**：通过对Grid边界的偏好选择，提高了路径搜索效率。
- **延迟检测**：允许在降低计算开销的同时调整路径，优化了碰撞检测的性能。
- **适用性**：适用于各种状态空间，特别是高维空间。

### 缺点

- **依赖投影**：算法性能高度依赖于投影评估器的质量，错误的投影可能导致有效性降低。
- **参数敏感性**：各参数（如距离范围和有效路径比例）需要小心调整，以获得最佳性能。
- **复杂性**：对于使用者来说，理解双树和延迟碰撞检测的机制可能会增加学习成本。

### 示例代码

```cpp
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/base/spaces/SE3StateSpace.h>

// 创建规划环境
ompl::base::StateSpacePtr space(new ompl::base::SE3StateSpace());
ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

// 创建 LBKPIECE1 规划器
ompl::geometric::LBKPIECE1 lbkPiece(si);

// 设置距离范围
lbkPiece.setRange(0.5); // 设置每次扩展的最大长度为0.5

// 设置边界选择比例
lbkPiece.setBorderFraction(0.9); // 设置90%的时间专注于边界选择

// 设置最小有效路径比例
lbkPiece.setMinValidPathFraction(0.7); // 设置最小有效路径比率为70%

// 设置投影评估器 
// lbkPiece.setProjectionEvaluator(projectionEvaluator); // 这里需要设置合适的评估器
```

- 参数说明

  - `setRange(double distance)`：
    - 设置规划器在扩展时使用的最大运动长度。

  - `setBorderFraction(double bp)`：
    - 设置专注于边界的时间比例，值在0到1之间。

  - `setMinValidPathFraction(double fraction)`：
    - 设置在扩展过程中允许的最小有效路径比例。

  - `setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)`：
    - 指定规划器使用的投影评估器。

  - `setProjectionEvaluator(const std::string &name)`：
    - 从已注册的状态空间中选择一个投影评估器。

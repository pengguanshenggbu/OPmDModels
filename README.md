# CompactModelOPmD

本项目用于求解 OP-mD（Orienteering Problem with multiple Drones）问题。代码实现了两类 Gurobi 精确模型：

- `BranchAndCutModel.py`：Branch-and-Cut 模型，使用 lazy constraints / user cuts 处理子回路、卡车-无人机路径一致性和时间同步约束。
- `CompactModel.py`：紧凑混合整数规划模型，显式建模卡车到达/离开时间、无人机起飞/降落时间和无人机占用状态。

项目还包含算例生成、参数构造、批量实验、结果统计、绘图以及 OP solver 格式转换等辅助脚本。

## 项目结构

```text
.
├── BranchAndCutModel.py        # Branch-and-Cut 主模型和命令行入口
├── CompactModel.py             # Compact Model 主模型和命令行入口
├── Get_OPmD_data.py            # 从坐标/收益生成 OP-mD 模型输入集合与参数
├── Prepare_parameters.py       # 根据 L_ratio、T_ratio 等批量生成实验参数
├── BatchSolve.py               # 批量参数预处理示例
├── run_all_experiments_BC.sh   # Branch-and-Cut 批量实验脚本
├── Data/                       # .inst 算例数据
│   ├── random/
│   ├── clustered/
│   └── mixed/
├── Result_Model/               # 模型求解结果与解文件
├── Utils/                      # 通用工具：实例解析、TSP、解检查、结果排序等
├── InstanceGenerator/          # 随机/聚类/混合算例生成脚本
├── OPsolve/                    # OP solver 格式转换和日志解析
├── Plot/                       # 结果绘图脚本和图片
├── VNS_Statistic/              # VNS 结果统计脚本
├── EvaluationCompare/          # 不同实验结果对比脚本
└── Tunning/                    # 参数配置抽样和应用脚本
```

## 环境依赖

建议使用 Python 3.10+。核心求解依赖 Gurobi，因此需要本机已安装 Gurobi Optimizer，并配置好有效 license。

主要 Python 包：

```bash
pip install gurobipy pandas numpy matplotlib
```

说明：

- `gurobipy` 是求解模型的必需依赖。
- `pandas` 主要用于批量参数、统计和结果表处理。
- `numpy`、`matplotlib` 主要用于绘图脚本。

## 数据格式

算例文件位于 `Data/` 下，常用入口默认读取 `Data/random/`。每个 `.inst` 文件格式如下：

```text
节点数
x坐标 y坐标 奖励
x坐标 y坐标 奖励
...
```

第 0 个节点为 depot，通常奖励为 0。例如：

```text
10
43.6424 6.77746 0
4.86666 29.3957 15
...
```

模型参数含义：

- `alpha`：无人机相对速度参数。无人机两点飞行时间为欧式距离除以 `alpha` 后向下取整。
- `beta`：无人机访问节点的奖励系数，`q = floor(p * beta)`。
- `L`：无人机单次 sortie 最大续航距离。
- `m`：无人机数量。
- `Tmax`：总完工时间上限。

卡车旅行时间使用曼哈顿距离向下取整。

## 单实例求解

Branch-and-Cut：

```bash
python3 BranchAndCutModel.py poi-10-1 1 0.67 15 1 59
```

Compact Model：

```bash
python3 CompactModel.py poi-10-1 1 0.67 15 1 59
```

输出为一行 tab 分隔结果：

```text
name alpha beta L m T OBJ GAP CPU Nodes obj_bound
```

示例：

```text
poi-10-1  1.0  0.67  15.0  1  59.0  201  0.0  0.0045  1.0  201
```

两个入口还支持可选参数：

```bash
python3 BranchAndCutModel.py <name> <alpha> <beta> <L> <m> <Tmax> <IsPrint> <IsOutput>
```

- `IsPrint=True`：在控制台打印卡车路径和无人机 sorties。
- `IsOutput=True`：将汇总结果和解文件写入 `Result_Model/`。

注意：当前 argparse 将 `IsPrint`、`IsOutput` 声明为 `bool`，命令行传入字符串时 Python 会按非空字符串转为 `True`。如果需要关闭，建议直接省略这两个参数。

## 批量实验

运行 Branch-and-Cut 批量脚本：

```bash
bash run_all_experiments_BC.sh
```

脚本会：

1. 设置最大并行任务数 `MAX_JOBS=8`。
2. 初始化结果文件 `Result/BC_result_new.txt`。
3. 读取脚本自身所有以 `python3` 开头的命令，并用 `xargs -P` 并行执行。

如果要调整批量实验：

- 修改 `MAX_JOBS` 控制并行度。
- 在脚本末尾增删 `python3 BranchAndCutModel.py ...` 命令。
- 确保输出目录存在或按需要修改脚本中的结果路径。

## 结果文件

常见输出位置：

- `Result_Model/BC_result*.txt`：Branch-and-Cut 汇总结果。
- `Result_Model/CompactModel_result*.txt`：Compact Model 汇总结果。
- `Result_Model/solution_BC/`：Branch-and-Cut 解文件。
- `Result_Model/solution_compact/`：Compact Model 解文件。

解文件内容通常包含：

```text
Route: 0, ...
Drone 0: (launch,visit,land), ...
```

其中 truck route 表示卡车闭环路径；drone sortie 三元组 `(i,k,j)` 表示无人机从节点 `i` 起飞，访问客户 `k`，在节点 `j` 回收。

## 关键模块说明

### `Get_OPmD_data.py`

从实例坐标和参数构建模型输入：

- `N`：节点集合。
- `A`：卡车有向弧集合。
- `H`：满足续航约束的无人机 sortie 集合。
- `D`：无人机集合，如 `d1, d2, ...`。
- `p`、`q`：卡车/无人机访问奖励。
- `t`、`tprime`：卡车旅行时间和无人机 sortie 时间。
- `M`：时间同步约束使用的 Big-M 参数。

### `BranchAndCutModel.py`

核心变量：

- `x[i,j]`：卡车是否使用弧 `(i,j)`。
- `y[i]`：节点 `i` 是否由卡车访问。
- `z[h,d]`：无人机 `d` 是否执行 sortie `h=(i,k,j)`。
- `W[i]`：节点总等待时间。
- `w[i,d]`：无人机 `d` 在节点 `i` 相关等待时间。

模型目标为最大化卡车与无人机收集的总奖励。求解时间上限当前设为 3600 秒。

### `CompactModel.py`

相比 Branch-and-Cut，紧凑模型显式使用时间变量：

- `arrivalTruck`、`departTruck`
- `arrivalDrone`、`launchDrone`
- `droneUse`

并使用 MTZ 约束处理卡车子回路。

### `Utils/`

- `Utils.py`：实例解析、旅行时间计算、结果打印。
- `TSPmodel.py`：用于计算 TSP 基准时间的 Gurobi 模型。
- `SolutionChecker.py`：对 Compact Model 解进行可行性检查的辅助类。
- `GetL.py`、`GetTmax.py`：辅助生成续航和时间预算参数。
- `Reorder_parallel_result.py`：整理并行实验输出顺序。

## 算例生成

`InstanceGenerator/generate_optw_instances.py` 支持生成：

- uniform random 实例：`r-n-id.inst`
- clustered 实例：`c*-n-id.inst`
- mixed 实例：`rc*-n-id.inst`

脚本默认路径为 `InstanceGenerator/data/mixed/`，如需将新实例用于主模型，建议复制到 `Data/random/`、`Data/clustered/` 或 `Data/mixed/`，并在入口脚本中调整读取目录。

## OP solver 格式转换

`OPsolve/Transfer2Oplib.py` 可将项目 `.inst` 数据转换为 OP solver 使用的 `.oplib` 格式；`parse_opsolver_log_to_csv.py` 用于解析外部 solver 日志。更多细节见 `OPsolve/README.md`。

## 当前注意事项

- `BranchAndCutModel.py` 和 `CompactModel.py` 默认只从 `./Data/random/` 读取实例。
- `Prepare_parameters.py` 的 `__main__` 示例里仍有旧路径 `./analysis/data/random/`，直接运行前需要改成当前数据目录。
- `Utils/Utils.py` 的 `save_result_txt` 使用了 `os.path`，但该文件当前未显式 `import os`；若开启 `IsOutput=True` 时遇到 `NameError`，需要补充该 import。
- `OPmDsolver.py` 看起来是早期封装入口，当前与 `get_OPmD_data` 和结果字段命名不完全一致，建议优先使用 `BranchAndCutModel.py` 或 `CompactModel.py`。

## 推荐工作流

1. 将 `.inst` 算例放入 `Data/random/`。
2. 先用小规模实例验证参数：

   ```bash
   python3 BranchAndCutModel.py poi-10-1 1 0.67 15 1 59
   ```

3. 确认 Gurobi license、路径和输出正常后，再运行批量脚本。
4. 使用 `Result_Model/` 下的汇总文件和 `Plot/`、`VNS_Statistic/` 脚本进行后处理。

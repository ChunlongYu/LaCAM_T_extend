# LaCAM\* 的最优性、Gap 终止与全局 Lower Bound 讨论笔记

## 1. 问题背景

LaCAM\* 是 LaCAM 的 anytime 版本。它的基本思想是：

1. 先快速找到一个可行解；
2. 不立即停止，而是继续 lazy search；
3. 在搜索过程中不断改进当前最好解；
4. 当搜索空间被完全证明不会再产生更优解时，才声明最优。

论文中的 LaCAM\* 不是传统意义上的 Branch-and-Bound（B&B），但它具有一些类似 B&B 的结构：

- 当前最好解可以看作上界 \(UB\)；
- 启发式 \(h\) 是剩余代价的 admissible lower bound；
- 对某些节点可以基于 \(g+h\) 做剪枝；
- 找到 goal 后继续搜索，最终收敛到最优解。

但是，LaCAM\* 原始算法并没有像 MIP 一样提供可靠的 MIPGap-style 提前终止证书。

---

## 2. 为什么不能直接用 \(\min_{N\in Open} g_t(N)+h(N)\) 作为全局下界？

最初容易产生一个想法：

\[
LB_t = \min_{N\in Open}\{g_t(N)+h(N)\}
\]

然后如果已有当前最好解：

\[
UB_t = g_t(G)
\]

则计算：

\[
gap_t = \frac{UB_t-LB_t}{UB_t}
\]

如果 gap 足够小，就提前终止。

这个想法看起来很像 A\* 或 B&B，但对 LaCAM\* 来说是不严谨的。

原因在于，LaCAM\* 是 lazy successor generation。当前已经发现的 configuration graph 只是完整 configuration graph 的一部分，记为：

\[
H_t=(V_t,E_t)
\]

其中 \(E_t\) 只是目前已经生成出来的边。

因此：

\[
g_t(N)
\]

只是当前已知图 \(H_t\) 中从起点 \(S\) 到配置 \(N\) 的最短距离，而不是完整 configuration graph 中从 \(S\) 到 \(N\) 的真实最短距离。于是可能有：

\[
g_t(N) > g^*(N)
\]

也就是说，\(g_t(N)\) 可能是高估的。

即便 \(h(N)\) 是 admissible 的，只要 \(g_t(N)\) 高估，那么：

\[
f_t(N)=g_t(N)+h(N)
\]

也可能高估经过 \(N\) 的真实最优路径代价。

所以：

\[
\min_{N\in Open} f_t(N)
\]

不一定是全局 lower bound。

---

## 3. 一个反例：为什么直接用 \(g_t(X)+h(X)\) 会错？

考虑一个抽象 configuration graph。真实图中有以下边：

\[
S\rightarrow A\rightarrow G
\]

\[
S\rightarrow X
\]

\[
S\rightarrow B\rightarrow X\rightarrow G
\]

边代价为：

\[
c(S,A)=4,\quad c(A,G)=4
\]

\[
c(S,X)=10
\]

\[
c(S,B)=1,\quad c(B,X)=1,\quad c(X,G)=1
\]

真实最优路径是：

\[
S\rightarrow B\rightarrow X\rightarrow G
\]

总代价为：

\[
1+1+1=3
\]

但假设当前 LaCAM\* 只 lazy 生成了这些边：

\[
S\rightarrow A,\quad A\rightarrow G,\quad S\rightarrow X
\]

还没有生成：

\[
S\rightarrow B,\quad B\rightarrow X,\quad X\rightarrow G
\]

那么当前已知图是：

```text
S --4--> A --4--> G      当前已有解 UB = 8

S --10--> X              当前 g_t(X)=10
```

此时：

\[
UB=8
\]

且：

\[
g_t(X)=10
\]

如果 \(h(X)=1\)，则：

\[
g_t(X)+h(X)=11
\]

如果误把 \(11\) 当成经过 \(X\) 的路径下界，并认为：

\[
11>8
\]

于是剪掉 \(X\)，那就错了。

因为真实图中存在路径：

\[
S\rightarrow B\rightarrow X\rightarrow G
\]

其总代价为：

\[
3<8
\]

这个例子说明：

> \(g_t(X)+h(X)\) 不是经过配置 \(X\) 的全局有效下界，因为 \(g_t(X)\) 可能高估了真实到达 \(X\) 的最短代价。

---

## 4. LaCAM\* 原文中的剪枝为什么不破坏最优性？

LaCAM\* Algorithm 3 中有类似这样的剪枝：

\[
g_t(N)+h(N)\geq UB
\]

则当前节点可以被 pop。

但这不是传统 B&B 中的“永久剪枝”。

更准确地说，这只是：

> 在当前已知到达 \(N\) 的前缀代价 \(g_t(N)\) 下，暂时不值得继续扩展 \(N\)。

这个节点并没有从 `Explored` 中彻底删除。之后如果 LaCAM\* 发现了新的边，使得某个已知节点的 \(g\)-value 下降，算法会通过 Dijkstra-style rewiring 更新 \(g\) 和 parent，并可能重新激活该节点。

例如在上面的反例中，当前：

\[
g_t(X)=10
\]

所以 \(X\) 可能暂时不被扩展。

但如果之后生成了：

\[
S\rightarrow B,\quad B\rightarrow X
\]

则 rewiring 会更新：

\[
g_t(X):10\rightarrow 2
\]

于是：

\[
g_t(X)+h(X)=2+1=3
\]

此时 \(X\) 又变成一个有希望的节点，可以被重新加入搜索。

因此，LaCAM\* 的最优性不是靠 \(g+h\) 作为全局下界来保证的，而是靠：

1. lazy successor generation 最终是 exhaustive 的；
2. 已发现图 \(H_t\) 中的最短路径信息通过 rewiring 维护；
3. 当所有后继都被完全枚举、Open 为空时，完整搜索空间中不存在更优解。

---

## 5. 如何构造真正的全局 lower bound？

关键思路是：不要对任意配置节点 \(N\) 使用 \(g_t(N)+h(N)\)，而是考虑一条完整路径中**第一条尚未生成的边**。

定义当前已知 configuration graph：

\[
H_t=(V_t,E_t)
\]

其中：

- \(V_t\)：当前已经发现的 configuration；
- \(E_t\)：当前已经 lazy 生成的 transition；
- \(g_t(X)\)：在 \(H_t\) 中从 \(S\) 到 \(X\) 的最短路径代价。

对于一个已发现配置 \(X\)，定义：

\[
U_t(X)=Succ(X)\setminus Gen_t(X)
\]

其中：

- \(Succ(X)\)：完整 configuration graph 中从 \(X\) 出发的所有合法后继；
- \(Gen_t(X)\)：当前已经从 \(X\) lazy 生成过的后继；
- \(U_t(X)\)：从 \(X\) 出发尚未生成的后继集合。

也就是说，\(U_t(X)\) 表示“还没有探索过的出口”。

定义 frontier 集合：

\[
F_t=\{X\in V_t: U_t(X)\neq \emptyset\}
\]

即所有“已经到达，但后继还没完全枚举”的配置。

对每个 frontier 配置 \(X\)，定义：

\[
\ell_t(X)=
\min_{Y\in U_t(X)}
\left\{
c(X,Y)+h(Y)
\right\}
\]

那么可以构造一个 frontier-based lower bound：

\[
LB_t^{frontier}
=
\min_{X\in F_t}
\left[
g_t(X)+\ell_t(X)
\right]
\]

如果已有 incumbent：

\[
UB_t
\]

那么全局 lower bound 可以写成：

\[
LB_t=
\min\left\{
UB_t,\ 
LB_t^{frontier}
\right\}
\]

其中加入 \(UB_t\) 是为了覆盖“某条完整路径已经完全在当前已知图中”的情况。

---

## 6. 为什么这个 frontier lower bound 是对的？

任取一条真实完整路径：

\[
\pi: S\rightarrow \cdots \rightarrow G
\]

分两种情况。

### 情况 1：\(\pi\) 的所有边都已经在当前已知图 \(H_t\) 中

那么这条路径已经完全可见。当前已知的 incumbent \(UB_t\) 至少不会比这条已知路径更差，因此：

\[
UB_t \leq cost(\pi)
\]

所以：

\[
\min\{UB_t,LB_t^{frontier}\}\leq cost(\pi)
\]

### 情况 2：\(\pi\) 中存在尚未生成的边

取 \(\pi\) 上的第一条尚未生成边：

\[
X\rightarrow Y
\]

由于它是第一条尚未生成边，所以从 \(S\) 到 \(X\) 的前缀路径全部由已生成边组成。于是，在当前已知图 \(H_t\) 中，已经存在这条 \(S\rightarrow X\) 的前缀。

因此：

\[
g_t(X)
\leq
cost(S\rightarrow X \text{ along } \pi)
\]

同时，由于 \(h(Y)\) 是 admissible heuristic：

\[
h(Y)\leq cost(Y\rightarrow G \text{ along optimal suffix})
\]

所以：

\[
cost(\pi)
\geq
g_t(X)+c(X,Y)+h(Y)
\]

又因为：

\[
Y\in U_t(X)
\]

所以：

\[
\ell_t(X)
\leq
c(X,Y)+h(Y)
\]

因此：

\[
cost(\pi)
\geq
g_t(X)+\ell_t(X)
\geq
LB_t^{frontier}
\]

由此可知：

\[
LB_t \leq OPT
\]

所以这个 \(LB_t\) 是一个真正的全局 lower bound。

---

## 7. 用前面的例子再解释一次

真实最优路径是：

\[
S\rightarrow B\rightarrow X\rightarrow G
\]

但当前只生成了：

\[
S\rightarrow A,\quad A\rightarrow G,\quad S\rightarrow X
\]

当前图：

```text
S --4--> A --4--> G      UB = 8
S --10--> X
```

真实最优路径的第一条未生成边是：

\[
S\rightarrow B
\]

注意，它的出发点是 \(S\)，不是 \(X\)。

因此 frontier lower bound 用：

\[
g_t(S)+c(S,B)+h(B)
\]

而不是：

\[
g_t(X)+h(X)
\]

若：

\[
g_t(S)=0,\quad c(S,B)=1,\quad h(B)=2
\]

则：

\[
LB=0+1+2=3
\]

于是此时只能得到：

\[
3\leq OPT\leq 8
\]

不能提前终止。

之后如果生成了 \(S\rightarrow B\)，但还没生成 \(B\rightarrow X\)，当前最优路径的第一条未生成边变成：

\[
B\rightarrow X
\]

此时：

\[
g_t(B)=1
\]

所以：

\[
LB=g_t(B)+c(B,X)+h(X)=1+1+1=3
\]

仍然是安全的。

再之后生成 \(B\rightarrow X\)，rewiring 会更新：

\[
g_t(X):10\rightarrow 2
\]

然后如果生成 \(X\rightarrow G\)，就能找到更优解：

\[
UB=3
\]

---

## 8. 这样是否可以实现 MIPGap-style termination？

理论上可以。

如果维护：

\[
LB_t\leq OPT\leq UB_t
\]

则可以定义：

\[
gap_t=\frac{UB_t-LB_t}{UB_t}
\]

当：

\[
gap_t\leq \epsilon
\]

就可以提前终止，并声明当前解满足 \(\epsilon\)-optimality。

也可以用比值形式：

\[
\frac{UB_t}{LB_t}\leq 1+\epsilon
\]

则当前解满足：

\[
UB_t\leq (1+\epsilon)OPT
\]

这才是类似 MIPGap 的可靠提前终止条件。

---

## 9. 实现上的关键难点

这个 frontier lower bound 的理论逻辑是清楚的，但实现并不简单。

最难的是安全、高效地维护：

\[
U_t(X)=Succ(X)\setminus Gen_t(X)
\]

因为 LaCAM 的 successor 是通过 constraint tree lazy 生成的，并不是显式列出所有后继。

因此要计算：

\[
\ell_t(X)=
\min_{Y\in U_t(X)}
\{c(X,Y)+h(Y)\}
\]

可能本身就是一个 constrained successor optimization 问题。

可以考虑三个层次。

### 层次 1：使用所有 successor 的超集

用：

\[
\underline{\ell}(X)=
\min_{Y\in Succ(X)}
\{c(X,Y)+h(Y)\}
\]

代替：

\[
\ell_t(X)
\]

这样一定安全，因为它可能把已经生成过的 successor 也算进去，只会让 lower bound 更小、更松，不会破坏有效性。

缺点是：这个 bound 可能很弱，而且不会随着 lazy generation 明显收紧。

### 层次 2：利用 constraint tree 剩余 frontier

LaCAM 的每个高层节点内部有一个 low-level constraint tree。每个未处理的 constraint-tree 节点代表一批尚未尝试的 next configurations。

可以对每个剩余 constraint region \(C\) 计算：

\[
\min_{Y\in Succ(X),\,Y satisfies C}
\{c(X,Y)+h(Y)\}
\]

再对所有剩余 region 取最小。

这会比层次 1 更强，但实现复杂度更高。

### 层次 3：一阶 MAPF assignment relaxation

从配置 \(X\) 到后继配置 \(Y\)，每个 agent 只能选择：

\[
Y[i]\in neigh(X[i])\cup\{X[i]\}
\]

同时需要避免 vertex collision 和 edge collision。

若目标是 sum-of-loss 或 sum-of-fuels，且 \(h(Y)\) 是 agent-wise additive，则可以把 one-step lower bound 近似为一个 assignment / min-cost matching relaxation。

若目标是 makespan，则可以考虑 bottleneck assignment 或阈值可行性判断。

这些 relaxation 只要不高估，就可以作为安全 lower bound。

---

## 10. 可进一步增强的 lower bound

frontier lower bound 可以与其他 MAPF lower bound 取最大：

\[
LB_t^{final}
=
\max
\left\{
LB_t^{frontier},
LB^{dist},
LB^{pair},
LB^{cut},
LB^{LP}
\right\}
\]

因为多个有效 lower bound 的最大值仍然是有效 lower bound。

可能的增强包括：

### 10.1 单机器人距离下界

makespan：

\[
LB^{dist}=\max_i dist(s_i,g_i)
\]

sum-of-loss / sum-of-fuels：

\[
LB^{dist}=\sum_i dist(s_i,g_i)
\]

这是最便宜的下界，但通常较弱。

### 10.2 Pairwise conflict lower bound

对每一对 agent \((i,j)\)，求只考虑这两个 agent 时的最优代价，得到相对于单机器人独立路径的额外冲突代价：

\[
\Delta_{ij}
\]

为了避免重复计算同一个 agent 的 penalty，可以在 pair graph 上做 maximum weight matching：

\[
LB^{pair}=LB^{dist}+\max matching(\Delta_{ij})
\]

### 10.3 Corridor / cut capacity lower bound

对窄通道、瓶颈 cut、单向通行区域，可以统计必须通过该 cut 的 agent 数量与单位时间容量。

makespan 下可形成类似：

\[
T\geq
\left\lceil
\frac{\#required\ crossings}{capacity}
\right\rceil
\]

的下界。

这类 bound 对 warehouse、maze、tunnel 等地图尤其有用。

### 10.4 Time-expanded LP / min-cost flow relaxation

构造 time-expanded network，把 MAPF 放松为 fractional flow 或 aggregate flow。

- 对 makespan：判断某个 horizon \(T\) 下 relaxation 是否可行；若不可行，则 \(T\) 是有效下界；
- 对 sum-of-loss / sum-of-fuels：求 relaxation 的最小代价作为下界。

这类下界更强，但计算量也更大。

---

## 11. 最终判断

可以形成一个 **Bounded-gap LaCAM\*** 的扩展框架：

\[
UB_t=g_t(G)
\]

\[
LB_t=
\max
\left\{
LB_t^{frontier},
LB^{dist},
LB^{pair},
LB^{cut},
LB^{LP}
\right\}
\]

然后使用：

\[
\frac{UB_t-LB_t}{UB_t}\leq\epsilon
\]

作为提前终止条件。

核心观点是：

> 不能直接使用 \(\min_{N\in Open} g_t(N)+h(N)\) 作为全局 lower bound，因为 \(g_t(N)\) 可能高估。  
> 但可以围绕“第一条未生成边”的 frontier 构造有效 lower bound，因为在这条边之前，路径前缀完全由已生成边组成，\(g_t(X)\) 对这一类路径不会高估。

这个方向理论上是可行的。真正的研究难点是：

1. 如何高效维护所有 frontier configurations；
2. 如何安全估计未生成 successor 集合；
3. 如何让 lower bound 足够强，否则虽然 gap 证书是真的，但 gap 可能长期很大；
4. 如何在不显著破坏 LaCAM\* scalability 的前提下加入这些 bound。

---

## 12. 可以考虑的论文贡献表述

如果要把这个方向写成论文，可以考虑如下贡献点：

1. **Bounded-gap extension of LaCAM\***  
   提出一种支持 anytime bounded-gap termination 的 LaCAM\* 扩展，而不仅仅是 eventually optimal。

2. **Frontier-based global lower bound for lazy MAPF search**  
   针对 lazy successor generation 中 \(g_t(N)\) 可能高估的问题，提出基于第一条未生成边的全局 lower bound。

3. **Efficient lower-bound maintenance under constraint-tree successor generation**  
   设计适用于 LaCAM constraint tree 的未生成后继下界计算方法。

4. **Hybrid lower bound integration**  
   将 frontier bound 与 single-agent distance、pairwise conflict、corridor/cut capacity、time-expanded relaxation 等下界结合，提高 gap 收敛速度。

---

## 13. 一句话总结

LaCAM\* 原始算法能 eventually optimal，但不能直接给出 MIPGap-style guarantee。  
直接用 \(g_t+h\) 作为 Open 节点下界是不安全的，因为 \(g_t\) 可能高估。  
一个更严谨的方向是基于“第一条未生成边”构造 frontier lower bound，从而得到真正满足：

\[
LB_t\leq OPT\leq UB_t
\]

的 bounded-gap LaCAM\* 框架。

---

## 14. 精确 frontier successor bound 的计算代价问题

前面讨论过一个较紧的 frontier lower bound：

\[
LB_t^{frontier}
=
\min_{X\in F_t}
\left[
g_t(X)+
\min_{Y\in U_t(X)}
\{c(X,Y)+h(Y)\}
\right]
\]

其中：

\[
U_t(X)=Succ(X)\setminus Gen_t(X)
\]

表示从配置 \(X\) 出发尚未 lazy 生成的后继集合。

这个下界理论上更紧，但存在一个重要问题：

> 如果为了计算这个下界，需要遍历所有节点的所有可能 successor，那么就会抵消 LaCAM 的 lazy successor generation 优势。

LaCAM 的核心价值在于避免一次性展开 MAPF configuration graph 中指数规模的后继。如果为了 lower bound 又显式枚举这些后继，就可能把算法重新推回传统大分支搜索的困境。

因此，精确计算：

\[
\min_{Y\in U_t(X)}
\{c(X,Y)+h(Y)\}
\]

更适合作为理论基准或选择性加强手段，而不适合作为每轮都执行的默认操作。

---

## 15. 一个更便宜的全局 lower bound

为了避免枚举所有未生成 successor，可以使用一个更便宜但更松的全局下界。

定义：

\[
\mathcal{F}_t=
\{X\in V_t: X\text{ 的 successor 还没有全部 lazy 生成完}\}
\]

也就是说，\(\mathcal{F}_t\) 包含所有已经发现、但 constraint tree 尚未完全展开的 configuration。

注意：

\[
\mathcal{F}_t \neq Open
\]

因为有些节点可能已经被 LaCAM\* 暂时 pop 出 Open，但其 successor 并没有全部生成完。只要它未来仍可能因为 rewiring 而重新激活，就应当仍然属于 unfinished frontier。

定义 cheap global lower bound：

\[
LB_t^{cheap}
=
\min
\left\{
g_t(G),\
\min_{X\in \mathcal{F}_t}
[g_t(X)+h(X)]
\right\}
\]

如果当前尚未找到 goal，则：

\[
g_t(G)=+\infty
\]

这个下界不需要枚举 successor。它只需要维护每个 unfinished configuration 的：

\[
g_t(X)+h(X)
\]

其中 \(h(X)\) 是从 \(X\) 到目标的 admissible heuristic。

---

## 16. 为什么 cheap lower bound 是安全的？

任取一条真实完整路径：

\[
\pi:S\rightarrow \cdots \rightarrow G
\]

分两种情况。

### 情况 1：\(\pi\) 的所有边都已经生成

如果这条路径的所有 transition 都已经在当前已知图 \(H_t\) 中，那么它已经是一条已知图中的完整路径。

当前已知图中到达 \(G\) 的最短代价为：

\[
g_t(G)
\]

因此：

\[
g_t(G)\leq cost(\pi)
\]

### 情况 2：\(\pi\) 中存在尚未生成的边

取 \(\pi\) 上的第一条尚未生成边：

\[
X\rightarrow Y
\]

因为这是第一条尚未生成边，所以从 \(S\) 到 \(X\) 的前缀全部由已经生成的边构成。因此，在当前已知图 \(H_t\) 中已经存在这条 \(S\rightarrow X\) 前缀。

所以：

\[
g_t(X)
\leq
cost(S\rightarrow X\text{ along }\pi)
\]

又因为 \(h(X)\) 是从 \(X\) 到 \(G\) 的 admissible heuristic：

\[
h(X)
\leq
cost(X\rightarrow G\text{ along }\pi)
\]

于是：

\[
g_t(X)+h(X)
\leq
cost(\pi)
\]

同时，因为 \(X\) 至少还有一条尚未生成边 \(X\rightarrow Y\)，所以：

\[
X\in \mathcal{F}_t
\]

因此：

\[
\min_{X\in \mathcal{F}_t}
[g_t(X)+h(X)]
\leq
cost(\pi)
\]

综合两种情况：

\[
LB_t^{cheap}
=
\min
\left\{
g_t(G),\
\min_{X\in \mathcal{F}_t}
[g_t(X)+h(X)]
\right\}
\leq
OPT
\]

所以 \(LB_t^{cheap}\) 是一个真正的全局 lower bound。

---

## 17. cheap LB 与精确 frontier successor LB 的关系

精确 frontier successor bound：

\[
g_t(X)+
\min_{Y\in U_t(X)}
\{c(X,Y)+h(Y)\}
\]

更紧，因为它利用了这样一个事实：

> 如果路径从 \(X\) 离开当前已知图，那么下一步必须走某个尚未生成的 successor \(Y\)。

但它需要理解和优化 \(U_t(X)\)，实现代价较高。

cheap lower bound：

\[
g_t(X)+h(X)
\]

更松，因为它不关心下一步走向哪个 successor，而是直接估计从 \(X\) 到目标的剩余代价。

二者关系可以理解为：

- \(g_t(X)+h(X)\)：便宜、简单、安全，但较松；
- \(g_t(X)+\min_{Y\in U_t(X)}\{c(X,Y)+h(Y)\}\)：更紧，但可能很贵；
- 后者更适合作为 selective tightening，而不是默认对所有节点计算。

---

## 18. cheap LB 的维护方式

可以额外维护一个 heap：

\[
key(X)=g_t(X)+h(X)
\]

heap 中放入所有：

\[
X\in \mathcal{F}_t
\]

即所有 successor 尚未完全生成的配置。

每次发生以下事件时更新 heap：

1. 新 configuration \(X\) 加入 \(V_t\)：加入 heap；
2. 某个 \(X\) 的 constraint tree 为空，即 successor 全部生成完：从 heap 中移除，或标记为 invalid；
3. Dijkstra-style rewiring 使 \(g_t(X)\) 下降：更新 \(key(X)\)；
4. 找到或改进 goal：更新 \(UB\)。

于是：

\[
LB_t^{cheap}
=
\min
\{g_t(G),\ heap.min()\}
\]

如果已有 incumbent：

\[
UB_t=g_t(G)
\]

则可以定义 gap：

\[
gap_t=
\frac{UB_t-LB_t^{cheap}}{UB_t}
\]

当：

\[
gap_t\leq \epsilon
\]

即可提前终止，并给出 bounded-gap certificate。

---

## 19. 这个 LB 对加速 LaCAM\* 搜索有帮助吗？

需要区分两种意义的“加速”。

### 19.1 对 exact optimality 的直接加速有限

如果目标是继续运行到 exact optimality，那么 cheap lower bound 本身通常不会大幅减少搜索。

原因是 LaCAM\* 原文已经有类似的 temporary pruning：

\[
g_t(N)+h(N)\geq UB
\]

则当前节点可以暂时不扩展。之后如果 rewiring 使其 \(g\)-value 下降，该节点又可以被重新激活。

因此，单独维护：

\[
LB_t^{cheap}
=
\min_{X\in \mathcal{F}_t}
[g_t(X)+h(X)]
\]

通常不会比原本的节点级 pruning 多剪掉大量搜索空间。

它能够省掉的主要是收尾工作：如果发现

\[
LB_t^{cheap}\geq UB
\]

可以直接证明 optimal，而不必逐个处理所有剩余 unfinished nodes。

但这通常不是 LaCAM\* 的主要瓶颈。

### 19.2 对 bounded-gap stopping 很有价值

如果允许提前停止并给出 gap 证书，那么 cheap LB 的价值就很明确。

例如当前已有解：

\[
UB=105
\]

如果 cheap global lower bound 能证明：

\[
LB=100
\]

则：

\[
gap=\frac{105-100}{105}=4.76\%
\]

此时可以直接停止，并声明当前解在 5% optimality gap 以内。

这种能力是原始 LaCAM\* 没有显式提供的。原始 LaCAM\* 被中断时只返回 sub-optimal solution，而不是 bounded-gap solution。

因此，cheap LB 的主要价值不是更快找到解，而是：

> 让 LaCAM\* 能够更早“带证书地停止”。

---

## 20. cheap LB 可能很松

cheap LB 最大的问题是可能长期被早期 frontier 卡住。

例如，如果起点 \(S\) 的 successor 还没有全部生成完，则：

\[
S\in \mathcal{F}_t
\]

于是：

\[
LB_t^{cheap}
\leq
g_t(S)+h(S)=h(S)
\]

如果 \(h(S)\) 只是单机器人最短路距离之和，或者 makespan 下的最大单机器人距离，那么它会忽略机器人之间的冲突、等待、窄通道瓶颈和互相避让。

例如：

\[
UB=150
\]

\[
h(S)=80
\]

则：

\[
gap=\frac{150-80}{150}=46.7\%
\]

即便当前解已经接近最优，cheap LB 也可能无法证明出来。

所以 cheap LB 的优点是：

> 计算便宜，理论安全。

缺点是：

> 下界可能很松，gap 可能长期很大。

这和 MIP 中 LP relaxation 很松的情况类似：证书是真的，但 gap 不一定好看。

---

## 21. 更实际的加速策略：cheap LB + selective tightening

较合理的方向不是对所有 frontier 节点计算强下界，而是：

> cheap LB 负责全局维护；强下界只对当前阻碍 gap 收敛的少数 frontier 计算。

具体做法是维护：

\[
key(X)=g_t(X)+h(X)
\]

找到当前最小 key 的 unfinished frontier：

\[
X^*=
\arg\min_{X\in\mathcal{F}_t}
[g_t(X)+h(X)]
\]

如果当前 gap 主要由 \(X^*\) 决定，则只对 \(X^*\) 做 tightening，而不是遍历所有节点的所有 successor。

例如把：

\[
g_t(X^*)+h(X^*)
\]

加强为：

\[
g_t(X^*)+\underline{R}(X^*)
\]

其中 \(\underline{R}(X^*)\) 是一个更强但仍然 admissible 的剩余代价下界。

可能的 selective tightening 包括：

1. corridor lower bound；
2. pairwise conflict lower bound；
3. one-step assignment relaxation；
4. local time-expanded flow relaxation；
5. constraint-tree remaining-region lower bound；
6. 针对窄通道、环路、交换结构的 pattern-based lower bound。

如果 tightening 后得到：

\[
g_t(X^*)+\underline{R}(X^*)\geq UB
\]

则该 frontier 可以进入 dormant 状态，直到 rewiring 使其 \(g_t(X^*)\) 下降。

这种策略更符合 LaCAM 的 lazy 思路：

> 哪个 frontier 阻碍 gap 收敛，就只处理哪个 frontier。

---

## 22. 三种可能的用法

### 用法 A：bounded-gap certificate

这是最直接、最可靠的用途。

流程如下：

1. LaCAM\* 按原策略快速找到初始解；
2. 维护 \(UB\) 和 cheap/global \(LB\)；
3. 如果用户接受 5% 或 10% gap，则在满足 gap 条件时提前停止；
4. 否则继续搜索，直到 exact optimality。

这不会破坏 LaCAM\* 的快速初解能力，同时增强了 anytime solution 的可解释性和可验证性。

### 用法 B：best-bound search

找到 incumbent 后，可以将搜索策略切换为更接近 A\*/B&B 的 best-bound 方式：

\[
\text{优先扩展 }g_t(X)+h(X)\text{ 最小的 unfinished node}
\]

这样有助于更快提高 lower bound 或证明 optimality。

但风险是：

> 过早使用 best-bound search 可能牺牲 LaCAM 快速找到初始可行解的优势。

因此更合理的是两阶段策略：

- **Phase 1**：使用 LaCAM 原策略快速找初始解；
- **Phase 2**：找到 incumbent 后，切换到 gap-oriented / best-bound search。

### 用法 C：局部加强当前最小 LB frontier

当：

\[
LB_t=\min_{X\in \mathcal{F}_t}
[g_t(X)+h(X)]
\]

长期由某个 \(X\) 决定时，只对这个 \(X\) 做额外分析。

这避免了全局遍历 successor，同时有可能显著提高 lower bound。

---

## 23. 对搜索加速的总体判断

单独的 cheap LB 不是“银弹”。

它对 LaCAM\* 的帮助主要是：

1. 提供 bounded-gap stopping；
2. 在 exact optimality 收尾阶段减少部分无意义处理；
3. 为 gap-oriented search 提供评价指标；
4. 指导 selective tightening 的位置选择。

但它本身通常不会显著加速初始解发现，也不一定会显著减少主搜索空间。

一个更有研究价值的框架是：

\[
\text{cheap global LB}
+
\text{selective tightening}
+
\text{best-bound expansion after incumbent}
\]

也就是：

> Bounded-gap LaCAM\*：保持 LaCAM\* 快速找初解的能力，同时通过 frontier lower bound 给出可验证 gap，并只对阻碍 gap 收敛的少数 frontier 做懒式下界加强。

---

## 24. 修正后的研究判断

原始想法中“精确 frontier successor lower bound”理论上成立，但计算代价可能过高，不应作为主路径。

更实际的研究路线是：

1. 先构建 cheap global LB：

\[
LB_t^{cheap}
=
\min
\left\{
g_t(G),\
\min_{X\in \mathcal{F}_t}
[g_t(X)+h(X)]
\right\}
\]

2. 用它实现 bounded-gap termination：

\[
\frac{UB_t-LB_t^{cheap}}{UB_t}\leq\epsilon
\]

3. 当 gap 长期无法收敛时，只对决定当前 LB 的少数 frontier 节点做 selective tightening；

4. 将 selective tightening 与 corridor、pairwise、flow relaxation 等 MAPF 结构下界结合；

5. 在找到 incumbent 后切换到 gap-oriented search，而不是从一开始就牺牲 LaCAM 的快速初解能力。

这样既保留了 LaCAM 的 lazy 优势，又让 LaCAM\* 从 eventually optimal anytime solver 向 anytime bounded-gap solver 发展。

---

## 25. 可以更新的一句话总结

精确 frontier successor bound 虽然理论上更紧，但如果需要遍历所有未生成 successor，就会破坏 LaCAM 的 lazy 优势。  
更实际的做法是维护一个便宜的全局 lower bound：

\[
LB_t^{cheap}
=
\min
\left\{
g_t(G),\
\min_{X\in \mathcal{F}_t}
[g_t(X)+h(X)]
\right\}
\]

它不显著增加计算负担，能够提供 bounded-gap certificate，但通常较松。  
真正有潜力的方向是：用 cheap LB 做全局证书和搜索监控，只对当前阻碍 gap 收敛的少数 frontier 做 selective tightening。

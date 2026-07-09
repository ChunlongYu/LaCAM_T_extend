#!/bin/bash
# ============================================================
# 大规模实验:多张地图 × 多算例,固定 agent 数。
# 每个算例同时导出:
#   1) inherit 日志 —— 逐拓展的 (节点特征 h/g/f, 继承数, 奖励, outcome)
#   2) conv    日志 —— 上下界(UB/LB)随时间的收敛记录
# 供 analyze_inherit.py / plot_convergence.py 分析。
#
# 用法(仓库根目录,先编译好 build/main):
#   bash run_experiments_full.sh
# 可用环境变量覆盖默认:
#   MAPS  SCENS  AGENTS  OBJECTIVE  TIME_LIMIT  PAIR_LB_MS  OUTDIR  BIN
# 例:
#   TIME_LIMIT=30 SCENS=10 bash run_experiments_full.sh
# ============================================================
set -u

# ---- 默认配置(可用环境变量覆盖) ----
MAPS_DEFAULT=("empty-32-32" "random-32-32-20" "room-64-64-8")
if [ -n "${MAPS:-}" ]; then read -r -a MAPS_ARR <<< "$MAPS"; else MAPS_ARR=("${MAPS_DEFAULT[@]}"); fi
SCENS="${SCENS:-20}"           # 每张图算例数 (场景 1..SCENS)
AGENTS="${AGENTS:-50}"         # agent 数
OBJECTIVE="${OBJECTIVE:-1}"    # 1=makespan, 2=sum_of_loss (决定 h/g/f 与界的单位)
TIME_LIMIT="${TIME_LIMIT:-60}" # 每次运行秒数上限
PAIR_LB_MS="${PAIR_LB_MS:-0}"  # >0 则开 pairwise 根下界(仅 makespan);0=关
OUTDIR="${OUTDIR:-experiment/full}"
BIN="${BIN:-build/main}"
# -------------------------------------

if [ ! -x "$BIN" ]; then
  echo "找不到可执行文件: $BIN"
  echo "请先编译:  cmake -B build && make -C build"
  exit 1
fi
mkdir -p "$OUTDIR"

total=$(( ${#MAPS_ARR[@]} * SCENS ))
est_min=$(( total * TIME_LIMIT / 60 ))
echo "maps=[${MAPS_ARR[*]}]  scens=1..$SCENS  agents=$AGENTS  objective=$OBJECTIVE"
echo "每算例上限 ${TIME_LIMIT}s  共 ${total} 个算例  最坏耗时约 ${est_min} 分钟"
echo "pair_lb_ms=$PAIR_LB_MS  输出目录: $OUTDIR"
echo

done_cnt=0
ok_cnt=0
SUMMARY="${OUTDIR}/summary.csv"
echo "map,scen,agents,solved,optimal,makespan,soc,comp_time_ms,final_gap,inherit_rows" > "$SUMMARY"

for MAP in "${MAPS_ARR[@]}"; do
  MAPFILE="assets/${MAP}.map"
  if [ ! -f "$MAPFILE" ]; then echo "跳过(缺地图): $MAPFILE"; continue; fi
  for i in $(seq 1 "$SCENS"); do
    SCEN="assets/${MAP}/${MAP}-random-${i}.scen"
    INH="${OUTDIR}/inherit_${MAP}_N${AGENTS}_s${i}.csv"
    CONV="${OUTDIR}/conv_${MAP}_N${AGENTS}_s${i}.csv"
    TXT="${OUTDIR}/result_${MAP}_N${AGENTS}_s${i}.txt"
    done_cnt=$((done_cnt + 1))
    if [ ! -f "$SCEN" ]; then
      echo "[$done_cnt/$total] 跳过(缺场景): $SCEN"
      continue
    fi
    printf "[%d/%d] %-16s scen=%-2d N=%d ... " "$done_cnt" "$total" "$MAP" "$i" "$AGENTS"

    "$BIN" -m "$MAPFILE" -i "$SCEN" -N "$AGENTS" -O "$OBJECTIVE" \
           -t "$TIME_LIMIT" --pair_lb_ms "$PAIR_LB_MS" \
           --inherit_log "$INH" --conv_log "$CONV" -o "$TXT" >/dev/null 2>&1

    # 从结果 txt 提取指标
    get() { grep -E "^$1=" "$TXT" 2>/dev/null | head -1 | cut -d= -f2; }
    solved=$(get solved); optimal=$(get optimal); mk=$(get makespan)
    soc=$(get soc); ct=$(get comp_time); gap=$(get final_gap)
    rows=0; [ -f "$INH" ] && rows=$(( $(wc -l < "$INH") - 1 ))
    echo "${MAP},${i},${AGENTS},${solved},${optimal},${mk},${soc},${ct},${gap},${rows}" >> "$SUMMARY"

    if [ "$rows" -gt 0 ]; then
      echo "ok (makespan=${mk}, inherit ${rows} 行, gap=${gap})"
      ok_cnt=$((ok_cnt + 1))
    else
      echo "无 inherit 数据(未进入优化阶段/无解?) makespan=${mk}"
    fi
  done
done

echo
echo "完成: ${ok_cnt}/${total} 个算例产出了 inherit 数据。汇总表: $SUMMARY"
echo
echo "分析命令:"
echo "  # inherit 相关性(全部合并 / 按图)"
echo "  python analyze_inherit.py ${OUTDIR}/inherit_*.csv -o ${OUTDIR}/analysis_all.png"
for MAP in "${MAPS_ARR[@]}"; do
  echo "  python analyze_inherit.py ${OUTDIR}/inherit_${MAP}_*.csv -o ${OUTDIR}/analysis_${MAP}.png"
done
echo "  # 上下界收敛(单算例示例)"
echo "  python plot_convergence.py ${OUTDIR}/conv_${MAPS_ARR[0]}_N${AGENTS}_s1.csv"

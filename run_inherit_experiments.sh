#!/bin/bash
# ============================================================
# 批量跑 inherit 实验:每张地图 N 个算例,固定 agent 数,
# 每次运行导出一份 per-run 的 (继承数, 奖励) CSV,供 analyze_inherit.py 汇总分析。
#
# 用法(在仓库根目录):
#   bash run_inherit_experiments.sh
# 可用环境变量覆盖默认(便于调参 / 测试):
#   MAPS  SCENS  AGENTS  OBJECTIVE  TIME_LIMIT  OUTDIR  BIN
# 例:
#   SCENS=5 TIME_LIMIT=20 bash run_inherit_experiments.sh
# ============================================================
set -u

# ---- 默认配置(可用环境变量覆盖) ----
MAPS_DEFAULT=("random-32-32-20" "room-64-64-8")
if [ -n "${MAPS:-}" ]; then read -r -a MAPS_ARR <<< "$MAPS"; else MAPS_ARR=("${MAPS_DEFAULT[@]}"); fi
SCENS="${SCENS:-10}"          # 每张图算例数 (场景 1..SCENS)
AGENTS="${AGENTS:-50}"        # agent 数
OBJECTIVE="${OBJECTIVE:-1}"   # 1=makespan, 2=sum_of_loss (决定 h/g/f 的单位)
TIME_LIMIT="${TIME_LIMIT:-10}" # 每次运行秒数(需足够进入优化阶段才有奖励)
OUTDIR="${OUTDIR:-experiment/inherit}"
BIN="${BIN:-build/main}"
# -------------------------------------

if [ ! -x "$BIN" ]; then
  echo "找不到可执行文件: $BIN"
  echo "请先编译:  cmake -B build && make -C build"
  exit 1
fi
mkdir -p "$OUTDIR"

echo "maps=[${MAPS_ARR[*]}]  scens=1..$SCENS  agents=$AGENTS  objective=$OBJECTIVE  t=${TIME_LIMIT}s"
echo "输出目录: $OUTDIR"
echo

total=$(( ${#MAPS_ARR[@]} * SCENS ))
done_cnt=0
ok_cnt=0
for MAP in "${MAPS_ARR[@]}"; do
  MAPFILE="assets/${MAP}.map"
  if [ ! -f "$MAPFILE" ]; then echo "跳过(缺地图): $MAPFILE"; continue; fi
  for i in $(seq 1 "$SCENS"); do
    SCEN="assets/${MAP}/${MAP}-random-${i}.scen"
    CSV="${OUTDIR}/inherit_${MAP}_N${AGENTS}_s${i}.csv"
    TXT="${OUTDIR}/result_${MAP}_N${AGENTS}_s${i}.txt"
    done_cnt=$((done_cnt + 1))
    if [ ! -f "$SCEN" ]; then
      echo "[$done_cnt/$total] 跳过(缺场景): $SCEN"
      continue
    fi
    printf "[%d/%d] %-16s scen=%-2d N=%d ... " "$done_cnt" "$total" "$MAP" "$i" "$AGENTS"
    "$BIN" -m "$MAPFILE" -i "$SCEN" -N "$AGENTS" -O "$OBJECTIVE" \
           -t "$TIME_LIMIT" --inherit_log "$CSV" -o "$TXT" >/dev/null 2>&1
    if [ -f "$CSV" ] && [ "$(wc -l < "$CSV")" -gt 1 ]; then
      rows=$(( $(wc -l < "$CSV") - 1 ))
      mk=$(grep -E "^makespan=" "$TXT" 2>/dev/null | cut -d= -f2)
      echo "ok (${rows} 行, makespan=${mk})"
      ok_cnt=$((ok_cnt + 1))
    else
      echo "无数据(未进入优化阶段/无解?)"
    fi
  done
done

echo
echo "完成: ${ok_cnt}/${total} 个算例产出了数据。"
echo
echo "分析(需要 analyze_inherit.py):"
echo "  # 全部合并"
echo "  python analyze_inherit.py ${OUTDIR}/inherit_*.csv -o ${OUTDIR}/analysis_all.png"
echo "  # 按地图分开"
for MAP in "${MAPS_ARR[@]}"; do
  echo "  python analyze_inherit.py ${OUTDIR}/inherit_${MAP}_*.csv -o ${OUTDIR}/analysis_${MAP}.png"
done

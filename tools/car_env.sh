# 실차(젯슨 핫스팟 HY_MIRU) 접속용 랩탑 ROS 환경 — 터미널마다 `source`로 불러 쓴다:
#   source ~/2026_IFAC/tools/car_env.sh
# (lut_lap_runner.sh는 자체 설정하므로 불필요 — RViz·f1rec·수동 ros2 명령 터미널용)
export ROS_DOMAIN_ID=70
# 젯슨 핫스팟은 AP→클라이언트 멀티캐스트가 불안정 → 유니캐스트 디스커버리 고정
export ROS_STATIC_PEERS="10.1.1.1;10.1.1.10"  # HY_MIRU(핫스팟 AP)=1, MIRU_5G에서의 젯슨=10 — 둘 다 등록
echo "car_env: ROS_DOMAIN_ID=$ROS_DOMAIN_ID, ROS_STATIC_PEERS=$ROS_STATIC_PEERS"

# 실차(젯슨) 접속용 랩탑 ROS 환경 — ROS를 쓰는 터미널마다 `source`로 불러 쓴다:
#   source ~/2026_IFAC/tools/car_env.sh
# (lut_lap_runner.sh는 자체 설정하므로 불필요 — RViz·f1rec·수동 ros2 명령 터미널용)
export ROS_DOMAIN_ID=70
# 젯슨 핫스팟은 AP→클라이언트 멀티캐스트가 불안정 → 유니캐스트 디스커버리 고정
export ROS_STATIC_PEERS="10.1.1.1;10.1.1.10"  # HY_MIRU(핫스팟 AP)=1, MIRU_5G에서의 젯슨=10 — 둘 다 등록
# ⚠️ 핵심(2026-08-13): STATIC_PEERS만으로는 원격 참가자 인덱스 0~3까지만 유니캐스트
# 디스커버리가 닿는다(FastDDS maxInitialPeersRange 기본 4). 젯슨 풀스택은 프로세스가
# 8개+ 라 늦게 뜬 노드는 미발견 위험 → 아래 프로필이 0~31까지 커버.
# 경로는 이 스크립트 자신의 위치에서 유도한다 — $HOME/2026_IFAC 하드코딩이면 다른
# 경로에 clone한 머신에서 FastDDS가 "파일 없음" 한 줄만 남기고 기본값(0~3)으로
# 조용히 폴백해, 픽스가 적용된 척하는 최악의 상태가 된다(검증 워크플로 실측).
if [ -n "${ZSH_VERSION:-}" ]; then
  _car_env_self=${(%):-%x}
else
  _car_env_self=${BASH_SOURCE[0]:-$0}
fi
_car_env_dir=$(cd "$(dirname "$_car_env_self")" 2>/dev/null && pwd)
_car_env_profile="$_car_env_dir/fastdds_car_client.xml"
if [ -f "$_car_env_profile" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$_car_env_profile"
  echo "car_env: DOMAIN=$ROS_DOMAIN_ID PEERS=$ROS_STATIC_PEERS PROFILE=$_car_env_profile"
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  echo "car_env: DOMAIN=$ROS_DOMAIN_ID PEERS=$ROS_STATIC_PEERS"
  echo "car_env: 🔴 fastdds_car_client.xml 을 찾지 못함($_car_env_profile) —"
  echo "car_env:    유니캐스트 디스커버리가 젯슨 참가자 0~3까지만 닿는다(복불복 재발 위험)."
fi
unset _car_env_self _car_env_dir _car_env_profile
echo "car_env: ⚠️ ros2 daemon stop 을 한 번 해줘야 데몬도 이 환경으로 재기동됨"

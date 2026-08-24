# usbc_power.sh — 젯슨 USB-C 포트 소프트웨어 전원 제어

`tools/usbc_power.sh`. 젯슨 Orin Nano Dev Kit **Super** 의 USB-C 포트를 소프트웨어로
껐다 켜고, 먹통이 된 C타입 허브를 재부팅 없이 되살린다.

---

## 1. 왜 필요한가

차량의 C타입 허브에는 **라이다 이더넷(RTL8152 → 192.168.0.x)** 과 **조이스틱(F710)** 이
물려 있다. 이 허브가 종종 죽는데, 죽는 방식이 특이하다.

2026-08-24 실측으로 잡은 먹통 상태:

```
fusb301: ATTACH=1  VBUSOK=1  ftype=SINK  롤=host     ← 전기적으로는 완전 정상
xHCI   : 열거된 장치 0개                              ← 그런데 USB는 하나도 안 올라옴
dmesg  : disconnect 없음, error 없음, over_current_count=0
```

전원 부족도, 케이블도, 과전류도 아니다. 부팅 시 fusb301이 `DRP+ACC`로 토글하다 이미
꽂혀 있는 허브를 `detach`로 흘려버리고, 92초 뒤 뒤늦게 `SRC`로 붙어도 padctl 롤 전환
핸드오프가 어긋나 데이터 라인이 안 살아나는 레이스다. dmesg에 아래 경고가 같이 찍힌다.

```
/bus@0/padctl@3520000/ports/usb2-0: Fixed dependency cycle(s) with
        /bus@0/i2c@c240000/fusb301@25/connector@0
```

---

## 2. 하드웨어 구조

| 항목 | 값 |
|---|---|
| 보드 | Jetson Orin Nano Engineering Reference Developer Kit **Super** |
| 커널 | 6.8.12-1021-tegra (JetPack R39.2) |
| USB-C 포트 | xHCI 루트포트 `usb1-port1` = pad `usb2-0` |
| Type-C 컨트롤러 | **fusb301** @ I2C `1-0025` |
| 제어 sysfs | `/sys/bus/i2c/devices/1-0025/fusb301/` |
| 루트포트 sysfs | `/sys/bus/usb/devices/usb1/1-0:1.0/usb1-port1/` |

`lsusb`에 보이는 Realtek 4-Port 허브(`0bda:5489` / `0bda:0489`)는 **온보드 허브**로
Type-A 4포트를 담당한다. C타입 허브가 아니므로 헷갈리지 말 것.

---

## 3. 제어 수단이 두 개이고 효과가 다르다

실측으로 확인한 내용이다. 하나로는 안 되고 **둘 다 필요하다.**

| 수단 | 지속 차단 | 먹통 복구 | 스크립트 명령 |
|---|:---:|:---:|---|
| 루트포트 `usb1-port1/disable` = 1 / 0 | **O** | X | `off` / `on` |
| fusb301 `fmode` SNK(4) → SRC(1) | X | **O** | `cycle` |
| fusb301 `freset` (칩 리셋) | X | X | (안 씀) |
| `uhubctl` | — | — | PPPS 미지원, 해당 없음 |

- **`disable`** 은 장치를 전부 떼어 내고 그 상태를 유지한다(VBUS는 계속 공급됨).
  0으로 되돌리면 2초 안에 복귀한다. 그래서 **off/on 스위치**로 쓴다.
  단 **이미 먹통이 된 포트에는 아무 반응이 없다** — dmesg에 한 줄도 안 남는다.
- **`fmode`** 는 쓰는 순간 detach/attach가 한 번 일어나고 드라이버가 곧바로 SRC로
  되돌아온다(MODES 레지스터가 항상 0x01로 복귀). 그래서 "계속 꺼두기"에는 못 쓴다.
  대신 **이것만이 먹통을 실제로 깬다.** 그래서 **복구(cycle)** 로 쓴다.
  `fmode` 값은 FUSB301 MODES 레지스터 비트다 — `1`=SRC, `4`=SNK, `16`=DRP, `32`=DRP+ACC.

---

## 4. 사용법

젯슨에서 직접 실행한다. `/usr/local/bin/usbc_power.sh` 에 설치돼 있다.

```bash
usbc_power.sh status    # 현재 상태 + 허브에 물린 장치 목록
usbc_power.sh off       # 포트 차단 (on 할 때까지 유지)
usbc_power.sh on        # 차단 해제 + 복귀 대기. 안 오면 자동으로 cycle까지 간다
usbc_power.sh cycle     # 강제 재부착 — 허브가 먹통일 때 쓰는 복구 명령
```

옵션:

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--force` | 꺼짐 | 주행 스택이 떠 있어도 강행 |
| `--off-seconds N` | 3 | `cycle`에서 VBUS를 내려 두는 시간 |
| `--wait-seconds N` | 15 | 복귀 대기 한도 |

환경변수로도 덮어쓸 수 있다: `FUSB301_DIR`, `USBC_PORT`, `USBC_ROLE_FILE`,
`USBC_DEV_PREFIXES`, `USBC_STACK_PATTERN`, `USBC_STATE_FILE`.

### 실행 예

```
$ usbc_power.sh status
USB-C 포트 상태
  fusb301   mode=SRC(1) type=SINK(16) CC=1 STATUS=0x19 (ATTACH=1 VBUSOK=1)
  롤        host
  루트포트  disable=0 state=configured over_current_count=0
  열거 장치 3개
    1-1      214b:7250 USB2.0 HUB
    1-1.1    0bda:8152 USB 10/100 LAN
    1-1.3    046d:c21f Wireless Gamepad F710

$ usbc_power.sh cycle
사이클 시작 — 현재 3개, 목표 3개
▶ 강제 재부착 (fmode SNK→SRC)
  +02s 장치 3/3개
✅ 복귀 완료 (3개)
```

`status`는 상태를 읽고 스스로 판정해 준다.

- `disable=1` → `off 상태다. on 으로 켤 것.`
- `ATTACH=1 VBUSOK=1` 인데 장치 0개 → `먹통 상태다. cycle 로 복구할 것.`
- `ATTACH=0` → 케이블이 안 꽂힌 것이므로 `on`/`cycle`이 소프트웨어로 할 게 없다고
  분명히 말하고 멈춘다.

---

## 5. 안전장치

**허브를 끊으면 `/scan`과 `/joy`가 통째로 죽는다.** 그래서 기본적으로 거부한다.

`urg_node`, `vesc_driver`, `joy_node`, `ackermann_to_vesc`, `controller_node`,
`local_planner`, `state_machine`, `global_planner` 중 하나라도 돌고 있으면 `off`와
`cycle`은 실행을 거부하고 무엇이 떠 있는지 보여 준다. 정말 필요하면 `--force`.

> ⚠️ **주행 중에는 절대 쓰지 말 것.** 사이클에 5~7초가 걸리고 그동안 라이다와
> 조이스틱이 없다. 차를 세우고 스택을 내린 뒤에 실행한다.

`on`은 마지막으로 정상이던 장치 개수를 `/tmp/usbc_power.baseline`에 기억해 두고
그만큼 돌아올 때까지 기다린다. 부팅하면 초기화되며, 그때는 1개만 확인하고 넘어간다.

---

## 6. 복구가 안 될 때

1. `usbc_power.sh cycle` — 내부적으로 1회 자동 재시도한다.
2. `usbc_power.sh status`로 `ATTACH` 확인. **0이면 케이블이 빠진 것**이라
   소프트웨어로 할 게 없다. 물리적으로 꽂는다.
3. 그래도 안 되면 케이블을 뽑았다 꽂거나 재부팅한다.

### 복귀 후 라이다 이더넷 확인

USB 장치가 돌아와도 `enx…` 인터페이스가 IP를 받기까지는 몇 초 더 걸린다.

```bash
ip -br addr show | grep enx      # 192.168.0.x 가 붙었는지
cat /sys/class/net/enx*/carrier  # 1이어야 링크가 살아 있는 것
```

`carrier=0`이면 USB는 정상이고 **라이다 쪽 전원/랜선 문제**다. USB-C와 무관하므로
`cycle`을 반복해도 소용없다.

> 📌 NetworkManager에 Hokuyo 연결 프로필이 `hokuyo`, `Hokuyo_hub`(2개),
> `Hokuyo-LiDAR`, `Hokuyo_ㅗㅕㅠ` 로 5개나 중복 등록돼 있다. 재연결 때 엉뚱한
> 프로필이 잡혀 IP가 달라질 수 있으니 하나만 남기고 정리하는 게 좋다.

---

## 7. 확인된 시험 결과 (2026-08-24 ~ 25)

| 시험 | 결과 |
|---|---|
| 먹통 상태에서 `cycle` | 약 5초 만에 전 장치 복귀, 2회 연속 재현 |
| `off` → 장치 0개 유지 | 15초 이상 유지 확인 (net 인터페이스·`/dev/input/js0` 소멸) |
| `on` → 베이스라인 복귀 | 1~2초 만에 3/3 복귀, IP `192.168.0.15` 재획득 확인 |
| 루트포트 `disable` 로 먹통 복구 | **실패** — dmesg 무반응 |
| `freset` 로 먹통 복구 | **실패** — dmesg 무반응 |
| 온보드 Type-A 허브·블루투스 영향 | **없음** — 사이클 내내 유지 |
| 주행 스택 가드 | 가짜 `joy_node`로 거부(exit 1) 및 `--force` 강행 확인 |

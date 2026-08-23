#pragma once

#include <algorithm>
#include <cmath>

namespace f1tenth_control {

// 명령 자체를 목표 쪽으로만 움직이는 비대칭 rate limiter.
// 실측 속도는 아래 ramp_lead_max 안티와인드업에서 별도로 사용한다. 분기 방향을
// target-current_speed로 정하면 실측이 목표보다 높은 동안 목표가 상승할 때
// last_cmd를 건너뛰고 target으로 clamp되는 역방향 계단이 생긴다(0821 23시 bag).
inline double rate_limit_speed_command(double last_cmd, double target, double dt,
                                       double max_accel, double max_decel) {
    const double safe_dt = std::max(0.0, dt);
    const double accel_step = std::max(0.0, max_accel) * safe_dt;
    const double decel_step = std::max(0.0, max_decel) * safe_dt;
    if (target > last_cmd) return std::min(target, last_cmd + accel_step);
    if (target < last_cmd) return std::max(target, last_cmd - decel_step);
    return target;
}

enum class HfiLaunchEvent {
    kNone,
    kStarted,
    kReleased,
    kMovingBypass,
    kRetryScheduled,
    kRetryStarted,
    kTimedOut,
    kFailureReset,
    kLatchAutoReset,
};

enum class HfiLaunchFailureReason {
    kNone,
    kTimeout,
    kNoForwardProgress,
    kSustainedReverse,
};

struct HfiLaunchGuardConfig {
    bool enabled = false;
    // 4420 ERPM/(m/s) 기준 0.7 m/s는 3094 ERPM으로 VESC의 HFI→sensorless 전환
    // 문턱(3000 ERPM)에 3%밖에 여유가 없었다. 060605/060751 실차에서 이 상한에
    // 머문 시도가 반복적으로 제자리 덜걱임을 보였으므로 0.9(3978 ERPM)로 둔다.
    double speed_cap = 0.9;
    double exit_speed = 0.5;
    // 정지 진입/해제 Schmitt trigger. 0822 VESC /odom의 0.13~0.15 m/s 정지 노이즈가
    // 0.5초 재무장 dwell을 계속 초기화하지 않되, 실제 0.20 m/s 이상 움직임은 즉시 깬다.
    double standstill_speed = 0.12;
    double standstill_exit_speed = 0.20;
    double standstill_filter_tau = 0.10;
    double timeout = 4.0;
    double exit_hold = 0.1;
    double relatch_time = 0.5;
    // 이미 전진 중인 수동→자율/일시 정지명령 복귀는 정지출발이 아니다. 이 속도를
    // 연속 유지하면 relatch_pending을 우회한다. 0 이하면 우회 비활성.
    double moving_bypass_speed = 0.5;
    double moving_bypass_hold = 0.1;
    double retry_cooldown = 0.5;
    unsigned int max_attempts = 2;
    // 060751에서 정상 포착도 1.58~1.62초가 걸렸고 1.2초 watchdog이 아직 살아날
    // 시도를 강제로 0으로 잘랐다. 2초까지 5 cm도 전진하지 못한 시도만 조기 중단한다.
    double no_progress_timeout = 2.0;
    double no_progress_min_distance = 0.05;
    double reverse_abort_speed = 0.10;
    double reverse_abort_hold = 0.15;

    // ── HFI 포착 실패 대응 (2026-08-23 신설) ──────────────────────────────
    // 🔑 0822 bag 자율 시도 65개 실측: "끝내 못 뚫는 시도"와 "느리지만 결국 뚫는
    //    시도"는 **첫 1.5초 신호로 구분되지 않는다** (최장 전진런 p50 0.02 vs
    //    0.04 s / 듀티 0.03 vs 0.06 / vmax 0.24 vs 0.32 — 범위가 거의 겹친다).
    //    즉 "이건 못 뚫는다"를 조기 판정할 근거가 데이터에 없다. 그런데 현재
    //    로직은 그 판정을 전제로 토크를 **0으로 뺀다**. 유일하게 근거 있는 대응은
    //    반대다 — 안 나가면 상한까지 **더 밀어보고**, 그래도 안 되면 그때 쉬는 것.
    // ⚠️ 이건 "판단"이 아니라 액추에이터 포화 대응이다(②-i와 같은 계열). 플래너가
    //    요구한 속도를 넘기지 않으며(cap은 언제나 상한이지 하한이 아니다), 상한
    //    escalate_cap_max가 안전 천장으로 남는다.
    bool escalate_enable = false;
    double escalate_start_delay = 0.6;      // 정체가 이만큼 이어지면 cap 상승 개시 [s]
    double escalate_rate = 0.5;             // cap 상승률 [m/s per s]
    double escalate_cap_max = 1.6;          // cap 절대 상한 [m/s] — 안전 천장
    double escalate_progress_speed = 0.30;  // 이 속도 이상 전진하면 정체 타이머 리셋
    // 최종 실패 래치를 "정지 유지" 이 시간[s] 뒤 스스로 해제한다. 0이면 구 거동.
    // 🔴 구 거동은 탈출 조건이 `target_speed <= 0.1 || disengaged`뿐이라, 플래너가
    //    계속 속도를 요구하면 영원히 안 풀린다. 0822 `run_20260822_060751` 실측:
    //    최종 래치 2회 모두 `drive_mode == autonomous` 중에 걸렸고 **사람이 manual로
    //    내릴 때까지** 각각 3.1 s / 30.0 s 정지했다.
    double latch_release_time = 0.0;
    // dwell 판정용 누출 적분 시상수 [s]. 굴러가면 v*tau 만큼 쌓여 문턱을 넘는다.
    double settle_tau = 1.5;
};

struct HfiLaunchGuardState {
    bool armed = true;
    bool active = false;
    bool retry_waiting = false;
    bool relatch_pending = false;
    bool failure_latched = false;
    bool standstill_filter_initialized = false;
    bool standstill_latched = true;
    double elapsed = 0.0;
    double exit_hold_elapsed = 0.0;
    double relatch_elapsed = 0.0;
    double moving_bypass_elapsed = 0.0;
    double retry_cooldown_elapsed = 0.0;
    double standstill_filtered_speed = 0.0;
    double launch_signed_distance = 0.0;
    double reverse_elapsed = 0.0;
    // 현재 시도에 적용 중인 상한. escalate_enable=false면 항상 config.speed_cap이다.
    double cap_now = 0.0;
    double stall_elapsed = 0.0;
    double latch_elapsed = 0.0;
    // 최근 변위의 누출 적분(τ=settle_tau). '제자리 로킹'과 '실제로 굴러감'을 가른다.
    // 🔴 2026-08-23: dwell 3곳이 전부 standstill(속도 문턱 0.20)에만 걸려 있었는데,
    //    실측 로킹 진폭이 0.24라 문턱을 넘나들며 dwell을 계속 0으로 리셋해
    //    재시도/재무장/래치해제가 **영원히 안 되는** 구간이 있었다(fuzz로 확인).
    double settle_distance = 0.0;
    double escalate_peak = 0.0;
    HfiLaunchFailureReason last_failure_reason = HfiLaunchFailureReason::kNone;
    unsigned int attempt = 0;
    unsigned long release_count = 0;
    unsigned long moving_bypass_count = 0;
    unsigned long retry_count = 0;
    unsigned long attempt_timeout_count = 0;
    unsigned long failure_count = 0;
    unsigned long no_progress_abort_count = 0;
    unsigned long reverse_abort_count = 0;
    unsigned long latch_auto_reset_count = 0;
};

struct HfiLaunchDecision {
    bool constrain_to_cap = false;
    bool force_stop = false;
    // constrain_to_cap일 때 실제로 적용할 상한. 호출부는 config.speed_cap이 아니라
    // **이 값**을 써야 한다(에스컬레이션이 반영된 값).
    double cap = 0.0;
    HfiLaunchEvent event = HfiLaunchEvent::kNone;
};

inline HfiLaunchDecision update_hfi_launch_guard(
    HfiLaunchGuardState& state, const HfiLaunchGuardConfig& config,
    bool disengaged, double target_speed, double current_speed, double dt,
    bool launch_allowed = true) {
    HfiLaunchDecision decision;
    decision.cap = config.speed_cap;
    if (!config.enabled) {
        state.active = false;
        state.retry_waiting = false;
        state.relatch_pending = false;
        state.failure_latched = false;
        state.elapsed = 0.0;
        state.exit_hold_elapsed = 0.0;
        state.relatch_elapsed = 0.0;
        state.moving_bypass_elapsed = 0.0;
        state.retry_cooldown_elapsed = 0.0;
        state.standstill_filter_initialized = false;
        state.standstill_latched = true;
        state.standstill_filtered_speed = 0.0;
        state.launch_signed_distance = 0.0;
        state.reverse_elapsed = 0.0;
        state.last_failure_reason = HfiLaunchFailureReason::kNone;
        state.attempt = 0;
        state.cap_now = 0.0;
        state.stall_elapsed = 0.0;
        state.latch_elapsed = 0.0;
        state.armed = true;
        return decision;
    }

    const double safe_dt = std::max(0.0, dt);
    const double abs_speed = std::abs(current_speed);
    const double standstill_enter = std::max(0.0, config.standstill_speed);
    const double standstill_exit = std::max(
        standstill_enter, config.standstill_exit_speed);
    const double standstill_filter_tau = std::max(0.0, config.standstill_filter_tau);
    if (!state.standstill_filter_initialized) {
        state.standstill_filtered_speed = abs_speed;
        state.standstill_latched = abs_speed < standstill_enter;
        state.standstill_filter_initialized = true;
    } else if (safe_dt > 0.0) {
        const double alpha = standstill_filter_tau > 0.0
            ? 1.0 - std::exp(-safe_dt / standstill_filter_tau)
            : 1.0;
        state.standstill_filtered_speed +=
            alpha * (abs_speed - state.standstill_filtered_speed);
        if (state.standstill_latched) {
            // 실제 exit 이상 움직임은 필터 지연 없이 즉시 정지 판정을 해제한다.
            if (abs_speed >= standstill_exit ||
                state.standstill_filtered_speed >= standstill_exit) {
                state.standstill_latched = false;
            }
        } else if (abs_speed < standstill_enter ||
                   state.standstill_filtered_speed < standstill_enter) {
            // 0속도 명령 직후 실제 raw 속도가 enter 아래로 들어오면 즉시 dwell을 시작한다.
            // 이후 0.13~0.15 잡음은 exit(0.20) 미만이므로 dwell을 깨지 않는다.
            state.standstill_latched = true;
        }
    }
    const bool standstill = state.standstill_latched;
    // 🔑 dwell 판정은 "정지"가 아니라 **"전진하지 않음"**이어야 한다.
    //    ±0.24 m/s로 제자리 로킹하는 차는 정지가 아니지만 나아가지도 않는다 —
    //    그 구간에서 재시도 dwell이 리셋되면 상태기가 빠져나오지 못한다.
    //    누출 적분이라 굴러가면 금방 문턱을 넘고, 로킹이면 0 근처에 머문다.
    {
        const double tau = std::max(0.05, config.settle_tau);
        state.settle_distance += current_speed * safe_dt;
        state.settle_distance -= state.settle_distance * (safe_dt / tau);
    }
    const bool rolling =
        std::fabs(state.settle_distance) > std::max(1e-3, config.no_progress_min_distance);
    const bool settled = standstill || !rolling;
    // 열린 safe-stop 경로는 앞쪽 감속 웨이포인트가 양수여도 말단 vx=0이 플래너의
    // 정지 의도다. launch_allowed=false 동안에는 그 중간 양수값으로 HFI를 재기동하지 않는다.
    const bool reset_requested = disengaged || target_speed <= 0.1 || !launch_allowed;
    const double relatch_time = std::max(0.0, config.relatch_time);
    const bool moving_bypass_enabled = config.moving_bypass_speed > 0.0;
    const bool moving_forward = moving_bypass_enabled &&
        current_speed >= config.moving_bypass_speed;
    const double moving_bypass_hold = std::max(0.0, config.moving_bypass_hold);
    const unsigned int max_attempts = std::max(1U, config.max_attempts);

    // 순간적인 target=0이나 모드 채터로 즉시 재무장하지 않는다. 정지 명령(또는 자율
    // 미체결)과 실제 정지가 relatch_time 동안 함께 유지되어야 다음 출발을 허용한다.
    if (reset_requested) {
        // 출발 시도/완료 뒤의 정지 요청은 relatch dwell이 끝날 때까지 다음 양의
        // 목표를 차단한다. target 채터가 0을 한 번 찍은 직후 보호 없이 재출발하는
        // 우회 경로를 없앤다. 최초 기동의 armed=true 상태에는 이 대기를 추가하지 않는다.
        // 최초 armed 상태라도 수동 주행/타력 중이면 다음 체결은 정지출발이 아니다. 첫
        // reset 사이클부터 pending을 세워 아래 signed moving bypass로만 통과시킨다.
        if (!state.armed || !standstill) state.relatch_pending = true;
        state.active = false;
        state.retry_waiting = false;
        state.elapsed = 0.0;
        state.exit_hold_elapsed = 0.0;
        state.retry_cooldown_elapsed = 0.0;
        state.launch_signed_distance = 0.0;
        state.reverse_elapsed = 0.0;
        state.last_failure_reason = HfiLaunchFailureReason::kNone;
        state.attempt = 0;
        // 새 출발 요청이므로 에스컬레이션도 보수적 기본값에서 다시 시작한다.
        state.cap_now = config.speed_cap;
        state.stall_elapsed = 0.0;
        state.latch_elapsed = 0.0;

        if (moving_forward) {
            state.moving_bypass_elapsed += safe_dt;
        } else {
            state.moving_bypass_elapsed = 0.0;
        }

        if (settled) {
            state.relatch_elapsed += safe_dt;
            if (state.relatch_elapsed >= relatch_time) {
                const bool reset_failure = state.failure_latched;
                state.armed = true;
                state.relatch_pending = false;
                state.failure_latched = false;
                state.moving_bypass_elapsed = 0.0;
                if (reset_failure) decision.event = HfiLaunchEvent::kFailureReset;
            }
        } else {
            state.relatch_elapsed = 0.0;
            state.armed = false;
        }
        // 감속 경로는 움직이는 동안 그대로 추종하고, 실제 정지한 뒤에만 0을 고정한다.
        // 플래너가 closed handoff/creep 경로로 바꿔 launch_allowed=true가 되면 armed 상태에서
        // 정상 HFI 출발을 시작한다. 이 게이트가 없으면 safe-stop의 앞쪽 양수 waypoint만 보고
        // 4초 안전 래치를 조기에 우회할 수 있다.
        if (!launch_allowed && (standstill || abs_speed < standstill_enter)) {
            decision.force_stop = true;
        }
        return decision;
    }

    if (state.relatch_pending) {
        // 수동→자율 전환 또는 순간 target=0 뒤에도 차가 충분히 빠르게 **전진 중**이면
        // HFI 정지출발을 다시 걸 이유가 없다. abs(speed)를 쓰지 않아 역주행은 우회하지
        // 못한다. reset_requested 구간에서 이미 쌓인 dwell도 이어받아 bumpless transfer한다.
        if (!state.failure_latched && moving_forward) {
            state.relatch_elapsed = 0.0;
            state.moving_bypass_elapsed += safe_dt;
            // 1 m/s 이상은 HFI 포착구간을 충분히 벗어난 실측이므로 hold를 확인하는 동안도
            // 0/brake를 삽입하지 않는다. 그렇지 않으면 바로 그 전환 계단이 새 위험이 된다.
            if (state.moving_bypass_elapsed >= moving_bypass_hold) {
                state.relatch_pending = false;
                state.armed = false;
                state.moving_bypass_elapsed = 0.0;
                state.cap_now = config.speed_cap;
                state.stall_elapsed = 0.0;
                ++state.moving_bypass_count;
                decision.event = HfiLaunchEvent::kMovingBypass;
            }
            return decision;
        }

        state.moving_bypass_elapsed = 0.0;
        decision.force_stop = true;

        // 핵심 교착 수리: raw target이 다시 양수여도 이 분기 자체가 실제 발행을 0으로
        // 강제하고 있으므로, VESC도 정지해 있으면 유효한 "정지 명령+실측 정지" dwell이다.
        // 종전 코드는 양수 target에서 relatch_elapsed를 매번 0으로 만들어 영원히 못 풀렸다.
        if (settled) {
            state.relatch_elapsed += safe_dt;
        } else {
            state.relatch_elapsed = 0.0;
        }
        if (settled && state.relatch_elapsed >= relatch_time) {
            state.relatch_pending = false;
            state.failure_latched = false;
            state.relatch_elapsed = 0.0;
            state.armed = false;
            state.active = true;
            state.elapsed = 0.0;
            state.exit_hold_elapsed = 0.0;
            state.retry_cooldown_elapsed = 0.0;
            state.launch_signed_distance = 0.0;
            state.reverse_elapsed = 0.0;
            state.last_failure_reason = HfiLaunchFailureReason::kNone;
            state.attempt = 1;
            state.stall_elapsed = 0.0;
            state.cap_now = std::max(state.cap_now, config.speed_cap);
            decision.force_stop = false;
            decision.constrain_to_cap = true;
            decision.cap = state.cap_now;
            decision.event = HfiLaunchEvent::kStarted;
        }
        return decision;
    }

    state.relatch_elapsed = 0.0;
    state.moving_bypass_elapsed = 0.0;

    if (state.failure_latched) {
        decision.force_stop = true;
        // ── 교착 탈출: 정지가 유지되면 스스로 재무장한다 ──────────────────
        // 조건이 "경과 시간"이 아니라 **"실제 VESC 정지 유지"**인 것이 핵심이다.
        // 굴러가는 중에는 절대 재무장하지 않는다(launch_relatch_time과 같은 규약).
        // 재무장은 항상 보수적 기본 cap에서 다시 시작하므로 "실패할수록 세진다"가
        // 되지 않는다.
        if (config.latch_release_time > 0.0) {
            if (settled) {
                state.latch_elapsed += safe_dt;
            } else {
                state.latch_elapsed = 0.0;
            }
            if (state.latch_elapsed >= config.latch_release_time) {
                state.failure_latched = false;
                state.latch_elapsed = 0.0;
                state.armed = false;
                state.active = true;
                state.elapsed = 0.0;
                state.exit_hold_elapsed = 0.0;
                state.retry_cooldown_elapsed = 0.0;
                state.launch_signed_distance = 0.0;
                state.reverse_elapsed = 0.0;
                state.stall_elapsed = 0.0;
                state.cap_now = config.speed_cap;
                state.last_failure_reason = HfiLaunchFailureReason::kNone;
                state.attempt = 1;
                ++state.latch_auto_reset_count;
                decision.force_stop = false;
                decision.constrain_to_cap = true;
                decision.cap = state.cap_now;
                decision.event = HfiLaunchEvent::kLatchAutoReset;
            }
        }
        return decision;
    }

    // 한 시도가 timeout되면 바로 다시 토크를 걸지 않고, 실제 정지가 cooldown 동안
    // 유지된 뒤에만 같은 저속 포착 시도를 한 번 더 한다.
    if (state.retry_waiting) {
        decision.force_stop = true;
        if (settled) {
            state.retry_cooldown_elapsed += safe_dt;
        } else {
            state.retry_cooldown_elapsed = 0.0;
        }

        if (settled &&
            state.retry_cooldown_elapsed >= std::max(0.0, config.retry_cooldown)) {
            state.retry_waiting = false;
            state.active = true;
            state.elapsed = 0.0;
            state.exit_hold_elapsed = 0.0;
            state.retry_cooldown_elapsed = 0.0;
            state.launch_signed_distance = 0.0;
            state.reverse_elapsed = 0.0;
            state.last_failure_reason = HfiLaunchFailureReason::kNone;
            state.stall_elapsed = 0.0;
            // ⚠️ cap_now는 **리셋하지 않는다**. 같은 상한으로 다시 시도하는 것은
            //    이미 실패한 토크 수준을 반복하는 것이다(0822 060751에서 재시도가
            //    3/8 또 실패해 최종 래치로 갔다). 요청 단위로 단조 유지한다.
            state.cap_now = std::max(state.cap_now, config.speed_cap);
            ++state.attempt;
            ++state.retry_count;
            decision.force_stop = false;
            decision.constrain_to_cap = true;
            decision.cap = state.cap_now;
            decision.event = HfiLaunchEvent::kRetryStarted;
        }
        return decision;
    }

    if (state.active) {
        state.elapsed += safe_dt;
        state.launch_signed_distance += current_speed * safe_dt;
        if (config.reverse_abort_speed > 0.0 &&
            current_speed <= -config.reverse_abort_speed) {
            state.reverse_elapsed += safe_dt;
        } else {
            state.reverse_elapsed = 0.0;
        }

        // ── cap 에스컬레이션 (요청 단위 단조 비감소) ──────────────────────
        // 🔑 "정체"의 정의가 **연속 정체 시간**이라 한 번 튀어나간 뒤 다시 멈추는
        //    로킹에서도 계속 밀어준다. cap을 내리지 않는 것도 의도적이다 — 내리면
        //    명령이 톱니가 되고, 그 톱니 자체가 VESC 속도 PID에 계단이 된다.
        if (config.escalate_enable) {
            if (state.cap_now < config.speed_cap) state.cap_now = config.speed_cap;
            if (current_speed >= config.escalate_progress_speed) {
                state.stall_elapsed = 0.0;      // 먹고 있다 — 더 밀지 않는다
            } else {
                state.stall_elapsed += safe_dt;
            }
            if (state.stall_elapsed >= std::max(0.0, config.escalate_start_delay)) {
                const double ceiling = std::max(config.escalate_cap_max, config.speed_cap);
                state.cap_now = std::min(ceiling,
                    state.cap_now + std::max(0.0, config.escalate_rate) * safe_dt);
            }
            state.escalate_peak = std::max(state.escalate_peak, state.cap_now);
        } else {
            state.cap_now = config.speed_cap;
        }

        // 전진 명령이므로 양의 속도만 성공이다. 역방향 -exit_speed는 절대 성공으로
        // 취급하지 않으며, 한 샘플 스파이크가 아니라 exit_hold 연속 유지를 요구한다.
        if (current_speed >= config.exit_speed) {
            state.exit_hold_elapsed += safe_dt;
        } else {
            state.exit_hold_elapsed = 0.0;
        }
        if (current_speed >= config.exit_speed &&
            state.exit_hold_elapsed >= std::max(0.0, config.exit_hold)) {
            state.active = false;
            state.armed = false;
            ++state.release_count;
            decision.event = HfiLaunchEvent::kReleased;
            return decision;
        }

        const bool sustained_reverse = config.reverse_abort_hold > 0.0 &&
            state.reverse_elapsed >= config.reverse_abort_hold;
        const bool no_forward_progress = config.no_progress_timeout > 0.0 &&
            state.elapsed >= config.no_progress_timeout &&
            state.launch_signed_distance < config.no_progress_min_distance;
        // timeout 직전에 exit_speed를 처음 넘은 경우에는 연속 hold를 끝낼 기회를 준다.
        // 060605에서는 2차 시도가 4.00초에 +0.55 m/s까지 실제로 관통했는데, hold가
        // 한 샘플만 쌓였다는 이유로 같은 사이클에 최종 실패 래치됐다. 속도가 다시
        // 문턱 아래로 내려가면 exit_hold_elapsed가 0이 되어 다음 사이클 즉시 실패한다.
        const bool hard_timeout = config.timeout > 0.0 &&
            state.elapsed >= config.timeout && state.exit_hold_elapsed <= 0.0;
        if (sustained_reverse || no_forward_progress || hard_timeout) {
            state.active = false;
            state.armed = false;
            state.exit_hold_elapsed = 0.0;
            ++state.attempt_timeout_count;
            if (sustained_reverse) {
                state.last_failure_reason = HfiLaunchFailureReason::kSustainedReverse;
                ++state.reverse_abort_count;
            } else if (no_forward_progress) {
                state.last_failure_reason = HfiLaunchFailureReason::kNoForwardProgress;
                ++state.no_progress_abort_count;
            } else {
                state.last_failure_reason = HfiLaunchFailureReason::kTimeout;
            }
            decision.force_stop = true;
            if (state.attempt < max_attempts) {
                state.retry_waiting = true;
                state.retry_cooldown_elapsed = 0.0;
                decision.event = HfiLaunchEvent::kRetryScheduled;
            } else {
                state.failure_latched = true;
                ++state.failure_count;
                decision.event = HfiLaunchEvent::kTimedOut;
            }
            return decision;
        }
        decision.constrain_to_cap = true;
        decision.cap = state.cap_now;
        return decision;
    }

    if (state.armed && standstill) {
        state.active = true;
        state.armed = false;
        state.elapsed = 0.0;
        state.exit_hold_elapsed = 0.0;
        state.retry_cooldown_elapsed = 0.0;
        state.launch_signed_distance = 0.0;
        state.reverse_elapsed = 0.0;
        state.last_failure_reason = HfiLaunchFailureReason::kNone;
        state.attempt = 1;
        state.stall_elapsed = 0.0;
        state.cap_now = config.speed_cap;
        decision.constrain_to_cap = true;
        decision.cap = state.cap_now;
        decision.event = HfiLaunchEvent::kStarted;
    }
    return decision;
}

}  // namespace f1tenth_control

#include "hexapod_core/tripod_phase.hpp"
#include "hexapod_core/foot_trajectory.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace hexapod_core;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const std::string& message) {
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-12,
        message);
}

using phase_array = std::array<leg_phase_state, leg_count>;

void expect_groups(const phase_array& phases,
    leg_phase phase_a, double progress_a,
    leg_phase phase_b, double progress_b, const std::string& message) {
    // 显式使用既定六腿顺序，独立检查生产代码的分组填写。
    const std::array<leg_phase, 6> expected_phases{
        phase_a, phase_b, phase_a, phase_b, phase_a, phase_b};
    const std::array<double, 6> expected_progress{
        progress_a, progress_b, progress_a, progress_b, progress_a, progress_b};
    for (std::size_t index = 0; index < phases.size(); ++index) {
        require(phases[index].phase == expected_phases[index],
            message + " leg phase " + std::to_string(index));
        near(phases[index].phase_ratio, expected_progress[index],
            message + " leg progress " + std::to_string(index));
    }
}

void test_known_samples() {
    // 占空比 5/8：支撑 5 个时间单位，摆动 3 个时间单位。
    expect_groups(sample_tripod_phase({0.0, 0.625}),
        leg_phase::stance, 0.0, leg_phase::stance, 0.8, "cycle start");
    expect_groups(sample_tripod_phase({0.125, 0.625}),
        leg_phase::stance, 0.2, leg_phase::swing, 0.0, "B lift boundary");
    expect_groups(sample_tripod_phase({0.25, 0.625}),
        leg_phase::stance, 0.4, leg_phase::swing, 1.0 / 3.0, "quarter");
    expect_groups(sample_tripod_phase({0.5, 0.625}),
        leg_phase::stance, 0.8, leg_phase::stance, 0.0, "B wrap boundary");
    expect_groups(sample_tripod_phase({0.625, 0.625}),
        leg_phase::swing, 0.0, leg_phase::stance, 0.2, "A lift boundary");
    expect_groups(sample_tripod_phase({0.75, 0.625}),
        leg_phase::swing, 1.0 / 3.0, leg_phase::stance, 0.4, "three quarters");
    expect_groups(sample_tripod_phase({0.875, 0.625}),
        leg_phase::swing, 2.0 / 3.0, leg_phase::stance, 0.6, "seven eighths");

    expect_groups(sample_tripod_phase({0.0, 0.5}),
        leg_phase::stance, 0.0, leg_phase::swing, 0.0, "no-overlap start");
    expect_groups(sample_tripod_phase({0.25, 0.5}),
        leg_phase::stance, 0.5, leg_phase::swing, 0.5, "no-overlap quarter");
    expect_groups(sample_tripod_phase({0.5, 0.5}),
        leg_phase::swing, 0.0, leg_phase::stance, 0.0, "no-overlap exchange");
}

void test_integer_timeline() {
    // 独立的整数时间表：一个周期 256 个 tick，B 组偏移 128 tick。
    // 用整数判定阶段，避免复制生产函数的浮点边界分支。
    constexpr int period_ticks = 256;
    constexpr std::array<int, 6> offsets{0, 128, 0, 128, 0, 128};
    for (int stance_ticks = 128; stance_ticks < period_ticks; ++stance_ticks) {
        std::array<int, 6> counted_stance{};
        int all_stance_ticks = 0;
        for (int tick = 0; tick < period_ticks; ++tick) {
            const auto phases = sample_tripod_phase({
                static_cast<double>(tick) / period_ticks,
                static_cast<double>(stance_ticks) / period_ticks});
            int support_count = 0;
            for (std::size_t index = 0; index < phases.size(); ++index) {
                const int local_tick = (tick + offsets[index]) % period_ticks;
                const bool stance = local_tick < stance_ticks;
                const auto& actual = phases[index];
                require(actual.phase == (stance ? leg_phase::stance : leg_phase::swing),
                    "integer timeline stage");
                require(std::isfinite(actual.phase_ratio)
                    && actual.phase_ratio >= 0.0 && actual.phase_ratio <= 1.0,
                    "finite normalized stage progress");
                if (stance) {
                    near(actual.phase_ratio,
                        static_cast<double>(local_tick) / stance_ticks,
                        "integer stance progress");
                    ++counted_stance[index];
                    ++support_count;
                } else {
                    near(actual.phase_ratio,
                        static_cast<double>(local_tick - stance_ticks)
                            / (period_ticks - stance_ticks),
                        "integer swing progress");
                }
            }
            require(support_count == 3 || support_count == 6,
                "three or six planned support legs");
            if (support_count == 6) ++all_stance_ticks;

            // 半周期后，两组的阶段和阶段进度互换。
            const int shifted_tick = (tick + 128) % period_ticks;
            const auto shifted = sample_tripod_phase({
                static_cast<double>(shifted_tick) / period_ticks,
                static_cast<double>(stance_ticks) / period_ticks});
            require(shifted[0].phase == phases[1].phase
                && shifted[1].phase == phases[0].phase, "half-cycle group exchange");
            near(shifted[0].phase_ratio, phases[1].phase_ratio, "exchanged A progress");
            near(shifted[1].phase_ratio, phases[0].phase_ratio, "exchanged B progress");
        }
        for (const int count : counted_stance) {
            require(count == stance_ticks, "per-leg stance duration matches duty factor");
        }
        require(all_stance_ticks == 2 * stance_ticks - period_ticks,
            "planned double-support duration");
    }
}

void test_floating_boundaries() {
    const double last_cycle = std::nextafter(1.0, 0.0);
    const double before_half = std::nextafter(0.5, 0.0);
    const double after_half = std::nextafter(0.5, 1.0);
    const std::array<double, 5> duties{
        0.5, std::nextafter(0.5, 1.0), 0.6, 0.625, last_cycle};
    for (const double duty : duties) {
        const auto at_a_lift = sample_tripod_phase({duty, duty});
        require(at_a_lift[0].phase == leg_phase::swing
            && at_a_lift[0].phase_ratio == 0.0, "A exact swing boundary");
        require(sample_tripod_phase({std::nextafter(duty, 0.0), duty})[0].phase
            == leg_phase::stance, "A immediately before swing");

        const double b_lift = duty - 0.5;
        const auto at_b_lift = sample_tripod_phase({b_lift, duty});
        require(at_b_lift[1].phase == leg_phase::swing
            && at_b_lift[1].phase_ratio == 0.0, "B exact swing boundary");
        if (b_lift > 0.0) {
            require(sample_tripod_phase({std::nextafter(b_lift, 0.0), duty})[1].phase
                == leg_phase::stance, "B immediately before swing");
        }
        require(sample_tripod_phase({std::nextafter(b_lift, 0.5), duty})[1].phase
            == leg_phase::swing, "B immediately after swing boundary");
        require(sample_tripod_phase({before_half, duty})[1].phase == leg_phase::swing,
            "B does not wrap early when addition would round to one");
        const auto at_half = sample_tripod_phase({0.5, duty});
        require(at_half[1].phase == leg_phase::stance
            && at_half[1].phase_ratio == 0.0, "B wraps exactly at half cycle");
        require(sample_tripod_phase({after_half, duty})[1].phase == leg_phase::stance,
            "B immediately after wrap");

        // 合法输入的极端占空比和相位也必须返回有限归一化结果。
        for (const double cycle : {0.0, std::nextafter(0.0, 1.0), before_half,
                 0.5, after_half, duty, last_cycle}) {
            for (const auto& state : sample_tripod_phase({cycle, duty})) {
                require(std::isfinite(state.phase_ratio)
                    && state.phase_ratio >= 0.0 && state.phase_ratio <= 1.0,
                    "floating boundary progress remains normalized");
            }
        }
    }

    // 对 0.6 的 B 组边界，先相加会把边界前一点舍入到边界。
    const double before_b_lift = std::nextafter(0.6 - 0.5, 0.0);
    require(sample_tripod_phase({before_b_lift, 0.6})[1].phase == leg_phase::stance,
        "B does not begin swing early due to addition rounding");
}

void test_invalid_input() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const auto rejected = [](const tripod_phase_request& request) {
        try {
            (void)sample_tripod_phase(request);
        } catch (const std::invalid_argument&) {
            return;
        }
        throw std::runtime_error("invalid request was not rejected");
    };
    for (const double cycle : {-1.0, std::nextafter(0.0, -1.0),
             1.0, std::nextafter(1.0, 2.0), nan, inf, -inf}) {
        rejected({cycle, 0.625});
    }
    for (const double duty : {-1.0, 0.0, std::nextafter(0.5, 0.0),
             1.0, std::nextafter(1.0, 2.0), nan, inf, -inf}) {
        rejected({0.25, duty});
    }
    // 不保存历史，倒序采样不会改变同一输入的结果或请求本身。
    const tripod_phase_request request{0.25, 0.625};
    const auto before = sample_tripod_phase(request);
    (void)sample_tripod_phase({0.875, 0.75});
    const auto after = sample_tripod_phase(request);
    for (std::size_t index = 0; index < before.size(); ++index) {
        require(before[index].phase == after[index].phase
            && before[index].phase_ratio == after[index].phase_ratio,
            "sampling is independent of call history");
    }
    require(request.cycle_ratio == 0.25 && request.stance_ratio == 0.625,
        "request remains unchanged");
}

void test_swing_consumer() {
    const auto phases = sample_tripod_phase({0.8125, 0.625});
    require(phases[0].phase == leg_phase::swing, "A swing consumer stage");
    const swing_trajectory_request swing{
        {0.0, 0.0, -0.1}, {0.02, 0.0, -0.1}, {0.0, 0.0, 1.0}, 0.01};
    const auto point = sample_swing_trajectory(swing, phases[0].phase_ratio);
    near(point.x, 0.01, "stage progress drives swing midpoint x");
    near(point.y, 0.0, "stage progress drives swing midpoint y");
    near(point.z, -0.09, "stage progress drives swing apex");
}

}  // namespace

int main() {
    try {
        test_known_samples();
        test_integer_timeline();
        test_floating_boundaries();
        test_invalid_input();
        test_swing_consumer();
        std::cout << "PASS: tripod groups, 32768 integer timeline samples, duty duration, "
            "half-cycle symmetry, floating boundaries, invalid input and swing consumer.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

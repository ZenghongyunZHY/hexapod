#include "hexapod_core/foot_clearance_policy.hpp"

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
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-15, message);
}

template <typename function_type>
void require_invalid(function_type function, const std::string& message) {
    try { function(); } catch (const std::invalid_argument&) { return; }
    throw std::runtime_error("missing invalid_argument: " + message);
}

void test_contact_and_swing() {
    // 合成且不对称的离线预算，用来区分三个字段，不作为运行默认值。
    const foot_clearance_policy policy{0.0002, 0.0004, 0.0007};
    require_foot_clearance_policy(policy);
    for (const double ratio : {0.0, 0.1, 0.5, 0.9, 1.0}) {
        const auto contact = make_foot_clearance_bounds({leg_phase::stance, ratio}, 0.0, policy);
        near(contact.minimum_clearance_m, -0.0002, "stance lower uses below budget");
        require(contact.maximum_clearance_m.has_value(), "stance requires upper bound");
        near(*contact.maximum_clearance_m, 0.0004, "stance upper uses above budget");
    }
    for (const double ratio : {0.0, 1.0}) {
        const auto contact = make_foot_clearance_bounds({leg_phase::swing, ratio}, 0.0, policy);
        near(contact.minimum_clearance_m, -0.0002, "exact swing endpoint uses contact lower");
        require(contact.maximum_clearance_m.has_value(), "exact swing endpoint has upper");
        near(*contact.maximum_clearance_m, 0.0004, "exact swing endpoint uses contact upper");
    }
    for (const double ratio : {std::nextafter(0.0, 1.0), 0.2, 0.5, std::nextafter(1.0, 0.0)}) {
        const auto raised = make_foot_clearance_bounds({leg_phase::swing, ratio}, 0.003, policy);
        near(raised.minimum_clearance_m, 0.0023, "interior height minus swing budget");
        require(!raised.maximum_clearance_m, "interior never inherits contact upper");
        const auto near_ground = make_foot_clearance_bounds({leg_phase::swing, ratio}, 0.0001, policy);
        near(near_ground.minimum_clearance_m, -0.0002, "contact floor dominates near ground");
        require(!near_ground.maximum_clearance_m, "near-ground interior still has no upper");
        const auto at_switch = make_foot_clearance_bounds({leg_phase::swing, ratio}, 0.0005, policy);
        near(at_switch.minimum_clearance_m, -0.0002, "equal lower candidates");
    }
    const auto zero = make_foot_clearance_bounds({leg_phase::stance, 0.5}, 0.0, {0.0, 0.0, 0.0});
    near(zero.minimum_clearance_m, 0.0, "zero budget permits exact contact only");
    require(zero.maximum_clearance_m && *zero.maximum_clearance_m == 0.0, "zero contact upper");
    const auto strict_swing = make_foot_clearance_bounds({leg_phase::swing, 0.5}, 0.003, {0.0, 0.0, 0.0});
    near(strict_swing.minimum_clearance_m, 0.003, "zero swing budget preserves ideal height");
    require(!strict_swing.maximum_clearance_m, "zero swing budget does not create upper");
    require(policy.contact_below_tolerance_m == 0.0002
        && policy.contact_above_tolerance_m == 0.0004 && policy.swing_height_error_m == 0.0007,
        "policy input unchanged");
}

void test_invalid_inputs() {
    const foot_clearance_policy valid{0.0002, 0.0004, 0.0007};
    const std::array<double foot_clearance_policy::*, 3> fields{
        &foot_clearance_policy::contact_below_tolerance_m,
        &foot_clearance_policy::contact_above_tolerance_m,
        &foot_clearance_policy::swing_height_error_m};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const auto field : fields) {
        for (const double bad : {-0.001, nan, inf, -inf}) {
            auto policy = valid;
            policy.*field = bad;
            require_invalid([&] { require_foot_clearance_policy(policy); }, "each policy field validated");
            require_invalid([&] {
                (void)make_foot_clearance_bounds({leg_phase::stance, 0.0}, 0.0, policy);
            }, "contact branch cannot skip any policy validation");
        }
    }
    for (const auto phase : {leg_phase::stance, leg_phase::swing}) {
        for (const double bad : {-0.001, std::nextafter(1.0, 2.0), nan, inf, -inf}) {
            require_invalid([&] { (void)make_foot_clearance_bounds({phase, bad}, 0.0, valid); }, "phase ratio range");
        }
        for (const double bad : {-0.001, nan, inf, -inf}) {
            require_invalid([&] { (void)make_foot_clearance_bounds({phase, 0.0}, bad, valid); }, "ideal height before endpoint return");
        }
    }
    require_invalid([&] {
        (void)make_foot_clearance_bounds({static_cast<leg_phase>(99), 0.0}, 0.0, valid);
    }, "unknown phase");
}

}  // namespace

int main() {
    try {
        test_contact_and_swing();
        test_invalid_inputs();
        std::cout << "PASS: asymmetric contact bounds, exact endpoints, swing height floor, zero budgets and invalid inputs.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

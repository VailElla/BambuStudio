#include <catch2/catch.hpp>

#include "libslic3r/Arachne/SkeletalTrapezoidation.hpp"
#include "libslic3r/Arachne/BeadingStrategy/DistributedBeadingStrategy.hpp"

using namespace Slic3r;
using namespace Slic3r::Arachne;

namespace {
class InterpolationFixture : public SkeletalTrapezoidation
{
public:
    using SkeletalTrapezoidation::SkeletalTrapezoidation;
    using SkeletalTrapezoidation::interpolate;
};
using Beading = BeadingStrategy::Beading;

void require_same(const Beading& actual, const Beading& expected)
{
    REQUIRE(actual.total_thickness == expected.total_thickness);
    REQUIRE(actual.bead_widths.size() == expected.bead_widths.size());
    REQUIRE(actual.toolpath_locations.size() == expected.toolpath_locations.size());
    for (size_t i = 0; i < actual.bead_widths.size(); ++i)
        REQUIRE(actual.bead_widths[i] == Approx(expected.bead_widths[i]).margin(1));
    for (size_t i = 0; i < actual.toolpath_locations.size(); ++i)
        REQUIRE(actual.toolpath_locations[i] == Approx(expected.toolpath_locations[i]).margin(1));
    REQUIRE(actual.left_over == expected.left_over);
}
}

TEST_CASE("Arachne switching interpolation handles absent corresponding insets", "[arachne]")
{
    DistributedBeadingStrategy strategy(400000, 400000, 0.785398, 0.5, 0.5, 2);
    Polygons outline{Polygon{Points{{0, 0}, {10000000, 0}, {10000000, 10000000}, {0, 10000000}}}};
    InterpolationFixture fixture(outline, strategy, 0.785398, 200000, 100000, 25000, 400000, false, {});

    SECTION("zero-bead right layout selected as the result, as in Lucy's crash") {
        Beading left{400000, {200000, 200000}, {100000, 300000}, 0};
        Beading right{400000, {}, {}, 400000};
        auto result = fixture.interpolate(left, 0.5, right, 200000);
        require_same(result, fixture.interpolate(left, 0.5, right));
        REQUIRE(result.toolpath_locations.empty());
    }
    SECTION("nonempty result shorter than the left-hand index") {
        Beading left{600000, {200000, 200000, 200000}, {100000, 300000, 500000}, 0};
        Beading right{600000, {600000}, {300000}, 0};
        require_same(fixture.interpolate(left, 0.5, right, 400000), fixture.interpolate(left, 0.5, right));
    }
    SECTION("result has the inset but the right-hand layout does not") {
        Beading left{800000, {200000, 200000, 200000, 200000}, {100000, 300000, 500000, 700000}, 0};
        Beading right{400000, {400000}, {200000}, 0};
        require_same(fixture.interpolate(left, 0.5, right, 400000), fixture.interpolate(left, 0.5, right));
    }
    SECTION("empty left layout needs no switching correction") {
        Beading left{400000, {}, {}, 400000};
        Beading right{400000, {200000, 200000}, {100000, 300000}, 0};
        require_same(fixture.interpolate(left, 0.5, right, 200000), fixture.interpolate(left, 0.5, right));
    }
    SECTION("inconsistent bead and toolpath vectors are truncated safely") {
        Beading left{600000, {200000, 200000}, {}, 600000};
        Beading right{400000, {200000, 200000}, {100000, 300000}, 0};
        const auto result = fixture.interpolate(left, 0.5, right, 200000);
        REQUIRE(result.bead_widths.empty());
        REQUIRE(result.toolpath_locations.empty());
    }
    SECTION("shared inset still receives the original switching correction") {
        Beading left{600000, {200000, 400000}, {100000, 400000}, 0};
        Beading right{600000, {500000, 100000}, {300000, 550000}, 0};
        const auto result = fixture.interpolate(left, 0.5, right, 150000);
        REQUIRE(result.toolpath_locations.size() == 2);
        REQUIRE(result.toolpath_locations.front() <= 150000);
        REQUIRE(result.bead_widths.front() > 0);
        const auto expected = fixture.interpolate(left, 0.85, right);
        for (size_t i = 0; i < result.toolpath_locations.size(); ++i) {
            REQUIRE(result.toolpath_locations[i] == Approx(expected.toolpath_locations[i]).margin(1));
            REQUIRE(result.bead_widths[i] == Approx(expected.bead_widths[i]).margin(1));
        }
    }
}

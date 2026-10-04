#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
#include "landmark_server/course_model.hpp"
#include "landmark_server/retained_landmarks.hpp"
#include "test_map_utils.hpp"

namespace vortex::mission {

using namespace test;

namespace {

/// Course frame = odom (the gate at the origin, x through it).
CourseGeometry at_gate() {
    CourseGeometry g;
    g.set = true;
    g.at_gate = true;
    g.through_yaw = 0.0;
    g.to_odom = [](const Eigen::Vector2d& v) { return v; };
    g.to_course = [](const Eigen::Vector2d& v) { return v; };
    return g;
}

/// Two slalom sets, the torpedo board (two decal versions) and the gate.
const char* kCourse = R"(
course:
  enable: true
  start: [-4.0, 0.0]
  extra_tracks_per_kind: 2
  variant_votes: 20
  min_part_detections: 0
  class_groups:
    slalom_pipes: [SLALOM_PIPE_WHITE, SLALOM_PIPE_RED]
    torpedo_hazards: [TORPEDO_ICON_FIRE, TORPEDO_ICON_BLOOD]
    torpedo_vehicles: [TORPEDO_ICON_FIRETRUCK, TORPEDO_ICON_AMBULANCE]
  templates:
    gate:
      members:
        whole: {class: GATE_WHOLE, offset: [0.0, 0.0, 0.0]}
        pole_left: {class: GATE_POLE_EDGE, offset: [0.0, 1.55, 0.0]}
        pole_right: {class: GATE_POLE_EDGE, offset: [0.0, -1.55, 0.0]}
    slalom_set:
      sigma: [0.2, 0.2, 0.3]
      members:
        white_left: {class: SLALOM_PIPE_WHITE, offset: [0.0, -1.52, 0.0]}
        red: {class: SLALOM_PIPE_RED, offset: [0.0, 0.0, 0.0]}
        white_right: {class: SLALOM_PIPE_WHITE, offset: [0.0, 1.52, 0.0]}
    torpedo_board:
      sigma: [0.15, 0.06, 0.06]
      variants:
        version_1:
          members:
            board: {class: TORPEDO_BOARD_WHOLE, offset: [0.0, 0.0, 0.0]}
            fire: {class: TORPEDO_ICON_FIRE, offset: [0.0, 0.206, -0.214]}
            firetruck: {class: TORPEDO_ICON_FIRETRUCK, offset: [0.0, -0.186, -0.193]}
            blood: {class: TORPEDO_ICON_BLOOD, offset: [0.0, -0.219, 0.058]}
            ambulance: {class: TORPEDO_ICON_AMBULANCE, offset: [0.0, 0.182, 0.202]}
        version_2:
          members:
            board: {class: TORPEDO_BOARD_WHOLE, offset: [0.0, 0.0, 0.0]}
            blood: {class: TORPEDO_ICON_BLOOD, offset: [0.0, 0.210, -0.222]}
            ambulance: {class: TORPEDO_ICON_AMBULANCE, offset: [0.0, -0.178, -0.204]}
            fire: {class: TORPEDO_ICON_FIRE, offset: [0.0, -0.213, 0.056]}
            firetruck: {class: TORPEDO_ICON_FIRETRUCK, offset: [0.0, 0.183, 0.183]}
  tasks:
    gate: {template: gate, prior: [0.0, 0.0, 180.0], region_radius_m: 3.0,
           yaw_window_deg: 30.0, min_parts: 1}
    slalom_1: {template: slalom_set, prior: [4.0, 0.0, 0.0], region_radius_m: 1.2,
               yaw_window_deg: 20.0, symmetric: true, min_parts: 3}
    slalom_2: {template: slalom_set, prior: [6.0, 0.5, 0.0], region_radius_m: 1.2,
               yaw_window_deg: 20.0, symmetric: true, min_parts: 3}
    torpedo: {template: torpedo_board, prior: [13.0, -5.0, 180.0], region_radius_m: 2.0,
              part_radius_m: 0.5, yaw_window_deg: 30.0, min_parts: 3}
)";

LandmarkMapConfig course_config() {
    auto cfg = example_config();
    cfg.course = parse_map_config(YAML::Load(kCourse)).course;
    return cfg;
}

const LandmarkClassKey kPipe{LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE};  // kind
const LandmarkClassKey kHazard{LT::TORPEDO_BOARD, LS::TORPEDO_ICON_FIRE};
const LandmarkClassKey kVehicle{LT::TORPEDO_BOARD, LS::TORPEDO_ICON_FIRETRUCK};
const LandmarkClassKey kBoard{LT::TORPEDO_BOARD, LS::TORPEDO_BOARD_WHOLE};

Eigen::Vector3d v(double x, double y, double z = 2.6) { return {x, y, z}; }

/// The torpedo board of the course at (13, -5), facing -x (board +Y = -y).
Eigen::Vector3d board_part(double by, double bz) {
    return {13.0, -5.0 - by, 2.55 + bz};
}

struct World {
    RetainedLandmarks map{course_config()};
    RetainedLandmarks::CourseInput input;
    double now{0.0};

    World() { input.geometry = at_gate(); }

    void tick(const std::vector<Track>& tracks, const TrackVotes& votes = {}) {
        input.votes = votes;
        map.update(tracks, now, {}, input);
        now += 0.2;
    }
    int count(uint16_t type) const {
        int n = 0;
        for (const auto& l : map.landmarks()) {
            n += l.key.type == type ? 1 : 0;
        }
        return n;
    }
    const RetainedLandmark* in_slot(const std::string& slot) const {
        for (const auto& l : map.landmarks()) {
            if (l.course_slot == slot) {
                return &l;
            }
        }
        return nullptr;
    }
    const CourseModel::Task& task(const std::string& name) const {
        for (const auto& t : map.course().tasks()) {
            if (t.spec->name == name) {
                return t;
            }
        }
        throw std::runtime_error(name);
    }
};

Track pipe(int id, const Eigen::Vector3d& p) {
    return make_track(id, kPipe.type, kPipe.subtype, p, true, false);
}

/// The first slalom set as the course has it, 0.2 m off.
std::vector<Track> set_1() {
    return {pipe(1, v(4.1, -1.32)), pipe(2, v(4.1, 0.2)), pipe(3, v(4.1, 1.72))};
}

std::vector<Track> board_tracks(int first_id = 10) {
    return {make_track(first_id, kBoard.type, kBoard.subtype, board_part(0, 0), true, false),
            make_track(first_id + 1, kHazard.type, kHazard.subtype, board_part(0.206, -0.214), true, false),
            make_track(first_id + 2, kVehicle.type, kVehicle.subtype, board_part(-0.186, -0.193), true, false),
            make_track(first_id + 3, kHazard.type, kHazard.subtype, board_part(-0.219, 0.058), true, false),
            make_track(first_id + 4, kVehicle.type, kVehicle.subtype, board_part(0.182, 0.202), true, false)};
}

}  // namespace

TEST(CourseModel, TheLayoutParses) {
    const auto cfg = course_config().course;
    EXPECT_TRUE(cfg.enable);
    EXPECT_EQ(cfg.tasks.size(), 4u);
    EXPECT_EQ(cfg.kind_of({LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED}), kPipe);
    EXPECT_EQ(cfg.kind_of({LT::TORPEDO_BOARD, LS::TORPEDO_ICON_BLOOD}), kHazard);
    EXPECT_EQ(cfg.kind_of(kBoard), kBoard);
    EXPECT_NEAR(cfg.task("gate")->prior_yaw, M_PI, 1e-12);
}

TEST(CourseModel, MistakesAreRejectedWithTheKey) {
    const auto bad = [](const std::string& yaml) {
        EXPECT_THROW(parse_course_config(YAML::Load(yaml)["course"]), std::runtime_error)
            << yaml;
    };
    // unknown template
    bad(R"(course: {enable: true, tasks: {a: {template: nope, prior: [0, 0, 0]}}})");
    // enabled without tasks
    bad(R"(course: {enable: true})");
    // a part of two kinds (no class group for them)
    bad(R"(course:
  templates: {t: {members: {a: {class: [SLALOM_PIPE_WHITE, SLALOM_PIPE_RED], offset: [0, 0, 0]},
                            b: {class: GATE_WHOLE, offset: [1, 0, 0]}}}})");
    // a type is not a part
    bad(R"(course:
  templates: {t: {members: {a: {class: SLALOM_PIPE, offset: [0, 0, 0]},
                            b: {class: GATE_WHOLE, offset: [1, 0, 0]}}}})");
    // variants with another geometry
    bad(R"(course:
  templates:
    t:
      variants:
        a: {members: {x: {class: GATE_WHOLE, offset: [0, 0, 0]}, y: {class: GATE_POLE_EDGE, offset: [0, 1, 0]}}}
        b: {members: {x: {class: GATE_WHOLE, offset: [0, 0, 0]}, y: {class: GATE_POLE_EDGE, offset: [0, 2, 0]}}})");
}

TEST(CourseModel, ASetIsPlacedFromItsPipesAndNothingElseIsAPipe) {
    World w;
    auto tracks = set_1();
    tracks.push_back(pipe(4, v(4.0, 3.5)));    // outside every region
    tracks.push_back(pipe(5, v(-0.1, 1.5)));   // a gate post seen as a pipe
    for (int i = 0; i < 3; ++i) {
        w.tick(tracks);
    }
    EXPECT_TRUE(w.task("slalom_1").placed);
    EXPECT_EQ(w.count(LT::SLALOM_PIPE), 3);
    ASSERT_NE(w.in_slot("slalom_1/red"), nullptr);
    // The class comes from the template, not from the track's kind.
    EXPECT_EQ(w.in_slot("slalom_1/red")->key.subtype, LS::SLALOM_PIPE_RED);
    EXPECT_EQ(w.in_slot("slalom_1/white_left")->key.subtype, LS::SLALOM_PIPE_WHITE);
    EXPECT_NEAR(w.in_slot("slalom_1/red")->position.y(), 0.2, 1e-9);
}

TEST(CourseModel, PipesAlongTheCourseAreNoSet) {
    World w;
    for (int i = 0; i < 3; ++i) {
        w.tick({pipe(1, v(2.5, 0.0)), pipe(2, v(4.0, 0.0)), pipe(3, v(5.5, 0.0))});
    }
    EXPECT_FALSE(w.task("slalom_1").placed);
    EXPECT_EQ(w.count(LT::SLALOM_PIPE), 0);
}

TEST(CourseModel, TwoPipesDoNotPlaceASetThatNeedsThree) {
    // A real white and a false red at the right spacing.
    World w;
    for (int i = 0; i < 3; ++i) {
        w.tick({pipe(1, v(4.0, 0.0)), pipe(2, v(4.0, 1.52))});
    }
    EXPECT_FALSE(w.task("slalom_1").placed);
    EXPECT_EQ(w.count(LT::SLALOM_PIPE), 0);
}

TEST(CourseModel, APipeSeenAgainKeepsItsIdWhateverItIsCalled) {
    World w;
    w.tick(set_1());
    const int red_id = w.in_slot("slalom_1/red")->id;
    // The red's track is gone; the pipe is seen again as a new track (the
    // detector called it white this time: the same kind).
    w.tick({pipe(1, v(4.1, -1.32)), pipe(3, v(4.1, 1.72))});
    EXPECT_FALSE(w.in_slot("slalom_1/red")->is_live());
    w.tick({pipe(1, v(4.1, -1.32)), pipe(3, v(4.1, 1.72)), pipe(9, v(4.15, 0.25))},
           {{9, {{{LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE}, 3}}}});
    EXPECT_EQ(w.in_slot("slalom_1/red")->id, red_id);
    EXPECT_EQ(w.in_slot("slalom_1/red")->live_track_id, 9);
    EXPECT_EQ(w.in_slot("slalom_1/red")->key.subtype, LS::SLALOM_PIPE_RED);
    EXPECT_EQ(w.count(LT::SLALOM_PIPE), 3);
}

TEST(CourseModel, ATrackFillsAPartOnlyAfterEnoughDetections) {
    auto cfg = course_config();
    cfg.course.min_part_detections = 8;
    RetainedLandmarks map(cfg);
    RetainedLandmarks::CourseInput in;
    in.geometry = at_gate();
    const auto votes = [](int n) {
        TrackVotes v;
        for (int id : {1, 2, 3}) {
            v[id] = {{{LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE}, n}};
        }
        return v;
    };
    in.votes = votes(3);
    map.update(set_1(), 0.0, {}, in);
    EXPECT_FALSE(map.course().tasks()[1].placed);  // 3 detections each
    in.votes = votes(5);
    map.update(set_1(), 0.2, {}, in);
    EXPECT_TRUE(map.course().tasks()[1].placed);   // 8 each
}

TEST(CourseModel, PartsAreRememberedForTheWholeRun) {
    World w;
    w.tick(set_1());
    for (int i = 0; i < 500; ++i) {
        w.tick({});  // 100 s without the set
    }
    EXPECT_EQ(w.count(LT::SLALOM_PIPE), 3);
}

TEST(CourseModel, AFalsePipeNextToAPlacedSetIsNoObject) {
    World w;
    w.tick(set_1());
    // 1.6 m from every part of the set and outside the next set's region:
    // not a part, never a landmark, and dropped before the tracker.
    for (int i = 0; i < 5; ++i) {
        auto tracks = set_1();
        tracks.push_back(pipe(7, v(3.0, 2.9)));
        w.tick(tracks);
    }
    EXPECT_EQ(w.count(LT::SLALOM_PIPE), 3);
    const auto& course = w.map.course();
    EXPECT_EQ(course.intake_reject(kPipe, v(3.0, 2.9), at_gate()), "outside_tasks");
    EXPECT_FALSE(course.intake_reject(kPipe, v(4.2, 0.1), at_gate()));
}

TEST(CourseModel, SmallPartsAreTakenOnlyUpClose) {
    auto cfg = course_config();
    for (auto& t : cfg.course.tasks) {
        if (t.name == "torpedo") {
            t.max_range_m = 5.0;
        }
    }
    RetainedLandmarks map(cfg);
    const auto icon = board_part(0.206, -0.214);
    EXPECT_EQ(map.course().intake_reject(kHazard, icon, at_gate(), v(6.0, -5.0)), "too_far");
    EXPECT_FALSE(map.course().intake_reject(kHazard, icon, at_gate(), v(10.0, -5.0)));
}

TEST(CourseModel, TheNextSetIsSearchedWhereTheFirstSaysItIs) {
    World w;
    // The first set stands 0.8 m to the right of its prior.
    w.tick({pipe(1, v(4.0, -0.72)), pipe(2, v(4.0, 0.8)), pipe(3, v(4.0, 2.32))});
    ASSERT_TRUE(w.task("slalom_1").placed);
    const auto pose = w.map.course().working_pose(w.task("slalom_2"), at_gate());
    ASSERT_TRUE(pose);
    EXPECT_NEAR(pose->translation().y(), 0.5 + 0.8, 1e-6);
}

TEST(CourseModel, BeforeTheGateThePriorsAreFromTheStart) {
    World w;
    CourseGeometry g = at_gate();
    g.at_gate = false;  // origin at the start, 4 m before the gate
    const auto pose = w.map.course().working_pose(w.task("slalom_1"), g);
    ASSERT_TRUE(pose);
    EXPECT_NEAR(pose->translation().x(), 8.0, 1e-9);
}

TEST(CourseModel, WithoutACourseFrameNoTaskIsMapped) {
    World w;
    w.input.geometry = CourseGeometry{};
    for (int i = 0; i < 3; ++i) {
        w.tick(set_1());
    }
    EXPECT_EQ(w.count(LT::SLALOM_PIPE), 0);
    EXPECT_EQ(w.map.course().intake_reject(kPipe, v(4.1, 0.2), CourseGeometry{}),
              "course_frame_unset");
    // A class that is no part of any task is a free landmark as before.
    EXPECT_FALSE(w.map.course().intake_reject({LT::PATH_MARKER, 0}, v(2, 0), CourseGeometry{}));
}

TEST(CourseModel, TheGateIsPlacedFromOnePartAtThePriorYaw) {
    World w;
    w.tick({make_track(30, LT::GATE, LS::GATE_WHOLE, v(0.3, -0.2, 2.7), true, false)});
    const auto& gate = w.task("gate");
    ASSERT_TRUE(gate.placed);
    EXPECT_NEAR(std::abs(std::atan2(gate.pose.linear()(1, 0), gate.pose.linear()(0, 0))),
                M_PI, 1e-9);
    // A post seen later joins at its side of the drawing.
    w.tick({make_track(30, LT::GATE, LS::GATE_WHOLE, v(0.3, -0.2, 2.7), true, false),
            make_track(31, LT::GATE, LS::GATE_POLE_EDGE, v(0.3, 1.35, 2.7), true, false)});
    ASSERT_NE(w.in_slot("gate/pole_right"), nullptr);
    EXPECT_EQ(w.in_slot("gate/pole_right")->live_track_id, 31);
}

TEST(CourseModel, TheVotesDecideTheTorpedoVersion) {
    World w;
    // The icon at the top left (board frame +y, up) is reported as blood:
    // version 2.
    TrackVotes votes;
    votes[11] = {{{LT::TORPEDO_BOARD, LS::TORPEDO_ICON_BLOOD}, 6},
                 {{LT::TORPEDO_BOARD, LS::TORPEDO_ICON_FIRE}, 1}};
    for (int i = 0; i < 6; ++i) {
        w.tick(board_tracks(), votes);
    }
    const auto& board = w.task("torpedo");
    ASSERT_TRUE(board.placed);
    EXPECT_TRUE(board.variant_fixed);
    EXPECT_EQ(w.map.course().variant_name(board), "version_2");
    EXPECT_EQ(w.in_slot("torpedo/fire")->key.subtype, LS::TORPEDO_ICON_BLOOD);
}

TEST(CourseModel, WithoutGroupsTheFitFindsTheVersion) {
    // The icons as their own classes: the placement itself says which decal
    // version it is (the icons stand where version 2 has them).
    auto cfg = example_config();
    std::string yaml = kCourse;
    for (const char* line : {"    torpedo_hazards: [TORPEDO_ICON_FIRE, TORPEDO_ICON_BLOOD]\n",
                             "    torpedo_vehicles: [TORPEDO_ICON_FIRETRUCK, TORPEDO_ICON_AMBULANCE]\n"}) {
        yaml.erase(yaml.find(line), std::string(line).size());
    }
    cfg.course = parse_map_config(YAML::Load(yaml)).course;
    RetainedLandmarks map(cfg);
    RetainedLandmarks::CourseInput in;
    in.geometry = at_gate();
    const auto icon = [](int id, uint16_t sub, double by, double bz) {
        return make_track(id, LT::TORPEDO_BOARD, sub, board_part(by, bz), true, false);
    };
    const std::vector<Track> v2 = {
        make_track(10, kBoard.type, kBoard.subtype, board_part(0, 0), true, false),
        icon(11, LS::TORPEDO_ICON_BLOOD, 0.210, -0.222),
        icon(12, LS::TORPEDO_ICON_AMBULANCE, -0.178, -0.204),
        icon(13, LS::TORPEDO_ICON_FIRE, -0.213, 0.056),
        // a blood seen as fire at the firetruck's place: a track that fits
        // no part
        icon(14, LS::TORPEDO_ICON_FIRE, 0.183, 0.183)};
    map.update(v2, 0.0, {}, in);
    const auto& board = map.course().tasks().back();
    ASSERT_EQ(board.spec->name, "torpedo");
    ASSERT_TRUE(board.placed);
    EXPECT_EQ(board.tmpl->variants[board.variant].name, "version_2");
    int icons = 0;
    for (const auto& l : map.landmarks()) {
        icons += l.key.type == LT::TORPEDO_BOARD ? 1 : 0;
        EXPECT_NE(l.live_track_id, 14);
    }
    EXPECT_EQ(icons, 4);
}

TEST(CourseModel, AMislabelledIconCannotRemoveAnother) {
    // Blood seen as fire and a phantom fire 1.6 m away: one kind, one slot
    // each, and a track outside its slot's gate maps nothing.
    World w;
    for (int i = 0; i < 3; ++i) {
        w.tick(board_tracks());
    }
    const int fire_id = w.in_slot("torpedo/fire")->id;
    const int blood_id = w.in_slot("torpedo/blood")->id;
    auto tracks = board_tracks();
    tracks.erase(tracks.begin() + 1);  // the fire's track died
    tracks.push_back(make_track(40, kHazard.type, kHazard.subtype,
                                board_part(-0.219, 0.058) + Eigen::Vector3d(0, 0.03, 0),
                                true, false));  // second track on blood
    tracks.push_back(make_track(41, kHazard.type, kHazard.subtype,
                                board_part(1.6, 0.0), true, false));  // phantom
    for (int i = 0; i < 5; ++i) {
        w.tick(tracks);
    }
    ASSERT_NE(w.in_slot("torpedo/blood"), nullptr);
    EXPECT_EQ(w.in_slot("torpedo/blood")->id, blood_id);
    EXPECT_EQ(w.in_slot("torpedo/fire")->id, fire_id);
    EXPECT_LT((w.in_slot("torpedo/fire")->position - board_part(0.206, -0.214)).norm(), 1e-9);
    EXPECT_EQ(w.count(LT::TORPEDO_BOARD), 5);
}

TEST(CourseModel, OutsideTheFocusTasksAreFrozen) {
    World w;
    ASSERT_FALSE(w.map.course().set_focus({"slalom_1"}, true));
    EXPECT_TRUE(w.map.course().set_focus({"nope"}, true).has_value());
    const std::vector<Track> set_2 = {pipe(21, v(6.0, -1.02)), pipe(22, v(6.0, 0.5)),
                                      pipe(23, v(6.0, 2.02))};
    for (int i = 0; i < 3; ++i) {
        w.tick(set_2);
    }
    EXPECT_FALSE(w.task("slalom_2").placed);
    // Only the locked set could have this pipe.
    EXPECT_EQ(w.map.course().intake_reject(kPipe, v(6.5, 2.02), at_gate()), "task_locked");
    ASSERT_FALSE(w.map.course().set_focus({}, false));
    w.tick(set_2);
    EXPECT_TRUE(w.task("slalom_2").placed);
}

TEST(CourseModel, ACommittedTaskKeepsItsPose) {
    World w;
    w.tick(set_1());
    const Eigen::Vector3d before = w.task("slalom_1").pose.translation();
    ASSERT_FALSE(w.map.course().commit("slalom_1", true));
    for (int i = 0; i < 5; ++i) {
        w.tick({pipe(1, v(4.25, -1.32)), pipe(2, v(4.25, 0.2)), pipe(3, v(4.25, 1.72))});
    }
    EXPECT_LT((w.task("slalom_1").pose.translation() - before).norm(), 1e-9);
}

TEST(CourseModel, TheTrackerMakesNoMoreTracksThanThereArePartsPlusAFew) {
    World w;
    const auto limits = w.map.course().track_limits();
    EXPECT_EQ(limits.at({kPipe.type, kPipe.subtype}), 6 + 2);
    EXPECT_EQ(limits.at({kHazard.type, kHazard.subtype}), 2 + 2);
    EXPECT_EQ(limits.at({LT::GATE, LS::GATE_POLE_EDGE}), 2 + 2);
}

}  // namespace vortex::mission

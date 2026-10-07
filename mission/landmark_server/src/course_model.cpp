#include "landmark_server/course_model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include "landmark_server/class_config.hpp"
#include "landmark_server/retained_landmarks.hpp"

namespace vortex::mission {

namespace {

using ClassPair = std::pair<uint16_t, uint16_t>;

ClassPair pair_of(const LandmarkClassKey& k) {
    return {k.type, k.subtype};
}

LandmarkClassKey parse_key(const std::string& name, const std::string& where) {
    const auto parsed = parse_class_name(name);
    if (!parsed || parsed->second == 0) {
        throw std::runtime_error(where + ": '" + name +
                                 "' is not a full class name (TYPE_SUBTYPE)");
    }
    return {parsed->first, parsed->second};
}

// The keys a course file may have. Anything else is a typo: it would be
// ignored and its default used without a word.
const std::vector<std::string> kTaskFieldKeys = {
    "region_radius_m", "part_radius_m", "yaw_window_deg",
    "symmetric",       "min_parts",     "max_range_m"};
const std::vector<std::string> kCourseKeys = {"enable",
                                              "start",
                                              "class_groups",
                                              "templates",
                                              "tasks",
                                              "extra_tracks_per_kind",
                                              "variant_votes",
                                              "variant_ratio",
                                              "min_slot_gate_m",
                                              "lane_margin_m",
                                              "max_align_deg",
                                              "min_class_agreement",
                                              "min_part_detections"};

std::vector<std::string> with_task_fields(std::vector<std::string> keys) {
    keys.insert(keys.end(), kTaskFieldKeys.begin(), kTaskFieldKeys.end());
    return keys;
}

const std::vector<std::string> kTemplateKeys = with_task_fields(
    {"members", "variants", "points", "sigma", "balanced_classes"});
const std::vector<std::string> kTaskKeys =
    with_task_fields({"template", "prior"});

void check_keys(const YAML::Node& node,
                const std::vector<std::string>& allowed,
                const std::string& where) {
    if (!node || !node.IsMap()) {
        return;
    }
    for (const auto& kv : node) {
        const auto key = kv.first.as<std::string>();
        if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
            throw std::runtime_error(where + ": unknown key '" + key + "'");
        }
    }
}

Eigen::Matrix3d yaw_rotation(double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

double yaw_of(const Eigen::Isometry3d& T) {
    return std::atan2(T.linear()(1, 0), T.linear()(0, 0));
}

Eigen::Isometry3d make_pose(double yaw, const Eigen::Vector3d& t) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = yaw_rotation(yaw);
    T.translation() = t;
    return T;
}

/// Largest distance (xy) of a member from the template origin.
double extent(const StructureTemplate& tmpl) {
    double r = 0.0;
    for (const auto& v : tmpl.variants) {
        for (const auto& m : v.members) {
            r = std::max(r, m.offset.head<2>().norm());
        }
    }
    return r;
}

/// The template with every member's classes replaced by its kind. Variants
/// that are the same in kinds (they differ only in classes of one group)
/// cannot be told apart by fitting: then only the first is kept and the
/// votes decide.
StructureTemplate kind_template(const StructureTemplate& tmpl,
                                const CourseConfig& cfg) {
    StructureTemplate out = tmpl;
    for (auto& v : out.variants) {
        for (auto& m : v.members) {
            m.classes = {cfg.kind_of(m.classes.front())};
        }
    }
    const auto same_kinds = [&](const StructureVariant& a,
                                const StructureVariant& b) {
        for (const auto& m : b.members) {
            const bool found = std::any_of(
                a.members.begin(), a.members.end(), [&](const auto& n) {
                    return (n.offset - m.offset).norm() < 0.05 &&
                           n.classes == m.classes;
                });
            if (!found) {
                return false;
            }
        }
        return true;
    };
    if (std::all_of(out.variants.begin() + 1, out.variants.end(),
                    [&](const auto& v) {
                        return same_kinds(out.variants.front(), v);
                    })) {
        out.variants.resize(1);
    }
    return out;
}

constexpr double kVariantOffsetTol = 0.05;
/// Parts are told apart horizontally (parts of one kind never stand above
/// each other); the height only has to be roughly right, because floor and
/// surface objects are moved to the floor/surface depth.
constexpr double kMaxSlotDz = 0.8;

double slot_distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    if (std::abs(a.z() - b.z()) > kMaxSlotDz) {
        return std::numeric_limits<double>::infinity();
    }
    return (a - b).head<2>().norm();
}
constexpr int kMinVotesForAgreement = 5;

}  // namespace

const TaskSpec* CourseConfig::task(const std::string& name) const {
    for (const auto& t : tasks) {
        if (t.name == name) {
            return &t;
        }
    }
    return nullptr;
}

const StructureTemplate* CourseConfig::template_for(const TaskSpec& t) const {
    for (const auto& tmpl : templates) {
        if (tmpl.name == t.template_name) {
            return &tmpl;
        }
    }
    return nullptr;
}

LandmarkClassKey CourseConfig::kind_of(const LandmarkClassKey& key) const {
    for (const auto& g : class_groups) {
        if (std::find(g.classes.begin(), g.classes.end(), key) !=
            g.classes.end()) {
            return g.classes.front();
        }
    }
    return key;
}

std::set<ClassPair> CourseConfig::templated_kinds() const {
    std::set<ClassPair> out;
    if (!enable) {
        return out;
    }
    for (const auto& t : tasks) {
        const auto* tmpl = template_for(t);
        for (const auto& v : tmpl->variants) {
            for (const auto& m : v.members) {
                for (const auto& c : m.classes) {
                    out.insert(pair_of(kind_of(c)));
                }
            }
        }
    }
    return out;
}

CourseConfig parse_course_config(const YAML::Node& node) {
    CourseConfig cfg;
    if (!node) {
        return cfg;
    }
    const auto get_d = [](const YAML::Node& n, const char* key, double fb) {
        return n[key] ? n[key].as<double>() : fb;
    };
    check_keys(node, kCourseKeys, "course");
    if (const auto templates = node["templates"]) {
        for (const auto& kv : templates) {
            check_keys(kv.second, kTemplateKeys,
                       "course.templates." + kv.first.as<std::string>());
        }
    }
    cfg.enable = node["enable"] ? node["enable"].as<bool>() : false;
    if (const auto s = node["start"]) {
        if (!s.IsSequence() || s.size() != 2) {
            throw std::runtime_error("course.start must be [x, y]");
        }
        cfg.start_xy = {s[0].as<double>(), s[1].as<double>()};
    }
    if (node["extra_tracks_per_kind"]) {
        cfg.extra_tracks_per_kind = node["extra_tracks_per_kind"].as<int>();
    }
    if (node["variant_votes"]) {
        cfg.variant_votes = node["variant_votes"].as<int>();
    }
    cfg.variant_ratio = get_d(node, "variant_ratio", cfg.variant_ratio);
    cfg.min_slot_gate_m = get_d(node, "min_slot_gate_m", cfg.min_slot_gate_m);
    cfg.lane_margin_m = get_d(node, "lane_margin_m", cfg.lane_margin_m);
    cfg.max_align_rad =
        get_d(node, "max_align_deg", cfg.max_align_rad * 180.0 / M_PI) * M_PI /
        180.0;
    cfg.min_class_agreement =
        get_d(node, "min_class_agreement", cfg.min_class_agreement);
    if (node["min_part_detections"]) {
        cfg.min_part_detections = node["min_part_detections"].as<int>();
    }

    if (const auto groups = node["class_groups"]) {
        for (const auto& kv : groups) {
            ClassGroup g;
            g.name = kv.first.as<std::string>();
            const std::string where = "course.class_groups." + g.name;
            if (!kv.second.IsSequence() || kv.second.size() < 2) {
                throw std::runtime_error(where + " needs two or more classes");
            }
            for (const auto& c : kv.second) {
                g.classes.push_back(parse_key(c.as<std::string>(), where));
            }
            for (const auto& other : cfg.class_groups) {
                for (const auto& c : g.classes) {
                    if (std::find(other.classes.begin(), other.classes.end(),
                                  c) != other.classes.end()) {
                        throw std::runtime_error(where + ": " + class_name(c) +
                                                 " is already in " +
                                                 other.name);
                    }
                }
            }
            cfg.class_groups.push_back(std::move(g));
        }
    }

    cfg.templates = parse_structures(node["templates"]);
    for (const auto& tmpl : cfg.templates) {
        const std::string where = "course.templates." + tmpl.name;
        for (const auto& v : tmpl.variants) {
            for (const auto& m : v.members) {
                for (const auto& c : m.classes) {
                    if (c.subtype == 0) {
                        throw std::runtime_error(
                            where + "." + m.name +
                            ": a part needs full class names, not a type");
                    }
                    if (cfg.kind_of(c) != cfg.kind_of(m.classes.front())) {
                        throw std::runtime_error(
                            where + "." + m.name +
                            ": the classes of a part must be one class group");
                    }
                }
            }
        }
        // Variants differ in classes, not in geometry: every member of a
        // variant is at a member of the first.
        const auto& base = tmpl.variants.front().members;
        for (std::size_t vi = 1; vi < tmpl.variants.size(); ++vi) {
            const auto& members = tmpl.variants[vi].members;
            if (members.size() != base.size()) {
                throw std::runtime_error(where +
                                         ": variants need the same parts");
            }
            for (const auto& m : members) {
                const bool found =
                    std::any_of(base.begin(), base.end(), [&](const auto& b) {
                        return (b.offset - m.offset).norm() < kVariantOffsetTol;
                    });
                if (!found) {
                    throw std::runtime_error(
                        where + ".variants." + tmpl.variants[vi].name + "." +
                        m.name + ": no part at this offset in " +
                        tmpl.variants.front().name);
                }
            }
        }
    }

    if (const auto tasks = node["tasks"]) {
        for (const auto& kv : tasks) {
            TaskSpec t;
            t.name = kv.first.as<std::string>();
            const std::string where = "course.tasks." + t.name;
            const YAML::Node& n = kv.second;
            check_keys(n, kTaskKeys, where);
            if (!n["template"]) {
                throw std::runtime_error(where + ": missing template");
            }
            t.template_name = n["template"].as<std::string>();
            const auto prior = n["prior"];
            if (!prior || !prior.IsSequence() || prior.size() != 3) {
                throw std::runtime_error(where +
                                         ".prior must be [x, y, yaw_deg]");
            }
            t.prior_xy = {prior[0].as<double>(), prior[1].as<double>()};
            t.prior_yaw = prior[2].as<double>() * M_PI / 180.0;
            const auto* tmpl = cfg.template_for(t);
            if (tmpl == nullptr) {
                throw std::runtime_error(where + ": unknown template '" +
                                         t.template_name + "'");
            }
            // A task value is the task's own, else its template's (what
            // every task of that prop shares), else the default.
            const YAML::Node tn = node["templates"][t.template_name];
            const auto value = [&](const char* key) {
                return n[key] ? n[key] : tn[key];
            };
            const auto num = [&](const char* key, double fallback) {
                const auto v = value(key);
                return v ? v.as<double>() : fallback;
            };
            t.region_radius_m = num("region_radius_m", t.region_radius_m);
            t.part_radius_m = num("part_radius_m", t.part_radius_m);
            t.yaw_window_rad =
                num("yaw_window_deg", t.yaw_window_rad * 180.0 / M_PI) * M_PI /
                180.0;
            t.symmetric =
                value("symmetric") ? value("symmetric").as<bool>() : false;
            t.max_range_m = num("max_range_m", t.max_range_m);
            t.min_parts = value("min_parts") ? value("min_parts").as<int>()
                                             : tmpl->min_members;
            const int parts =
                static_cast<int>(tmpl->variants.front().members.size());
            if (t.min_parts < 1 || t.min_parts > parts) {
                throw std::runtime_error(where + ".min_parts must be 1.." +
                                         std::to_string(parts));
            }
            if (t.region_radius_m <= 0.0 || t.part_radius_m <= 0.0) {
                throw std::runtime_error(where + ": radii must be > 0");
            }
            cfg.tasks.push_back(std::move(t));
        }
    }
    if (cfg.enable && cfg.tasks.empty()) {
        throw std::runtime_error(
            "course.enable is true but course.tasks is empty");
    }
    return cfg;
}

CourseModel::CourseModel(CourseConfig config) {
    reset(std::move(config));
}

void CourseModel::reset(CourseConfig config) {
    config_ = std::move(config);
    focus_.clear();
    lock_others_ = false;
    build_tasks();
}

void CourseModel::clear() {
    build_tasks();
    track_votes_.clear();
    track_last_vote_.clear();
}

std::optional<CourseModel::LayoutUpdate> CourseModel::update_layout(
    CourseConfig config) {
    if (config.enable != config_.enable ||
        config.tasks.size() != config_.tasks.size()) {
        return std::nullopt;
    }
    // In the order of the tasks there are: tasks_[i] is config_.tasks[i].
    std::vector<TaskSpec> ordered;
    ordered.reserve(config_.tasks.size());
    for (const auto& old : config_.tasks) {
        const TaskSpec* t = config.task(old.name);
        if (t == nullptr || t->template_name != old.template_name) {
            return std::nullopt;
        }
        ordered.push_back(*t);
    }
    config.tasks = std::move(ordered);
    config_ = std::move(config);

    LayoutUpdate update;
    for (std::size_t i = 0; i < tasks_.size(); ++i) {
        Task& t = tasks_[i];
        t.spec = &config_.tasks[i];
        t.tmpl = config_.template_for(*t.spec);
        (t.placed ? update.kept : update.applied).push_back(t.spec->name);
    }
    update_lane();
    return update;
}

void CourseModel::update_lane() {
    lane_min_ = lane_max_ = config_.start_xy;
    for (const auto& spec : config_.tasks) {
        const double r =
            spec.region_radius_m + extent(*config_.template_for(spec));
        lane_min_ =
            lane_min_.cwiseMin(spec.prior_xy - Eigen::Vector2d::Constant(r));
        lane_max_ =
            lane_max_.cwiseMax(spec.prior_xy + Eigen::Vector2d::Constant(r));
    }
    lane_min_ -= Eigen::Vector2d::Constant(config_.lane_margin_m);
    lane_max_ += Eigen::Vector2d::Constant(config_.lane_margin_m);
}

void CourseModel::build_tasks() {
    tasks_.clear();
    update_lane();
    for (const auto& spec : config_.tasks) {
        Task t;
        t.spec = &spec;
        t.tmpl = config_.template_for(spec);
        const auto& base = t.tmpl->variants.front().members;
        for (std::size_t i = 0; i < base.size(); ++i) {
            Slot s;
            s.name = base[i].name;
            for (const auto& v : t.tmpl->variants) {
                std::size_t best = 0;
                double best_d = std::numeric_limits<double>::infinity();
                for (std::size_t j = 0; j < v.members.size(); ++j) {
                    const double d =
                        (v.members[j].offset - base[i].offset).norm();
                    if (d < best_d) {
                        best_d = d;
                        best = j;
                    }
                }
                s.member.push_back(best);
                s.kinds.push_back(
                    config_.kind_of(v.members[best].classes.front()));
            }
            double nearest = std::numeric_limits<double>::infinity();
            for (std::size_t j = 0; j < base.size(); ++j) {
                if (j != i) {
                    nearest = std::min(
                        nearest, (base[j].offset - base[i].offset).norm());
                }
            }
            s.gate_m = std::isfinite(nearest)
                           ? std::max(config_.min_slot_gate_m, 0.5 * nearest)
                           : spec.part_radius_m;
            t.slots.push_back(std::move(s));
        }
        tasks_.push_back(std::move(t));
    }
}

bool CourseModel::in_focus(const Task& t) const {
    return focus_.empty() || std::find(focus_.begin(), focus_.end(),
                                       t.spec->name) != focus_.end();
}

bool CourseModel::locked(const Task& t) const {
    return lock_others_ && !focus_.empty() && !in_focus(t);
}

std::optional<std::string> CourseModel::set_focus(
    const std::vector<std::string>& tasks,
    bool lock_others) {
    for (const auto& name : tasks) {
        if (config_.task(name) == nullptr) {
            return "unknown task '" + name + "'";
        }
    }
    focus_ = tasks;
    lock_others_ = lock_others;
    return std::nullopt;
}

std::optional<std::string> CourseModel::commit(const std::string& name,
                                               bool on) {
    for (auto& t : tasks_) {
        if (t.spec->name == name) {
            t.committed = on;
            return std::nullopt;
        }
    }
    return "unknown task '" + name + "'";
}

Eigen::Vector3d CourseModel::slot_position(const Task& t,
                                           std::size_t slot) const {
    const auto& m =
        t.tmpl->variants[t.variant].members[t.slots[slot].member[t.variant]];
    return t.pose * m.offset;
}

LandmarkClassKey CourseModel::slot_class(const Task& t,
                                         std::size_t slot) const {
    const Slot& s = t.slots[slot];
    if (s.assigned) {
        return *s.assigned;
    }
    const auto& m = t.tmpl->variants[t.variant].members[s.member[t.variant]];
    // The classes this part can have: of the variant in use, or, while the
    // variant is not decided, of any variant (then the part shows what was
    // seen there, not a guess of the variant).
    std::vector<LandmarkClassKey> classes = m.classes;
    if (t.tmpl->variants.size() > 1 && !t.variant_fixed && !t.committed) {
        for (std::size_t vi = 0; vi < t.tmpl->variants.size(); ++vi) {
            for (const auto& c :
                 t.tmpl->variants[vi].members[s.member[vi]].classes) {
                if (std::find(classes.begin(), classes.end(), c) ==
                    classes.end()) {
                    classes.push_back(c);
                }
            }
        }
    }
    if (classes.size() == 1) {
        return classes.front();
    }
    LandmarkClassKey best = m.classes.front();
    int best_votes = 0;
    const auto votes = part_votes(t, slot);
    for (const auto& c : classes) {
        const auto it = votes.find(pair_of(c));
        const int n = it == votes.end() ? 0 : it->second;
        if (n > best_votes) {
            best_votes = n;
            best = c;
        }
    }
    return best;
}

int CourseModel::detections(int track_id) const {
    const auto it = track_votes_.find(track_id);
    int n = 0;
    if (it != track_votes_.end()) {
        for (const auto& [c, k] : it->second) {
            n += k;
        }
    }
    return n;
}

void CourseModel::assign_balanced(Task& t) {
    // The parts that allow several classes, and those classes.
    std::vector<std::size_t> multi;
    std::vector<LandmarkClassKey> classes;
    for (std::size_t i = 0; i < t.slots.size(); ++i) {
        const auto& m =
            t.tmpl->variants[t.variant].members[t.slots[i].member[t.variant]];
        if (m.classes.size() < 2) {
            continue;
        }
        multi.push_back(i);
        for (const auto& c : m.classes) {
            if (std::find(classes.begin(), classes.end(), c) == classes.end()) {
                classes.push_back(c);
            }
        }
    }
    if (multi.empty() || classes.empty() ||
        multi.size() % classes.size() != 0) {
        return;
    }
    const int per_class = static_cast<int>(multi.size() / classes.size());
    std::vector<std::map<ClassPair, int>> votes;
    int total = 0;
    for (const auto i : multi) {
        votes.push_back(part_votes(t, i));
        for (const auto& [c, n] : votes.back()) {
            total += n;
        }
    }
    if (total == 0) {
        return;  // nothing seen yet: keep what there is
    }
    // Every assignment with each class per_class times (at most 4! of them).
    std::vector<int> used(classes.size(), 0);
    std::vector<std::size_t> pick(multi.size(), 0);
    std::vector<std::size_t> best_pick;
    int best = -1;
    const std::function<void(std::size_t, int)> search = [&](std::size_t k,
                                                             int score) {
        if (k == multi.size()) {
            if (score > best) {
                best = score;
                best_pick = pick;
            }
            return;
        }
        for (std::size_t c = 0; c < classes.size(); ++c) {
            if (used[c] == per_class) {
                continue;
            }
            const auto it = votes[k].find(pair_of(classes[c]));
            ++used[c];
            pick[k] = c;
            search(k + 1, score + (it == votes[k].end() ? 0 : it->second));
            --used[c];
        }
    };
    search(0, 0);
    for (std::size_t k = 0; k < multi.size(); ++k) {
        t.slots[multi[k]].assigned = classes[best_pick[k]];
    }
}

void CourseModel::publish_pose(Task& t, Store& store) {
    const double yaw = yaw_of(t.pose);
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    bool any_live = false;
    const auto& variant = t.tmpl->variants[t.variant];
    std::map<std::string, const RetainedLandmark*> by_member;
    for (std::size_t i = 0; i < t.slots.size(); ++i) {
        RetainedLandmark* lm = t.slots[i].landmark_id >= 0
                                   ? store.find(t.slots[i].landmark_id)
                                   : nullptr;
        if (lm == nullptr) {
            continue;
        }
        // A part's yaw is the task's (+X out of the front of the prop).
        lm->orientation = q;
        lm->has_orientation = true;
        lm->yaw_locked = true;
        any_live = any_live || lm->is_live();
        by_member[variant.members[t.slots[i].member[t.variant]].name] = lm;
    }
    // Points only once the variant is known: an opening of a guessed decal
    // version would be a wrong target.
    if (t.tmpl->variants.size() > 1 && !t.variant_fixed && !t.committed) {
        return;
    }
    for (const auto& p : variant.points) {
        Eigen::Vector3d base = t.pose.translation();
        double last = 0.0;
        bool live = any_live;
        if (!p.from.empty()) {
            const auto it = by_member.find(p.from);
            if (it != by_member.end()) {
                base = it->second->position;
                last = it->second->last_measurement;
                live = it->second->is_live();
            } else {
                for (std::size_t i = 0; i < t.slots.size(); ++i) {
                    if (variant.members[t.slots[i].member[t.variant]].name ==
                        p.from) {
                        base = slot_position(t, i);
                    }
                }
            }
        }
        RetainedLandmark& lm =
            store.derived(t.spec->name + "/" + p.name, p.cls);
        lm.position = base + q * p.offset;
        lm.orientation = Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw + p.yaw, Eigen::Vector3d::UnitZ()));
        lm.has_orientation = true;
        lm.yaw_locked = true;
        lm.derived_live = live;
        lm.absorbed_by = -1;
        if (last > 0.0) {
            lm.last_measurement = last;
        }
    }
}

void CourseModel::release(Slot& s) {
    const auto it = track_votes_.find(s.track_id);
    if (it != track_votes_.end()) {
        for (const auto& [c, n] : it->second) {
            s.past_votes[c] += n;
        }
    }
    s.track_id = -1;
}

std::map<ClassPair, int> CourseModel::part_votes(const Task& t,
                                                 std::size_t slot) const {
    const Slot& s = t.slots[slot];
    auto out = s.past_votes;
    if (s.track_id >= 0) {
        const auto it = track_votes_.find(s.track_id);
        if (it != track_votes_.end()) {
            for (const auto& [c, n] : it->second) {
                out[c] += n;
            }
        }
    }
    return out;
}

bool CourseModel::slot_accepts(const Task& t,
                               std::size_t slot,
                               const LandmarkClassKey& kind) const {
    const auto& kinds = t.slots[slot].kinds;
    if (t.variant_fixed || t.committed) {
        return kinds[t.variant] == kind;
    }
    return std::find(kinds.begin(), kinds.end(), kind) != kinds.end();
}

std::string CourseModel::variant_name(const Task& t) const {
    if (t.tmpl->variants.size() > 1 && !t.variant_fixed) {
        return "";
    }
    return t.tmpl->variants[t.variant].name;
}

std::map<ClassPair, int> CourseModel::track_limits() const {
    std::map<ClassPair, int> out;
    if (!config_.enable) {
        return out;
    }
    for (const auto& t : tasks_) {
        std::map<ClassPair, int> most;
        for (std::size_t vi = 0; vi < t.tmpl->variants.size(); ++vi) {
            std::map<ClassPair, int> n;
            for (const auto& s : t.slots) {
                ++n[pair_of(s.kinds[vi])];
            }
            for (const auto& [k, c] : n) {
                most[k] = std::max(most[k], c);
            }
        }
        for (const auto& [k, c] : most) {
            out[k] += c;
        }
    }
    for (auto& [kind, n] : out) {
        n += config_.extra_tracks_per_kind;
    }
    return out;
}

Eigen::Vector2d CourseModel::layout_to_odom(const Eigen::Vector2d& l,
                                            const CourseGeometry& geo) const {
    // Before the gate locks the course frame its origin is the start pose.
    return geo.to_odom(geo.at_gate ? l : Eigen::Vector2d(l - config_.start_xy));
}

Eigen::Vector2d CourseModel::odom_to_layout(const Eigen::Vector2d& o,
                                            const CourseGeometry& geo) const {
    const Eigen::Vector2d c = geo.to_course(o);
    return geo.at_gate ? c : Eigen::Vector2d(c + config_.start_xy);
}

Eigen::Isometry2d CourseModel::alignment(const CourseGeometry& geo) const {
    // Where the placed tasks are (layout frame) against their priors: one
    // gives the translation, two or more also the rotation (least squares).
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> pairs;
    for (const auto& t : tasks_) {
        if (t.placed) {
            pairs.emplace_back(
                t.spec->prior_xy,
                odom_to_layout(t.pose.translation().head<2>(), geo));
        }
    }
    Eigen::Isometry2d A = Eigen::Isometry2d::Identity();
    if (pairs.empty()) {
        return A;
    }
    Eigen::Vector2d ps = Eigen::Vector2d::Zero();
    Eigen::Vector2d qs = Eigen::Vector2d::Zero();
    for (const auto& [p, q] : pairs) {
        ps += p;
        qs += q;
    }
    ps /= static_cast<double>(pairs.size());
    qs /= static_cast<double>(pairs.size());
    double angle = 0.0;
    if (pairs.size() >= 2) {
        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (const auto& [p, q] : pairs) {
            const Eigen::Vector2d a = p - ps;
            const Eigen::Vector2d b = q - qs;
            sin_sum += a.x() * b.y() - a.y() * b.x();
            cos_sum += a.dot(b);
        }
        angle = std::clamp(std::atan2(sin_sum, cos_sum), -config_.max_align_rad,
                           config_.max_align_rad);
    }
    A.linear() = Eigen::Rotation2Dd(angle).toRotationMatrix();
    A.translation() = qs - A.linear() * ps;
    return A;
}

Eigen::Isometry3d CourseModel::prior_pose(const Task& t,
                                          const CourseGeometry& geo) const {
    const Eigen::Isometry2d A = alignment(geo);
    const Eigen::Vector2d odom = layout_to_odom(A * t.spec->prior_xy, geo);
    const double turn = std::atan2(A.linear()(1, 0), A.linear()(0, 0));
    return make_pose(
        geo.through_yaw + t.spec->prior_yaw + turn,
        Eigen::Vector3d(odom.x(), odom.y(), t.pose.translation().z()));
}

bool CourseModel::lane_allows(const Eigen::Vector3d& position,
                              const CourseGeometry& geo) const {
    if (!config_.enable || !geo.set) {
        return true;
    }
    const Eigen::Vector2d l =
        alignment(geo).inverse() * odom_to_layout(position.head<2>(), geo);
    return (l.array() >= lane_min_.array()).all() &&
           (l.array() <= lane_max_.array()).all();
}

std::vector<Eigen::Vector2d> CourseModel::lane_corners(
    const CourseGeometry& geo) const {
    if (!config_.enable || !geo.set) {
        return {};
    }
    const Eigen::Isometry2d A = alignment(geo);
    std::vector<Eigen::Vector2d> out;
    for (const auto& c : {Eigen::Vector2d(lane_min_.x(), lane_min_.y()),
                          Eigen::Vector2d(lane_max_.x(), lane_min_.y()),
                          Eigen::Vector2d(lane_max_.x(), lane_max_.y()),
                          Eigen::Vector2d(lane_min_.x(), lane_max_.y())}) {
        out.push_back(layout_to_odom(A * c, geo));
    }
    return out;
}

std::optional<Eigen::Isometry3d> CourseModel::working_pose(
    const Task& t,
    const CourseGeometry& geo) const {
    if (t.placed) {
        return t.pose;
    }
    if (!geo.set) {
        return std::nullopt;
    }
    return prior_pose(t, geo);
}

std::optional<std::string> CourseModel::intake_reject(
    const LandmarkClassKey& kind,
    const Eigen::Vector3d& position,
    const CourseGeometry& geo,
    const std::optional<Eigen::Vector3d>& vehicle) const {
    if (!config_.enable) {
        return std::nullopt;
    }
    bool templated = false;
    bool in_locked = false;
    bool too_far = false;
    // A detection on a part of a placed task that has no part of this kind
    // (a gate post called a pipe) is that part, not an object of its own.
    for (const auto& t : tasks_) {
        if (!t.placed || !geo.set) {
            continue;
        }
        bool has_kind = false;
        for (std::size_t i = 0; i < t.slots.size() && !has_kind; ++i) {
            has_kind = slot_accepts(t, i, kind);
        }
        if (has_kind) {
            continue;
        }
        for (std::size_t i = 0; i < t.slots.size(); ++i) {
            if (t.slots[i].landmark_id >= 0 &&
                slot_distance(slot_position(t, i), position) <
                    t.slots[i].gate_m) {
                return std::string("other_task");
            }
        }
    }
    for (const auto& t : tasks_) {
        bool has_kind = false;
        for (std::size_t i = 0; i < t.slots.size() && !has_kind; ++i) {
            has_kind = slot_accepts(t, i, kind);
        }
        if (!has_kind) {
            continue;
        }
        templated = true;
        if (!geo.set) {
            continue;
        }
        bool inside = false;
        if (t.placed) {
            for (std::size_t i = 0; i < t.slots.size() && !inside; ++i) {
                inside = slot_accepts(t, i, kind) &&
                         (slot_position(t, i) - position).head<2>().norm() <=
                             t.spec->part_radius_m;
            }
        } else {
            inside = (prior_pose(t, geo).translation() - position)
                         .head<2>()
                         .norm() <= t.spec->region_radius_m + extent(*t.tmpl);
        }
        if (!inside) {
            continue;
        }
        if (t.spec->max_range_m > 0.0 && vehicle &&
            (position - *vehicle).norm() > t.spec->max_range_m) {
            too_far = true;
            continue;
        }
        if (!locked(t)) {
            return std::nullopt;
        }
        in_locked = true;
    }
    if (!templated) {
        return std::nullopt;  // not a part of any task: a free class
    }
    if (!geo.set) {
        return std::string("course_frame_unset");
    }
    if (in_locked) {
        return std::string("task_locked");
    }
    return std::string(too_far ? "too_far" : "outside_tasks");
}

std::set<int> CourseModel::update(const std::vector<KindTrack>& tracks,
                                  const TrackVotes& votes,
                                  const CourseGeometry& geo,
                                  Store& store) {
    std::set<int> claimed;
    // Votes per track over its whole life, confirmed or not; a track
    // without detections for a while is gone.
    ++tick_;
    for (const auto& [id, by_class] : votes) {
        for (const auto& [c, n] : by_class) {
            track_votes_[id][c] += n;
        }
        track_last_vote_[id] = tick_;
    }
    std::set<int> alive;
    for (const auto& k : tracks) {
        alive.insert(k.id);
    }
    constexpr int kForgetTicks = 100;
    for (auto it = track_last_vote_.begin(); it != track_last_vote_.end();) {
        if (!alive.contains(it->first) && tick_ - it->second > kForgetTicks) {
            track_votes_.erase(it->first);
            it = track_last_vote_.erase(it);
        } else {
            ++it;
        }
    }
    if (!config_.enable || !geo.set) {
        return claimed;
    }

    // Placed tasks first: they keep their parts.
    for (auto& t : tasks_) {
        if (!t.placed) {
            continue;
        }
        keep_and_fill(t, tracks, claimed, store);
        if (!locked(t) && !t.committed) {
            refit(t, geo, store);
            decide_variant(t);
        }
    }
    for (auto& t : tasks_) {
        if (t.placed) {
            continue;
        }
        t.pose = prior_pose(t, geo);
        if (!locked(t)) {
            place(t, tracks, claimed, geo, store);
        }
    }
    // Classes follow the variant and the votes; parts and points follow
    // the task pose.
    for (auto& t : tasks_) {
        if (t.tmpl->balanced_classes) {
            assign_balanced(t);
        }
        if (t.placed) {
            publish_pose(t, store);
        }
        for (std::size_t i = 0; i < t.slots.size(); ++i) {
            if (RetainedLandmark* lm = t.slots[i].landmark_id >= 0
                                           ? store.find(t.slots[i].landmark_id)
                                           : nullptr) {
                lm->key = slot_class(t, i);
            }
        }
    }
    return claimed;
}

void CourseModel::attach(Task& t,
                         std::size_t slot,
                         int track_id,
                         Store& store) {
    Slot& s = t.slots[slot];
    RetainedLandmark* lm =
        s.landmark_id >= 0 ? store.find(s.landmark_id) : nullptr;
    if (lm == nullptr) {
        lm = &store.create(slot_class(t, slot), t.spec->name + "/" + s.name);
        s.landmark_id = lm->id;
    }
    s.track_id = track_id;
    store.follow(*lm, track_id);
}

void CourseModel::keep_and_fill(Task& t,
                                const std::vector<KindTrack>& tracks,
                                std::set<int>& claimed,
                                Store& store) {
    const auto find_track = [&](int id) -> const KindTrack* {
        for (const auto& k : tracks) {
            if (k.id == id) {
                return &k;
            }
        }
        return nullptr;
    };
    // Parts that stay where the template puts them keep their track.
    for (std::size_t i = 0; i < t.slots.size(); ++i) {
        Slot& s = t.slots[i];
        if (s.track_id < 0) {
            continue;
        }
        const KindTrack* k = find_track(s.track_id);
        const bool keep =
            k != nullptr && slot_accepts(t, i, k->kind) &&
            !claimed.contains(k->id) &&
            slot_distance(k->position, slot_position(t, i)) <= s.gate_m;
        if (keep) {
            claimed.insert(k->id);
            if (RetainedLandmark* lm = store.find(s.landmark_id)) {
                store.follow(*lm, k->id);
            }
            continue;
        }
        if (RetainedLandmark* lm =
                s.landmark_id >= 0 ? store.find(s.landmark_id) : nullptr) {
            lm->live_track_id = -1;  // remembered where it was
        }
        release(s);
    }
    if (locked(t)) {
        return;
    }
    // Free parts take the nearest unclaimed track of their kind in the gate.
    std::vector<std::tuple<double, std::size_t, int>> pairs;
    for (std::size_t i = 0; i < t.slots.size(); ++i) {
        const Slot& s = t.slots[i];
        if (s.track_id >= 0) {
            continue;
        }
        const Eigen::Vector3d expected = slot_position(t, i);
        for (const auto& k : tracks) {
            if (!slot_accepts(t, i, k.kind) || claimed.contains(k.id) ||
                detections(k.id) < config_.min_part_detections) {
                continue;
            }
            const double d = slot_distance(k.position, expected);
            if (d <= s.gate_m) {
                pairs.emplace_back(d, i, k.id);
            }
        }
    }
    std::sort(pairs.begin(), pairs.end());
    for (const auto& [d, i, id] : pairs) {
        if (t.slots[i].track_id >= 0 || claimed.contains(id)) {
            continue;
        }
        claimed.insert(id);
        attach(t, i, id, store);
    }
}

void CourseModel::refit(Task& t, const CourseGeometry& geo, Store& store) {
    const auto& variant = t.tmpl->variants[t.variant];
    std::vector<std::pair<std::size_t, FitLandmark>> members;
    for (const auto& s : t.slots) {
        const RetainedLandmark* lm =
            s.landmark_id >= 0 ? store.find(s.landmark_id) : nullptr;
        if (lm == nullptr) {
            continue;
        }
        FitLandmark f;
        f.id = lm->id;
        f.key = s.kinds[t.variant];
        f.position = lm->position;
        f.covariance = lm->covariance.topLeftCorner<3, 3>() +
                       Eigen::Matrix3d::Identity() * 0.05 * 0.05;
        members.emplace_back(s.member[t.variant], f);
    }
    if (members.empty()) {
        return;
    }
    Eigen::Isometry3d pose = t.pose;
    if (members.size() == 1) {
        const auto& m = variant.members[members.front().first];
        pose.translation() =
            members.front().second.position - pose.linear() * m.offset;
    } else {
        pose = refit_pose(variant, t.pose, members);
    }
    FitPrior prior;
    prior.yaw = geo.through_yaw + t.spec->prior_yaw;
    prior.yaw_window = t.spec->yaw_window_rad;
    prior.symmetric = t.spec->symmetric;
    if (pose.matrix().allFinite() && prior.allows(pose)) {
        t.pose = pose;
    }
}

void CourseModel::decide_variant(Task& t) {
    const auto& variants = t.tmpl->variants;
    if (variants.size() < 2 || t.variant_fixed) {
        return;
    }
    std::vector<int> score(variants.size(), 0);
    for (std::size_t vi = 0; vi < variants.size(); ++vi) {
        for (std::size_t i = 0; i < t.slots.size(); ++i) {
            const auto& m = variants[vi].members[t.slots[i].member[vi]];
            for (const auto& [c, n] : part_votes(t, i)) {
                if (std::find(m.classes.begin(), m.classes.end(),
                              LandmarkClassKey{c.first, c.second}) !=
                    m.classes.end()) {
                    score[vi] += n;
                }
            }
        }
    }
    std::vector<std::size_t> order(variants.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
        order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return score[a] > score[b];
    });
    const int best = score[order[0]];
    const int second = score[order[1]];
    if (best > second) {
        t.variant = order[0];
    }
    if (best >= config_.variant_votes &&
        best >= config_.variant_ratio * static_cast<double>(second)) {
        t.variant_fixed = true;
    }
}

double CourseModel::class_agreement(
    const Task& t,
    const std::vector<std::pair<std::size_t, int>>& parts,
    const TrackVotes& votes) const {
    int agree = 0;
    int total = 0;
    for (const auto& [slot, track_id] : parts) {
        const auto it = votes.find(track_id);
        if (it == votes.end()) {
            continue;
        }
        int n_track = 0;
        for (const auto& [c, n] : it->second) {
            n_track += n;
        }
        if (n_track < kMinVotesForAgreement) {
            continue;
        }
        // Classes this part can have in any variant.
        std::set<ClassPair> ok;
        for (std::size_t vi = 0; vi < t.tmpl->variants.size(); ++vi) {
            for (const auto& c : t.tmpl->variants[vi]
                                     .members[t.slots[slot].member[vi]]
                                     .classes) {
                ok.insert(pair_of(c));
            }
        }
        for (const auto& [c, n] : it->second) {
            total += n;
            agree += ok.contains(c) ? n : 0;
        }
    }
    return total == 0 ? 1.0 : static_cast<double>(agree) / total;
}

void CourseModel::place(Task& t,
                        const std::vector<KindTrack>& tracks,
                        std::set<int>& claimed,
                        const CourseGeometry& geo,
                        Store& store) {
    const Eigen::Isometry3d prior = t.pose;
    const double reach = t.spec->region_radius_m + extent(*t.tmpl);
    std::vector<KindTrack> cands;
    for (const auto& k : tracks) {
        if (claimed.contains(k.id) ||
            detections(k.id) < config_.min_part_detections ||
            (k.position - prior.translation()).head<2>().norm() > reach) {
            continue;
        }
        bool has_kind = false;
        for (std::size_t i = 0; i < t.slots.size() && !has_kind; ++i) {
            has_kind = slot_accepts(t, i, k.kind);
        }
        if (has_kind) {
            cands.push_back(k);
        }
    }
    if (static_cast<int>(cands.size()) < t.spec->min_parts) {
        return;
    }

    std::vector<std::pair<std::size_t, int>> parts;  // (slot, track id)
    Eigen::Isometry3d pose = prior;
    if (t.spec->min_parts == 1) {
        // The nearest track to where the template puts a part of its kind,
        // at the prior yaw.
        double best_d = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < t.slots.size(); ++i) {
            const auto& m = t.tmpl->variants[t.variant]
                                .members[t.slots[i].member[t.variant]];
            for (const auto& k : cands) {
                if (!slot_accepts(t, i, k.kind)) {
                    continue;
                }
                const Eigen::Vector3d p = prior * m.offset;
                const double d = (k.position - p).head<2>().norm();
                if (d <= t.spec->region_radius_m && d < best_d) {
                    best_d = d;
                    parts = {{i, k.id}};
                    pose.translation() = k.position - prior.linear() * m.offset;
                }
            }
        }
        if (parts.empty()) {
            return;
        }
    } else {
        const StructureTemplate kt = kind_template(*t.tmpl, config_);
        std::vector<FitLandmark> free;
        for (const auto& k : cands) {
            free.push_back(
                {k.id, k.kind, k.position,
                 k.covariance + Eigen::Matrix3d::Identity() * 0.05 * 0.05});
        }
        FitPrior fp;
        fp.yaw = yaw_of(prior);
        fp.yaw_window = t.spec->yaw_window_rad;
        fp.symmetric = t.spec->symmetric;
        fp.center = prior.translation().head<2>();
        fp.radius = t.spec->region_radius_m;
        const auto fit = fit_structure(kt, free, fp, t.spec->min_parts);
        if (!fit) {
            return;
        }
        pose = fit->pose;
        // The kind template keeps the variants only when their kinds
        // differ; then the fit also says which variant this is.
        const std::size_t vi =
            kt.variants.size() > 1 ? fit->variant : t.variant;
        if (kt.variants.size() > 1) {
            t.variant = vi;
        }
        for (const auto& [member, id] : fit->members) {
            for (std::size_t i = 0; i < t.slots.size(); ++i) {
                if (t.slots[i].member[vi] == member) {
                    parts.emplace_back(i, id);
                }
            }
        }
    }
    if (class_agreement(t, parts, track_votes_) < config_.min_class_agreement) {
        return;
    }
    t.placed = true;
    t.pose = pose;
    for (const auto& [slot, id] : parts) {
        claimed.insert(id);
        attach(t, slot, id, store);
    }
    (void)geo;
}

void CourseModel::apply_correction(const Eigen::Isometry3d& delta) {
    const double dyaw = std::atan2(delta.linear()(1, 0), delta.linear()(0, 0));
    for (auto& t : tasks_) {
        if (!t.placed) {
            continue;
        }
        t.pose = make_pose(yaw_of(t.pose) + dyaw, delta * t.pose.translation());
    }
}

}  // namespace vortex::mission

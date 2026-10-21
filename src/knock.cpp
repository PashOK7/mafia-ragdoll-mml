// knock: bodies knock living people over (see knock.h).
#include "fmt.h"
#include "knock.h"
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <unordered_map>

namespace knock {
namespace {


// ---- settings (`knock set`, the effects panel's "Knocked over by bodies" tab)
enum Tune { kOn, kSpeed, kMass, kWindow, kTunes };
using Tunable = tunables::Row<Tune>;
Tunable g_tune[kTunes] = {   // each row names its id: Init refuses a table out of order
    // id, name, value, slider min, max, step, section, what
    {kOn, "on", 1, 0, 1, 1, "Knocked over by bodies",
     "1: a body (anything loose in the physics) that strikes a living person hard enough knocks him over, as a car "
     "does, and his body may knock over the next one"},
    {kSpeed, "speed", 0.5f, 0.1f, 3, 0, "Knocked over by bodies",
     "m/s: the speed the blows would give the whole person that knocks him over (a body thrown at about 6 m/s into "
     "someone gives about 0.7)"},
    {kMass, "mass", 75, 40, 150, 0, "Knocked over by bodies", "kg: a person's mass, for that speed"},
    {kWindow, "window", 0.15f, 0.02f, 1, 0, "Knocked over by bodies",
     "s: how long a person's blows add up (a body strikes with several parts over a few steps)"},
};
float T(Tune t) { return g_tune[t].value; }

LogFn g_log;
#define KLOG(...) (g_log ? g_log(__VA_ARGS__) : (void)0)

struct Acc { V3 J{0, 0, 0}, at{0, 0, 0}; float best = 0, peak = 0; double since = 0, knocked = -1e9; };
std::unordered_map<const void *, Acc> g_acc;
double g_time;
unsigned long long g_blows, g_knocks;
std::deque<std::string> g_recent;   // the last decisions and near ones, for `knock`

void Remember(const std::string &s) {
    g_recent.push_back(Fmt("%.1f s: ", g_time) + s);
    if (g_recent.size() > 10) g_recent.pop_front();
}

} // namespace

void Init(LogFn log) {
    g_log = log;
    if (!tunables::InOrder(g_tune)) KLOG("knock: the settings table is out of order");
    tunables::Defaults(g_tune);
}

void Tick(float dt, const std::vector<jolt_world::PersonBlow> &blows, std::vector<Knock> &out) {
    out.clear();
    g_time += dt;
    if (dt <= 0) return;
    const float fade = std::exp(-dt / std::max(T(kWindow), 0.01f));
    for (auto it = g_acc.begin(); it != g_acc.end();) {
        Acc &a = it->second;
        a.J = a.J * fade;
        if (Len(a.J) < 0.5f && g_time - a.knocked > 2.0) {
            // a blow's peak worth keeping for tuning: one that came close
            if (a.peak >= 0.5f * T(kSpeed) && a.peak < T(kSpeed))
                Remember(Fmt("%p: blows up to %.2f m/s, not enough (speed %.2f)", it->first, a.peak, T(kSpeed)));
            it = g_acc.erase(it);
            continue;
        }
        ++it;
    }
    if (T(kOn) < 0.5f) return;
    for (const jolt_world::PersonBlow &b : blows) {
        g_blows++;
        Acc &a = g_acc[b.who];
        a.J += b.J;
        float m = Len(b.J);
        if (m > a.best) { a.best = m; a.at = b.at; }
    }
    const float mass = std::max(T(kMass), 1.0f);
    for (auto &kv : g_acc) {
        Acc &a = kv.second;
        float dv = Len(a.J) / mass;
        a.peak = std::max(a.peak, dv);
        if (dv < T(kSpeed) || g_time - a.knocked < 1.0) continue;
        a.knocked = g_time;
        out.push_back(Knock{const_cast<void *>(kv.first), a.J, a.at, dv});
        a.J = {0, 0, 0};
        a.best = 0;
        a.peak = 0;
    }
}

void Note(const Knock &k, const char *result) {
    g_knocks++;
    Remember(Fmt("%p: %.2f m/s (%.0f N s): %s", k.who, k.dv, Len(k.J), result));
}

void Forget(const void *who) { g_acc.erase(who); }
void Clear() { g_acc.clear(); }

void Tunes(std::vector<tunables::Info> &out) { tunables::List(g_tune, out); }

bool SetTune(const char *name, float value, std::string *out) {
    const Tunable *t = tunables::Set(g_tune, name, value);
    if (out) *out = t ? Fmt("knock: %s = %g (%s)", t->name, value, t->what) : Fmt("knock: no setting '%s' (`knock` lists them)", name);
    return t != nullptr;
}

bool Exec(const std::string &cmd, const std::string &args, std::string *out) {
    if (cmd != "knock") return false;
    char what[32] = "", name[64] = "";
    float v = 0;
    if (sscanf(args.c_str(), "%31s %63s %f", what, name, &v) == 3 && std::string(what) == "set") {
        SetTune(name, v, out);
        return true;
    }
    std::string s = Fmt("knock: %llu blows, %llu knocked over; %u people being struck now;", g_blows, g_knocks,
                        (unsigned)g_acc.size());
    for (const Tunable &t : g_tune) s += Fmt(" %s=%g", t.name, t.value);
    for (const std::string &r : g_recent) s += " | " + r;
    *out = s;
    return true;
}

} // namespace knock

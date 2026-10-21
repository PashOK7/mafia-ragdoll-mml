// Live settings of the effects, the same shape for every effect: what `<effect> set` changes, what the effects
// panel shows (tools/effects_panel) and what effects:: keeps in the settings file next to Game.exe.
#pragma once
#include <string>
#include <vector>

namespace tunables {

// One setting as the panel sees it. `min`..`max`: the slider's range (the panel takes it as a log scale when it
// spans 100 times or more from a positive `min`); `step` 1 for whole numbers, else 0. `restart`: setting it rebuilds
// the effect (the PhysX smoke) or takes effect on its next start (Flow's grid size).
struct Info {
    std::string name;
    float value, def, min, max, step;
    std::string section, what;
    bool restart;
};

// An effect's table row: `id` is its index (the module checks the order once: a row out of order once turned a
// setting into another), the rest as above.
template <class Id> struct Row {
    Id id;
    const char *name;
    float value, min, max, step;
    const char *section, *what;
};

// The rows' first values, captured before anything changes them (call Defaults first in every path that sets one).
template <class R, int N> const float *Defaults(const R (&rows)[N]) {
    static float d[N];
    static bool done;
    if (!done) { for (int i = 0; i < N; i++) d[i] = rows[i].value; done = true; }
    return d;
}

template <class R, int N> bool InOrder(const R (&rows)[N]) {
    for (int i = 0; i < N; i++) if ((int)rows[i].id != i) return false;
    return true;
}

template <class R, int N> void List(const R (&rows)[N], std::vector<Info> &out, int restartRow = -1) {
    const float *d = Defaults(rows);
    for (int i = 0; i < N; i++)
        out.push_back(Info{rows[i].name, rows[i].value, d[i], rows[i].min, rows[i].max, rows[i].step, rows[i].section,
                           rows[i].what, i == restartRow});
}

// Sets a row by name (the range is the slider's, not a limit); null if there is none.
template <class R, int N> const R *Set(R (&rows)[N], const char *name, float v) {
    Defaults(rows);
    for (R &r : rows) {
        if (std::string(r.name) != name) continue;
        r.value = v;
        return &r;
    }
    return nullptr;
}

}

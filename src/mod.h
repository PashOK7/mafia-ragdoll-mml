// What every part of the mod shares: the loader's host table and the mod's handle, the log.
#pragma once
#include <string>
#include "mml/mml.h"

namespace mod {
extern const MmlHost *H;
extern MmlMod *M;
void Log(const char *fmt, ...);
void Warn(const char *fmt, ...);
void Reply(MmlReply *r, const std::string &text);
} // namespace mod

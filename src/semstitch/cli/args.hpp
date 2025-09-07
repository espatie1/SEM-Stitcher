#pragma once
#include <string>
#include <algorithm>

/*
  Simple CLI argument helpers.

  Conventions:
    - Key format: --key=value  (no spaces)
    - Flags:      --key        (boolean presence)
    - Parsing starts at argv[2] because argv[1] is often a subcommand.
      Example:   prog SUBCMD --fps=30 --save=out.png
    - Keys are case-sensitive.

  Notes:
    - If a key is missing or parsing fails, the default value is returned.
    - stoi/stod are used; on error (bad number) we fall back to default.
    - This parser does not handle quotes, repeated keys, or short flags (-k).
*/

/* Get string value for "--key=value". Returns 'def' if not found. */
inline std::string argValue(int argc, char** argv, const std::string& key, const std::string& def = {}) {
    const std::string pref = "--" + key + "=";
    for (int i = 2; i < argc; ++i) {
        std::string a(argv[i]);
        if (a.rfind(pref, 0) == 0) return a.substr(pref.size());
    }
    return def;
}

/* Get int value for "--key=value". Returns 'def' on missing or parse error. */
inline int argValueInt(int argc, char** argv, const std::string& key, int def) {
    try {
        std::string v = argValue(argc, argv, key, "");
        if (v.empty()) return def;
        return std::stoi(v);
    } catch (...) { return def; }
}

/* Get double value for "--key=value". Returns 'def' on missing or parse error. */
inline double argValueDouble(int argc, char** argv, const std::string& key, double def) {
    try {
        std::string v = argValue(argc, argv, key, "");
        if (v.empty()) return def;
        return std::stod(v);
    } catch (...) { return def; }
}

/* Check presence of a boolean flag "--key". */
inline bool argHas(int argc, char** argv, const std::string& key) {
    const std::string flag = "--" + key;
    for (int i = 2; i < argc; ++i) if (flag == argv[i]) return true;
    return false;
}

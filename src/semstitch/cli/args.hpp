#pragma once
#include <string>
#include <algorithm>

inline std::string argValue(int argc, char** argv, const std::string& key, const std::string& def = {}) {
    const std::string pref = "--" + key + "=";
    for (int i = 2; i < argc; ++i) {
        std::string a(argv[i]);
        if (a.rfind(pref, 0) == 0) return a.substr(pref.size());
    }
    return def;
}

inline int argValueInt(int argc, char** argv, const std::string& key, int def) {
    try {
        std::string v = argValue(argc, argv, key, "");
        if (v.empty()) return def;
        return std::stoi(v);
    } catch (...) { return def; }
}

inline double argValueDouble(int argc, char** argv, const std::string& key, double def) {
    try {
        std::string v = argValue(argc, argv, key, "");
        if (v.empty()) return def;
        return std::stod(v);
    } catch (...) { return def; }
}

inline bool argHas(int argc, char** argv, const std::string& key) {
    const std::string flag = "--" + key;
    for (int i = 2; i < argc; ++i) if (flag == argv[i]) return true;
    return false;
}

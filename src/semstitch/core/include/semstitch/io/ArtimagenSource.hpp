#pragma once

#include "semstitch/core/Frame.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// ARTIMAGEN C interface
extern "C" {
#include "libartimagen/artimagen.h" 
}

// C++ class CImage (to read the buffer). If this include is missing,
// you can replace with: #include "third_party/artimagen/src/artimagen_i.h"
#include "libartimagen/artimagen_i.h"

namespace semstitch {

class ArtimagenSource {
public:
    // Path for future use (Lua/YAML scene); not required right now
    explicit ArtimagenSource(const std::string& /*luaSceneFile*/ = "");
    ~ArtimagenSource();

    // Get the next frame (Gray8)
    std::optional<Frame> next();

private:
    // Filter verbose ARTIMAGEN messages
    static void msgCallback(t_message* m);

    // ARTIMAGEN descriptors
    t_gc_definition          gcDef_{};
    t_std_image_def_struct   imgDef_{};

    void* sample_{nullptr}; // CSample*
    void* image_{nullptr};  // CImage* — used temporarily inside next()

    std::vector<std::uint8_t> scratch_; // output 8-bit frame buffer
};

} // namespace semstitch

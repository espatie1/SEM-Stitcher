#pragma once

#include "semstitch/core/Frame.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// C-интерфейс ARTIMAGEN
extern "C" {
#include "libartimagen/artimagen.h" 
}

// C++-класс CImage (чтобы читать буфер). Если инклюд не найдётся,
// можно заменить на: #include "third_party/artimagen/src/artimagen_i.h"
#include "libartimagen/artimagen_i.h"

namespace semstitch {

class ArtimagenSource {
public:
    // путь на будущее (сцена Lua/YAML); сейчас не обязателен
    explicit ArtimagenSource(const std::string& /*luaSceneFile*/ = "");
    ~ArtimagenSource();

    // очередной кадр (Gray8)
    std::optional<Frame> next();

private:
    // фильтрация болтливых сообщений ARTIMAGEN
    static void msgCallback(t_message* m);

    // описатели ARTIMAGEN
    t_gc_definition          gcDef_{};
    t_std_image_def_struct   imgDef_{};

    void* sample_{nullptr}; // CSample*
    void* image_{nullptr};  // CImage* — используется временно внутри next()

    std::vector<std::uint8_t> scratch_; // 8-bit кадр на выход
};

} // namespace semstitch

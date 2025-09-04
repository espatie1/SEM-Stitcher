#pragma once
#include "semstitch/core/Frame.hpp"
#include "libartimagen/artimagen.h"    // ← тот самый оригинальный заголовок

#include <memory>
#include <optional>
#include <vector>
#include <string>

namespace semstitch {

class ArtimagenSource {
public:
    explicit ArtimagenSource(const std::string& luaSceneFile);   // .lua сцена
    ~ArtimagenSource();

    /// очередной кадр (Gray-8); std::nullopt — симулятор закончил последовательность
    std::optional<Frame> next();

private:
    /*  Держим «сырые» хэндлы, которые отдаёт ARTIMAGEN.
        Они освобождаются своими API-функциями.                              */
    void*                       sample_{nullptr};
    t_gc_definition             gcDef_{};
    t_std_image_def_struct      imgDef_{};
    std::vector<std::uint8_t>   scratch_;   // локальный буфер копии пикселей
};

} // namespace semstitch

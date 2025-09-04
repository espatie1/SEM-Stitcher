#pragma once

#include "semstitch/core/Frame.hpp"
#include <memory>
#include <optional>
#include <vector>

namespace artimagen { class Engine; }

namespace semstitch {

class ArtimagenSource {
public:
    explicit ArtimagenSource(const std::string& sceneConfigPath);
    std::optional<Frame> next(); 

private:
    std::unique_ptr<artimagen::Engine> engine_;
    std::vector<std::uint8_t>          buffer_;  
};

} 

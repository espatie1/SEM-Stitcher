cmake -S . -B build -G "Visual Studio 17 2022" `
      -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release

# Real Stella RGBA -> ORB / pose browser ABI target.
# NO pose simulation. No background mapper in this single-threaded build.
# Usage: include this from the pinned Stella root CMakeLists.txt.
add_executable(pai_stella_browser "${CMAKE_CURRENT_LIST_DIR}/stella-wasm-bridge.cc")
target_link_libraries(pai_stella_browser PRIVATE stella_vslam)
set_target_properties(pai_stella_browser PROPERTIES SUFFIX ".js")
target_link_options(pai_stella_browser PRIVATE
  "--no-entry"
  "-sMODULARIZE=1"
  "-sEXPORT_NAME=createPAIStella"
  "-sENVIRONMENT=web,worker,node"
  "-sEXIT_RUNTIME=0"
  "-sALLOW_MEMORY_GROWTH=1"
  "-sEXPORTED_FUNCTIONS=_malloc,_free,_pai_stella_create,_pai_stella_destroy,_pai_stella_last_error,_pai_stella_features_rgba,_pai_stella_track_rgba,_pai_stella_get_diagnostics"
  "-sEXPORTED_RUNTIME_METHODS=ccall,cwrap,UTF8ToString"
)

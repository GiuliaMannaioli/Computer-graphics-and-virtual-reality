cmake_minimum_required(VERSION 3.0)

SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

project(tpShadow)

add_executable(
  tpShadow
  main.cpp
  Error.cpp
  Mesh.cpp
  ShaderProgram.cpp)


target_sources(${PROJECT_NAME} PRIVATE dep/glad/src/glad.c)
target_include_directories(${PROJECT_NAME} PRIVATE dep/glad/include/)

add_subdirectory(dep/glfw)
target_link_libraries(${PROJECT_NAME} glfw)

add_subdirectory(dep/glm)
target_link_libraries(${PROJECT_NAME} glm)

target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS})

add_custom_command(TARGET tpShadow
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:tpShadow> ${CMAKE_CURRENT_SOURCE_DIR})

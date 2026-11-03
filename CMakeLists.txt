# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# 告诉系统，除了默认的 components，还有一个专门放中间件的目录，可以用这种方法
#set(EXTRA_COMPONENT_DIRS "xxx")


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(xbox_controller_driver)

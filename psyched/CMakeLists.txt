cmake_minimum_required(VERSION 3.8)
project(psyched)

find_package(ament_cargo REQUIRED)

ament_cargo_build()

ament_package()

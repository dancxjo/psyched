cmake_minimum_required(VERSION 3.8)
project(pilot)

find_package(ament_cargo REQUIRED)

cargo_ament_build()

ament_package()

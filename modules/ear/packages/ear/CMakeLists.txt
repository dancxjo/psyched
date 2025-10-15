cmake_minimum_required(VERSION 3.16)
project(ear)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

install(
	PROGRAMS
		scripts/ear_audio_capture
		scripts/ear_vad
		scripts/ear_silence
		scripts/ear_transcriber
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()

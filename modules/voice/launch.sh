#!/usr/bin/env bash

ros2 launch voice voice.launch.py \
	${ENGINE_ARG} \
	${TOPIC_ARG} \
	${PAUSE_ARG} \
	${RESUME_ARG} \
	${CLEAR_ARG} \
	${INTERRUPT_ARG} \
	${MODEL_ARG} \
	${@:-}

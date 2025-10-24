"""Launch description for the Conversant dialog manager."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _declare(name: str, default: str = "", description: str = "") -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description() -> LaunchDescription:
    mode = LaunchConfiguration("mode")
    silence_ms = LaunchConfiguration("silence_ms")
    thread_ttl = LaunchConfiguration("thread_ttl_seconds")
    filler_phrases = LaunchConfiguration("filler_phrases")
    local_llm_url = LaunchConfiguration("local_llm_url")
    local_llm_model = LaunchConfiguration("local_llm_model")
    memory_topic = LaunchConfiguration("memory_topic")
    vad_topic = LaunchConfiguration("vad_topic")
    silence_topic = LaunchConfiguration("silence_topic")
    speech_topic = LaunchConfiguration("speech_topic")
    pause_topic = LaunchConfiguration("pause_topic")
    resume_topic = LaunchConfiguration("resume_topic")
    clear_topic = LaunchConfiguration("clear_topic")
    hesitate_topic = LaunchConfiguration("hesitate_topic")
    concern_topic = LaunchConfiguration("concern_topic")
    turn_control_topic = LaunchConfiguration("turn_control_topic")
    spoken_topic = LaunchConfiguration("spoken_topic")

    arguments = [
        _declare("mode", "balanced", "Turn-taking preset: aggressive, balanced, or patient."),
        _declare("silence_ms", "900", "Silence threshold in milliseconds before resuming speech."),
        _declare("thread_ttl_seconds", "180", "Idle seconds before conversation threads expire."),
        _declare("filler_phrases", "", "Optional comma-separated list of filler phrases."),
        _declare("local_llm_url", "", "Optional HTTP endpoint for short concern responses."),
    _declare("local_llm_model", "", "Optional model identifier when using an Ollama endpoint."),
        _declare("memory_topic", "/conversant/memory_event", "Topic used to publish conversation memory events."),
        _declare("vad_topic", "/ear/speech_active", "Speech activity topic from the Ear module."),
        _declare("silence_topic", "/ear/silence", "Silence detection topic from the Ear module."),
        _declare("speech_topic", "/voice", "Voice module topic for queued speech."),
        _declare("pause_topic", "/voice/pause", "Voice pause topic."),
        _declare("resume_topic", "/voice/resume", "Voice resume topic."),
        _declare("clear_topic", "/voice/clear", "Voice clear topic."),
        _declare("hesitate_topic", "/conversant/hesitate", "Topic that triggers immediate filler speech."),
        _declare("concern_topic", "/conversant/concern", "Topic for concern payloads."),
        _declare("turn_control_topic", "/conversant/turn_control", "Topic for turn-taking adjustments."),
        _declare("spoken_topic", "/voice/spoken", "Topic that echoes spoken utterances."),
    ]

    parameters = [
        {
            "mode": mode,
            "silence_ms": ParameterValue(silence_ms, value_type=int),
            "thread_ttl_seconds": ParameterValue(thread_ttl, value_type=int),
            "filler_phrases": filler_phrases,
            "local_llm_url": local_llm_url,
            "local_llm_model": local_llm_model,
            "memory_topic": memory_topic,
            "vad_topic": vad_topic,
            "silence_topic": silence_topic,
            "speech_topic": speech_topic,
            "pause_topic": pause_topic,
            "resume_topic": resume_topic,
            "clear_topic": clear_topic,
            "hesitate_topic": hesitate_topic,
            "concern_topic": concern_topic,
            "turn_control_topic": turn_control_topic,
            "spoken_topic": spoken_topic,
        }
    ]

    node = Node(
        package="conversant",
        executable="conversant_agent",
        name="conversant",
        output="screen",
        emulate_tty=True,
        parameters=parameters,
    )

    return LaunchDescription([*arguments, node])

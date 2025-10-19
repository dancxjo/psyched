from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    sensor_type = LaunchConfiguration("sensor_type")
    gpio_pin = LaunchConfiguration("gpio_pin")
    poll_interval = LaunchConfiguration("poll_interval")
    frame_id = LaunchConfiguration("frame_id")
    simulate_when_unavailable = LaunchConfiguration("simulate_when_unavailable")
    simulation_seed = LaunchConfiguration("simulation_seed")
    temperature_topic = LaunchConfiguration("temperature_topic")
    temperature_fahrenheit_topic = LaunchConfiguration("temperature_fahrenheit_topic")
    humidity_topic = LaunchConfiguration("humidity_topic")
    humidity_percent_topic = LaunchConfiguration("humidity_percent_topic")
    status_topic = LaunchConfiguration("status_topic")

    launch_args = [
        DeclareLaunchArgument("sensor_type", default_value="DHT11", description="Physical DHT sensor type"),
        DeclareLaunchArgument("gpio_pin", default_value="D4", description="GPIO pin label for the sensor"),
        DeclareLaunchArgument("poll_interval", default_value="2.5", description="Sensor polling interval in seconds"),
        DeclareLaunchArgument("frame_id", default_value="environment_link", description="Frame id for published messages"),
        DeclareLaunchArgument(
            "simulate_when_unavailable",
            default_value="true",
            description="Fallback to simulated readings when hardware is unavailable",
        ),
        DeclareLaunchArgument("simulation_seed", default_value="-1", description="Deterministic seed for simulations"),
        DeclareLaunchArgument(
            "temperature_topic", default_value="/environment/temperature", description="Absolute temperature topic"
        ),
        DeclareLaunchArgument(
            "temperature_fahrenheit_topic",
            default_value="/environment/temperature_fahrenheit",
            description="Fahrenheit temperature topic",
        ),
        DeclareLaunchArgument("humidity_topic", default_value="/environment/humidity", description="Relative humidity topic"),
        DeclareLaunchArgument(
            "humidity_percent_topic",
            default_value="/environment/humidity_percent",
            description="Humidity percentage topic",
        ),
        DeclareLaunchArgument(
            "status_topic", default_value="/environment/thermostat_status", description="Status/debug topic for the node"
        ),
    ]

    node = Node(
        package="hypothalamus",
        executable="hypothalamus_node",
        name="hypothalamus",
        output="screen",
        parameters=[
            {
                "sensor_type": sensor_type,
                "gpio_pin": gpio_pin,
                "poll_interval": ParameterValue(poll_interval, value_type=float),
                "frame_id": frame_id,
                "simulate_when_unavailable": ParameterValue(simulate_when_unavailable, value_type=bool),
                "simulation_seed": ParameterValue(simulation_seed, value_type=int),
                "temperature_topic": temperature_topic,
                "temperature_fahrenheit_topic": temperature_fahrenheit_topic,
                "humidity_topic": humidity_topic,
                "humidity_percent_topic": humidity_percent_topic,
                "status_topic": status_topic,
            }
        ],
    )

    return LaunchDescription(launch_args + [node])

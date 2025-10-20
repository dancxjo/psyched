import { sourcePsychedEnv } from "./ros_env.ts";
import { runRosCommand } from "./ros_container.ts";

/**
 * Publishes a string message to a ROS 2 topic.
 * @param topic The topic to publish to.
 * @param message The message to publish.
 */
export async function publishString(topic: string, message: string) {
  const env = await sourcePsychedEnv();
  await runRosCommand([
    "ros2",
    "topic",
    "pub",
    "--once",
    topic,
    "std_msgs/msg/String",
    `data: ${message}`,
  ], { env });
}

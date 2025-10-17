import { $ } from "$dax";
import { sourcePsychedEnv } from "./ros_env.ts";

/**
 * Publishes a string message to a ROS 2 topic.
 * @param topic The topic to publish to.
 * @param message The message to publish.
 */
export async function publishString(topic: string, message: string) {
  const env = await sourcePsychedEnv();
  // `dax` doesn't handle nested quotes well, so we escape them for the shell.
  const escapedMessage = message.replace(/'/g, `'"'"'`);
  await $`ros2 topic pub --once ${topic} std_msgs/msg/String 'data: ${escapedMessage}'`
    .env(env);
}

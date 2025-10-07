// @ts-nocheck -- Kinect stream payloads remain untyped ROS messages.
import KinectStreamPanel from "../../islands/KinectStreamPanelIsland.tsx";

export default function EyeModulePage() {
  return (
    <section class="content">
      <KinectStreamPanel />
    </section>
  );
}

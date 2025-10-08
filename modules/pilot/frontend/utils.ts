import { createDefine } from "fresh";

export interface BuildInfo {
  version?: string;
}

export interface CockpitConfig {
  host?: string;
  port?: string;
  protocol?: string;
  url?: string;
}

export interface State {
  buildInfo?: BuildInfo;
  cockpit?: CockpitConfig;
}

const baseDefine = createDefine<State>();

export const define: typeof baseDefine & {
  route: typeof baseDefine.page;
} = {
  ...baseDefine,
  route: baseDefine.page,
};

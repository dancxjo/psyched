import { createDefine } from "fresh";

export interface BuildInfo {
  version?: string;
}

export interface State {
  buildInfo?: BuildInfo;
}

const baseDefine = createDefine<State>();

export const define: typeof baseDefine & {
  route: typeof baseDefine.page;
} = {
  ...baseDefine,
  route: baseDefine.page,
};

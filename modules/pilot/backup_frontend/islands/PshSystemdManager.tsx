import { useState } from "preact/hooks";

import { Card } from "@pilot/components/dashboard.tsx";

type Props = {
  modules: string[];
};

type ResponseBody = {
  ok: boolean;
  error?: string;
};

type Message = {
  kind: "success" | "error";
  text: string;
};

async function post(url: string, module: string) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ module }),
  });
  return response.json() as Promise<ResponseBody>;
}

export default function PshSystemdManager({ modules }: Props) {
  const [module, setModule] = useState(modules[0] ?? "");
  const [message, setMessage] = useState<Message | null>(null);
  const [loading, setLoading] = useState(false);

  async function run(endpoint: string) {
    if (!module) {
      setMessage({ kind: "error", text: "Select a module first" });
      return;
    }
    setLoading(true);
    setMessage(null);
    try {
      const res = await post(endpoint, module);
      if (!res.ok) {
        setMessage({ kind: "error", text: res.error ?? "Action failed" });
      } else {
        setMessage({ kind: "success", text: `Action completed for ${module}` });
      }
    } catch (error) {
      setMessage({ kind: "error", text: String(error) });
    } finally {
      setLoading(false);
    }
  }

  return (
    <Card
      title="Systemd units"
      subtitle="Create and control user-level wrappers for module launch scripts"
      tone="cyan"
    >
      <div class="form-grid">
        <label class="form-field">
          <span class="form-label">Module</span>
          <select
            class="form-control"
            value={module}
            onChange={(event) =>
              setModule((event.target as HTMLSelectElement).value)}
            disabled={loading}
          >
            {modules.map((name) => (
              <option key={name} value={name}>{name}</option>
            ))}
          </select>
        </label>
      </div>
      <div class="button-group button-group--wrap">
        <button
          class="button button--primary"
          type="button"
          disabled={loading}
          onClick={() => run("/api/psh/sys/setup")}
        >
          Setup
        </button>
        <button
          class="button button--danger"
          type="button"
          disabled={loading}
          onClick={() => run("/api/psh/sys/teardown")}
        >
          Teardown
        </button>
        <button
          class="button button--primary"
          type="button"
          disabled={loading}
          onClick={() => run("/api/psh/sys/enable")}
        >
          Enable
        </button>
        <button
          class="button button--ghost"
          type="button"
          disabled={loading}
          onClick={() => run("/api/psh/sys/disable")}
        >
          Disable
        </button>
        <button
          class="button button--primary"
          type="button"
          disabled={loading}
          onClick={() => run("/api/psh/sys/up")}
        >
          Start
        </button>
        <button
          class="button button--ghost"
          type="button"
          disabled={loading}
          onClick={() => run("/api/psh/sys/down")}
        >
          Stop
        </button>
      </div>
      {message && (
        <p
          class={`note ${
            message.kind === "success" ? "note--success" : "note--alert"
          }`}
        >
          {message.text}
        </p>
      )}
    </Card>
  );
}
